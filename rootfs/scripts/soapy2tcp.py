from os import environ
import socket
import atexit
from SoapySDR import Device, SOAPY_SDR_RX, SOAPY_SDR_CF32, SoapySDR_errToStr
import numpy as np
from numba import njit
from threading import Thread, Event
from queue import Queue, Full
from time import time, sleep
import traceback
from scipy.signal import iirdesign, sosfilt_zi
from scipy.signal._sosfilt import _sosfilt


def rx_thread(sdr, rxStream, rxcfg, tx_init, inbufs, rxq):
    bufidx = 0
    last_cleared = [time()] * len(rxq)

    sdr.activateStream(rxStream)
    atexit.register(sdr.deactivateStream, rxStream)

    status = sdr.readStream(rxStream, [inbufs[bufidx]], rxcfg["mtu"])
    print(f"[rx] Actual stream transfer size: {status.ret}/{rxcfg['mtu']}")

    while True:
        status = sdr.readStream(rxStream, [inbufs[bufidx]], rxcfg["mtu"])
        samps = status.ret
        if samps < 0:
            print(f"[rx] failed to read stream: {status} = {SoapySDR_errToStr(status)}")
            continue

        for i in range(len(rxq)):
            try:
                if tx_init[i].is_set():
                    rxq[i].put_nowait((bufidx, samps))
            except Full:
                print("[rx] TX %d: receive buffers full after %f seconds, clearing queue" %
                    (i, (time() - last_cleared[i])))
                last_cleared[i] = time()
                with rxq[i].mutex:
                    rxq[i].queue.clear()
        bufidx = (bufidx+1) % rxcfg["numbufs"]


@njit("(complex64[::1], complex64[::1])", nogil=True, fastmath=True)
def fastmult(x, y):
    x *= y


@njit("uint8[::1](float32[::1])", nogil=True, fastmath=True)
def fastshift(inbuf):
    return (inbuf - 127.5).astype(np.uint8)


@njit("uint8[::1](float32[::1])", nogil=True, fastmath=True)
def fastscale(inbuf):
    return (inbuf * 127.5 - 127.5).astype(np.uint8)


def tx_thread(rxcfg, txcfg, tx_init, inbufs, rxq):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        print(f"[tx {txcfg['idx']}] Listening on port {txcfg['baseport'] + txcfg['idx']}")
        print(f"[tx {txcfg['idx']}] Shifting by {(txcfg['fc'] - rxcfg['freq'])/1e6} MHz ({rxcfg['freq']/1e6} to {txcfg['fc']/1e6}) and decimating {rxcfg['rate']/1e6} by {txcfg['deci']} to {rxcfg['rate']/txcfg['deci']/1e6}")

        outbuf = np.zeros(rxcfg["mtu"] * 2 // txcfg['deci'], np.uint8)
        fdtype = np.complex64

        if txcfg['deci'] != 1:
            # setup mixer LO
            fmix = txcfg["fc"] - rxcfg["freq"] # amount to shift by
            mixper = int(np.lcm(fmix, rxcfg["rate"]) / fmix) # period of the mixer frequency sampled at the sample rate
            mixlen = int(np.ceil(rxcfg["mtu"] / mixper)) * mixper * 2 # the smallest periodic buffer length that fits the max MTU, times 2
            mixtime = np.arange(0, mixlen) / rxcfg["rate"] # sample times
            mix = np.exp(-1j * 2*np.pi * fmix * mixtime).astype(fdtype) * 255.0 # LO buffer, scaled to int8
            offset = 0 # keep track of where in the LO buffer we are
            # setup filter
            wp = 0.8 / txcfg['deci'] # passband stop (% Nyquist)
            ws = 1.0 / txcfg['deci'] # stopband start (% Nyquist)
            rp = 2.5 # passband ripple (dB)
            rs = 35 # stopband attenuation (dB)
            sos = iirdesign(wp, ws, rp, rs, output="sos").astype(fdtype) # get filter coefficients
            zi = sosfilt_zi(sos).astype(fdtype) # calculate initial conditions
            zi.shape = (1, *zi.shape) # add an empty dimension for _sosfilt
            # buffer for decimation
            decbuf = np.zeros(rxcfg["mtu"] // txcfg['deci'], fdtype)

        # wait for a connection, then signal RX thread to push to the queue
        sock.bind(("0.0.0.0", txcfg["baseport"] + txcfg["idx"]))
        sock.listen()
        conn, addr = sock.accept()
        tx_init[txcfg["idx"]].set()

        with conn:
            print(f"[tx {txcfg['idx']}] Connection accepted from {addr}")
            conn.sendall(b"RTL0\x00\x00\x00\x00\x00\x00\x00\x00") # rtl-tcp header

            while True:
                bufidx, insamps = rxq[txcfg['idx']].get() # receive a buffer index and length
                decsamps = insamps // txcfg['deci']
                outsamps = decsamps * 2
                # copy out the received samples, adding an empty dimension for _sosfilt
                sigbuf = np.array(inbufs[bufidx][:insamps], fdtype, order='C', ndmin=2)
                if txcfg['deci'] == 1:
                    # if no decimation, just scale and shift, don't mix/filter
                    outbuf[:outsamps] = fastscale(sigbuf[0].view(np.float32))
                else:
                    fastmult(sigbuf[0], mix[offset:offset+insamps]) # mix with LO
                    offset = (offset + insamps) % mixper
                    _sosfilt(sos, sigbuf, zi) # filter
                    decbuf[:decsamps] = sigbuf[0][:insamps:txcfg['deci']] # decimate
                    outbuf[:outsamps] = fastshift(decbuf[:decsamps].view(np.float32)) # shift to uint8 range
                try:
                    conn.sendall(outbuf[:outsamps])
                except BaseException:
                    print(f"[tx {txcfg['idx']}] Disconnected from {addr}")
                    tx_init[txcfg['idx']].clear()


# wrapper to catch exceptions and restart threads
def thread_wrapper(func, *args):
    while True:
        try:
            print(f"[{func.__name__}] starting thread")
            func(*args)
        except BaseException:
            print(traceback.format_exc())
            print(f"[{func.__name__}] exception; restarting thread")
        else:
            print(f"[{func.__name__}] thread function returned; restarting thread")
        sleep(1)


def main():
    rxcfg = {}
    txcfg = {}

    # parse params to open and initialize SoapySDR device + stream
    args = dict(kv.split("=") for kv in environ["SOAPY"].split(","))
    print(f"[rx] Opening SoapySDR device with parameters: {args}")
    sdr = Device(args)
    rxStream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
    atexit.register(sdr.closeStream, rxStream)

    try:
        txcfg["baseport"] = int(environ["BASEPORT"])
    except KeyError:
        txcfg["baseport"] = 1234
    print(f"[main] Starting output channels at port {txcfg['baseport']}")

    try:
        rxcfg["numbufs"] = int(environ["NUMBUFS"])
    except KeyError:
        rxcfg["numbufs"]  = 10
    print(f"[main] Using {rxcfg['numbufs']} bufs")

    try:
        rxcfg["rate"] = int(environ["RATE"])
        sdr.setSampleRate(SOAPY_SDR_RX, 0, rxcfg["rate"])
    except KeyError:
        rxcfg["rate"] = sdr.getSampleRate(SOAPY_SDR_RX, 0)
    print(f"[main] Sampling at {rxcfg['rate']} MHz")

    try:
        rxcfg["ppm"] = int(environ["PPM"])
        sdr.setFrequencyCorrection(SOAPY_SDR_RX, 0, rxcfg["ppm"])
        print(f"[main] Using {rxcfg['ppm']} ppm offset")
    except KeyError:
        pass

    try:
        rxcfg["freq"] = int(environ["FREQ"])
        sdr.setFrequency(SOAPY_SDR_RX, 0, rxcfg["freq"])
    except KeyError:
        rxcfg["freq"] = sdr.getFrequency(SOAPY_SDR_RX, 0)
    print(f"[main] Tuning to {rxcfg['freq']} MHz")

    try:
        rxcfg["bw"] = int(environ["BANDWIDTH"])
        sdr.setBandwidth(SOAPY_SDR_RX, 0, rxcfg["bw"])
        print(f"[main] Setting {rxcfg['bw']} MHz bandwidth")
    except KeyError:
        pass

    try:
        sdr.setGainMode(SOAPY_SDR_RX, 0, False)
        try:
            gain = float(environ["GAIN"])
            print(f"[rx] Setting gain to: {gain}")
            sdr.setGain(SOAPY_SDR_RX, 0, gain)
        except ValueError:
            gains = dict(kv.split("=") for kv in environ["GAIN"].split(","))
            for g in gains:
                if(g.lower() == "agc"):
                    print("[rx] Enabling AGC")
                    sdr.setGainMode(SOAPY_SDR_RX, 0, True)
                    if(gains[g].lower() != "true"):
                        print("[rx] Setting AGC setpoint to: %f" % float(gains[g]))
                        sdr.writeSetting("agc_setpoint", float(gains[g]))
                else:
                    print("[rx] Setting gain %s to: %f" % (g, float(gains[g])))
                    sdr.setGain(SOAPY_SDR_RX, 0, g, float(gains[g]))
    except KeyError:
        pass

    rxcfg["mtu"] = sdr.getStreamMTU(rxStream)
    print(f"[rx] Using stream MTU: {rxcfg['mtu']}")

    inbufs = np.zeros((rxcfg["numbufs"], rxcfg["mtu"]), np.complex64)
    rxq = []
    tx_init = []

    # semicolon separated list of comma separated channel settings
    # 0: center frequency
    # 1: decimation factor
    chans = list(tuple(map(int, c.split(","))) for c in environ["CHANS"].split(";"))
    # start new TX threads for each output channel
    for i, (fc, deci) in enumerate(chans):
        cfg = {"idx": i, "fc": fc, "deci": deci, **txcfg}
        rxq.append(Queue(rxcfg["numbufs"]))
        tx_init.append(Event())
        Thread(target=thread_wrapper, args=(tx_thread, rxcfg, cfg, tx_init, inbufs, rxq)).start()

    # start a thread to receive samples from the SDR
    rxt = Thread(target=thread_wrapper, args=(rx_thread, sdr, rxStream, rxcfg, tx_init, inbufs, rxq))
    rxt.start()

    rxt.join()


if __name__ == "__main__":
    main()
