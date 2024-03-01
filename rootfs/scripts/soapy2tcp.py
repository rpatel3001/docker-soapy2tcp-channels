import atexit
import numpy as np
import socket
import traceback
import prctl
from os import environ
from numba import njit
from scipy.signal import iirdesign, sosfilt_zi
from SoapySDR import Device, SoapySDR_errToStr, SOAPY_SDR_RX, SOAPY_SDR_CF32
from threading import Thread, Event
from time import time, sleep
from queue import Queue, Full
from _thread import interrupt_main

def rx_thread(sdrcfg, rxStream, rxcfg, tx_init, inbufs, rxq):
    prctl.set_name("rx")

    bufidx = 0
    last_cleared = [time()] * len(rxq)

    sdr = Device(sdrcfg)
    rxStream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
    sdr.setSampleRate(SOAPY_SDR_RX, 0, rxcfg["rate"])
    sdr.setFrequencyCorrection(SOAPY_SDR_RX, 0, rxcfg["ppm"])
    sdr.setFrequency(SOAPY_SDR_RX, 0, rxcfg["freq"])

    try:
        gain = float(environ.get("GAIN", 40))
        print(f"[rx] Setting gain to: {gain}")
        sdr.setGainMode(SOAPY_SDR_RX, 0, False)
        sdr.setGain(SOAPY_SDR_RX, 0, gain)
    except ValueError:
        gains = dict(kv.split("=") for kv in environ.get("GAIN", "agc=-30" if sdrcfg["driver"] == "sdrplay" else "Automatic=40").split(","))
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

    sdr.activateStream(rxStream)

    status = sdr.readStream(rxStream, [inbufs[bufidx]], rxcfg["mtu"])
    print(f"[rx] Actual stream transfer size: {status.ret}/{rxcfg['mtu']}")
    realmtu = status.ret

    while True:
        status = sdr.readStream(rxStream, [inbufs[bufidx]], rxcfg["mtu"])
        samps = status.ret
        if samps < 0:
            print(f"[rx] failed to read stream, restarting thread: {SoapySDR_errToStr(status.ret)}: {status}")
            sdr.deactivateStream(rxStream)
            sdr.closeStream(rxStream)
            if environ.get("EXIT_ON_ERROR"):
                print("[rx] Quitting script...")
                for i in range(len(rxq)):
                    rxq[i].put_nowait((-1, -1))
                raise StopIteration
            else:
                print("[rx] Restarting thread...")
                return
        elif samps < realmtu:
            print(f"[rx] Got only {samps} samples, expected {realmtu}")

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


@njit("(complex64[:, ::1], complex64[::1], complex64[:, ::1])", nogil=True, fastmath=True)
def fastfilt(sos, x, zi):
    for n in range(x.shape[0]):
        for s in range(sos.shape[0]):
            x_n = x[n]
            x[n] = sos[s, 0] * x_n + zi[s, 0]
            zi[s, 0] = (sos[s, 1] * x_n - sos[s, 4] * x[n] +
                            zi[s, 1])
            zi[s, 1] = (sos[s, 2] * x_n - sos[s, 5] * x[n])


def tx_thread(rxcfg, chancfg, tx_init, inbufs, rxq):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        prctl.set_name(f"tx {chancfg['idx']}")
        print(f"[tx {chancfg['idx']}] Listening on port {chancfg['baseport'] + chancfg['idx']}")
        print(f"[tx {chancfg['idx']}] Shifting by {(chancfg['fc'] - rxcfg['freq'])/1e6} MHz ({rxcfg['freq']/1e6} to {chancfg['fc']/1e6}) and decimating {rxcfg['rate']/1e6} by {chancfg['deci']} to {rxcfg['rate']/chancfg['deci']/1e6}")

        outbuf = np.zeros(rxcfg["mtu"] * 2 // chancfg['deci'], np.uint8)
        fdtype = np.complex64

        if chancfg['deci'] != 1:
            # setup mixer LO
            fmix = chancfg["fc"] - rxcfg["freq"] # amount to shift by
            mixper = int(np.lcm(fmix, rxcfg["rate"]) / fmix) # period of the mixer frequency sampled at the sample rate
            mixlen = int(np.ceil(rxcfg["mtu"] / mixper)) * mixper * 2 # the smallest periodic buffer length that fits the max MTU, times 2
            mixtime = np.arange(0, mixlen) / rxcfg["rate"] # sample times
            mix = np.exp(-1j * 2*np.pi * fmix * mixtime).astype(fdtype) * 255.0 # LO buffer, scaled to int8
            offset = 0 # keep track of where in the LO buffer we are
            # setup filter
            wp = 0.80 / chancfg['deci'] # passband stop (% Nyquist)
            ws = 1.20 / chancfg['deci'] # stopband start (% Nyquist)
            rp = 0.75 # passband ripple (dB)
            rs = 40 # stopband attenuation (dB)
            sos = iirdesign(wp, ws, rp, rs, output="sos").astype(fdtype) # get filter coefficients
            zi = sosfilt_zi(sos).astype(fdtype) # calculate initial conditions
            # buffer for decimation
            decbuf = np.zeros(rxcfg["mtu"] // chancfg['deci'], fdtype)

        # wait for a connection, then signal RX thread to push to the queue
        sock.bind(("0.0.0.0", chancfg["baseport"] + chancfg["idx"]))
        sock.listen()
        conn, addr = sock.accept()
        tx_init[chancfg["idx"]].set()

        with conn:
            print(f"[tx {chancfg['idx']}] Connection accepted from {addr}")
            conn.sendall(b"RTL0\x00\x00\x00\x00\x00\x00\x00\x00") # rtl-tcp header

            while True:
                bufidx, insamps = rxq.get() # receive a buffer index and length
                if bufidx < 0:
                    print(f"[tx {chancfg['idx']}] Got negative bufidx, quitting thread")
                    raise StopIteration
                decsamps = insamps // chancfg['deci']
                outsamps = decsamps * 2
                # copy out the received samples
                sigbuf = np.array(inbufs[bufidx][:insamps], fdtype, order='C')
                if chancfg['deci'] == 1:
                    # if no decimation, just scale and shift, don't mix/filter
                    outbuf[:outsamps] = fastscale(sigbuf.view(np.float32))
                else:
                    fastmult(sigbuf, mix[offset:offset+insamps]) # mix with LO
                    offset = (offset + insamps) % mixper
                    fastfilt(sos, sigbuf, zi) # filter
                    decbuf[:decsamps] = sigbuf[:insamps:chancfg['deci']] # decimate
                    outbuf[:outsamps] = fastshift(decbuf[:decsamps].view(np.float32)) # shift to uint8 range
                try:
                    conn.sendall(outbuf[:outsamps])
                except BaseException:
                    print(f"[tx {chancfg['idx']}] Disconnected from {addr}")
                    tx_init[txcfg['idx']].clear()
                    return


# wrapper to catch exceptions and restart threads
def thread_wrapper(func, *args):
    while True:
        try:
            print(f"[{func.__name__}] starting thread")
            func(*args)
        except StopIteration:
            print(traceback.format_exc())
            print(f"[{func.__name__}] exception; quitting script")
            return
        except BaseException:
            print(traceback.format_exc())
            print(f"[{func.__name__}] exception; restarting thread")
        else:
            print(f"[{func.__name__}] thread function returned; restarting thread")
        sleep(1)


def main():
    prctl.set_name("main")
    rxcfg = {}
    txcfg = {}

    sdrcfg = dict(kv.split("=") for kv in environ["SOAPY"].split(","))
    print(f"[rx] Using SoapySDR device with parameters: {sdrcfg}")

    txcfg["baseport"] = int(environ.get("BASEPORT", 1234))

    rxcfg["numbufs"] = int(environ.get("NUMBUFS", 100))
    print(f"[main] Using {rxcfg['numbufs']} bufs")

    rxcfg["rate"] = int(environ.get("RATE", 2100000))
    print(f"[main] Sampling at {rxcfg['rate']} MHz")

    rxcfg["ppm"] = int(environ.get("PPM", 0))
    print(f"[main] Using {rxcfg['ppm']} ppm offset")

    rxcfg["freq"] = int(environ.get("FREQ", 136500000))
    print(f"[main] Tuning to {rxcfg['freq']} MHz")

    sdr = Device(sdrcfg)
    rxStream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
    rxcfg["mtu"] = sdr.getStreamMTU(rxStream)
    sdr.closeStream(rxStream)
    del sdr
    print(f"[rx] Using stream MTU: {rxcfg['mtu']}")

    inbufs = np.zeros((rxcfg["numbufs"], rxcfg["mtu"]), np.complex64)
    rxq = []
    tx_init = []

    # semicolon separated list of comma separated channel settings
    # 0: center frequency
    # 1: decimation factor
    chans = list(tuple(map(int, c.split(","))) for c in environ.get("CHANS", f"rxcfg['freq'],1").split(";"))
    # start new TX threads for each output channel
    for i, (fc, deci) in enumerate(chans):
        chancfg = {"idx": i, "fc": fc, "deci": deci, **txcfg}
        rxq.append(Queue(rxcfg["numbufs"]))
        tx_init.append(Event())
        Thread(name=f"tx {i}", target=thread_wrapper, args=(tx_thread, rxcfg, chancfg, tx_init, inbufs, rxq[i])).start()

    # start a thread to receive samples from the SDR
    rxt = Thread(name="rx", target=thread_wrapper, args=(rx_thread, sdrcfg, rxStream, rxcfg, tx_init, inbufs, rxq))
    rxt.start()

    rxt.join()


if __name__ == "__main__":
    main()
