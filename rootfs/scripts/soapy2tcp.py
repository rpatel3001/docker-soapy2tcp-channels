from os import environ
import socket
import atexit
import SoapySDR
from SoapySDR import *
import numpy as np
from threading import Thread, Event
from queue import Queue, Full
from time import time, sleep
import traceback
from scipy.signal import cheby2, sosfilt, sosfilt_zi


def rx_thread(sdr, rxStream, rxcfg, tx_init, inbufs, rxq):
    bufidx = 0
    last_cleared = time()

    sdr.activateStream(rxStream)
    atexit.register(sdr.deactivateStream, rxStream)

    status = sdr.readStream(rxStream, [inbufs[bufidx]], rxcfg["mtu"])
    print(f"[rx] Actual stream transfer size: {status.ret}")

    while True:
        status = sdr.readStream(rxStream, [inbufs[bufidx]], rxcfg["mtu"])
        samps = status.ret
        if samps < 0:
            print(f"[rx] failed to read stream: {status}")
            continue

        for i in range(len(rxq)):
            try:
                if tx_init[i].is_set():
                    rxq[i].put_nowait((bufidx, samps))
            except Full:
                print("[rx] TX %d receive buffers full after %f seconds, clearing queue" %
                    (i, (time() - last_cleared)))
                last_cleared = time()
                with rxq[i].mutex:
                    rxq[i].queue.clear()
        bufidx = (bufidx+1) % rxcfg["numbufs"]


def tx_thread(rxcfg, txcfg, tx_init, inbufs, rxq):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        print(f"[tx {txcfg['idx']}] Listening on port {txcfg['baseport'] + txcfg['idx']}")
        print(f"[tx {txcfg['idx']}] Shifting by {(txcfg['fc'] - rxcfg['freq'])/1e6} MHz ({rxcfg['freq']/1e6} to {txcfg['fc']/1e6}) and decimating {rxcfg['rate']/1e6} by {txcfg['deci']} to {rxcfg['rate']/txcfg['deci']/1e6}")
        sock.bind(("0.0.0.0", txcfg["baseport"] + txcfg["idx"]))
        sock.listen()
        conn, addr = sock.accept()
        tx_init[txcfg["idx"]].set()

        with conn:
            print(f"[tx {txcfg['idx']}] Connection accepted from {addr}")
            conn.sendall(b"RTL0\x00\x00\x00\x00\x00\x00\x00\x00")

            outbuf = np.zeros(rxcfg["mtu"] * 2 // txcfg['deci'], np.uint8)
            sos = cheby2(4, 20, 0.9 / txcfg['deci'], output="sos")
            zi = sosfilt_zi(sos)

            fmix = txcfg["fc"] - rxcfg["freq"]
            mixper = int(np.lcm(fmix, rxcfg["rate"]) / fmix)
            mixlen = np.ceil(rxcfg["mtu"] / mixper) * mixper * 2
            mixtime = np.arange(0, mixlen) / rxcfg["rate"]
            mix = np.exp(-1j * 2*np.pi * fmix * mixtime)
            offset = 0

            while True:
                bufidx, insamps = rxq[txcfg['idx']].get()
                outsamps = insamps * 2 // txcfg['deci']
                sigbuf = inbufs[bufidx, :insamps]
                if txcfg['deci'] == 1:
                    outbuf[0:outsamps:2] = np.real(sigbuf) * 127.5 + 127.5
                    outbuf[1:outsamps:2] = np.imag(sigbuf) * 127.5 + 127.5
                else:
                    aabuf, zi = sosfilt(sos, sigbuf * mix[offset:offset+insamps], zi=zi)
                    offset = (offset + insamps) % mixper
                    decbuf = aabuf[::txcfg['deci']]

                    outbuf[0:outsamps:2] = np.real(decbuf) * 127.5 + 127.5
                    outbuf[1:outsamps:2] = np.imag(decbuf) * 127.5 + 127.5
                try:
                    conn.sendall(outbuf[:outsamps])
                except BaseException:
                    print(f"[tx {txcfg['idx']}] Disconnected from {addr}")
                    tx_init[txcfg['idx']].clear()


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

    # parse params to open and initialize SoapySDR device
    args = dict(kv.split("=") for kv in environ["SOAPY"].split(","))
    print(f"[rx] Opening SoapySDR device with parameters: {args}")
    sdr = SoapySDR.Device(args)

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

    # start a thread to receive samples from the SDR
    inbufs = np.zeros((rxcfg["numbufs"], rxcfg["mtu"]), np.complex64)
    rxq = []
    tx_init = []

    rxt = Thread(target=thread_wrapper, args=(rx_thread, sdr, rxStream, rxcfg, tx_init, inbufs, rxq))
    rxt.start()

    # semicolon separated list of comma separated channel settings
    # 0: center frequency
    # 1: decimation factor
    # 2: output port
    chans = list(tuple(map(int, c.split(","))) for c in environ["CHANS"].split(";"))
    # start new TX threads for each output channel
    for i, (fc, deci) in enumerate(chans):
        cfg = {"idx": i, "fc": fc, "deci": deci, **txcfg}
        rxq.append(Queue(rxcfg["numbufs"]))
        tx_init.append(Event())
        Thread(target=thread_wrapper, args=(tx_thread, rxcfg, cfg, tx_init, inbufs, rxq)).start()

    rxt.join()


if __name__ == "__main__":
    main()
