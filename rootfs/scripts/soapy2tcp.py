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


def rx_thread():
    global freq, rate, inbufs, mtu, rxq, rx_init, tx_init

    # parse params to open and initialize SoapySDR device
    args = dict(kv.split("=") for kv in environ["SOAPY"].split(","))
    print(f"[rx] Opening SoapySDR device with parameters: {args}")
    sdr = SoapySDR.Device(args)

    try:
        rate = int(environ["RATE"])
        print(f"[rx] Setting sample rate to: {rate/1e6} MHz")
        sdr.setSampleRate(SOAPY_SDR_RX, 0, rate)
    except KeyError:
        print("[rx] Missing RATE env var!")
        rate = sdr.getSampleRate(SOAPY_SDR_RX, 0)

    try:
        ppm = int(environ["PPM"])
        print(f"[rx] Setting frequency correction to: {ppm} ppm")
        sdr.setFrequencyCorrection(SOAPY_SDR_RX, 0, ppm)
    except KeyError:
        pass

    try:
        freq = int(environ["FREQ"])
        print(f"[rx] Setting center frequency to: {freq/1e6} MHz")
        sdr.setFrequency(SOAPY_SDR_RX, 0, freq)
    except KeyError:
        print("[rx] Missing FREQ env var!")

    try:
        bw = int(environ["BANDWIDTH"])
        print(f"[rx] Setting filter bandwidth to: {bw/1e6} MHz")
        sdr.setBandwidth(SOAPY_SDR_RX, 0, bw)
    except KeyError:
        pass

    try:
        sdr.setGainMode(SOAPY_SDR_RX, 0, False)
        try:
            gain = float(environ["GAIN"])
            print(f"[rx] Setting gain to: {gain} dB")
            sdr.setGain(SOAPY_SDR_RX, 0, gain)
        except ValueError:
            gains = dict(kv.split("=") for kv in environ["GAIN"].split(","))
            for g in gains:
                if(g.lower() == "agc"):
                    print("[rx] Enabling AGC")
                    sdr.setGainMode(SOAPY_SDR_RX, 0, True)
                    if(gains[g].lower() != "true"):
                        print(f"[rx] Setting AGC setpoint to: {gains[g]} dB")
                        sdr.writeSetting("agc_setpoint", float(gains[g]))
                else:
                    print(f"[rx] Setting gain {g} to: {gains[g]}")
                    sdr.setGain(SOAPY_SDR_RX, 0, g, float(gains[g]))
    except KeyError:
        pass

    rxStream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
    atexit.register(sdr.closeStream, rxStream)

    mtu = sdr.getStreamMTU(rxStream)
    print(f"[rx] Using stream MTU: {mtu}")

    numbufs = 10
    print(f"[rx] Using {numbufs} receive buffers")

    sdr.activateStream(rxStream)
    atexit.register(sdr.deactivateStream, rxStream)

    inbufs = np.zeros((numbufs, mtu), np.complex64)
    bufidx = 0
    rxq = Queue(numbufs)
    last = time()

    rx_init.set()

    status = sdr.readStream(rxStream, [inbufs[bufidx]], mtu)
    print(f"[rx] Actual stream transfer size: {status.ret}")

    while True:
        status = sdr.readStream(rxStream, [inbufs[bufidx]], mtu)
        samps = status.ret
        if samps < 0:
            print(f"[rx] failed to read stream: {status}")
            continue
        try:
            if tx_init.is_set():
                rxq.put_nowait((bufidx, samps))
                bufidx = (bufidx+1) % numbufs
        except Full:
            print("[rx] %d element receive queue full, after %f seconds" %
                  (numbufs, (time() - last)))
            last = time()
            with rxq.mutex:
                rxq.queue.clear()


def tx_thread(fmix, deci, port):
    global freq, rate, inbufs, mtu, rxq, tx_init

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        print(f"[tx] Listening on port {port}")
        sock.bind(("0.0.0.0", port))
        sock.listen()
        conn, addr = sock.accept()
        tx_init.set()

        with conn:
            print(f"[tx] Connection accepted from {addr}")
            conn.sendall(b"RTL0\x00\x00\x00\x00\x00\x00\x00\x00")

            outbuf = np.zeros(mtu * 2 // deci, np.uint8)
            sos = cheby2(4, 20, 0.9 / deci, output="sos")
            zi = sosfilt_zi(sos)

            mixper = int(np.lcm(fmix, rate) / fmix)
            mixlen = np.ceil(mtu / mixper) * mixper * 2
            mixtime = np.arange(0, mixlen) / rate
            mix = np.exp(-1j * 2*np.pi * fmix * mixtime)
            offset = 0

            while True:
                bufidx, insamps = rxq.get()
                outsamps = insamps * 2 // deci
                sigbuf = inbufs[bufidx, :insamps]
                if deci == 1:
                    outbuf[0:outsamps:2] = np.real(sigbuf) * 127.5 + 127.5
                    outbuf[1:outsamps:2] = np.imag(sigbuf) * 127.5 + 127.5
                else:
                    aabuf, zi = sosfilt(sos, sigbuf * mix[offset:offset+insamps], zi=zi)
                    offset = (offset + insamps) % mixper
                    decbuf = aabuf[::deci]

                    outbuf[0:outsamps:2] = np.real(decbuf) * 127.5 + 127.5
                    outbuf[1:outsamps:2] = np.imag(decbuf) * 127.5 + 127.5
                try:
                    conn.sendall(outbuf[:outsamps])
                except BaseException:
                    print(f"[tx] Disconnected from {addr}")
                    return


def thread_wrapper(label, func, *args):
    while True:
        try:
            print(f"[{label}] starting thread")
            func(*args)
        except BaseException:
            print(traceback.format_exc())
            print(f"[{label}] exception; restarting thread")
        else:
            print(f"[{label}] thread function returned; restarting thread")
        sleep(1)


def main():
    global rx_init, rxq, freq, tx_init

    rx_init = Event()
    tx_init = Event()

    # start a thread to receive samples from the SDR
    rxt = Thread(target=thread_wrapper, args=("rx", rx_thread))
    rxt.start()
    rx_init.wait()

    # semicolon separated list of comma separated channel settings
    # 0: center frequency
    # 1: decimation factor
    # 2: output port
    chans = list(tuple(map(int, c.split(","))) for c in environ["CHANS"].split(";"))

    # start new TX threads for each output channel
    for i, (fc, deci, port) in enumerate(chans):
        fmix = fc - freq
        print(
            f"[main] Shifting by {fmix/1e6} MHz ({freq/1e6} to {fc/1e6}) and decimating {rate/1e6} by {deci} to {rate/deci/1e6} on port {port}")
        Thread(target=thread_wrapper, args=("tx", tx_thread, fmix, deci, port)).start()

    rxt.join()


if __name__ == "__main__":
    main()
