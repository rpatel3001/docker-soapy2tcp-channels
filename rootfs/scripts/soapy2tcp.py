from os import environ
import socket
import atexit
import SoapySDR
from SoapySDR import *
import numpy as np
from scipy.signal import decimate
from threading import Thread, Event
from queue import Queue, Full
from time import sleep


def rx_thread_worker():
    global freq, rate, inbufs, rxq, rx_init
    
    # parse params to open and initialize SoapySDR device
    args = dict(kv.split("=") for kv in environ["SOAPY"].split(","))
    print("[rx] Opening SoapySDR device with parameters: %s"%args)
    sdr = SoapySDR.Device(args)

    try:
        rate = int(environ["RATE"])
        print("[rx] Setting sample rate to: %d"%rate)
        sdr.setSampleRate(SOAPY_SDR_RX, 0, rate)
    except KeyError:
        print("[rx] Missing RATE env var!")
        rate = sdr.getSampleRate(SOAPY_SDR_RX, 0)

    try:
        ppm = int(environ["PPM"])
        print("[rx] Setting frequency correction to: %d"%ppm)
        sdr.setFrequencyCorrection(SOAPY_SDR_RX, 0, ppm)
    except KeyError:
        pass

    try:
        freq = int(environ["FREQ"])
        print("[rx] Setting center frequency to: %d"%freq)
        sdr.setFrequency(SOAPY_SDR_RX, 0, freq)
    except KeyError:
        print("[rx] Missing FREQ env var!")

    try:
        bw = int(environ["BANDWIDTH"])
        print("[rx] Setting filter bandwidth to: %d"%bw)
        sdr.setBandwidth(SOAPY_SDR_RX, 0, bw)
    except KeyError:
        pass

    try:
        sdr.setGainMode(SOAPY_SDR_RX, 0, False)
        try:
            gain = float(environ["GAIN"])
            print("[rx] Setting gain to: %f"%gain)
            sdr.setGain(SOAPY_SDR_RX, 0, gain)
        except ValueError:
            gains = dict(kv.split("=") for kv in environ["GAIN"].split(","))
            for g in gains:
                if(g.lower() == "agc"):
                    print("[rx] Enabling AGC")
                    sdr.setGainMode(SOAPY_SDR_RX, 0, True)
                    if(gains[g].lower() != "true"):
                        print("[rx] Setting AGC setpoint to: %f"%gains[g])
                        sdr.writeSetting("agc_setpoint", gains[g])
                else:
                    print("[rx] Setting gain %s to: %f"%(g, float(gains[g])))
                    sdr.setGain(SOAPY_SDR_RX, 0, g, float(gains[g]))
    except KeyError:
        pass

    rxStream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
    atexit.register(sdr.closeStream, rxStream)

    mtu = sdr.getStreamMTU(rxStream)
    print("[rx] Using stream MTU: %d"%mtu)

    numbufs = 500
    print("[rx] Using %d receive buffers"%numbufs)

    sdr.activateStream(rxStream)
    atexit.register(sdr.deactivateStream, rxStream)

    inbufs = np.zeros((numbufs, mtu), np.complex64)
    bufidx = 0
    rxq = Queue(numbufs)

    rx_init.set()

    while True:
        status = sdr.readStream(rxStream, [inbufs[bufidx]], mtu)
        if status.ret < 0:
            print("[rx] failed to read stream: %s"%status)
            sdr.deactivateStream(rxStream)
            sdr.closeStream(rxStream)
            del sdr
            raise BufferError
        rxq.put_nowait((bufidx, status.ret))
        bufidx = (bufidx+1) % numbufs


def tx_thread(fc, deci, conn, addr):
    global freq, rate, inbufs, rxq, rx_init

    with conn:
        print(f"[chan] Connection accepted from {addr}")
        conn.sendall(b"RTL0\x00\x00\x00\x00\x00\x00\x00\x00")
        
        offset = 0

        while True:
            b = rxq.get()
            sigbuf = inbufs[b[0], :b[1]]
            if deci == 1:
                outbuf = np.zeros(len(sigbuf) * 2, np.uint8)
                outbuf[0::2] = np.real(sigbuf) * 127 + 127
                outbuf[1::2] = np.imag(sigbuf) * 127 + 127
            else:
                t1 = np.arange(offset, offset + len(sigbuf)) / rate
                offset = (offset + len(sigbuf)) % rate

                fmix = freq - fc

                mix = np.exp(1j * 2*np.pi * fmix * t1)

                sig = decimate(sigbuf * mix, deci)

                outbuf = np.zeros(len(sigbuf) * 2 // deci, np.uint8)
                outbuf[0::2] = np.real(sig) * 127 + 127
                outbuf[1::2] = np.imag(sig) * 127 + 127
            try:
                conn.sendall(outbuf)
            except BaseException:
                print(f"[chan] Disconnected from {addr}")
                return


def chan_thread_worker(fc, deci, port):
    global freq, rate

    rx_init.wait()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        print("[chan] Shifting by %f MHz (%f to %f) and decimating to %f MHz (%f / %d)"%((freq-fc)/1e6, freq/1e6, fc/1e6,(rate/deci)/1e6, rate/1e6, deci))
        print(f"[chan] Listening on port {port}")
        sock.bind(("0.0.0.0", port))
        sock.listen()

        while True:
            try:
                conn, addr = sock.accept()
                conn.settimeout(None)
                Thread(target=tx_thread, args=(fc, deci, conn, addr)).start()
            except BaseException:
                pass


def rx_thread():
    while True:
        try:
            rx_thread_worker()
        except BufferError:
            print("[rx] soapysdr buffer overflow, read the buffer faster! restarting thread")
        except Full:
            print("[rx] receive queue full, process data faster! restarting thread")
        except BaseException as e:
            print("[rx] {!r}; restarting thread".format(e))
        else:
            print("[rx] exited normally, bad thread; restarting")


def chan_thread(fc, deci, port):
    while True:
        try:
            chan_thread_worker(fc, deci, port)
        except BaseException as e:
            print("[chan] {!r}; restarting thread".format(e))
        else:
            print("[chan] exited normally; restarting thread")


def main():
    global rx_init, rxq

    socket.setdefaulttimeout(0)

    rx_init = Event()

    # start a thread to receive samples from the SDR
    rxt = Thread(target=rx_thread)
    rxt.start()

    # semicolon separated list of comma separated channel settings
    # 0: center frequency
    # 1: decimation factor
    # 2: output port
    chans = list(tuple(map(int, c.split(","))) for c in environ["CHANS"].split(";"))

    # start new TX threads for each output channel
    for i, (fc, deci, port) in enumerate(chans):
        print(f"[main] Starting new tx channel thread {i} with params: fc = {fc}, deci = {deci}, port = {port}")
        Thread(target=chan_thread, args=(fc,deci, port)).start()
    
    rxt.join()


if __name__ == "__main__":
    main()