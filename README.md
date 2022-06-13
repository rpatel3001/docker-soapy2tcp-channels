# docker-soapy2tcp-channels
[![GitHub Workflow Status](https://img.shields.io/github/workflow/status/rpatel3001/docker-soapy2tcp-channels/Build%20and%20deploy%20to%20ghcr.io)](https://github.com/rpatel3001/docker-soapy2tcp-channels/actions/workflows/deploy.yml)
[![Discord](https://img.shields.io/discord/734090820684349521)](https://discord.gg/sTf9uYF)

A Docker image which opens a SoapySDR device, mixes and downsamples the stream into channels, and exposes the channels as rtl_tcp compatible streams. Numba is used to JIT compile vector math operations for speed.

Note that this is quite CPU intensive and may not work on a Raspberry Pi. My machine is a Dell Wyse 5020 thin client with a quad core AMD GX-415GA SOC, which is slightly more performant than the Broadcom BCM2711 in the Pi 4B. With the example setup below, plus ADS-B and UAT decoders, all 4 cores average around 90-95% with a load average ~7. Each downsampled stream is ~50% utilization, the passthrough stream is ~15%, and the SDRplay driver API service is also ~50%. Acarsdec, dumpvdl2 and RTLSDR-airband may also consume significant CPU, depending on how many channels you have setup.

The intended use is to take advantage of the wide bandwidth of the SDRplay RSP1 (and clones) to simultaneously receive ACARS, VDL-M2, and voice transmissions from aircraft using [acarsdec](https://github.com/TLeconte/acarsdec) ([Docker](https://github.com/sdr-enthusiasts/docker-acarsdec)), [dumpvdl2](https://github.com/szpajder/dumpvdl2) ([Docker](https://github.com/sdr-enthusiasts/docker-dumpvdl2)), and [RTLSDR-airband](https://github.com/szpajder/RTLSDR-Airband) ([Docker](https://github.com/sdr-enthusiasts/docker-rtlsdrairband)).

Note: This has only been tested with an AliExpress clone of an RSP1.

---

## Theory of Operation

Once the device is opened, samples are constantly read into receive buffers. The RX thread notifies all the TX threads that samples are available by pushing the buffer index and number of samples onto a queue.

Each TX thread will then:

1. Copy out the samples into a local buffer
2. Mix the signal with the LO frequency determined by the channel center frequency and device tuner frequency
3. Filter the signal to remove out of band frequencies
4. Decimate the signal by the given factor
5. Convert samples from complex float to uint8
6. Send the samples out on the TCP socket

Steps 2-4 are skipped when no decimation is to be applied.

The output sample rate is `RATE`/`DECI`. The output bandwidth of each downsampled channel is only 80% of that due to filtering. The filter used will be adjustable in a future release, but for now uses parameters that I found to provide adequate filtering while not overflowing the buffers on my machine. The input sample rate of dumpvdl2 and acarsdec can be adjusted using the `OVERSAMPLE` and `RATE_MULT` options, respectively. Note that the output rate for acarsdec must be a multiple of 12500 Hz and the rate for dumpvdl2 must be a multiple of 105000 Hz. The example below uses an input rate of 8.4 MHz and a decimation factor of 4 for an output rate of 2.1 MHz on both channels.

## Environment Variables

These control the SoapySDR device that is opened as the original input stream. Not all settings are supported by all devices. Values that are not set are set to default values by the SoapySDR driver in use.

| Variable | Description | Default |
|----------|-------------|---------|
| `SOAPYSDR` | SoapySDR device string that identifies your device. | Unset |
| `RATE` | Sampling rate to set. | SoapySDR driver default |
| `PPM` | PPM frequency correction to set. | Unset |
| `FREQ` | Frequency to tune to. | SoapySDR driver default |
| `BANDWIDTH` | Filter bandwidth to set. | Unset |
| `GAIN` | Set to a numerical value to set the overall gain. Set to a comma separated list of key/value pair such as `IFGR=49,RFGR=0` to set individual gain components. Set to `agc=true` to enable AGC, or `agc=-XX.X` to enable AGC with a specific setpoint. | Unset |
| `CHANS` | Semicolon separated list of comma separated settings per output channel: `centerfrequency,decimationfactor` | Unset |
| `BASEPORT` | Port to start counting from for output channels. | 1234 |
| `NUMBUFS` | Number of receive buffers to use. Each one allocates 2 \* 8 \* MTU bytes, where MTU is set by the SoapySDR driver in use. If you are having problems with buffer overflows, setting this higher will increase the time between overflows at the expense of increased RAM usage and more data lost per overflow. | 100 |

## Docker Compose

This minimal snippet includes this container as a source and acarsdec, dumpvdl2, and RTLSDR-airband as sinks. It also include [rtlmuxer](https://github.com/rpatel3001/docker-rtlmuxer) to proxy the TCP streams to multiple clients. This allows you to view the output stream while the sinks are running in SDR#, SDR++, etc. Your sinks may need additional configuration options, as described in their respective documentation.

Your RTLSDR-airband device configuration should include:

```
    type = "soapysdr";
    device_string = "driver=rtltcp,rtltcp=airbandmuxer:7374";
    centerfreq = 133.500;
    sample_rate = 8.400;
    mode = "multichannel";
```

```
services:
  soapy2chans:
    container_name: soapy2chans
    hostname: soapy2chans
    image: ghcr.io/rpatel3001/docker-soapy2tcp-channels
    restart: always
    devices:
      - /dev/bus/usb
    environment:
      - TZ=America/New_York
      - SOAPY=driver=sdrplay
      - RATE=8400000
      - FREQ=133500000
      - GAIN=agc=-30
      - BASEPORT=1234
      - CHANS=131000000,4;136600000,4;133500000,1

  acarsmuxer:
    container_name: acarsmuxer
    hostname: acarsmuxer
    image: ghcr.io/rpatel3001/docker-rtlmuxer
    restart: always
    depends_on:
      - soapy2chans
    ports:
      - 7374:7374
    environment:
      - TZ=America/New_York
      - SRC_ADDR=soapy2chans
      - SRC_PORT=1234

  vdlm2muxer:
    container_name: vdlm2muxer
    hostname: vdlm2muxer
    image: ghcr.io/rpatel3001/docker-rtlmuxer
    restart: always
    depends_on:
      - soapy2chans
    ports:
      - 7375:7374
    environment:
      - TZ=America/New_York
      - SRC_ADDR=soapy2chans
      - SRC_PORT=1235

  airbandmuxer:
    container_name: airbandmuxer
    hostname: airbandmuxer
    image: ghcr.io/rpatel3001/docker-rtlmuxer
    restart: always
    depends_on:
      - soapy2chans
    ports:
      - 7376:7374
    environment:
      - TZ=America/New_York
      - SRC_ADDR=soapy2chans
      - SRC_PORT=1236

  airband:
    container_name: airband
    hostname: airband
    image: fredclausen/rtlsdrairband
    restart: always
    depends_on:
      - airbandmuxer
    volumes:
      - ./config:/run/rtlsdr-airband
    environment:
      - TZ=America/New_York
      - RTLSDRAIRBAND_CUSTOMCONFIG=true
    ports:
      - 8000:8000
    tmpfs:
      - /tmp

  dumpvdl2:
    container_name: dumpvdl2
    hostname: dumpvdl2
    image: ghcr.io/sdr-enthusiasts/docker-dumpvdl2
    restart: always
    depends_on:
      - vdlm2muxer
    environment:
      - TZ=America/New_York
      - SOAPYSDR=driver=rtltcp,rtltcp=vdlm2muxer:7374
      - CENTER_FREQ=136600000
      - FREQUENCIES=136.650;136.800;136.975
    tmpfs:
      - /run:exec,size=64M
      - /var/log

  acarsdec:
    container_name: acarsdec
    hostname: acarsdec
    build: https://github.com/rpatel3001/docker-acarsdec.git
    restart: always
    depends_on:
      - acarsmuxer
    environment:
      - TZ=America/New_York
      - SOAPYSDR=rtltcp,rtltcp=acarsmuxer:7374
      - FREQUENCIES=130.025;130.450;130.825;131.425;131.475;131.550;131.725;131.825
      - RATEMULT=168
      - CENTER_FREQ=131000000
    tmpfs:
      - /run:exec,size=64M
      - /var/log,size=64M
```
