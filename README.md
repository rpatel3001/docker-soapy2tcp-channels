# docker-soapy2tcp-channels
[![GitHub Workflow Status](https://img.shields.io/github/workflow/status/rpatel3001/docker-soapy2tcp-channels/Build%20and%20deploy%20to%20ghcr.io)](https://github.com/rpatel3001/docker-soapy2tcp-channels/actions/workflows/deploy.yml)
[![Discord](https://img.shields.io/discord/734090820684349521)](https://discord.gg/sTf9uYF)

A Docker image which opens a SoapySDR device, mixes and downsamples the stream into channels, and exposes the channels as rtl_tcp compatible streams.

Note: This has only been tested with an AliExpress clone of an RSP1.

---

## Up and running

```
version: '3'

services:
  soapy2tcp:
    container_name: soapy2tcp
    hostname: soapy2tcp
    image: ghcr.io/rpatel3001/docker-soapy2tcp-channels
    restart: always
    ports:
      - 1234:1234
    volumes:
      - /etc/timezone:/etc/timezone:ro
      - /etc/localtime:/etc/localtime:ro
    devices:
      - /dev/bus/usb
    environment:
      - SOAPY=driver=sdrplay
      - RATE=8400000
      - FREQ=136000000
      - GAIN=IFGR=49,RFGR=0
      - CHANS=136900000,16,1234
```

## SoapySDR Options

These control the SoapySDR device that is opened as the original input stream. Not all settings are supported by all devices. Values that are not set are set to default values by the SoapySDR driver in use.

| Variable | Description | Default |
|----------|-------------|---------|
| `RATE` | Sampling rate to set. | Unset |
| `PPM` | PPM frequency correction to set. | Unset |
| `FREQ` | Frequency to tune to. | Unset |
| `BANDWIDTH` | Filter bandwidth to set. | Unset |
| `GAIN` | Set to a numerical value to set the overall gain. Set to a comma separated list of key/value pair such as `IFGR=49,RFGR=0` to set individual gain components. Set to `agc=true` to enable AGC, or `agc=-XX.X` to enable AGC with a specific setpoint. | Unset |
| `CHANS` | Semicolon separated list of comma separated settings per output channel: `centerfrequency,decimationfactor,outputport` | Unset |
