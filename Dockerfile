FROM ghcr.io/sdr-enthusiasts/docker-baseimage@sha256:bafdf9e89a7371651bed9b78f5c26440bd2839943f5d88ead6d1bf8e19be7103

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY sdrplay/ /src/sdrplay/

# hadolint ignore=DL3008,SC2086,DL4006,SC2039
RUN set -x && \
    TEMP_PACKAGES=() && \
    KEPT_PACKAGES=() && \
    # packages needed to install
    TEMP_PACKAGES+=(git) && \
    # packages needed for my sanity
    KEPT_PACKAGES+=(nano) && \
    # packages needed for DSP
    KEPT_PACKAGES+=(python3-scipy) && \
    KEPT_PACKAGES+=(python3-numba) && \
    KEPT_PACKAGES+=(python3-prctl) && \
    # packages needed to build
    TEMP_PACKAGES+=(build-essential) && \
    TEMP_PACKAGES+=(cmake) && \
    TEMP_PACKAGES+=(pkg-config) && \
    TEMP_PACKAGES+=(libusb-1.0-0-dev) && \
    KEPT_PACKAGES+=(libusb-1.0-0) && \
    # install packages
    apt-get update && \
    apt-get install -y --no-install-recommends \
        "${KEPT_PACKAGES[@]}" \
        "${TEMP_PACKAGES[@]}" && \
    git clone https://github.com/ericek111/libmirisdr-5.git /src/libmirisdr-5 && \
    pushd /src/libmirisdr-5 && \
    mkdir build && \
    pushd build && \
    cmake .. && \
    make && \
    make install && \
    popd && popd && \
    git clone https://github.com/ericek111/SoapyMiri.git /src/SoapyMiri && \
    pushd /src/SoapyMiri && \
    mkdir build && \
    pushd build && \
    cmake .. && \
    make -j4 && \
    make install && \
    popd && popd && \
    # install SDRPlay driver
    pushd /src/sdrplay && \
    chmod +x install.sh && \
    ./install.sh && \
#    popd
#RUN set -x && \
    # install SoapySDRPlay
    git clone https://github.com/pothosware/SoapySDRPlay2.git /src/sdrplay/SoapySDRPlay2 && \
    pushd /src/sdrplay/SoapySDRPlay2 && \
    sed -i 's#// OVERLOAD DETECTED#SoapySDR_log(SOAPY_SDR_WARNING, "ADC OVERLOAD DETECTED");#' Streaming.cpp && \
#    sed -i 's#// OVERLOAD CORRECTED#SoapySDR_log(SOAPY_SDR_WARNING, "ADC OVERLOAD CORRECTED");#' Streaming.cpp && \
    mkdir build && \
    pushd build && \
    cmake .. && \
    make && \
    make install && \
    popd && popd && \
    # Clean up
    apt-get remove -y "${TEMP_PACKAGES[@]}" && \
    apt-get autoremove -y && \
    rm -rf /src/* /tmp/* /var/lib/apt/lists/*

COPY rootfs/ /

#RUN set -x && \
#    pip install line_profiler && \
#    touch /etc/services.d/soapy/down && \
#    sed -i "s#def tx_thread#@profile\ndef tx_thread#" /scripts/soapy2tcp.py && \
#    sed -i "s#python -u#kernprof -l#" /etc/services.d/soapy/run
