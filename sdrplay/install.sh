#!/bin/bash

ARCH=$(uname -m)

case $ARCH in
  x86_64)
    BINARY=SDRplay_RSP_API-Linux-2.13.1.run
    ;;
  armv*)
    BINARY=SDRplay_RSP_API-RPi-2.13.1.run
    ARCH=armv7l
    ;;
  aarch64)
    BINARY=SDRplay_RSP_API-ARM64-2.13.1.run
    ;;
esac

sh $BINARY --noexec --target sdrplay
patch --verbose -Np0 < ./install-lib.patch

cd sdrplay
sed -i "s#sudo ##" install_lib.sh
./install_lib.sh
cd ..
