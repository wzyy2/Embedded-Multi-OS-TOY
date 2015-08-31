#!/bin/sh

set -e

KDIR=~/kernel/kernel


scons -j20


(
cd ./linux_vmm/
make -C $KDIR M=$PWD VMM_HDR_DIR=$PWD/../ ARCH=arm CROSS_COMPILE=arm-linux-gnueabi-
cp rtvmm.ko $PWD/../
)
