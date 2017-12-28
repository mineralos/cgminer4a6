#!/bin/sh

#make clean
#make distclean

ROOTFS_DIR=/home/public/huwt/cgminer-for-test/rootfs
MAKE_JOBS=4
ROOTFS_DIR_G19=$3

## A6
#CHIP_TYPE=A6
#sed -i "s/#define CHIP_A[0-9]/#define CHIP_A6/g" miner.h
#./autogen.sh
#LDFLAGS=-L${ROOTFS_DIR}/lib \
#CFLAGS="-I${ROOTFS_DIR}/include -Wall " \
#./configure --prefix=${ROOTFS_DIR} \
#--enable-bitmine_${CHIP_TYPE} --without-curses --host=arm-xilinx-linux-gnueabi --build=x86_64-pc-linux-gnu # --target=arm
make -j${MAKE_JOBS}
arm-xilinx-linux-gnueabi-strip cgminer
cp cgminer innominer_T2

#cp ./cgminer /home/public/update/cgminer_huwt.$1
#chmod 777 /home/public/update/cgminer_huwt.$1

