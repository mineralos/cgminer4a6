#!/bin/sh

ROOTFS_DIR=$1
ROOTFS_DIR_G19=$2

# 安装json库(删除cgminer)
make install
rm -rf ${ROOTFS_DIR}/bin/cgminer

# 安装多版本cgminer
cp innominer_* ${ROOTFS_DIR}/bin/
mv innominer_* ${ROOTFS_DIR_G19}/bin/
