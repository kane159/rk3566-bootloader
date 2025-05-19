#!/bin/bash
set -e
export CROSS_COMPILE=aarch64-linux-gnu-
make rk3566_miyoo_defconfig
make -j$(nproc)
cd ..
git clone --depth=1 https://github.com/rockchip-linux/rkbin.git
cd rkbin
./tools/mkimage -n rk3566 -T rksd -d bin/rk35/rk3566_ddr_1056MHz_v1.08.bin:../u-boot/spl/u-boot-spl.bin ../u-boot/idblock.bin
cd ../u-boot
cat idblock.bin u-boot.bin > bootloader.bin
