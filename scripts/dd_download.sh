#!/bin/sh

# make partition table by fdisk command
# reserve part for fex binaries download 0~204799
# partition1 /dev/sdc1 vfat 204800~327679
# partition2 /dev/sdc2 ext4 327680~end

die() {
        echo "$*" >&2
        exit 1
}

[ -s "../chosen_board.mk" ] || die "please run ./configure first."
[ $# -eq 1 ] || die "Usage: $0 /dev/sdc"

set -e

. ../chosen_board.mk

O=$1
P=../download/$BOARD

sudo dd if=$P/boot0_sdcard.fex 	of=$O bs=1k seek=8
sudo dd if=$P/u-boot.fex 	of=$O bs=1k seek=19096
sudo dd if=$P/sunxi_mbr.fex 	of=$O bs=1k seek=20480
sudo dd if=$P/boot-resource.fex	of=$O bs=1k seek=36864
sudo dd if=$P/env.fex 		of=$O bs=1k seek=69632
sudo dd if=$P/boot.fex 		of=$O bs=1k seek=86016
