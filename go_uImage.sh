#!/bin/sh

rm linux
ln -s linux-beagle-xm/ linux
#ln -s linux-beagle-xm_from_volodya/ linux
#ln -s linux-3.2.51/ linux
#ln -s linux-3.0.1/ linux

set -e

export PATH=$PATH:$(cd ..; pwd -P)/u-boot/u-boot/tools

export ARCH=arm
#export CROSS_COMPILE=$(cd ../..; pwd -P)/openembedded/bin/arm-angstrom-linux-gnueabi-
#export CROSS_COMPILE=$(cd ../..; pwd -P)/openembedded_4.6.3/usr/local/angstrom/arm/bin/arm-angstrom-linux-gnueabi-
#export CROSS_COMPILE=$(cd ../..; pwd -P)/arm-2013.05-24/bin/arm-none-linux-gnueabi-
export CROSS_COMPILE=$(cd ../..; pwd -P)/arm-2010q1-202/bin/arm-none-linux-gnueabi-
is_zImage=0

if test -z "$@"; then
    is_zImage=1
fi

#cp -f -T .config_2011.10.19 linux/.config;

for k in $@; do
    case $k in
	(zImage) is_zImage=1;
#	(oldconfig) cp -f -T config20091228 linux/.config;;
#	(oldconfig) cp -f -T config20100408 linux/.config;;
#	(oldconfig) cp -f -T config20100527 linux/.config;;
#	(oldconfig) cp -f -T .config linux/.config;;
#	(?);;
    esac;
done

cd linux

#make distclean
make clean;
make -j5 ARCH=${ARCH} CROSS_COMPILE=${CROSS_COMPILE} uImage


#if [ "${is_zImage}" -eq "1" ]; then
#    cp -f -T ./arch/arm/boot/zImage ../mkkernel/zImage
#    cd ../mkkernel
#    ./mkkernel.sh
#    cp -f -T uImage ../../tftpboot/uImage
#fi

#make menuconfig
