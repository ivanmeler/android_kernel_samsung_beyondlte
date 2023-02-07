#!/bin/bash

export MODEL=$1
export BUILD_CROSS_COMPILE=$(pwd)/toolchains/aarch64-linux-android-4.9/bin/aarch64-linux-androidkernel-
export BUILD_JOB_NUMBER=`grep -c ^processor /proc/cpuinfo`
RDIR=$(pwd)

case $MODEL in
beyond2lte)
    KERNEL_DEFCONFIG=exynos9820-beyond2lte_defconfig
    SOC=9820
    echo SRPRI17C014KU > ramdisk/split_img/boot.img-board
;;
beyond1lte)
    KERNEL_DEFCONFIG=exynos9820-beyond1lte_defconfig
    SOC=9820
    echo SRPRI28B014KU > ramdisk/split_img/boot.img-board
;;
beyond0lte)
    KERNEL_DEFCONFIG=exynos9820-beyond0lte_defconfig
    SOC=9820
    echo SRPRI28A014KU > ramdisk/split_img/boot.img-board
;;
beyondx)
    KERNEL_DEFCONFIG=exynos9820-beyondx_defconfig
    SOC=9820
    echo SRPSC04B011KU > ramdisk/split_img/boot.img-board
;;
d1)
    KERNEL_DEFCONFIG=exynos9820-d1_defconfig
    SOC=9825
    echo SRPSD26B007KU > ramdisk/split_img/boot.img-board
;;
d1x)
    KERNEL_DEFCONFIG=exynos9820-d1xks_defconfig
    SOC=9825
    echo SRPSD23A002KU > ramdisk/9825/split_img/boot.img-board
;;
d2s)
    KERNEL_DEFCONFIG=exynos9820-d2s_defconfig
    SOC=9825
    echo SRPSC14B007KU > ramdisk/split_img/boot.img-board
;;
d2x)
    KERNEL_DEFCONFIG=exynos9820-d2x_defconfig
    SOC=9825
    echo SRPSC14C007KU > ramdisk/split_img/boot.img-board
;;
*)
    echo "Unknown device: $MODEL setting to beyond2lte"
    KERNEL_DEFCONFIG=exynos9820-beyond2lte_defconfig
    SOC=9820
    echo SRPRI17C014KU > ramdisk/split_img/boot.img-board
esac

FUNC_BUILD_KERNEL()
{
    echo " Starting a kernel build using "$KERNEL_DEFCONFIG ""
    # No this is not a typo, samsung left it this way on 12
    export PLATFORM_VERSION=11
    export ANDROID_MAJOR_VERSION=r

    make -j$BUILD_JOB_NUMBER ARCH=arm64 \
        CROSS_COMPILE=$BUILD_CROSS_COMPILE \
        $KERNEL_DEFCONFIG || exit -1

    make -j$BUILD_JOB_NUMBER ARCH=arm64 \
        CROSS_COMPILE=$BUILD_CROSS_COMPILE || exit -1

    $RDIR/toolchains/mkdtimg cfg_create build/dtb_$SOC.img \
        $RDIR/toolchains/configs/exynos$SOC.cfg \
        -d $RDIR/arch/arm64/boot/dts/exynos

    echo " Finished kernel build"
}

FUNC_BUILD_DTBO()
{
    $RDIR/toolchains/mkdtimg cfg_create build/dtbo_$MODEL.img \
        $RDIR/toolchains/configs/$MODEL.cfg \
        -d $RDIR/arch/arm64/boot/dts/samsung
}

FUNC_BUILD_RAMDISK()
{
    rm -f $RDIR/ramdisk/split_img/boot.img-kernel
    cp $RDIR/arch/arm64/boot/Image $RDIR/ramdisk/split_img/boot.img-kernel
    # This is kinda ugly hack, we could as well touch .placeholder to all of those
    mkdir -p $RDIR/ramdisk/ramdisk/debug_ramdisk
    mkdir -p $RDIR/ramdisk/ramdisk/dev
    mkdir -p $RDIR/ramdisk/ramdisk/mnt
    mkdir -p $RDIR/ramdisk/ramdisk/proc
    mkdir -p $RDIR/ramdisk/ramdisk/sys

    rm -rf $RDIR/ramdisk/ramdisk/fstab.exynos9820
    rm -rf $RDIR/ramdisk/ramdisk/fstab.exynos9825

    cp $RDIR/ramdisk/fstab.exynos$SOC $RDIR/ramdisk/ramdisk/

    cd $RDIR/ramdisk/
    ./repackimg.sh --nosudo
}

FUNC_BUILD_ZIP()
{
    cd $RDIR/build
    rm -rf $MODEL-boot-ramdisk.img
    mv $RDIR/ramdisk/image-new.img $RDIR/build/$MODEL-boot-ramdisk.img
}

# MAIN FUNCTION
rm -rf ./build.log
(
	START_TIME=`date +%s`

	FUNC_BUILD_KERNEL
	FUNC_BUILD_RAMDISK
	FUNC_BUILD_ZIP

	END_TIME=`date +%s`

	let "ELAPSED_TIME=$END_TIME-$START_TIME"
	echo "Total compile time was $ELAPSED_TIME seconds"

) 2>&1	| tee -a ./build.log
