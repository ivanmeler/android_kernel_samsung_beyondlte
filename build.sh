#!/bin/bash
# kernel build script by geiti94 v0.1 (made for s10e/s10/s10/n10/n10+ sources)

export MODEL=$1
export VARIANT=eur
export ARCH=arm64
export BUILD_CROSS_COMPILE=$(pwd)/toolchains/aarch64-linux-android-4.9/bin/aarch64-linux-android-
export BUILD_JOB_NUMBER=12

RDIR=$(pwd)
OUTDIR=$RDIR/arch/arm64/boot
DTSDIR=$RDIR/arch/arm64/boot/dts/exynos
DTBDIR=$OUTDIR/dtb
INCDIR=$RDIR/include

case $MODEL in
beyond2lte)
	case $VARIANT in
	can|duos|eur|xx)
		KERNEL_DEFCONFIG=exynos9820-beyond2lte_defconfig
		;;
	*)
		echo "Unknown variant: $VARIANT"
		exit 1
		;;
	esac
;;
beyond1lte)
	case $VARIANT in
	can|duos|eur|xx)
		KERNEL_DEFCONFIG=exynos9820-beyond1lte_defconfig
		;;
	*)
		echo "Unknown variant: $VARIANT"
		exit 1
		;;
	esac
;;
beyond0lte)
	case $VARIANT in
	can|duos|eur|xx)
		KERNEL_DEFCONFIG=exynos9820-beyond0lte_defconfig
		;;
	*)
		echo "Unknown variant: $VARIANT"
		exit 1
		;;
	esac
;;
*)
	echo "Unknown device: $MODEL"
	exit 1
	;;
esac


FUNC_CLEAN_DTB()
{
	if ! [ -d $RDIR/arch/$ARCH/boot/dts ] ; then
		echo "no directory : "$RDIR/arch/$ARCH/boot/dts""
	else
		echo "rm files in : "$RDIR/arch/$ARCH/boot/dts/*.dtb""
		rm $RDIR/arch/$ARCH/boot/dts/*.dtb
		rm $RDIR/arch/$ARCH/boot/dtb/*.dtb
		rm $RDIR/arch/$ARCH/boot/boot.img-dtb
		rm $RDIR/arch/$ARCH/boot/boot.img-zImage
	fi
}

FUNC_BUILD_KERNEL()
{
	echo ""
        echo "=============================================="
        echo "START : FUNC_BUILD_KERNEL"
        echo "=============================================="
        echo ""
        echo "build common config="$KERNEL_DEFCONFIG ""
        echo "build model config="$MODEL ""


	export ANDROID_MAJOR_VERSION=q


	make -j$BUILD_JOB_NUMBER ARCH=$ARCH \
			CROSS_COMPILE=$BUILD_CROSS_COMPILE \
			$KERNEL_DEFCONFIG || exit -1

	make -j$BUILD_JOB_NUMBER ARCH=$ARCH \
			CROSS_COMPILE=$BUILD_CROSS_COMPILE || exit -1

	
	echo ""
	echo "================================="
	echo "END   : FUNC_BUILD_KERNEL"
	echo "================================="
	echo ""
}

FUNC_BUILD_RAMDISK()
{
	mv $RDIR/arch/$ARCH/boot/Image $RDIR/arch/$ARCH/boot/boot.img-zImage

	case $MODEL in
	beyond2lte)
		case $VARIANT in
		can|duos|eur|xx)
			rm -f $RDIR/ramdisk/G975/split_img/boot.img-zImage
			mv -f $RDIR/arch/$ARCH/boot/boot.img-zImage $RDIR/ramdisk/G975/split_img/boot.img-zImage
			cd $RDIR/ramdisk/G975
			./repackimg.sh --nosudo
			echo SEANDROIDENFORCE >> image-new.img
			;;
		*)
			echo "Unknown variant: $VARIANT"
			exit 1
			;;
		esac
	;;
	beyond1lte)
		case $VARIANT in
		can|duos|eur|xx)
			rm -f $RDIR/ramdisk/G973/split_img/boot.img-zImage
			mv -f $RDIR/arch/$ARCH/boot/boot.img-zImage $RDIR/ramdisk/G973/split_img/boot.img-zImage
			cd $RDIR/ramdisk/G973
			./repackimg.sh --nosudo
			echo SEANDROIDENFORCE >> image-new.img
			;;
		*)
			echo "Unknown variant: $VARIANT"
			exit 1
			;;
		esac
	;;
	beyond0lte)
		case $VARIANT in
		can|duos|eur|xx)
			rm -f $RDIR/ramdisk/G970/split_img/boot.img-zImage
			mv -f $RDIR/arch/$ARCH/boot/boot.img-zImage $RDIR/ramdisk/G970/split_img/boot.img-zImage
			cd $RDIR/ramdisk/G970
			./repackimg.sh --nosudo
			echo SEANDROIDENFORCE >> image-new.img
			;;
		*)
			echo "Unknown variant: $VARIANT"
			exit 1
			;;
		esac
	;;
	*)
		echo "Unknown device: $MODEL"
		exit 1
		;;
	esac
}

FUNC_BUILD_ZIP()
{
	cd $RDIR/build
	rm $MODEL-$VARIANT.img
	case $MODEL in
	beyond2lte)
		case $VARIANT in
		can|duos|eur|xx)
			mv -f $RDIR/ramdisk/G975/image-new.img $RDIR/build/$MODEL-$VARIANT.img
			;;
		*)
			echo "Unknown variant: $VARIANT"
			exit 1
			;;
		esac
	;;
	beyond1lte)
		case $VARIANT in
		can|duos|eur|xx)
			mv -f $RDIR/ramdisk/G973/image-new.img $RDIR/build/$MODEL-$VARIANT.img
			;;
		*)
			echo "Unknown variant: $VARIANT"
			exit 1
			;;
		esac
	;;
	beyond0lte)
		case $VARIANT in
		can|duos|eur|xx)
			mv -f $RDIR/ramdisk/G970/image-new.img $RDIR/build/$MODEL-$VARIANT.img
			;;
		*)
			echo "Unknown variant: $VARIANT"
			exit 1
			;;
		esac
	;;
	*)
		echo "Unknown device: $MODEL"
		exit 1
		;;
	esac
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
