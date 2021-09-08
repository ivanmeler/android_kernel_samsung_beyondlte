#!/bin/bash


export PLATFORM_VERSION=11
export ANDROID_MAJOR_VERSION=r
export ARCH=arm64

make ARCH=arm64 exynos9820-beyond2lte_defconfig
make ARCH=arm64 -j16