#!/bin/bash

export BRANCH=$1
color_red='\033[0;31m'
color_reset='\033[0m'

case $BRANCH in
stable)
	wget `curl -s https://raw.githubusercontent.com/topjohnwu/magisk-files/master/stable.json | jq -r '.magisk.link'` -O Magisk.zip
	magisk_ver=`curl -s https://raw.githubusercontent.com/topjohnwu/magisk-files/master/stable.json | jq -r '.magisk.version'`
        should_proceed=1
	;;
canary)
	wget `curl -s https://raw.githubusercontent.com/topjohnwu/magisk-files/master/canary.json | jq -r '.magisk.link'` -O Magisk.zip
	magisk_ver=`curl -s https://raw.githubusercontent.com/topjohnwu/magisk-files/master/canary.json | jq -r '.magisk.version'`
        should_proceed=1
	;;
*)
        echo -e "$color_red"
	echo -e "Run script with branch you want to updaye to"
	echo -e "e.g."
	echo -e "update_magisk.sh canary or update_magisk.sh stable"
        echo -e "$color_reset"
	should_proceed=0
	;;
esac

if [ $should_proceed -eq "1" ]; then

# Delete old Magisk version
rm -rf usr/init/magisk32.xz
rm -rf usr/init/magisk64.xz
rm -rf usr/init/init

unzip -p Magisk.zip lib/x86/libmagiskboot.so > magiskboot
chmod a+x magiskboot

unzip -p Magisk.zip lib/armeabi-v7a/libmagisk32.so > libmagisk32.so
unzip -p Magisk.zip lib/arm64-v8a/libmagisk64.so > libmagisk64.so
unzip -p Magisk.zip lib/arm64-v8a/libmagiskinit.so > usr/init/init

./magiskboot compress=xz libmagisk32.so usr/init/magisk32.xz
./magiskboot compress=xz libmagisk64.so usr/init/magisk64.xz

chmod 755 usr/init/magisk32.xz
chmod 755 usr/init/magisk64.xz

# Remove leftover files
rm -rf Magisk.zip
rm -rf libmagisk32.so
rm -rf libmagisk64.so
rm -rf magiskboot

git add -A
git commit --author="ivanmeler <i_ivan@windowslive.com>" -m "Update magisk to $magisk_ver"
fi
