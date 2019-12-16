#
# Makefile for the Linux kernel device drivers.
#
# Sep 2018, Binse Park <h22yap@gmail.com>
# Rewritten to use lists instead of if-statements.
#

#$(info kperfmon_CURDIR="$(shell ls $(srctree/../../*)")
ifeq (exist, $(shell [ -e $(srctree)/../../system/core/liblog/include/log/perflog.h ] && echo exist))
	obj-y				+= kperfmon.o
else
	obj-y				+= ologk.o
endif
