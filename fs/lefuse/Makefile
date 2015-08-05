#
# Makefile for the FUSE filesystem.
#

obj-$(CONFIG_FUSE_FS) += fuse.o
obj-$(CONFIG_CUSE) += cuse.o

ifeq ($(CONFIG_MTK_MET), y)
# ccflags-y += -DMET_FUSEIO_TRACE
endif

ccflags-y += -DUSER_BUILD_KERNEL
fuse-objs := dev.o dir.o file.o leinode.o control.o
