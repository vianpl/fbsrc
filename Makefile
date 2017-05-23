fbsrc-objs := fbsrc-v4l2.o
obj-m  := fbsrc.o

ccflags-y += -g -DDEBUG

# normal makefile
KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	rm -rf   Module.symvers modules.order *.o *.ko *.mod.c
