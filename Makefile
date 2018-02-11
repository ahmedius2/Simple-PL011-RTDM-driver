# invoke like this: make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- KSRC=~/raspberry/linux/
# export LD_LIBRARY_PATH=/usr/xenomai/lib:$LD_LIBRARY_PATH

### List of modules to be build
MODULES = rtdm_pl011_uart

###### KERNEL MODULE BUILD (no change required normally) ######
### Default to sources of currently running kernel
KSRC ?= /lib/modules/$(shell uname -r)/build

OBJS     := ${patsubst %, %.o, $(MODULES)}
CLEANMOD := ${patsubst %, .%*, $(MODULES)}
PWD      := $(shell if [ "$$PWD" != "" ]; then echo $$PWD; else pwd; fi)

obj-m        := $(OBJS)
rtdm_pl011_uart-y := rtdm_pl011.o bcm2835.o ringbuf.o
ccflags-y    := -I$(KSRC)/include/xenomai -I$(KSRC)/include/xenomai/posix $(ADD_CFLAGS)

all::
	$(MAKE) -C $(KSRC) M=$(PWD) modules

clean::
	$(RM) $(CLEANMOD) *.o *.ko *.mod.c Module*.symvers Module.markers modules.order
	$(RM) -R .tmp*
