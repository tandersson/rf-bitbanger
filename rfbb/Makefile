ifneq ($(KERNELRELEASE),)
#invoked from the kernel build system

obj-m += rfbb.o

else
#invoked directly from the command line
ifneq ($(KERNEL_DIR),)
	KERNELDIR := $(KERNEL_DIR)
endif

ifneq ($(EBT_ARCH),)
	ARGS += ARCH=$(EBT_ARCH)
endif

ifneq ($(EBT_HOST_CROSS),)
	ARGS += CROSS_COMPILE=$(EBT_HOST_CROSS)
endif


PWD  := $(shell pwd) 

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) $(ARGS) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) $(ARGS) clean

endif
