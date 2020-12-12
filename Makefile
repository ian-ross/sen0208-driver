obj-m = sen0208.o

KPATH=/big/kernel/raspberry-pi/linux
PWD=$(shell pwd)
CROSS=aarch64-linux-gnu-
CC=$(CROSS)gcc
CPP := gcc -E
DTC := dtc
DTC_FLAGS += -Wno-unit_address_vs_reg

DTBOS := $(patsubst %-overlay.dts, %.dtbo, $(wildcard *.dts))

all: sen0208 sen0208.dtbo

sen0208: sen0208.c
	make -C $(KPATH) M=$(PWD) ARCH=arm64 CROSS_COMPILE=$(CROSS) SUBDIRS=$(PWD) modules

%.dtbo : %-overlay.dts
	$(call cmd_dtc)

clean:
	rm -rf *.o *.ko .*cmd .tmp* core *.i *.a *.mod *.mod.c modules.* Module.* *.dtbo *.dts.tmp


dtc-tmp = $(<).tmp
dtc_cpp_flags = -nostdinc -I$(KPATH)/include -undef -D__DTS__

define cmd_dtc
	$(CPP) $(dtc_cpp_flags) -x assembler-with-cpp -o $(dtc-tmp) $< ;
  $(DTC) -I dts -O dtb -o $@ -b 0 -@ $(DTC_FLAGS) $(dtc-tmp) ;
endef
