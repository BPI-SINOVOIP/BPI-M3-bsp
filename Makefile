.PHONY: all clean help
.PHONY: u-boot kernel kernel-config
.PHONY: linux pack

include chosen_board.mk

SUDO=sudo
CROSS_COMPILE=$(COMPILE_TOOL)/arm-linux-gnueabi-
#CROSS_COMPILE=arm-linux-gnueabi-
#NEW_CROSS_COMPILE=$(NEW_COMPILE_TOOL)/arm-linux-gnueabihf-
NEW_CROSS_COMPILE=arm-linux-gnueabihf-
U_CROSS_COMPILE=$(CROSS_COMPILE)
#K_CROSS_COMPILE=$(CROSS_COMPILE)
K_CROSS_COMPILE=$(NEW_CROSS_COMPILE)

#unmark for use new toolchain , this time just for KERNEL TEST ONLY
#DONOT USE FOR UBOOT this time, 
#due to allwinner UBOOT release without some source code issue. 

OUTPUT_DIR=$(CURDIR)/output

U_CONFIG_H=$(U_O_PATH)/include/config.h
K_DOT_CONFIG=$(K_O_PATH)/.config

LICHEE_KDIR=$(CURDIR)/linux-sunxi
ROOTFS=$(CURDIR)/rootfs/linux/default_linux_rootfs.tar.gz

Q=
J=$(shell expr `grep ^processor /proc/cpuinfo  | wc -l` \* 2)

all: bsp

## DK, if u-boot and kernel KBUILD_OUT issue fix, u-boot-clean and kernel-clean
## are no more needed
clean: u-boot-clean kernel-clean
	rm -f chosen_board.mk

## pack
pack: sunxi-pack
	$(Q)scripts/mk_pack.sh

# u-boot
$(U_CONFIG_H): u-boot-sunxi
	$(Q)$(MAKE) -C u-boot-sunxi $(UBOOT_CONFIG)_config CROSS_COMPILE=$(U_CROSS_COMPILE) -j$J

u-boot: $(U_CONFIG_H)
	$(Q)$(MAKE) -C u-boot-sunxi all CROSS_COMPILE=$(U_CROSS_COMPILE) -j$J

u-boot-clean:
	rm -f sunxi-pack/chips/sun8iw6p1/bin/u-boot-sun8iw6p1.bin
	rm -f u-boot-sunxi/tools/sunxi_env_gen
	$(Q)$(MAKE) -C u-boot-sunxi CROSS_COMPILE=$(U_CROSS_COMPILE) -j$J distclean

## linux
$(K_DOT_CONFIG): linux-sunxi
	$(Q)$(MAKE) -C linux-sunxi ARCH=arm $(KERNEL_CONFIG)

kernel: $(K_DOT_CONFIG)
	$(Q)$(MAKE) -C linux-sunxi ARCH=arm CROSS_COMPILE=${K_CROSS_COMPILE} -j$J INSTALL_MOD_PATH=output uImage modules
	$(Q)$(MAKE) -C linux-sunxi/modules/gpu CROSS_COMPILE=$(K_CROSS_COMPILE) ARCH=arm LICHEE_KDIR=${LICHEE_KDIR}
	$(Q)$(MAKE) -C linux-sunxi ARCH=arm CROSS_COMPILE=${K_CROSS_COMPILE} -j$J INSTALL_MOD_PATH=output modules_install
	$(Q)$(MAKE) -C linux-sunxi ARCH=arm CROSS_COMPILE=${K_CROSS_COMPILE} -j$J headers_install
	cd linux-sunxi && ${K_CROSS_COMPILE}objcopy -R .note.gnu.build-id -S -O binary vmlinux bImage

kernel-clean:
	$(Q)$(MAKE) -C linux-sunxi/arch/arm/mach-sunxi/pm/standby ARCH=arm CROSS_COMPILE=${K_CROSS_COMPILE} clean
	$(Q)$(MAKE) -C linux-sunxi/modules/gpu CROSS_COMPILE=$(K_CROSS_COMPILE) ARCH=arm LICHEE_KDIR=${LICHEE_KDIR} clean
	$(Q)$(MAKE) -C linux-sunxi ARCH=arm CROSS_COMPILE=${K_CROSS_COMPILE} -j$J distclean
	rm -rf linux-sunxi/output/
	rm -f linux-sunxi/bImage

kernel-config: $(K_DOT_CONFIG)
	$(Q)$(MAKE) -C linux-sunxi ARCH=arm CROSS_COMPILE=${K_CROSS_COMPILE} -j$J menuconfig
	cp linux-sunxi/.config linux-sunxi/arch/arm/configs/$(KERNEL_CONFIG)

## bsp
bsp: u-boot kernel

## linux
linux: 
	$(Q)scripts/mk_linux.sh $(ROOTFS)

help:
	@echo ""
	@echo "Usage:"
	@echo "  make bsp             - Default 'make'"
	@echo "  make linux         - Build target for linux platform, as ubuntu, need permisstion confirm during the build process"
	@echo "   Arguments:"
	@echo "    ROOTFS=            - Source rootfs (ie. rootfs.tar.gz with absolute path)"
	@echo ""
	@echo "  make pack            - pack the images and rootfs to a PhenixCard download image."
	@echo "  make clean"
	@echo ""
	@echo "Optional targets:"
	@echo "  make kernel           - Builds linux kernel"
	@echo "  make kernel-config    - Menuconfig"
	@echo "  make u-boot          - Builds u-boot"
	@echo ""

