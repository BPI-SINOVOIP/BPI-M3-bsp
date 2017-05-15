ifneq ($(LICHEE_KDIR),)
KERNEL_DIR = $(LICHEE_KDIR)
GPU_BUILD_TYPE = release
endif

$(warning ---------------------------kernel dir is $(KERNEL_DIR))

include $(KERNEL_DIR)/.config


##############################################################
##################   Set ARCH and CROSS_COMPILE  #############
##############################################################
ifdef CONFIG_ARM
export ARCH ?= arm
ifeq ($(CROSS_COMPILE),)
export CROSS_COMPILE ?= arm-linux-gnueabi-
endif
endif
	
ifdef CONFIG_ARM64
export ARCH ?= arm64
ifeq ($(CROSS_COMPILE),)
export CROSS_COMPILE ?= aarch64-linux-gnu-
endif
endif

SYSTEM_TYPE = linux
GPU_TYPE = sgx544

#####################################
#For sgx544 Platform
#####################################
ifeq ($(GPU_TYPE),sgx544)
GPU_BUILD_DIR = sgx544/android/kernel_mode/eurasia_km/eurasiacon/build/linux2/sunxi_linux
GPU_KO_NAME   = sgx544/android/kernel_mode/eurasia_km/eurasiacon/binary2_sunxi_$(SYSTEM_TYPE)_$(GPU_BUILD_TYPE)/target/pvrsrvkm.ko
GPU_CLEAN_DIR = sgx544/android/kernel_mode/eurasia_km/eurasiacon/binary2_sunxi_$(SYSTEM_TYPE)_$(GPU_BUILD_TYPE)
ifdef CONFIG_OF
export USING_DT = 1
endif
export KERNELDIR=$(KERNEL_DIR)
endif

##############################################################

all: build

build:
	$(MAKE) -j -C $(GPU_BUILD_DIR) BUILD=$(GPU_BUILD_TYPE) KERNELDIR=$(KERNEL_DIR)

clean:
	$(MAKE) -C $(GPU_BUILD_DIR) clean
	if [ $(GPU_CLEAN_DIR) ]; then \
		rm -rf $(GPU_CLEAN_DIR); \
	fi
