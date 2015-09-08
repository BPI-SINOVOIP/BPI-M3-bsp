########################################################################### ###
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Dual MIT/GPLv2
# 
# The contents of this file are subject to the MIT license as set out below.
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# Alternatively, the contents of this file may be used under the terms of
# the GNU General Public License Version 2 ("GPL") in which case the provisions
# of GPL are applicable instead of those above.
# 
# If you wish to allow use of your version of this file only under the terms of
# GPL, and not to allow others to use your version of this file under the terms
# of the MIT license, indicate your decision by deleting the provisions above
# and replace them with the notice and other provisions required by GPL as set
# out in the file called "GPL-COPYING" included in this distribution. If you do
# not delete the provisions above, a recipient may use your version of this file
# under the terms of either the MIT license or GPL.
# 
# This License is also included in this distribution in the file called
# "MIT-COPYING".
# 
# EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
# PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
# BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
### ###########################################################################

include ../config/preconfig.mk

HAL_VARIANT ?= sunxi
PVR_SYSTEM := rgx_sunxi
NO_HARDWARE := 0
LDM_PLATFORM := 1

RGX_BVNC ?= 1.75.2.30

PVRSRV_NEED_PVR_DPF := 1
BUILD := release
SUPPORT_DISPLAY_CLASS = 0

include ../common/android/paths.mk
include ../common/android/arch.mk
include ../common/android/features.mk

ifeq ($(SUPPORT_ADF),1)
ifneq ($(is_future_version),1)
SYS_INCLUDES += \
 -isystem $(ANDROID_ROOT)/device/img/generic/adf/libadf/include \
 -isystem $(ANDROID_ROOT)/device/img/generic/adf/libadfhwc/include
endif
endif

PVR_LDM_PLATFORM_PRE_REGISTERED := 0

CACHEFLUSH_TYPE ?= CACHEFLUSH_GENERIC

include ../config/core.mk
include ../common/3rdparty.mk
include ../common/android/extra_config.mk
