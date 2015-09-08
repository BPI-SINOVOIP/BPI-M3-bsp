/*************************************************************************/ /*!
@File           sunxi_init.h
@Title           Allwinnertech Platform Initialization Header
@Copyright      Copyright (c) Allwinnertech Co., Ltd. All Rights Reserved
@Description    Declare functions related to Allwinnertech Platform Initialization
@License        GPLv2

The contents of this file is used under the terms of the GNU General Public 
License Version 2 ("GPL") in which case the provisions of GPL are applicable
instead of those above.
*/ /**************************************************************************/
#if !defined(__SUNXI_INIT__)
#define __SUNXI_INIT__
long int GetConfigFreq(IMG_VOID);
IMG_VOID RgxSunxiInit(IMG_VOID);

#if defined(SUPPORT_ION)
PVRSRV_ERROR IonInit(void *pvPrivateData);
IMG_VOID IonDeinit(IMG_VOID);
#endif

PVRSRV_ERROR AwPrePowerState(PVRSRV_DEV_POWER_STATE eNewPowerState, PVRSRV_DEV_POWER_STATE eCurrentPowerState, IMG_BOOL bForced);
PVRSRV_ERROR AwPostPowerState(PVRSRV_DEV_POWER_STATE eNewPowerState, PVRSRV_DEV_POWER_STATE eCurrentPowerState, IMG_BOOL bForced);
PVRSRV_ERROR AwSysPrePowerState(PVRSRV_SYS_POWER_STATE eNewPowerState);
PVRSRV_ERROR AwSysPostPowerState(PVRSRV_SYS_POWER_STATE eNewPowerState);
#endif	/* __SUNXI_INIT__ */
