/*
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 *
 * Description: MMC driver for general mmc operations
 * Author: Aaron <leafy.myeh@allwinnertech.com>
 * Date: 2012-2-3 14:18:18
 */

#ifndef _MMC_DEF_H_
#define _MMC_DEF_H_

#include <common.h>
#include <asm/io.h>
#include <asm/arch/cpu.h>
#include <asm/arch/ccmu.h>

#define MAX_MMC_NUM			4

#define MMC_TRANS_BY_DMA
//#define MMC_DEBUG
#define MMC_REG_FIFO_OS		(0x200)

#define MMC_REG_BASE		SUNXI_MMC0_BASE
#define CCMU_HCLKGATE0_BASE CCM_AHB0_GATE0_CTRL
#define CCMU_HCLKRST0_BASE 	CCM_AHB0_RST_REG0
#define MMC_REG_COMM_BASE   SUNXI_MMC_COMMON_BASE

#ifdef MMC_DEBUG
#define mmcinfo(fmt...)	printf("[mmc]: "fmt)
#define mmcdbg(fmt...)	printf("[mmc]: "fmt)
#define mmcmsg(fmt...)	printf(fmt)
#else
#define mmcinfo(fmt...)	printf("[mmc]: "fmt)
#define mmcdbg(fmt...)
#define mmcmsg(fmt...)
#endif

#define DRIVER_VER  "2014-08-11 9:49"

#endif /* _MMC_H_ */
