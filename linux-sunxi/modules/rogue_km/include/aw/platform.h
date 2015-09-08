/*************************************************************************/ /*!
@File           platform.h
@Title          Platform Header
@Copyright      Copyright (c) Allwinnertech Co., Ltd. All Rights Reserved
@Description    Define struct type about RGX6230 for Allwinnertech sun9iw1p1 platform
@License        GPLv2

The contents of this file is used under the terms of the GNU General Public 
License Version 2 ("GPL") in which case the provisions of GPL are applicable
instead of those above.
*/ /**************************************************************************/

#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include <linux/hardirq.h>
#include <linux/clk.h>
#include <linux/clk-private.h>
#include <linux/io.h>
#include <linux/clk/sunxi_name.h>
#include <linux/clk/sunxi.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/stat.h>
#include <mach/irqs.h>
#include <mach/hardware.h>
#include <mach/platform.h>
#include <mach/sys_config.h>
#include <mach/sunxi-smc.h>
#include "power.h"

#if defined(SUPPORT_ION)
#include "ion_sys.h"
#endif /* defined(SUPPORT_ION) */

#define GPU_CTRL "gpuctrl"

struct aw_clk_data
{
	const char   *clk_name;    /* Clock name */
	const char   *clk_id;      /* Clock ID, which is in the system configuration head file */
	struct clk   *clk_handle;  /* Clock handle, we can set the frequency value via it */ 
	
	/* If the value is 1, the reset API while handle it to reset corresponding module */   
	int          need_reset;
	
	/* The meaning of the values are as follows:
	*  0: don't need to set frequency.
	*  1: use the public frequency
	*  >1: use the private frequency, the value is for frequency.
	*/
	unsigned int expected_freq;
};

struct aw_vf_table
{
	unsigned int vol;
	unsigned int max_freq;
};

struct aw_private_data
{
	char *regulator_id;
	int clk_enable_status;
	int max_level;
	int dvfs_status;
};

struct aw_thermal_ctrl_data
{
	unsigned int temp;
	int          dvfs_level;
};

#if defined(CONFIG_ARCH_SUN9IW1P1)
#include "sun9i/sun9iw1p1.h"
#else
#error "please select a platform\n"
#endif

#endif