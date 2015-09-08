/*************************************************************************/ /*!
@File           sun9iw1p1.h
@Title          SUN9IW1P1 Header
@Copyright      Copyright (c) Allwinnertech Co., Ltd. All Rights Reserved
@Description    Define Detailed data about RGX6230 for Allwinnertech sun9iw1p1 platform
@License        GPLv2

The contents of this file is used under the terms of the GNU General Public 
License Version 2 ("GPL") in which case the provisions of GPL are applicable
instead of those above.
*/ /**************************************************************************/

#ifndef _SUN9IW1P1_H_
#define _SUN9IW1P1_H_

static struct aw_vf_table vf_table[] =
{
	{
		.vol      = 700, /* mV */
		.max_freq = 48,  /* MHz */		
	},
	{
		.vol      = 800, /* mV */
		.max_freq = 120, /* MHz */
	},
	{
		.vol      = 800, /* mV */
		.max_freq = 240, /* MHz */
	},
	{
		.vol      = 900, /* mV */
		.max_freq = 384, /* MHz */
	},
	{
		.vol      = 1000, /* mV */
		.max_freq = 480, /* MHz */
	},
	{
		.vol      = 1100, /* mV */
		.max_freq = 528, /* MHz */
	},
};

static struct aw_private_data private_data =
{
	.regulator_id      = "axp22_dcdc2",
	.clk_enable_status = 0,
	.dvfs_status       = 1,
};

static struct aw_clk_data clk_data[] =
{
	{
		.clk_name      = "pll",
		.clk_id        = PLL9_CLK,
		.clk_handle    = NULL,
		.need_reset    = 0,
		.expected_freq = 1,
	},
	{
		.clk_name      = "core",
		.clk_id        = GPUCORE_CLK,
		.clk_handle    = NULL,
		.need_reset    = 1,
		.expected_freq = 1,
	},
	{
		.clk_name      = "mem",
		.clk_id        = GPUMEM_CLK,
		.clk_handle    = NULL,
		.need_reset    = 0,
		.expected_freq = 1,
	},
	{
		.clk_name      = "axi",
		.clk_id        = GPUAXI_CLK,
		.clk_handle    = NULL,
		.need_reset    = 0,
		.expected_freq = 320,
	},
	{
		.clk_name      = "ctrl",
		.clk_id        = GPU_CTRL,
		.clk_handle    = NULL,
		.need_reset    = 1,
		.expected_freq = 0,
	}
};
#endif