/*************************************************************************/ /*!
@File           sunxi_init.c
@Title          Allwinnertech Platform Initialization
@Copyright      Copyright (c) Allwinnertech Co., Ltd. All Rights Reserved
@Description    Define functions related to Allwinnertech Platform Initialization
@License        GPLv2

The contents of this file is used under the terms of the GNU General Public 
License Version 2 ("GPL") in which case the provisions of GPL are applicable
instead of those above.
*/ /**************************************************************************/
#include <aw/platform.h>
#include "sunxi_init.h"

#ifdef CONFIG_CPU_BUDGET_THERMAL
#include <linux/cpu_budget_cooling.h>
static int Is_powernow = 0;
#endif /* CONFIG_CPU_BUDGET_THERMAL */

extern struct platform_device *gpsPVRLDMDev;
static struct regulator *rgx_regulator = NULL;
static struct mutex dvfs_lock;

long int GetConfigFreq(IMG_VOID)
{
    return clk_get_rate(clk_data[0].clk_handle);
}

static IMG_VOID ResetAssert(struct aw_clk_data clk_data)
{
	if(clk_data.need_reset)
	{
		if(sunxi_periph_reset_assert(clk_data.clk_handle))
		{
			PVR_DPF((PVR_DBG_ERROR, "Failed to pull down gpu %s clock reset!", clk_data.clk_name));
		}
	}
}

static IMG_VOID ResetDeAssert(struct aw_clk_data clk_data)
{
	if(clk_data.need_reset)
	{
		if(sunxi_periph_reset_deassert(clk_data.clk_handle))
		{
			PVR_DPF((PVR_DBG_ERROR, "Failed to release gpu %s clock reset!", clk_data.clk_name));
		}
	}
}

static IMG_VOID AssertGpuResetSignal(IMG_VOID)
{
	int i;
	for(i = 0; i < sizeof(clk_data)/sizeof(clk_data[0]); i++)
	{
		ResetAssert(clk_data[i]);
	}
}

static IMG_VOID DeAssertGpuResetSignal(IMG_VOID)
{
	int i;
	for(i = sizeof(clk_data)/sizeof(clk_data[0]) - 1; i >= 0; i--)
	{
		ResetDeAssert(clk_data[i]);
	}
}

static IMG_VOID RgxEnableClock(IMG_VOID)
{
	int i;
	if(!private_data.clk_enable_status)
	{
		for(i = 0; i < sizeof(clk_data)/sizeof(clk_data[0]); i++)
		{
			if(clk_prepare_enable(clk_data[i].clk_handle))
			{
				printk(KERN_ERR "Failed to enable gpu %s clock!\n", clk_data[i].clk_name);
			}			
		}
		private_data.clk_enable_status = 1;
	}
}

static IMG_VOID RgxDisableClock(IMG_VOID)
{				
	int i;
	if(private_data.clk_enable_status)
	{
		for(i = sizeof(clk_data)/sizeof(clk_data[0]) - 1; i >= 0; i--)
		{
			clk_disable_unprepare(clk_data[i].clk_handle);			
		}
		private_data.clk_enable_status = 0;
	}
}

static IMG_VOID RgxEnablePower(IMG_VOID)
{
	if(!regulator_is_enabled(rgx_regulator))
	{
		regulator_enable(rgx_regulator); 		
	}
}

static IMG_VOID RgxDisablePower(IMG_VOID)
{
	if(regulator_is_enabled(rgx_regulator))
	{
		regulator_disable(rgx_regulator); 		
	}
}

static IMG_VOID SetGpuVol(int vol /* mV */)
{
	if(vol <= vf_table[private_data.max_level].vol)
	{
		if(vol != regulator_get_voltage(rgx_regulator))
		{
			if(regulator_set_voltage(rgx_regulator, vol*1000, vol*1000) != 0)
			{
				PVR_DPF((PVR_DBG_ERROR, "Failed to set gpu power voltage: Current voltage is %d mV, the voltage to be is %d mV", regulator_get_voltage(rgx_regulator)/1000, vol));
				return;
			}
			/* delay for gpu voltage stability */
			udelay(20);
		}
	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR, "Failed to set gpu power voltage: The voltage to be is %d mV, it is beyond the permitted voltage boundary %d mV", vol, vf_table[private_data.max_level].vol));
	}
}

static IMG_VOID SetClkVal(int freq /* MHz */)
{	
	int i;
	
	if(freq <= vf_table[private_data.max_level].max_freq)
	{
		if(freq != clk_get_rate(clk_data[0].clk_handle)/(1000*1000))
		{
			for(i = 0; i < sizeof(clk_data)/sizeof(clk_data[0]); i++)
			{
				if(clk_data[i].expected_freq <= 0)
				{
					continue;
				}
				else if(clk_data[i].expected_freq > 1)
				{
					freq = clk_data[i].expected_freq;
				}
				
				if(clk_set_rate(clk_data[i].clk_handle, freq*1000*1000))
				{
					PVR_DPF((PVR_DBG_ERROR, "Failed to set the frequency of gpu %s clock: Current frequency is %ld MHz, the frequency to be is %d MHz", clk_data[i].clk_name, clk_get_rate(clk_data[i].clk_handle)/(1000*1000), freq));
				}
			}
		}
	}
}

static IMG_VOID RgxSetClkVal(int freq /* MHz */)
{
	int i;
	PVRSRV_ERROR err;
	
	if(freq > vf_table[private_data.max_level].max_freq)
	{
		PVR_DPF((PVR_DBG_ERROR, "Failed to set the frequency of gpu: The frequency to be is %d MHz, but the current frequency is %ld MHz, it is beyond the permitted frequency boundary %d MHz", freq, clk_get_rate(clk_data[0].clk_handle)/(1000*1000), vf_table[private_data.max_level].max_freq));
		return;
	}
	
	for(i = 0; i <= private_data.max_level; i++)
	{
		if(freq > vf_table[i].max_freq)
		{
			continue;
		}
		mutex_lock(&dvfs_lock);
		err = PVRSRVDevicePreClockSpeedChange(0, IMG_TRUE, NULL);
		if(err == PVRSRV_OK)
		{
			if(vf_table[i].vol == regulator_get_voltage(rgx_regulator)/1000)
			{
				SetClkVal(freq);
			}
			else
			{
				if(freq > clk_get_rate(clk_data[0].clk_handle))
				{
					SetGpuVol(vf_table[i].vol);
					SetClkVal(freq);
				}
				else
				{
					SetClkVal(freq);
					SetGpuVol(vf_table[i].vol);
				}
			}
			PVRSRVDevicePostClockSpeedChange(0, IMG_TRUE, NULL);
		}
		mutex_unlock(&dvfs_lock);
		break;
	}
}

static ssize_t AndroidFreqValueShow(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%ld MHz\n", clk_get_rate(clk_data[0].clk_handle)/(1000*1000));
}

static ssize_t AndroidFreqValueStore(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long freq;	
	
	if(private_data.dvfs_status)
	{
		err = strict_strtoul(buf, 10, &freq);
		if (err)
		{
			PVR_DPF((PVR_DBG_ERROR, "Invalid parameter!"));
			goto err_out;
		}

		RgxSetClkVal(freq);
	}

err_out:	
	return count;
}

static ssize_t ManualFreqValueShow(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%ld MHz\n", clk_get_rate(clk_data[0].clk_handle)/(1000*1000));
}

static ssize_t ManualFreqValueStore(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long freq;
    err = strict_strtoul(buf, 10, &freq);
	
    if (err)
    {
		PVR_DPF((PVR_DBG_ERROR, "Invalid parameter!"));
		goto err_out;
	}
	
	RgxSetClkVal(freq);

err_out:
	return count;
}

static ssize_t DvfsStatusShow(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", private_data.dvfs_status);
}

static ssize_t DvfsStatusStore(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long status;
	err = strict_strtoul(buf, 10, &status);
	if (err)
    {
		PVR_DPF((PVR_DBG_ERROR, "Invalid parameter!"));
		goto err_out;
	}
	
	private_data.dvfs_status = (status == 0 ? 0 : 1);

err_out:	
	return count;
}

static DEVICE_ATTR(android, S_IRUGO|S_IWUSR|S_IWGRP, AndroidFreqValueShow, AndroidFreqValueStore);

static DEVICE_ATTR(manual, S_IRUGO|S_IWUSR|S_IWGRP, ManualFreqValueShow, ManualFreqValueStore);

static DEVICE_ATTR(dvfs_status, S_IRUGO|S_IWUSR|S_IWGRP, DvfsStatusShow, DvfsStatusStore);

static struct attribute *rgx_attributes[] =
{
	&dev_attr_android.attr,
    &dev_attr_manual.attr,
    &dev_attr_dvfs_status.attr,
    NULL
};

 struct attribute_group rgx_attribute_group = {
  .name = "dvfs",
  .attrs = rgx_attributes
};

PVRSRV_ERROR AwPrePowerState(PVRSRV_DEV_POWER_STATE eNewPowerState, PVRSRV_DEV_POWER_STATE eCurrentPowerState, IMG_BOOL bForced)
{
	if(eNewPowerState == PVRSRV_DEV_POWER_STATE_ON)
	{
		RgxEnableClock();
	}
	return PVRSRV_OK;
}

PVRSRV_ERROR AwPostPowerState(PVRSRV_DEV_POWER_STATE eNewPowerState, PVRSRV_DEV_POWER_STATE eCurrentPowerState, IMG_BOOL bForced)
{
	if(eNewPowerState == PVRSRV_DEV_POWER_STATE_OFF)
	{
		RgxDisableClock();
	}	
	return PVRSRV_OK;
}

PVRSRV_ERROR AwSysPrePowerState(PVRSRV_SYS_POWER_STATE eNewPowerState)
{
	if(eNewPowerState == PVRSRV_SYS_POWER_STATE_ON)
	{
		RgxEnablePower();
	
		mdelay(2);
	
		/* set external isolation invalid */
		sunxi_smc_writel(0, SUNXI_R_PRCM_VBASE + GPU_PWROFF_GATING);
	
		DeAssertGpuResetSignal();
		
		RgxEnableClock();
		
		/* set delay for internal power stability */
		sunxi_smc_writel(0x100, SUNXI_GPU_CTRL_VBASE + 0x18);
	}
	
	return PVRSRV_OK;
}

PVRSRV_ERROR AwSysPostPowerState(PVRSRV_SYS_POWER_STATE eNewPowerState)
{
	if(eNewPowerState == PVRSRV_SYS_POWER_STATE_OFF)
	{
		RgxDisableClock();
		
		AssertGpuResetSignal();
	
		/* set external isolation valid */
		sunxi_smc_writel(1, SUNXI_R_PRCM_VBASE + GPU_PWROFF_GATING);
	
		RgxDisablePower();
	}
	
	return PVRSRV_OK;
}

#ifdef CONFIG_CPU_BUDGET_THERMAL
static int rgx_throttle_notifier_call(struct notifier_block *nfb, unsigned long mode, void *cmd)
{
    int retval = NOTIFY_DONE;
	if(mode == BUDGET_GPU_THROTTLE)
    {
		if(Is_powernow)
		{
			RgxSetClkVal(vf_table[3].max_freq);
			Is_powernow = 0;
		}
    }
    else
	{
        if(cmd && (*(int *)cmd) == 1 && !Is_powernow)
		{
			RgxSetClkVal(vf_table[5].max_freq);
            Is_powernow = 1;
        }
		else if(cmd && (*(int *)cmd) == 0 && Is_powernow)
		{
			RgxSetClkVal(vf_table[3].max_freq);
            Is_powernow = 0;
        }
    }
	
	return retval;
}

static struct notifier_block rgx_throttle_notifier = {
.notifier_call = rgx_throttle_notifier_call,
};
#endif /* CONFIG_CPU_BUDGET_THERMAL */

IMG_VOID RgxSunxiInit(IMG_VOID)
{	
	int i;
	
	mutex_init(&dvfs_lock);
	
	private_data.max_level = sizeof(vf_table)/sizeof(vf_table[0]) - 1;
	
	rgx_regulator = regulator_get(NULL, private_data.regulator_id);
	if (IS_ERR(rgx_regulator)) 
	{
		PVR_DPF((PVR_DBG_ERROR, "Failed to get rgx regulator!"));
        rgx_regulator = NULL;
		return;
	}
	
	for(i = 0; i < sizeof(clk_data)/sizeof(clk_data[0]); i++)
	{
		clk_data[i].clk_handle = clk_get(NULL, clk_data[i].clk_id);
		if(NULL == clk_data[i].clk_handle)
		{
			PVR_DPF((PVR_DBG_ERROR, "Failed to get gpu %s clock id!", clk_data[i].clk_name));
			return;
		}
	}
	
	SetGpuVol(vf_table[3].vol);
	
	SetClkVal(vf_table[3].max_freq);
	
	(void) AwSysPrePowerState(PVRSRV_SYS_POWER_STATE_ON);
	
	sysfs_create_group(&gpsPVRLDMDev->dev.kobj, &rgx_attribute_group);
	
#ifdef CONFIG_CPU_BUDGET_THERMAL
	register_budget_cooling_notifier(&rgx_throttle_notifier);
#endif /* CONFIG_CPU_BUDGET_THERMAL */
}

#if defined(SUPPORT_ION)
struct ion_device *g_psIonDev;
extern struct ion_device *idev;

PVRSRV_ERROR IonInit(void *phPrivateData)
{
	g_psIonDev    = idev;
	return PVRSRV_OK;
}

struct ion_device *IonDevAcquire(IMG_VOID)
{
	return g_psIonDev;
}

IMG_VOID IonDevRelease(struct ion_device *psIonDev)
{
	/* Nothing to do, sanity check the pointer we're passed back */
	PVR_ASSERT(psIonDev == g_psIonDev);
}

IMG_UINT32 IonPhysHeapID(IMG_VOID)
{
	return 0;
}

IMG_VOID IonDeinit(IMG_VOID)
{
	g_psIonDev = NULL;
}
#endif /* defined(SUPPORT_ION) */
