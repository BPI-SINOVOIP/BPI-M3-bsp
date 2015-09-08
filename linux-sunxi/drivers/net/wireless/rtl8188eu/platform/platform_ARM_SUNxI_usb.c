/******************************************************************************
 *
 * Copyright(c) 2013 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/
/*
 * Description:
 *	This file can be applied to following platforms:
 *	CONFIG_PLATFORM_ARM_SUNXI Series platform 
 *	
 */
 
#include <drv_types.h>
#include <mach/sys_config.h>

#ifdef CONFIG_PLATFORM_ARM_SUNxI
extern int sunxi_usb_disable_hcd(__u32 usbc_no);
extern int sunxi_usb_enable_hcd(__u32 usbc_no);
extern void wifi_pm_power(int on);
static script_item_u item;
#endif


int platform_wifi_power_on(void)
{
	int ret = 0;

#if defined(CONFIG_PLATFORM_ARM_SUNxI)
	{
		script_item_value_type_e type;

		type = script_get_item("wifi_para", "wifi_usbc_id", &item);
		if(SCIRPT_ITEM_VALUE_TYPE_INT != type){
			printk("ERR: script_get_item wifi_usbc_id failed\n");
			ret = -ENOMEM;
			goto exit;
		}

		printk("sw_usb_enable_hcd: usbc_num = %d\n", item.val);
		wifi_pm_power(1);
		mdelay(10);
	
		#if !(defined(CONFIG_RTL8723A)) && !(defined(CONFIG_RTL8723B))
		sunxi_usb_enable_hcd(item.val);
		#endif
	}
#endif //CONFIG_PLATFORM_ARM_SUNxI

exit:
	return ret;
}

void platform_wifi_power_off(void)
{
#if defined(CONFIG_PLATFORM_ARM_SUNxI)
	#if !(defined(CONFIG_RTL8723A)) && !(defined(CONFIG_RTL8723B))
	sunxi_usb_disable_hcd(item.val);
	#endif
	wifi_pm_power(0);
#endif //defined(CONFIG_PLATFORM_ARM_SUNxI) 

}

