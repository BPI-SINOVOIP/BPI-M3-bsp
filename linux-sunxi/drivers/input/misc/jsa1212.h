/* drivers/input/misc/jsa1212.h
 *
 * Copyright (c) 2012 SOLTEAM.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

struct jsa1212_platform_data {
	int (*configure_vdd)(void); /*return 0: success, else error*/
	int (*configure_gpio)(void); /*return 0: success, else error*/
	unsigned int irq; /*irq number*/
};


enum {	
		DEBUG_INIT = 1U << 0,	
		DEBUG_REPORT_ALS_DATA = 1U << 1,
		DEBUG_REPORT_PS_DATA = 1U << 2,
		DEBUG_SUSPEND = 1U << 3,
		DEBUG_CONTROL_INFO = 1U << 4,
		DEBUG_INT = 1U << 5,
};

#define dprintk(level_mask, fmt, arg...) if (unlikely(debug_mask & level_mask))\
			printk("*jsa1212:*" fmt , ## arg)


#define PL_NAME "jsa1212"
#define JSA1212_I2C_ADDR(PIN_VALUE) (0x44 | PIN_VALUE) /*0x44 or 0x45*/