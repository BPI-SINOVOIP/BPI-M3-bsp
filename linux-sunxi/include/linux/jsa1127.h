/* drivers/input/misc/jsa1127.c
 *
 * Copyright (c) 2011 SOLTEAM.
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

/* jsa1127 chip platfrom data */
struct jsa1127_platform_data
{
	void (*configure_platform)(void);
};
