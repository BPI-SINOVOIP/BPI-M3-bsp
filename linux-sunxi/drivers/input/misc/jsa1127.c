/* drivers/input/sensor/jsa1127.c
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
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#include <linux/jsa1127.h>
#endif
#include <linux/init-input.h>
#include <mach/sys_config.h>

/* debug level */
#define DEBUG_MASK 0
#if DEBUG_MASK

#define DBG(fmt, ...)  do { \
							printk(KERN_DEBUG "[JSA1127_D] %s(%d): " fmt "\n",\
							__FUNCTION__, __LINE__, ## __VA_ARGS__); \
						} while(0)
#else
#define DBG(fmt, ...)
#endif

#define INFO(fmt, ...)  do { \
							printk(KERN_INFO "[JSA1127_I] %s(%d): " fmt "\n",\
							__FUNCTION__, __LINE__, ## __VA_ARGS__); \
						} while(0)

#define ERR(fmt, ...)  do { \
							printk(KERN_ERR "[JSA1127_E] %s(%d): " fmt "\n",\
							__FUNCTION__, __LINE__, ## __VA_ARGS__); \
						} while(0)

#define I2C_RETRY_TIME 10
#define PRODUCT_NAME "jsa1127"

#define MAX_DELAY  600 /* mSec */
#define MIN_DELAY  200  /* mSec */
#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)

/* Kernel has no float point, it would convert it by this*/
/* RINT = 100, specification page 12, lux/count = 1.67 */
#define DEFAULT_RESOLUTION_R100K 	1670 //Resolution
#define BASE_VALUE 			1000
#define INVALID_COUNT	0xFFFFFFFF
#define DEVICE_NAME jsa1127

struct sensor_config
{
	int twi_id;
	int int1;
	int int_mode;
}sensor_config;

static struct sensor_config_info ls_sensor_info = {
	.input_type = LS_TYPE,
	.int_number = 0,
	.ldo = NULL,
};

static u32 debug_mask = 0;

enum {
                DEBUG_INIT = 1U << 0,
                DEBUG_REPORT_ALS_DATA = 1U << 1,
                DEBUG_REPORT_PS_DATA = 1U << 2,
                DEBUG_SUSPEND = 1U << 3,
                DEBUG_CONTROL_INFO = 1U << 4,
                DEBUG_INT = 1U << 5,
};

#define dprintk(level_mask, fmt, arg...) if (unlikely(debug_mask & level_mask))\
                        printk("*jsa1127:*" fmt , ## arg)


static const unsigned short normal_i2c[] = {0x39,I2C_CLIENT_END};


/*
static void jsa1127_configure_platform(void)
{

}
static struct jsa1127_platform_data kai_jsa1127_pdata = {
	.configure_platform = jsa1127_configure_platform,
};
*/

/* jsa1127 driver data struct */
struct jsa1127_drv_data
{
	struct delayed_work work;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct jsa1127_platform_data *pdev;
	struct mutex mutex;
	struct early_suspend early_suspend;
	unsigned int delay;
	unsigned long lux;
	unsigned int enabled;
	unsigned long resolution;
	unsigned long compensate_rate;
	unsigned int first_boot;
};

static struct jsa1127_drv_data jsa1127_data = {
	.delay = MAX_DELAY,
	.lux = 0,
	.enabled = 0,
	.resolution = DEFAULT_RESOLUTION_R100K,
//	.compensate_rate = 50,//evt/dvt
	.compensate_rate = 40, //for bbs
	.first_boot = 1,
};

/* CMD definition */
#define CMD_SHUTDOWN_MODE 					0x8C
#define CMD_ACTIVATE_MODE 					0x0C
#define CMD_ACTIVATE_MODE_ONE_TIME 			0x04
#define CMD_START_INTEGRATION_ONE_TIME		0x08
#define CMD_STOP_INTEGRATION_ONE_TIME		0x30


static int jsa1127_cmd_send(unsigned char cmd)
{
	int ret = -EIO;
	unsigned char wbuf[5];
	int retry = I2C_RETRY_TIME;

	wbuf[0] = cmd;

	printk("jsa1127_cmd_send now !\n");

	for (; retry > 0; retry --)
	{
		ret = i2c_master_send(jsa1127_data.client, wbuf, 1);
		if (ret < 0)
		{
			ERR("write cmd[0x%2X] failed!, retry(%d)",
				cmd, (I2C_RETRY_TIME - retry));
		}
		else
		{
			DBG("write cmd[0x%2X] success!", cmd);
			break;
		}
	}

	return ret;
}

/* Using internal integration timing to read lux value */
static unsigned long jsa1127_read_lux(void)
{
	int ret = -EIO;
	int retry = I2C_RETRY_TIME;
	unsigned char rbuf[5];
	unsigned long lux = INVALID_COUNT;

	/* start to read data of lux */
	retry = I2C_RETRY_TIME;
	for(; retry > 0; retry --)
	{
		ret = i2c_master_recv(jsa1127_data.client, rbuf, 2);
		if (ret < 0)
		{
			ERR("read failed!, retry(%d)", (I2C_RETRY_TIME - retry));
		}
		else
		{
//			printk("read success!");
			break;
		}
	}

	if (ret > 0)
	{
		if (rbuf[1] && 0x80)
		{
			lux = (unsigned long)((((int)rbuf[1]&0x7F) << 8) | (int)rbuf[0]);
			DBG("lux value is valid!, count = %ld", lux);
		}
		else
		{
			lux = INVALID_COUNT;
			INFO("lux value is invalid!");
		}
	}

	return lux;
}


/* Initial chips stage, check chip connect with board is fine,
 * using external integration timing */
static int initial_jsa1127_chip(void)
{
	int ret = 0;
        dprintk(DEBUG_INIT,"%s enter\n",__FUNCTION__);
	ret = jsa1127_cmd_send(CMD_ACTIVATE_MODE);
	if (ret < 0)
	{
		ERR("Send CMD activiate one time failed!");
		ret = -EIO;
		goto i2c_err;
	}

#if 0
	ret = jsa1127_cmd_send(CMD_ACTIVATE_MODE_ONE_TIME);
	if (ret < 0)
	{
		ERR("Send CMD activiate one time failed!");
		ret = -EIO;
		goto i2c_err;
	}

	ret = jsa1127_cmd_send(CMD_START_INTEGRATION_ONE_TIME);
	if (ret < 0)
	{
		ERR("Send CMD start command failed!");
		ret =  -EIO;
		goto i2c_err;
	}

	ret = jsa1127_cmd_send(CMD_STOP_INTEGRATION_ONE_TIME);
	if (ret < 0)
	{
		ERR("Send CMD stop command failed!");
		ret =  -EIO;
		goto i2c_err;
	}

	ret = jsa1127_cmd_send(CMD_SHUTDOWN_MODE);
	if (ret < 0)
	{
		ERR("Send CMD shutdown failed!");
		ret =  -EIO;
		goto i2c_err;
	}
#endif 

	if (ret > 0)
		DBG("initial chip success!");

i2c_err:
	return ret;
}


/* work queue for update current light senosr lux*/
static void jsa1127_work_func(struct work_struct *work)
{
	unsigned long delay = delay_to_jiffies(jsa1127_data.delay);
	unsigned long lux;
	lux = jsa1127_read_lux();
	if (lux != INVALID_COUNT)
	{
		lux = (lux * jsa1127_data.resolution * jsa1127_data.compensate_rate) / BASE_VALUE;
		jsa1127_data.lux = lux;
		dprintk(DEBUG_INIT,"udpdate lux: %ld\n", lux);
	}
	else
	{
		dprintk(DEBUG_INIT,"report the prevous lux value\n");
		lux = jsa1127_data.lux;
	}

	if (jsa1127_data.input_dev)
	{
		dprintk(DEBUG_INIT,"report lux value: (%ld)\n", lux);
		input_report_abs(jsa1127_data.input_dev, ABS_MISC, lux);
		input_sync(jsa1127_data.input_dev);
	}

	if (jsa1127_data.enabled)
	{
		dprintk(DEBUG_INIT,"still scheduling light sensor workqueue!\n");
		schedule_delayed_work(&jsa1127_data.work, delay);
	}
}

static void jsa1127_early_suspend(struct early_suspend *h)
{
	int ret = 0;
	cancel_delayed_work_sync(&jsa1127_data.work);

	ret = jsa1127_cmd_send(CMD_SHUTDOWN_MODE);
	if (ret < 0)
		printk("Send CMD shutdown failed!\n");

}

static void jsa1127_late_resume(struct early_suspend *h)
{
	int ret = 0;
	unsigned long delay; 
	delay = delay_to_jiffies(jsa1127_data.delay);
	ret = jsa1127_cmd_send(CMD_ACTIVATE_MODE);
	if (ret < 0)
		printk("Send CMD activiate one time failed!\n");

	schedule_delayed_work(&jsa1127_data.work, delay);

}

static ssize_t sensor_data_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
	dprintk(DEBUG_INIT,"lux: %ld\n", jsa1127_data.lux);
       return sprintf(buf, "%lu\n", jsa1127_data.lux);
}

static ssize_t sensor_delay_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	dprintk(DEBUG_INIT,"delay: %d\n", jsa1127_data.delay);
       return sprintf(buf, "%u\n", jsa1127_data.delay);
}

static ssize_t sensor_delay_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int value = simple_strtoul(buf, NULL, 10);

	if (value < MIN_DELAY)
		value = MIN_DELAY;

	if (value >= MAX_DELAY)
		value = MAX_DELAY;

	mutex_lock(&jsa1127_data.mutex);
	jsa1127_data.delay = value;
	mutex_unlock(&jsa1127_data.mutex);

	dprintk(DEBUG_INIT,"set delay time as %d ms\n", jsa1127_data.delay);

	return count;
}

static ssize_t sensor_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	dprintk(DEBUG_INIT,"enabled: %d\n", jsa1127_data.enabled);
       return snprintf(buf, sizeof(buf), "%u\n", jsa1127_data.enabled);
}

static ssize_t sensor_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{ 
    
	int value;
	unsigned long delay;
	//----------------------------------------------------------------
	if(jsa1127_data.first_boot)
	{
			
		jsa1127_data.compensate_rate = 40;
		jsa1127_data.resolution = DEFAULT_RESOLUTION_R100K ;
			
		jsa1127_data.first_boot = 0;
	}
	//----------------------------------------------------------------
	value = simple_strtoul(buf, NULL, 10);
	delay = delay_to_jiffies(jsa1127_data.delay);
        dprintk(DEBUG_INIT,"%s enable value = %d\n",__FUNCTION__,value);
	mutex_lock(&jsa1127_data.mutex);

	if (value == 1 && jsa1127_data.enabled == 0)
	{
		if (jsa1127_cmd_send(CMD_ACTIVATE_MODE) < 0) //enable light sensor
		{
			printk("enable jsa1127 failed!\n");
			return 0;
		}
		else
		{
			dprintk(DEBUG_INIT,"enable light sensor success\n");
			jsa1127_data.enabled = 1;
			schedule_delayed_work(&jsa1127_data.work, delay);
		}
	}
	else if (jsa1127_data.enabled == 1 && value == 0)
	{
		if (jsa1127_cmd_send(CMD_SHUTDOWN_MODE) < 0) //disable light sensor
		{
			printk("disable jsa1127 failed!\n");
		}

		jsa1127_data.enabled = 0;
		cancel_delayed_work_sync(&jsa1127_data.work);
	}
	else
	{
		dprintk(DEBUG_INIT,"Do nothing at this operation time!\n");
	}

	mutex_unlock(&jsa1127_data.mutex);

	dprintk(DEBUG_INIT,"set enabled as %d\n", jsa1127_data.enabled);

	return count;
}

static ssize_t sensor_resolution_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	dprintk(DEBUG_INIT,"resolution: %ld\n", jsa1127_data.resolution);
       return sprintf(buf, "%lu\n", jsa1127_data.resolution);
}

static ssize_t sensor_resolution_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int value = simple_strtoul(buf, NULL, 10);

	if (value < 0)
		return 0;

	mutex_lock(&jsa1127_data.mutex);
	jsa1127_data.resolution = (unsigned long) value;
	mutex_unlock(&jsa1127_data.mutex);

	dprintk(DEBUG_INIT,"set resolution as %ld\n", jsa1127_data.resolution);

	return count;
}

///sys/class/input/input2/test
static ssize_t sensor_test_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int value = simple_strtoul(buf, NULL, 10);
	int ret = 0;
	unsigned long lux;

  switch(value)
  {
  	case 0:
				ret = jsa1127_cmd_send(CMD_ACTIVATE_MODE_ONE_TIME);
				if (ret < 0)
				{
					printk("Send CMD activiate one time failed!\n");
				}
  		break;
  	case 1:
				ret = jsa1127_cmd_send(CMD_START_INTEGRATION_ONE_TIME);
				if (ret < 0)
				{
					printk("Send CMD start command failed!\n");
				}
  		break;
  	case 2:
				ret = jsa1127_cmd_send(CMD_STOP_INTEGRATION_ONE_TIME);
				if (ret < 0)
				{
					printk("Send CMD stop command failed!\n");
				}
  		break;
  	case 3:
				ret = jsa1127_cmd_send(CMD_SHUTDOWN_MODE);
				if (ret < 0)
				{
					printk("Send CMD shutdown failed!\n");
				}
  		break;
  	case 4:  			
				lux = jsa1127_read_lux();
				if (lux != INVALID_COUNT)
				{
					lux = (lux * jsa1127_data.resolution) / BASE_VALUE;
					printk("jsa1127_read_lux lux: %ld\n", lux);
				}
				else
				{
					printk("Send CMD shutdown failed!\n");
				}
					
  		break;
  	default:
  		break;
  }

	return count;
}

static ssize_t sensor_compensate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	dprintk(DEBUG_INIT,"compensate: %ld\n", jsa1127_data.compensate_rate);
       return sprintf(buf,  "%lu\n", jsa1127_data.compensate_rate);
}

static ssize_t sensor_compensate_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int value = simple_strtoul(buf, NULL, 10);

	mutex_lock(&jsa1127_data.mutex);
	jsa1127_data.compensate_rate = value;
	mutex_unlock(&jsa1127_data.mutex);

	dprintk(DEBUG_INIT,"set delay time as %ld ms\n", jsa1127_data.compensate_rate);

	return count;
}

static struct device_attribute sensor_dev_attr_data = __ATTR(data, S_IRUGO,
	sensor_data_show, NULL);
static struct device_attribute sensor_dev_attr_delay = __ATTR(delay,
	S_IRUGO|S_IWUSR|S_IWGRP,
	sensor_delay_show, sensor_delay_store);
static struct device_attribute sensor_dev_attr_enable = __ATTR(enable,
	S_IRUGO|S_IWUSR|S_IWGRP,
	sensor_enable_show, sensor_enable_store);
static struct device_attribute sensor_dev_attr_resolution = __ATTR(resolution,
	S_IRUGO|S_IWUSR|S_IWGRP,
	sensor_resolution_show, sensor_resolution_store);

static struct device_attribute sensor_dev_attr_test = __ATTR(test,
	S_IRUGO|S_IWUSR|S_IWGRP,
	NULL, sensor_test_store);
static struct device_attribute sensor_dev_attr_compensate = __ATTR(compensate,
	S_IRUGO|S_IWUSR|S_IWGRP,
	sensor_compensate_show, sensor_compensate_store);
	
static struct attribute *sensor_attributes[] = {
	&sensor_dev_attr_data.attr,
	&sensor_dev_attr_delay.attr,
	&sensor_dev_attr_enable.attr,
	&sensor_dev_attr_resolution.attr,
	&sensor_dev_attr_test.attr,
	&sensor_dev_attr_compensate.attr,
	NULL
};

static struct attribute_group sensor_attribute_group = {
	.attrs = sensor_attributes
};

static int __devinit jsa1127_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
        dprintk(DEBUG_INIT,"%s enter\n",__FUNCTION__);
	/* Chcek I2C */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk("Check I2C functionality failed.\n");
		ret = -EIO;
		goto err_check_functionality_failed;
	}

	/* Get driver data and i2c client */
	jsa1127_data.client = client;
	
/*	jsa1127_data.pdev = (struct jsa1127_platform_data *) client->dev.platform_data;
	if (!jsa1127_data.pdev)
	{
		printk("Please check platform data!\n");
		ret = -EIO;
		goto err_platform_data;
	}

//	initial platform configuration
	if (!jsa1127_data.pdev && !jsa1127_data.pdev->configure_platform)
	{
		dprintk(DEBUG_INT,"initalize platform setting!\n");
		jsa1127_data.pdev->configure_platform();
	}
*/

	/* Initial jsa1127 chip and check connect with i2c bus */

	ret = initial_jsa1127_chip();
	if (ret < 0)
	{
		printk("initial chip error!\n");
		goto err_init_chip;
	}

	/* register input device and delay work queue for evnet polling */

	/* allocate input event to pass to user space*/
	/* Allocate Input Device */
	if (!jsa1127_data.input_dev)
		jsa1127_data.input_dev = input_allocate_device();

	if (!jsa1127_data.input_dev)
	{
		printk("Allocate input device failed.\n");
		ret = -ENOMEM;
		return ret;
	}

	jsa1127_data.input_dev->name = client->name;
	jsa1127_data.input_dev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, jsa1127_data.input_dev->evbit);
	input_set_capability(jsa1127_data.input_dev, EV_ABS, ABS_MISC);

	input_set_abs_params(jsa1127_data.input_dev, ABS_MISC, 0, 65535, 0, 0);

	/* Register Input Device */
	ret = input_register_device(jsa1127_data.input_dev);
	if (ret)
	{
		printk("Register input device failed.\n");
		input_free_device(jsa1127_data.input_dev);
		ret = -EIO;
	}

	if (ret < 0 )
	{
		printk("jsa1127 input register error!\n");
		goto err_input_failed;
	}

	/* initial delay workqueue */
	mutex_init(&jsa1127_data.mutex);
	mutex_lock(&jsa1127_data.mutex);
	jsa1127_data.delay = MAX_DELAY;
	jsa1127_data.first_boot = 1;
	INIT_DELAYED_WORK(&jsa1127_data.work, jsa1127_work_func);

	dprintk(DEBUG_INIT,"%s schedule_delayed_work\n",__FUNCTION__);
	jsa1127_data.enabled = 1;
        schedule_delayed_work(&jsa1127_data.work, msecs_to_jiffies(MAX_DELAY));

	jsa1127_data.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	jsa1127_data.early_suspend.suspend = jsa1127_early_suspend;
	jsa1127_data.early_suspend.resume = jsa1127_late_resume;
	register_early_suspend(&jsa1127_data.early_suspend);

	/* setup the attr for control by others */
	ret = sysfs_create_group(&jsa1127_data.input_dev->dev.kobj,
			&sensor_attribute_group);
	if (ret < 0)
	{
		printk("register attributes failed!\n");
		goto err_sysfs_register;
	}

	mutex_unlock(&jsa1127_data.mutex);
	dprintk(DEBUG_INIT,"Probe Done.\n");
	return 0;

err_sysfs_register:
err_input_failed:
err_init_chip:
err_platform_data:
err_check_functionality_failed:

	return ret;
}

static int jsa1127_remove(struct i2c_client *client)
{
	cancel_delayed_work_sync(&jsa1127_data.work);
	unregister_early_suspend(&jsa1127_data.early_suspend);
	sysfs_remove_group(&jsa1127_data.input_dev->dev.kobj,
		&sensor_attribute_group);
	input_unregister_device(jsa1127_data.input_dev);
	input_free_device(jsa1127_data.input_dev);

	dprintk(DEBUG_INIT,"Remove JSA1127 Module Done.\n");
	return 0;
}

static int jsa1127_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
        dprintk(DEBUG_INIT,"%s enter\n",__FUNCTION__);
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	if (ls_sensor_info.twi_id == adapter->nr){
		client->addr = 0x77;
		strlcpy(info->type,PRODUCT_NAME,I2C_NAME_SIZE);
		dprintk(DEBUG_INIT,"the device detect is ok!\n");
		return 0;
		}
	else
		return -ENODEV;
}

static int jsa1127_fetch_sysconfig_para(void)
{
	int ret = -1;
	int device_used = -1;
	int twi_id = 0;
	script_item_u	val;
	script_item_value_type_e  type;	
		

	
	type = script_get_item("ls_para", "ls_used", &val);
 
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		pr_err("%s: type err  device_used = %d. \n", __func__, val.val);
		goto script_get_err;
	}
	device_used = val.val;
	if (1 == device_used) {
		type = script_get_item("ls_para", "ls_twi_id", &val);	
		if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
			pr_err("%s: type err twi_id = %d. \n", __func__, val.val);
			goto script_get_err;
		}
		twi_id = val.val;
		ret = 0;
		
	} else {
		pr_err("%s: ls_unused. \n",  __func__);
		ret = -1;
	}
	sensor_config.twi_id = twi_id;

	return ret;

script_get_err:
	pr_notice("=========script_get_err============\n");
	return ret;

}

static const struct i2c_device_id jsa1127_id[] = {
	{PRODUCT_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, jsa1127_id);

static struct i2c_driver jsa1127_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.owner = THIS_MODULE,
		.name  = PRODUCT_NAME,
	},
	.id_table  = jsa1127_id,
	.probe     = jsa1127_probe,
	.remove    = jsa1127_remove,
	.address_list = normal_i2c,
	.detect    = jsa1127_detect,
};

/* driver initial part */
static int __init jsa1127_init(void)
{
    	dprintk(DEBUG_INIT,"%s enter\n",__FUNCTION__);
	if(jsa1127_fetch_sysconfig_para())
	{
		return -1;
	}

        dprintk(DEBUG_INIT,"%s input_fetch_sysconfig_para\n",__FUNCTION__);
	if (input_fetch_sysconfig_para(&(ls_sensor_info.input_type))) {
		printk("%s: ls_fetch_sysconfig_para err.\n", __func__);
		return 0;
	} 

	dprintk(DEBUG_INIT,"register jsa1127 i2c driver!\n");
	return i2c_add_driver(&jsa1127_driver);
}
module_init(jsa1127_init);
static void __exit jsa1127_exit(void)
{
	dprintk(DEBUG_INIT,"deregister jsa1127 i2c driver!\n");
	i2c_del_driver(&jsa1127_driver);
}
module_exit(jsa1127_exit);

module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

MODULE_AUTHOR("<Solteam Corp.>");
MODULE_DESCRIPTION("Solteam JSA1127 Ambient Light Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.0.0");
