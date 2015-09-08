#include <linux/init.h>
#include <mach/hardware.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <mach/sys_config.h>
#include <linux/io.h>

#include "rf_pm.h"

struct gps_config_info {
	int  gps_used;
	int  standby;
	int  reset;
	char gps_clk[10];
};

struct gps_dev {
	struct device *dev;
	struct gps_config_info gps_info;
	int  standby_state;
	int  reset_state;
};

static struct gps_dev *_gps_dev;
static struct clk *gps_32k = NULL;

/*for debug*/
#define GPS_DEBUG

#define GPS_ERR(x...) printk("=========GPS========= :" x)

#ifdef GPS_DEBUG
#define GPS_DBG(x...) printk("=========GPS========= :" x)
#else
#define GPS_DBG(x...)  
#endif


static int gps_power_init(void)
{
	struct gps_dev *dev = _gps_dev;
	struct gps_config_info *gps_info = &dev->gps_info;

	if(gps_info->standby){
		dev->standby_state = 0;
		gpio_set_value(gps_info->standby,dev->standby_state);
	}
	if(gps_info->reset){
		dev->reset_state = 1;
		gpio_set_value(gps_info->reset,dev->reset_state);
	}
	return 0;
}

static ssize_t standby_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct gps_dev *dev = _gps_dev;
	return sprintf(buf, "%d\n", dev->standby_state);
}

static ssize_t standby_store(struct device *pdev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	struct gps_dev *dev = _gps_dev;
	struct gps_config_info *gps_info = &dev->gps_info;
	int value;
	
	if(!gps_info->standby)
		return -EINVAL;

	if (sscanf(buf, "%d", &value) == 1) {
		dev->standby_state = value;
		gpio_set_value(gps_info->standby, value);
		GPS_DBG("%s:set standby to %d\n", __func__, value);
		return size;
	}

	return -EINVAL;
}

static ssize_t reset_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct gps_dev *dev = _gps_dev;
	return sprintf(buf, "%d\n", dev->reset_state);
}

static ssize_t reset_store(struct device *pdev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	struct gps_dev *dev = _gps_dev;
	struct gps_config_info *gps_info = &dev->gps_info;
	int value;

	if(!gps_info->reset)
		return -EINVAL;

	if (sscanf(buf, "%d", &value) == 1) {
		dev->reset_state = value;
		gpio_set_value(gps_info->reset, value);
		GPS_DBG("%s:set reset to %d\n", __func__, value);
		return size;
	}

	return -EINVAL;
}

static struct device_attribute attributes[] = {
	__ATTR(power_enable, S_IRUGO | S_IWUSR, standby_show, standby_store),
	__ATTR(reset, S_IRUGO | S_IWUSR, reset_show, reset_store),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	GPS_ERR("%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static void enable_gps_32k(int enable)
{
	int ret = 0;

	if (enable){
		ret = clk_prepare_enable(gps_32k);
		if (ret){
			GPS_ERR("enable ap 32k failed!\n");
		}
	} else {
		clk_disable_unprepare(gps_32k);
	}
}

static int __init gps_get_config(struct gps_config_info *gps_info)
{
	script_item_value_type_e type;
	script_item_u val;
	struct gpio_config  *gpio_p = NULL;

	memset(gps_info,0,sizeof(struct gps_config_info));
	type = script_get_item("gps_para", "gps_used", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		GPS_ERR("failed to fetch gps configuration!\n");
		return -1;
	}
	if (!val.val) {
		GPS_ERR("gsp not used in configuration\n");
		return -1;
	}
	gps_info->gps_used = val.val;
	
	type = script_get_item("gps_para", "gps_standby_n", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_PIO != type)
		GPS_DBG("mod has no gps_standby_n gpio\n");
	else {
		gpio_p = &val.gpio;
		gps_info->standby = gpio_p->gpio;
		sunxi_gpio_req(gpio_p);
	}

	type = script_get_item("gps_para", "gps_rst_n", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_PIO != type)
		GPS_DBG("mod has no gps_rst_n gpio\n");
	else {
		gpio_p = &val.gpio;
		gps_info->reset = gpio_p->gpio;
		sunxi_gpio_req(gpio_p);
	}
	
	type = script_get_item("gps_para", "gps_clk", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_STR != type) 
		GPS_DBG("failed to fetch gps_clk\n");
	else {
		strcpy(gps_info->gps_clk, val.str);
	}
	
	return 0;
}

static int gps_probe(struct platform_device *pdev)
{
	int err = 0;
	struct gps_dev *dev = _gps_dev;
	struct gps_config_info *gps_info = &dev->gps_info;
	
	if(gps_info->gps_clk && strcmp(gps_info->gps_clk, "")){
		gps_32k = clk_get(NULL, gps_info->gps_clk);
		if (IS_ERR(gps_32k)){
			GPS_ERR("Get ap 32k clk out failed!\n");
			return -1;
		}
		enable_gps_32k(1);
	}

	GPS_DBG("set %s 32k out\n", gps_info->gps_clk);
	
	gps_power_init();
	create_sysfs_interfaces(&pdev->dev);

	return err;
}

#ifdef CONFIG_PM
static int gps_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct gps_dev *dev = _gps_dev;
	struct gps_config_info *gps_info = &dev->gps_info;

	GPS_DBG("gps suspend!\n");
	if(dev->standby_state)
		gpio_set_value(gps_info->standby, 0);
	return 0;
}

static int gps_resume(struct platform_device *pdev)
{
	struct gps_dev *dev = _gps_dev;
	struct gps_config_info *gps_info = &dev->gps_info;

	GPS_DBG("gps resume!\n");
	if(dev->standby_state)
		gpio_set_value(gps_info->standby, 1);
	return 0;
}
#endif

static int gps_remove(struct platform_device *pdev)
{
	struct gps_dev *dev = _gps_dev;
	struct gps_config_info *gps_info = &dev->gps_info;
	
	remove_sysfs_interfaces(&pdev->dev);
	
	if(gps_info->gps_clk && strcmp(gps_info->gps_clk, "")){
		enable_gps_32k(0);
		clk_put(gps_32k);
	}
	return 0;
}

static struct platform_device gps_device = {
	.name   ="gps",
	.id     =-1,
};

static struct platform_driver gps_driver = {
	.probe = gps_probe,
#ifdef CONFIG_PM
	.suspend = gps_suspend,
	.resume = gps_resume,
#endif	
	.remove = gps_remove,
		
	.driver = {
		.name = "gps",
		.owner = THIS_MODULE,
	}
};

static int __init gps_init(void)
{
	int ret = 0;
	struct gps_dev *dev;
	struct gps_config_info *gps_info;
	
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	gps_info = &dev->gps_info;
	
	ret = gps_get_config(gps_info);
	if(ret){
		goto err0;
	}
	_gps_dev = dev;
	platform_device_register(&gps_device); 
	ret = platform_driver_register(&gps_driver);
	if (ret < 0) {
		GPS_DBG("platform_driver_register returned %d\n", ret);
		goto err1;
	}
	return 0;

err1:
	platform_device_unregister(&gps_device); 
err0:
	kfree(dev);
	return ret;
}

static void __exit gps_exit(void)
{
	platform_device_unregister(&gps_device); 
	platform_driver_unregister(&gps_driver);
	kfree(_gps_dev);
}

late_initcall_sync(gps_init);
module_exit(gps_exit);



