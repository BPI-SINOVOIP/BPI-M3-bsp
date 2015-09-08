/* drivers/input/misc/jsa1212.c
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
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/init-input.h>
#include <linux/wakelock.h>
#include <linux/timer.h>
#include <linux/suspend.h>
#include <mach/sys_config.h>

#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>

#include <linux/errno.h>
#include <linux/device.h>
#include <linux/fs.h>

#include <linux/irq.h>


#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "jsa1212.h"

#ifdef CONFIG_SCENELOCK
#include <linux/power/scenelock.h>
#endif

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif



#define ALS_EVENT ABS_X
#define PRX_EVENT ABS_Y
#define JSA1212_I2C_RETRY 5

#define JSA1212_SLAVE_ADDR 0x44
#define SENSOR_NAME PL_NAME

#define PS_DISTANCE  80

enum {
	LIGHT_ENABLED = BIT(0),
	PROXIMITY_ENABLED = BIT(1),
};


static struct sensor_config_info ls_sensor_info = {	
	.input_type = LS_TYPE,	
	.int_number = 0,	
	.ldo = NULL,
	};

static u32 debug_mask = 255;

/* Addresses to scan */
static const unsigned short normal_i2c[2] = {JSA1212_SLAVE_ADDR,I2C_CLIENT_END};



/* driver data */
struct jsa_data {	
	struct input_dev *proximity_input_dev;
	struct input_dev *light_input_dev;
	struct delayed_work ps_delay_work;
	struct work_struct irq_workqueue;
	struct i2c_client *i2c_client;
	int irq;
	struct work_struct work_light;
	struct hrtimer timer;
	ktime_t light_poll_delay;
	bool on;
	u8 power_state;
	unsigned long ps_poll_delay;
	struct mutex power_lock;
	struct wake_lock prx_wake_lock;
	struct workqueue_struct *wq;
	atomic_t jsa1212_init;
	atomic_t jsa1212_suspend;
};


static void jsa1212_resume_events(struct work_struct *work);
static void jsa1212_init_events(struct work_struct *work);
struct workqueue_struct *jsa1212_resume_wq;
struct workqueue_struct *jsa1212_init_wq;
static DECLARE_WORK(jsa1212_resume_work, jsa1212_resume_events);
static DECLARE_WORK(jsa1212_init_work, jsa1212_init_events);
struct jsa_data *jsa1212_data;



/*register define*/
struct jsa1212_register_data {
	unsigned int reg;
	unsigned char value;
};

/*register index*/
enum {
		CONFIGURE = 0x01, /*RW*/
		INTERRUPT, /*RW*/
		PROX_LT, /*RW*/
		PROX_HT, /*RW*/
		ALSIR_TH1, /*RW*/
		ALSIR_TH2, /*RW*/
		ALSIR_TH3, /*RW*/
		PROX_DATA, /*RO*/
		ALSIR_DT1, /*RO*/
		ALSIR_DT2, /*RO*/
		ALS_RNG, /*RW*/
		JSA1212_COUNT = ALS_RNG,
};

/*jsa1212 registers initialize table*/
static struct jsa1212_register_data jsa1212_init_register[] = {
		{CONFIGURE, 0x58},
		{INTERRUPT, 0x02},
		{PROX_LT, 0x32},
		{PROX_HT, 0x50},
		{ALSIR_TH1, 0x64},
		{ALSIR_TH2, 0x08},
		{ALSIR_TH3, 0x3E},
		{ALS_RNG, 0x00}
};

/*jsa1212 i2c func*/
static int jsa1212_i2c_write(unsigned char *data_in, unsigned char reg)
{
	int ret = 0;
	int retry_time = 0;
	unsigned char buf[2];


	buf[0] = reg;
	buf[1] = data_in[0];
	while (retry_time ++ < JSA1212_I2C_RETRY)
	{
		ret = i2c_master_send(jsa1212_data->i2c_client, buf, sizeof(buf));
		if (ret > 0)
			break;
		else
			printk(KERN_ERR "jsa1212 i2c write (0x%02X - 0x%02X) error(%d)\n", reg, data_in[0], ret);
	}
	
	return ret;
}

static int jsa1212_i2c_read(unsigned char *data_out, unsigned char reg, int size)
{
	int ret = 0;
	int retry_time = 0;
	unsigned char buf[1];

	buf[0] = reg;
	while (retry_time ++ < JSA1212_I2C_RETRY)
	{
		ret = i2c_master_send(jsa1212_data->i2c_client, buf, sizeof(buf));
		if (ret > 0)
			break;
		else
			printk(KERN_ERR "jsa1212 i2c read (0x%02X) error(%d)\n", reg, ret);
	}

	if (ret < 0)
		goto i2c_err;

	retry_time = 0;
	while (retry_time ++ < JSA1212_I2C_RETRY)
	{
		ret = i2c_master_recv(jsa1212_data->i2c_client, data_out, size);
		if (ret > 0)
			break;
		else
			printk(KERN_ERR "jsa1212 i2c read (0x%02X) error(%d)\n", reg, ret);
	}

i2c_err:
	return ret;
}


static int jsa1212_i2c_write_bytes(struct i2c_client * client, uint8_t * data, uint16_t len)
{
 	struct i2c_msg msg;
 	int ret=-1;

 	msg.flags = !I2C_M_RD;
 	msg.addr = client->addr;
 	msg.len = len;
 	msg.buf = data;
 
 	ret=i2c_transfer(client->adapter, &msg,1);
 	return ret;
}


static bool i2c_test(struct i2c_client * client)
{
	 int ret,retry;
	 uint8_t test_data[1] = { 0 };//only write a data address
	   
	 for(retry=0; retry < 5; retry++) {
			ret = jsa1212_i2c_write_bytes(client, test_data, 1);//Test i2c
	 		if (ret == 1)
			break;
			 msleep(10);
	 }
	   
	 return ret==1 ? true : false;
} 

static int jsa1212_ps_enable(struct jsa_data *jsa)
{
     unsigned char buf[2];
	 int ret = -1;

	 ret = jsa1212_i2c_read(buf, CONFIGURE, sizeof(buf));
	 if (ret < 0){
		printk(KERN_ERR "jsa1212 read out config data failed\n");
		return ret;
	 }
	 buf[1] = buf[0]|(1<<7);
	 ret = jsa1212_i2c_write(&buf[1],CONFIGURE);
	 if (ret < 0){
		printk(KERN_ERR "jsa1212 write config data failed\n");
		return ret;
	 }
	 queue_delayed_work(jsa->wq, &jsa->ps_delay_work, 0);
	 return 0;
}



static int jsa1212_ps_disable(struct jsa_data *jsa)
{
     unsigned char buf[2];
	 int ret = -1;
	 
	 ret = jsa1212_i2c_read(buf, CONFIGURE, sizeof(buf));
	 if (ret < 0){
		printk(KERN_ERR "jsa1212 read out config data failed\n");
		return ret;
	 }
	 
	 buf[1] = buf[0] & (~(1<<7));
	 ret = jsa1212_i2c_write(&buf[1],CONFIGURE);
	 if (ret < 0){
		printk(KERN_ERR "jsa1212 write config data failed\n");
		return ret;
	 }
	 cancel_delayed_work_sync(&jsa->ps_delay_work);
	 return 0;
}

static int jsa1212_als_enable(struct jsa_data *jsa)
{
     unsigned char buf[2];
	 int ret = -1;
   
	 ret = jsa1212_i2c_read(buf, CONFIGURE, sizeof(buf));
	 if (ret < 0){
		printk(KERN_ERR "jsa1212 read out config data failed\n");
		return ret;
	 }

	 buf[1] = buf[0] | (1<<2);
	 ret = jsa1212_i2c_write(&buf[1],CONFIGURE);
	 if (ret < 0){
		printk(KERN_ERR "jsa1212 write config data failed\n");
		return ret;
	 }

	 hrtimer_start(&jsa->timer, jsa->light_poll_delay, HRTIMER_MODE_REL);
	 return 0;
}
static int jsa1212_als_disable(struct jsa_data *jsa)
{
     unsigned char buf[2];
	 int ret = -1;

	 ret = jsa1212_i2c_read(buf, CONFIGURE, sizeof(buf));
	 if (ret < 0){
		printk(KERN_ERR "jsa1212 read out config data failed\n");
		return ret;
	 }
	 
	 buf[1] = buf[0] & (~(1<<2));
	 ret = jsa1212_i2c_write(&buf[1],CONFIGURE);
	 if (ret < 0){
		printk(KERN_ERR "jsa1212 write config data failed\n");
		return ret;
	 }
	 
	 hrtimer_cancel(&jsa->timer);
	 cancel_work_sync(&jsa->work_light);
	 return 0;
}

static int jsa1212_chip_init(void)
{    
    int index;
	int ret = -1;
	unsigned char buf[1];
     /*Config chip init data*/
	for (index = 0; index < (sizeof(jsa1212_init_register) / sizeof(jsa1212_init_register[0])); index ++)
	{
		buf[0] = jsa1212_init_register[index].value;
		ret = jsa1212_i2c_write(buf, jsa1212_init_register[index].reg);
		if (ret < 0)
		{
			printk(KERN_ERR "init jsa1212 failed \n");
			return ret;
		}
	}
	return 0;
}



irqreturn_t jsa_irq_handler(int irq, void *dev_id)
{

	dprintk(DEBUG_CONTROL_INFO, "in irq\n");
	return 0;
}


static int jsa_setup_irq(struct jsa_data *jsa)
{
	int ret = -EIO;

	dprintk(DEBUG_INT, "%s:light sensor irq_number= %d\n", __func__,
			ls_sensor_info.int_number);

	ls_sensor_info.dev = &(jsa->proximity_input_dev->dev);
	if (0 != ls_sensor_info.int_number) {
		ret = input_request_int(&(ls_sensor_info.input_type), jsa_irq_handler,
				IRQF_TRIGGER_FALLING, jsa);
		if (ret) {
			printk("Failed to request gpio irq \n");
			return ret;
		}
	}

	ret = 0;
	return ret;
}


static ssize_t ls_delay_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct jsa_data *jsa = dev_get_drvdata(dev);
	return sprintf(buf, "%lld\n", ktime_to_ns(jsa->light_poll_delay));
}


static ssize_t ls_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct jsa_data *jsa = dev_get_drvdata(dev);
	int64_t new_delay;
	int err;

	err = strict_strtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;

	dprintk(DEBUG_CONTROL_INFO, "ls new delay = %lldns, old delay = %lldns\n",
		    new_delay, ktime_to_ns(jsa->light_poll_delay));
	mutex_lock(&jsa->power_lock);
	if (new_delay != ktime_to_ns(jsa->light_poll_delay)) {
		jsa->light_poll_delay = ns_to_ktime(new_delay);
		if (jsa->power_state & LIGHT_ENABLED) {
			jsa1212_als_disable(jsa);
			jsa1212_als_enable(jsa);
		}
	}
	mutex_unlock(&jsa->power_lock);

	return size;
}

static ssize_t ps_delay_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct jsa_data *jsa = dev_get_drvdata(dev);
       return sprintf(buf, "%lu\n", jsa->ps_poll_delay);
}


static ssize_t ps_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct jsa_data *jsa = dev_get_drvdata(dev);
	int64_t new_delay;
	int err;

	err = strict_strtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;

	dprintk(DEBUG_CONTROL_INFO, "ps new delay = %lldms, old delay = %ldms\n",
		    new_delay,jsa->ps_poll_delay);
	
	jsa->ps_poll_delay = new_delay;

	return size;
}


static ssize_t ls_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct jsa_data *jsa = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n",
		       (jsa->power_state & LIGHT_ENABLED) ? 1 : 0);
}

static ssize_t ps_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct jsa_data *jsa = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n",
		       (jsa->power_state & PROXIMITY_ENABLED) ? 1 : 0);
}


static ssize_t ps_enable_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size)
{
	struct jsa_data *jsa = dev_get_drvdata(dev);
	bool new_value;


	if (atomic_read(&jsa->jsa1212_init) == 0) {
		mutex_lock(&jsa->power_lock);
		jsa->power_state |= PROXIMITY_ENABLED;
		mutex_unlock(&jsa->power_lock);
		return size;
	}
	
	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		printk("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	dprintk(DEBUG_CONTROL_INFO, "new_value = %d, old state = %d\n",
		    new_value, (jsa->power_state & PROXIMITY_ENABLED) ? 1 : 0);
	mutex_lock(&jsa->power_lock);
	if (new_value && !(jsa->power_state & PROXIMITY_ENABLED)) {
		jsa->power_state |= PROXIMITY_ENABLED;
		if(atomic_read(&jsa->jsa1212_suspend) != 1)
				jsa1212_ps_enable(jsa);
	} else if (!new_value && (jsa->power_state & PROXIMITY_ENABLED)) {
		jsa->power_state &= ~PROXIMITY_ENABLED;
		if(atomic_read(&jsa->jsa1212_suspend) != 1)
				jsa1212_ps_disable(jsa);
	}
	mutex_unlock(&jsa->power_lock);

	return size;
}

static ssize_t ls_enable_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size)
{
	struct jsa_data *jsa = dev_get_drvdata(dev);
	bool new_value;


    if (atomic_read(&jsa->jsa1212_init) == 0) {
		mutex_lock(&jsa->power_lock);
		jsa->power_state |= LIGHT_ENABLED;
		mutex_unlock(&jsa->power_lock);
		return size;
	}
	
	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		printk("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}


	dprintk(DEBUG_CONTROL_INFO, "new_value = %d, old state = %d\n",
		    new_value, (jsa->power_state & LIGHT_ENABLED) ? 1 : 0);
	
	mutex_lock(&jsa->power_lock);
	if (new_value && !(jsa->power_state & LIGHT_ENABLED)) {
			jsa->power_state |= LIGHT_ENABLED;
			if (atomic_read(&jsa->jsa1212_suspend) != 1)
					jsa1212_als_enable(jsa);
	} else if (!new_value && (jsa->power_state & LIGHT_ENABLED)) {
			jsa->power_state &= ~LIGHT_ENABLED;
			if (atomic_read(&jsa->jsa1212_suspend) != 1)
					jsa1212_als_disable(jsa);	
	}
	mutex_unlock(&jsa->power_lock);

	return size;
}



static struct device_attribute sensor_dev_attr_als_enable = __ATTR(enable, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP,
    ls_enable_show, ls_enable_store);
static struct device_attribute sensor_dev_attr_als_delay = __ATTR(delay, S_IRUSR|S_IRGRP|S_IWUSR |S_IWGRP,
    ls_delay_show, ls_delay_store);


static struct device_attribute sensor_dev_attr_prx_enable = __ATTR(enable, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP,
    ps_enable_show, ps_enable_store);
static struct device_attribute sensor_dev_attr_prx_delay = __ATTR(delay, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP,
    ps_delay_show, ps_delay_store);




static struct attribute *ls_attributes[] = {
	&sensor_dev_attr_als_enable.attr,
	&sensor_dev_attr_als_delay.attr,
	NULL
};

static struct attribute *ps_attributes[] = {
	&sensor_dev_attr_prx_enable.attr,
	&sensor_dev_attr_prx_delay.attr,	
	NULL
};


static struct attribute_group ls_attribute_group = {
    .attrs = ls_attributes
};

static struct attribute_group ps_attribute_group = {
    .attrs = ps_attributes
};


static int jsa1212_ps_read(void)
{
    int ret = -1;
	unsigned char buf[2];
	unsigned int val;
	/*Read out all data*/
	ret = jsa1212_i2c_read(buf, PROX_DATA, 1);
	if (ret < 0)
	{
		printk(KERN_ERR "jsa1212 read out data error\n");
		return ret;
	}
	val = buf[0] & 0xff;
	return val; 
}


static int jsa1212_als_read(void)
{
    int ret = -1;
	unsigned char buf[2];
	unsigned int val;

	/*Read out all data*/
	ret = jsa1212_i2c_read(buf, ALSIR_DT1, sizeof(buf));
	if (ret < 0)
	{
		printk(KERN_ERR "jsa1212 read out data error\n");
		return ret;
	}

	val = buf[0] | ((buf[1] & 0x0f) << 8);
	return val; 
}



static void jsa_ps_schedwork(struct work_struct *work)
{
	struct jsa_data *jsa = container_of((struct delayed_work *)work, struct jsa_data,ps_delay_work);
	int val=-1;
	val = jsa1212_ps_read();
	dprintk(DEBUG_REPORT_PS_DATA, " ps val =%d\n", val);
	
	/* 0 is close, 1 is far */
	val = val >= PS_DISTANCE ? 0:1;
	input_report_abs(jsa->proximity_input_dev, ABS_DISTANCE, val);
	input_sync(jsa->proximity_input_dev);
	queue_delayed_work(jsa->wq, &jsa->ps_delay_work, msecs_to_jiffies(jsa->ps_poll_delay));
}



static void jsa_als_work_func(struct work_struct *work)
{
	struct jsa_data *jsa = container_of(work, struct jsa_data,
			work_light);

	int adc = jsa1212_als_read();
	if (adc < 0)
	{
		printk("light val err");
		adc = 0; // no light
	}

	dprintk(DEBUG_REPORT_ALS_DATA, "light val=%d\n",  adc);
	input_report_abs(jsa->light_input_dev, ABS_MISC, adc);
	input_sync(jsa->light_input_dev);
}

/* This function is for light sensor.  It operates every a few seconds.
 * It asks for work to be done on a thread because i2c needs a thread
 * context (slow and blocking) and then reschedules the timer to run again.
 */
static enum hrtimer_restart jsa_als_timer_func(struct hrtimer *timer)
{
	struct jsa_data *jsa = container_of(timer, struct jsa_data, timer);
	queue_work(jsa->wq, &jsa->work_light);
	hrtimer_forward_now(&jsa->timer, jsa->light_poll_delay);
	return HRTIMER_RESTART;
}

static void jsa1212_init_events(struct work_struct *work)
{
	jsa1212_chip_init();

	mutex_lock(&jsa1212_data->power_lock);
	if (jsa1212_data->power_state &= LIGHT_ENABLED) {
		jsa1212_als_enable(jsa1212_data);
	}

	if (jsa1212_data->power_state &= PROXIMITY_ENABLED) {
		jsa1212_ps_enable(jsa1212_data);
	}
	mutex_unlock(&jsa1212_data->power_lock);

	atomic_set(&jsa1212_data->jsa1212_init, 1);
}


static int __devinit jsa1212_probe(struct i2c_client *client, 
											const struct i2c_device_id *id)
{
    int ret = -ENODEV;
	

	struct jsa_data *jsa;
	struct input_dev *input_dev;

    /* Chcek I2C */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
       		printk(KERN_ERR "Check I2C functionality failed.\n");
        	ret = -EIO;
    }

    jsa = kzalloc(sizeof(struct jsa_data), GFP_KERNEL);
	if (!jsa) {
		printk("%s: failed to alloc memory for module data\n", __func__);
		return -ENOMEM;
	}
	
	jsa->i2c_client = client;	
	i2c_set_clientdata(client, jsa);
	jsa->ps_poll_delay = 100;	
	
	/* the timer just fires off a work queue request.  we need a thread
	to read the i2c (can be slow and blocking). */	
	jsa->wq = create_singlethread_workqueue("jsa_wq");	
	if (!jsa->wq) {		
			ret = -ENOMEM;		
			printk("%s: could not create workqueue\n", __func__);		
			goto err_create_workqueue;
	}

	/* ==================proximity  sensor====================== */

	mutex_init(&jsa->power_lock);	
	INIT_DELAYED_WORK(&jsa->ps_delay_work, jsa_ps_schedwork);
	
	/* allocate proximity input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {		
			printk("%s: could not allocate input device\n", __func__);
			goto err_input_allocate_device_proximity;
	}	
	
	jsa->proximity_input_dev = input_dev;
	input_set_drvdata(input_dev, jsa);
	input_dev->name = "proximity";
	input_set_capability(input_dev, EV_ABS, ABS_DISTANCE);
	
	/* 0,close ,1 far */ 
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	
	printk("registering proximity input device\n");	
	ret = input_register_device(input_dev);	
	if (ret < 0){	
			printk("%s: could not register input device\n", __func__);
			input_free_device(input_dev);	
			goto err_input_register_device_proximity;
	}	
	
	ret = sysfs_create_group(&input_dev->dev.kobj,&ps_attribute_group);	
	if (ret) {	
			printk("%s: could not create sysfs group\n", __func__);	
			goto err_sysfs_create_group_proximity;

	}


    /* ==================light sensor====================== */	
    /* hrtimer settings.  we poll for light values using a timer. */
	
	hrtimer_init(&jsa->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	jsa->light_poll_delay = ns_to_ktime(200 * NSEC_PER_MSEC);
	jsa->timer.function = jsa_als_timer_func;	
	
	/* this is the thread function we run on the work queue */
	INIT_WORK(&jsa->work_light, jsa_als_work_func);
	
	/* allocate lightsensor-level input_device */	
	input_dev = input_allocate_device();	
	if (!input_dev) {	
			printk("%s: could not allocate input device\n", __func__);	
			ret = -ENOMEM;		
			goto err_input_allocate_device_light;	
	}	
	input_set_drvdata(input_dev, jsa);
	input_dev->name = "lightsensor";	
	input_set_capability(input_dev, EV_ABS, ABS_MISC);	

	
	input_set_abs_params(input_dev, ABS_MISC, 0, 4095, 0, 0);
	
	dprintk(DEBUG_INIT, "registering lightsensor-level input device\n");
	ret = input_register_device(input_dev);
	if (ret < 0) {	
			printk("%s: could not register input device\n", __func__);
			input_free_device(input_dev);	
			goto err_input_register_device_light;	
	}	

	jsa->light_input_dev = input_dev;	
	ret = sysfs_create_group(&input_dev->dev.kobj, &ls_attribute_group);	
	if (ret) {	
			printk("%s: could not create sysfs group\n", __func__);
			goto err_sysfs_create_group_light;	
	}

	
	atomic_set(&jsa->jsa1212_init, 0);
	atomic_set(&jsa->jsa1212_suspend, 0);
	jsa1212_data = jsa;	
	jsa1212_resume_wq = create_singlethread_workqueue("jsa1212_resume");
	if (jsa1212_resume_wq == NULL) {
			printk("create jsa1212_resume_wq fail!\n");	
			return -ENOMEM;
	}	
	jsa1212_init_wq = create_singlethread_workqueue("jsa1212_init");
	if (jsa1212_init_wq == NULL) {
			printk("create jsa1212_init_wq fail!\n");
			return -ENOMEM;	
	}

	
	
	/* Initialize the jsa1212 chip */
	queue_work(jsa1212_init_wq, &jsa1212_init_work);

	ret = jsa_setup_irq(jsa);
	if (ret) {
		printk("%s: could not setup irq\n", __func__);
		goto err_sysfs_create_group_light;
	}
    
	dprintk(DEBUG_INIT, "jsa probe OK!");

	return 0;
	
		/* error, unwind it all */
	err_sysfs_create_group_light:
		input_unregister_device(jsa->light_input_dev);
	err_input_register_device_light:
	err_input_allocate_device_light:
		sysfs_remove_group(&jsa->proximity_input_dev->dev.kobj,
				   &ps_attribute_group);
	err_sysfs_create_group_proximity:
		input_unregister_device(jsa->proximity_input_dev);
	err_input_register_device_proximity:
	err_input_allocate_device_proximity:
		mutex_destroy(&jsa->power_lock);
		destroy_workqueue(jsa->wq);
	err_create_workqueue:
		kfree(jsa);

	return ret;
}

static int jsa1212_remove(struct i2c_client *client)
{
	struct jsa_data *jsa = i2c_get_clientdata(client);

	cancel_work_sync(&jsa1212_init_work);
	destroy_workqueue(jsa1212_init_wq);
	cancel_work_sync(&jsa1212_resume_work);
	destroy_workqueue(jsa1212_resume_wq);
	sysfs_remove_group(&jsa->light_input_dev->dev.kobj,
			   &ls_attribute_group);
	input_unregister_device(jsa->light_input_dev);
	sysfs_remove_group(&jsa->proximity_input_dev->dev.kobj,
			   &ps_attribute_group);
	input_unregister_device(jsa->proximity_input_dev);
	if (0 != ls_sensor_info.int_number)
		input_free_int(&(ls_sensor_info.input_type), jsa);
	if (jsa->power_state) {
		if (jsa->power_state & LIGHT_ENABLED)
			jsa1212_als_disable(jsa);
		if (jsa->power_state & PROXIMITY_ENABLED)
		    jsa1212_ps_disable(jsa);
		jsa->power_state = 0;
	}
	cancel_delayed_work_sync(&jsa->ps_delay_work);
	destroy_workqueue(jsa->wq);
	mutex_destroy(&jsa->power_lock);
	kfree(jsa);
	return 0;
}




static void jsa1212_resume_events (struct work_struct *work)
{
	
    jsa1212_chip_init();
	
	mutex_lock(&jsa1212_data->power_lock);	
	if (jsa1212_data->power_state & LIGHT_ENABLED) 
		jsa1212_als_enable(jsa1212_data);
		
	if (jsa1212_data->power_state & PROXIMITY_ENABLED) {
#ifdef CONFIG_SCENELOCK
			if (check_scene_locked(SCENE_TALKING_STANDBY) != 0)
#endif
			jsa1212_ps_enable(jsa1212_data);
	}
	mutex_unlock(&jsa1212_data->power_lock);

	atomic_set(&jsa1212_data->jsa1212_suspend, 0);
}


#ifdef CONFIG_PM
static int jsa_suspend(struct i2c_client *client, pm_message_t mesg)
{
        /* We disable power only if proximity is disabled.  If proximity
	   is enabled, we leave power on because proximity is allowed
	   to wake up device.  We remove power without changing
	   ltr->power_state because we use that state in resume.
	*/
	struct jsa_data *jsa = i2c_get_clientdata(client);

	dprintk(DEBUG_SUSPEND, "==suspend=\n");

	atomic_set(&jsa->jsa1212_suspend, 1);

	if (jsa->power_state & LIGHT_ENABLED){
		jsa1212_als_disable(jsa);
	}

	if (jsa->power_state & PROXIMITY_ENABLED){
#ifdef CONFIG_SCENELOCK
		if (check_scene_locked(SCENE_TALKING_STANDBY) != 0)
#endif
		jsa1212_ps_disable(jsa);
	}

	return 0;
}

static int jsa_resume(struct i2c_client *client)
{
	/* Turn power back on if we were before suspend. */
	struct jsa_data *jsa = i2c_get_clientdata(client);

	dprintk(DEBUG_SUSPEND, "==resume=\n");
	
	if (NORMAL_STANDBY == standby_type) {
		if (jsa->power_state & LIGHT_ENABLED){
		    
			jsa1212_als_enable(jsa);
		}

		if (jsa->power_state & PROXIMITY_ENABLED){

			jsa1212_ps_enable(jsa);
		}
		atomic_set(&jsa->jsa1212_suspend, 0);
	} else if (SUPER_STANDBY == standby_type)
		queue_work(jsa1212_resume_wq, &jsa1212_resume_work);

	return 0;
		
}
#endif


static const struct i2c_device_id jsa1212_id[] = {
    {PL_NAME, 0},
    { }
};

MODULE_DEVICE_TABLE(i2c, jsa1212_id);

static struct i2c_driver jsa1212_driver = {
	.class = I2C_CLASS_HWMON,
    .driver = {
        .owner = THIS_MODULE,
        .name  = PL_NAME,
    },
    .id_table  = jsa1212_id,
    .probe     = jsa1212_probe,
    .remove    = __devexit_p(jsa1212_remove),
    .address_list = normal_i2c,
#ifdef CONFIG_PM	
    .suspend  	= jsa_suspend,	
    .resume  	= jsa_resume,
#endif	

};


static int ls_detect(struct i2c_client *client, struct i2c_board_info *info)
{	struct i2c_adapter *adapter = client->adapter;
    int ret;	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) 
			return -ENODEV;            	
	if (ls_sensor_info.twi_id == adapter->nr) {		
		    dprintk(DEBUG_INIT, "%s:addr= 0x%x\n",__func__,client->addr);
			ret=i2c_test(client);
			if(ret) {
					dprintk(DEBUG_INIT, "LS Device detected!\n" );		
					strlcpy(info->type, SENSOR_NAME, I2C_NAME_SIZE);	
					return 0;	
			}else{
	
					printk("%s:i2c connection maybe something wrong \n",__func__);	
					return -ENODEV; 
			}		
	} else {	
			return -ENODEV;
	}
}




/* driver initial part */
static int __init jsa1212_init(void)
{
    int ret = 0;
	dprintk(DEBUG_INIT, "%s:P & L sensor driver init\n", __func__ );

	if (input_fetch_sysconfig_para(&(ls_sensor_info.input_type))) {
		printk("%s: ls_fetch_sysconfig_para err.\n", __func__);
		return 0;
	} else {
		ret = input_init_platform_resource(&(ls_sensor_info.input_type));
		if (0 != ret) {
			printk("%s:ls_init_platform_resource err. \n", __func__);    
		}
	}

	if (ls_sensor_info.sensor_used == 0) {
		printk("*** ls_used set to 0 !\n");
		printk("*** if use light_sensor,please put the sys_config.fex ls_used set to 1. \n");
		return 0;
	}

	jsa1212_driver.detect = ls_detect;
	return i2c_add_driver(&jsa1212_driver);
}


static void  __exit jsa1212_exit(void)
{
    printk(KERN_INFO "deregister jsa1212 i2c driver! \n");
	
    i2c_del_driver(&jsa1212_driver);
	input_free_platform_resource(&(ls_sensor_info.input_type));
}


late_initcall(jsa1212_init);
module_exit(jsa1212_exit);

module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);


MODULE_AUTHOR("<Solteam Opto, Inc.>");
MODULE_DESCRIPTION("Solteam Opto JSA1212 Ambient Light & Proximity Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.8.0");
