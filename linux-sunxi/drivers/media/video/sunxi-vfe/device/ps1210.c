/*
 * A V4L2 driver for PS1210 Raw cameras.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-mediabus.h>
#include <linux/io.h>


#include "camera.h"


MODULE_AUTHOR("lwj");
MODULE_DESCRIPTION("A low-level driver for PS1210 Raw sensors");
MODULE_LICENSE("GPL");

//for internel driver debug
#define DEV_DBG_EN      1 
#if(DEV_DBG_EN == 1)    
#define vfe_dev_dbg(x,arg...) printk("[PS1210 Raw]"x,##arg)
#else
#define vfe_dev_dbg(x,arg...) 
#endif
#define vfe_dev_err(x,arg...) printk("[PS1210 Raw]"x,##arg)
#define vfe_dev_print(x,arg...) printk("[PS1210 Raw]"x,##arg)

#define LOG_ERR_RET(x)  { \
                          int ret;  \
                          ret = x; \
                          if(ret < 0) {\
                            vfe_dev_err("error at %s\n",__func__);  \
                            return ret; \
                          } \
                        }

//define module timing
#define MCLK              (24*1000*1000)
#define VREF_POL          V4L2_MBUS_VSYNC_ACTIVE_HIGH
#define HREF_POL          V4L2_MBUS_HSYNC_ACTIVE_HIGH
#define CLK_POL           V4L2_MBUS_PCLK_SAMPLE_RISING
#define V4L2_IDENT_SENSOR  0x1210


//define the voltage level of control signal
#define CSI_STBY_ON     1
#define CSI_STBY_OFF    0
#define CSI_RST_ON      0
#define CSI_RST_OFF     1
#define CSI_PWR_ON      1
#define CSI_PWR_OFF     0
#define CSI_AF_PWR_ON   1
#define CSI_AF_PWR_OFF  0
#define regval_list reg_list_a8_d8


#define REG_TERM 0xfffe
#define VAL_TERM 0xfe
#define REG_DLY  0xff

/*
 * Our nominal (default) frame rate.
 */

#define SENSOR_FRAME_RATE 30


/*
 * The ps1210 i2c address
 */
//#define I2C_ADDR 0x6c
#define PS1210_WRITE_ADDR (0xee)
#define PS1210_READ_ADDR  (0xef)

//static struct delayed_work sensor_s_ae_ratio_work;
static struct v4l2_subdev *glb_sd;
#define SENSOR_NAME "ps1210"

/*
 * Information we maintain about a known sensor.
 */
struct sensor_format_struct;  /* coming later */

struct cfg_array { /* coming later */
	struct regval_list * regs;
	int size;
};

static inline struct sensor_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sensor_info, sd);
}


/*
 * The default register settings
 *
 */

static struct regval_list sensor_default_regs[] = 
{
#if 0
	{0x03, 0x00},	 
	{0x29, 0x98}, 
		   
	{0x08, 0x04},
	{0x09, 0x64},
		   
	{0x03, 0x00},	  
	{0x25, 0x00},	   
	{0x26, 0xC3},	
		   
	{0x34, 0x05},
		   
	{0x41, 0x0B}, 
	{0x42, 0x04},	  
		   
	{0x03, 0x02},
	{0x2b, 0x08},
		   
	{0x03, 0x00},
		   
	{0x03, 0x02},
	{0x2c, 0x00},
	{0x2d, 0x00},
		   
	{0x9B, 0x20},	
		   
	{0x03, 0x00},
	{0x27, 0x67},	
	{0x28, 0xf0},
	{0x29, 0xf8},	
		   
	{0x03, 0x01}, 
		   
	{0x03, 0x02},
	{0x9b, 0x20},	
	{0x03, 0x01},
		   
	{0x16, 0x45},
		   
	{0x03, 0x00},	 
	{0x29, 0x98}, 
		   
	{0x06, 0x0f},
	{0x07, 0x00},
	{0x0c, 0x00},	 
	{0x0d, 0x0d},
	{0x0e, 0x00},
	{0x0f, 0x0e},
	{0x10, 0x07},	 
	{0x11, 0x8c},
	{0x12, 0x04},
	{0x13, 0x45},
		   
	{0x14, 0x00},	 
	{0x15, 0x19},	
	{0x16, 0x04},
	{0x17, 0x53},
		   
	{0x25, 0x00},	   
	{0x28, 0x30},	
		   
	{0x33, 0x01},
	{0x34, 0x02},
	{0x36, 0xc8},
	{0x38, 0x48},
	{0x3a, 0xc6},	//external LDO 0xc6, internal LDO 0x22
		   
	{0x41, 0x0c}, 
	{0x42, 0x03},	  
	{0x40, 0x10},
	{REG_DLY,0xff},	//delay
		   
	{0x40, 0x00},	   
	{0x03, 0x01},
	{0x26, 0x03},
	{0x03, 0x01},
	{0xc0, 0x04},
	{0xc1, 0x5f},
	{0xc2, 0x00},
	{0xc3, 0x60},
	{0xc4, 0x40},
	{0x03, 0x02},
	{0x2b, 0x60},
	{0x2c, 0x30},
		   
	{0x03, 0x00},	
	{0x33, 0x03},
	{0x34, 0x03},	
	{0x03, 0x01},
	{0x36, 0x04},	
	{0x37, 0x9e}, 
	{0x38, 0x0c},
	{0x39, 0x2e},
	{0xa1, 0x07},	
	{0xa2, 0xa4},
	{0x77, 0x0e},	 
	{0x78, 0x2e},	
	{0x79, 0x0e},
	{0x7a, 0x38},	
	{0x87, 0x0e},
	{0x88, 0x11},	
	{0x89, 0x0e}, 
	{0x8a, 0x16},
	{0x03, 0x01},	
	{0xb6, 0x94},
#endif

{0x03, 0x00},//
{0x29, 0x98},//	# output Hi-z release

{0x03, 0x00},//
{0x05, 0x03},//	# mirror/flip
{0x06, 0x09},//	# framewidth_h        (08)
{0x07, 0x5F},//	# framewidth_l        (97)

{0x14, 0x00},//	# vsyncstartrow_f0_h	(00)
{0x15, 0x0B},//	# vsyncstartrow_f0_l	(0D)

{0x25, 0x08},// # CLK DIV1

{0x33, 0x01},// # pixelbias
{0x34, 0x02},// # compbias

{0x36, 0x88},// # TX_Bias; DCDC 4.96 V, LDO 4.37 V => DCDC 4.48 V, LDO 4.37 V ( C8h => 88h )
{0x38, 0x48},// # black_bias, range_sel 0.4 V
{0x3A, 0x66},// # main regulator output -- external DVDD

{0x41, 0x1B},// # pll_m_cnt (21)
{0x42, 0x04},// # pll_r_cnt (04)

{0x40, 0x10},// # pll_control

	{REG_DLY,0xff},	//delay
	{REG_DLY,0xff},	//delay

{0x40, 0x00},// # pll_control on

{0x03, 0x01},//
{0x26, 0x03},// # blacksun_th_h
{0x62, 0x96},
{0x03, 0x01},//
{0xC0, 0x04},// # inttime_h  
{0xC1, 0x5F},// # inttime_m  
{0xC2, 0x00},// # inttime_l  
{0xC3, 0x00},// # globalgain 
{0xC4, 0x40},// # digitalgain
};

static struct regval_list sensor_1080p_regs[] = { //1080: 1920*1080@30fps

};



/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 * 
 */

static struct regval_list sensor_fmt_raw[] = {
//{REG_TERM,VAL_TERM},
};

/*
 * Low-level register I/O.
 *
 */


/*
 * On most platforms, we'd rather do straight i2c I/O.
 */

static int sensor_read(struct v4l2_subdev *sd, unsigned char reg,
    unsigned char *value)
{
	int ret=0;
	int cnt=0;
	
  ret = cci_read_a8_d8(sd,reg,value);
  while(ret!=0&&cnt<2)
  {
  	ret = cci_read_a8_d8(sd,reg,value);
		cnt++;
	}
	if(cnt>0)
		vfe_dev_dbg("sensor read retry=%d\n",cnt);
  
	return ret;
}

//static int reg_val_show(struct v4l2_subdev *sd,unsigned char reg)
//{
//	unsigned char tmp;
//	sensor_read(sd,reg,&tmp);
//	printk("0x%x value is 0x%x\n",reg,tmp);
//	return 0;
//}

/*static int show_regs_array (struct v4l2_subdev *sd,struct regval_list *array)
{	int i =0;
	unsigned char tmp;
	
	for(i=0;i<(sizeof(sensor_1080p_regs)/3);i++) 
	{		if((array+i)->addr == REG_DLY) 
		 {
               msleep((array+i)->data);
   	     } 
		sensor_read(sd,(array+i)->addr,&tmp);
		printk("ps1210 0x%x value is 0x%x\n",(array+i)->addr,tmp);
	}
	return 0;
}
*/

static int sensor_write(struct v4l2_subdev *sd, unsigned char reg,
    unsigned char value)
{
	int ret=0;
	int cnt=0;
	
//	usleep_range(10000,12000);

  ret = cci_write_a8_d8(sd,reg,value);
  while(ret!=0&&cnt<2)
  {
  	ret = cci_write_a8_d8(sd,reg,value);
		cnt++;
	}
	if(cnt>0)
		vfe_dev_dbg("sensor write retry=%d\n",cnt);

//	usleep_range(10000,12000);
//	reg_val_show(sd,reg);


	return ret;
}

/*
 * Write a list of register settings;
 */
static int sensor_write_array(struct v4l2_subdev *sd, struct regval_list *regs, int array_size)
{
	int i=0;
//	unsigned char read_value;
	
	if(!regs)
	return -EINVAL;

	while(i<array_size)
	{
		if(regs->addr == REG_DLY) {
			msleep(regs->data);
		} else {
//			vfe_dev_dbg("write 0x%x, 0x%x\n", regs->addr, regs->data);
			
			LOG_ERR_RET(sensor_write(sd, regs->addr, regs->data))
#if 0				
			sensor_read(sd,regs->addr,&read_value);
			if (read_value != regs->data)
			{
			    sensor_write(sd,regs->addr,regs->data);
				vfe_dev_dbg("sensor_write_array count=1\n");
			}

			
			sensor_read(sd,regs->addr,&read_value);
			if (read_value != regs->data)
			{
			    sensor_write(sd,regs->addr,regs->data);
				vfe_dev_dbg("sensor_write_array count=2\n");
			}

			sensor_read(sd,regs->addr,&read_value);
			if (read_value != regs->data)
			{
				sensor_write(sd,regs->addr,regs->data);
				vfe_dev_dbg("sensor_write_array count=3\n");
			}

			sensor_read(sd,regs->addr,&read_value);
			if (read_value != regs->data)
			{
				sensor_write(sd,regs->addr,regs->data);
				vfe_dev_dbg("sensor_write_array count=4\n");
			}
#endif
		}
		i++;
		regs++;
	}
	return 0;
}

/* 
 * Code for dealing with controls.
 * fill with different sensor module
 * different sensor module has different settings here
 * if not support the follow function ,retrun -EINVAL
 */
static int sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->exp;
	vfe_dev_dbg("sensor_get_exposure = %d\n", info->exp);
	return 0;
}

static int sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	unsigned char explow,expmid,exphigh;
	struct sensor_info *info = to_state(sd);
	
	if(exp_val>0x1fffff)
		exp_val=0x1fffff;
	
	exphigh	= (unsigned char) ( (0x0ff000&exp_val)>>12);
	expmid	= (unsigned char) ( (0x000ff0&exp_val)>>4);
	explow  = (unsigned char) ( (0x00000f&exp_val)<<4);
	
	sensor_write(sd, 0xc2, explow);
	sensor_write(sd, 0xc1, expmid);
	sensor_write(sd, 0xc0, exphigh);
//	printk("ps1210 sensor_set_exp = %d Done!\n", exp_val );

	info->exp = exp_val;
	return 0;
}


static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->gain;
	vfe_dev_dbg("sensor_get_gain = %d\n", info->gain);
	return 0;
}

static int analog_gain_table[112] =
{
    1024,   1088,   1152,   1216,   1280,   1344,   1408,   1472,   1536,   1600,   1664,   1728,  1792,   1856, 
    1920,   1984,   2048,   2176,   2304,   2432,   2560,   2688,   2816,   2944,   3072,   3200,  3328,   3456, 
    3584,   3712,   3840,   3968,   4096,   4352,   4608,   4864,   5120,   5376,   5632,   5888,  6144,   6400,
    6656,   6912,   7168,   7424,   7680,   7936,   8192,   8704,   9216,   9728,   10240, 10752, 11264, 11776,
    12288, 12800, 13312, 13824, 14336, 14848, 15360, 15872, 16384, 17408, 18432, 19456, 
    20480, 21504, 22528, 23552, 24576, 25600, 26624, 27648, 28672, 29696, 30720, 31744, 
    32768, 34816, 36864, 38912, 40960, 43008, 45056, 47104, 49152, 51200, 53248, 55296, 
    57344, 59392, 61440, 63488,67584,71680,75776,79872,83968,88064,92160,96256,100352,104448,
	108544,112640,116736,120832,124928,129024
};

static int sensor_s_gain(struct v4l2_subdev *sd, int gain_val)
{		
	struct sensor_info *info = to_state(sd);
	unsigned char gainlow=0;
	unsigned char i=0;
	unsigned int tmp_gain_val=0;
	
	tmp_gain_val = gain_val<<6;

    if (tmp_gain_val >= analog_gain_table[111])
    {
         gainlow = 111;
         return 0;
    }
    
    for(i = 1; i < 112; i++)
    {
        if(tmp_gain_val < analog_gain_table[i])
        {
            gainlow = i - 1;
            break;
        }
    }

	sensor_write(sd, 0xc3, gainlow);

	info->gain = gain_val;
	
	return 0;
	
}

static int ps1210_sensor_vts;
static int sensor_s_exp_gain(struct v4l2_subdev *sd, struct sensor_exp_gain *exp_gain)
{
  int exp_val, gain_val,shutter,frame_length;  
  struct sensor_info *info = to_state(sd);

  exp_val = exp_gain->exp_val;
  gain_val = exp_gain->gain_val;

  if(gain_val<1*16)
	  gain_val=16;
  if(gain_val>64*16-1)
	  gain_val=64*16-1;
  
  if(exp_val>0xfffff)
	  exp_val=0xfffff;

  shutter = exp_val/16;
  if(shutter > ps1210_sensor_vts- 4)
		frame_length = shutter + 4;
  else
		frame_length = ps1210_sensor_vts;

  sensor_write(sd, 0x09, (frame_length & 0xff));
  sensor_write(sd, 0x08, (frame_length >> 8));
  sensor_s_exp(sd,exp_val);
  sensor_s_gain(sd,gain_val);
  
//  printk("ps1210 sensor_set_gain gain= %d,exp= %d Done!\n", gain_val,exp_val);

  info->exp = exp_val;
  info->gain = gain_val;
  return 0;
}


static int sensor_s_sw_stby(struct v4l2_subdev *sd, int on_off)
{
	int ret = 0;
	return ret;
}

/*
 * Stuff that knows about the sensor.
 */
 
static int sensor_power(struct v4l2_subdev *sd, int on)
{
	//  struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	//insure that clk_disable() and clk_enable() are called in pair 
	//when calling CSI_SUBDEV_STBY_ON/OFF and CSI_SUBDEV_PWR_ON/OFF
	ret = 0;
	switch(on)
	{
		case CSI_SUBDEV_STBY_ON:
			vfe_dev_dbg("CSI_SUBDEV_STBY_ON!\n");
			//software standby on
			ret = sensor_s_sw_stby(sd, CSI_GPIO_HIGH);
			if(ret < 0)
				vfe_dev_err("soft stby falied!\n");
			usleep_range(10000,12000);
			//when using i2c_lock_adpater function, the following codes must not access i2c bus before calling i2c_unlock_adapter
			cci_lock(sd);
			//standby on io
//			vfe_gpio_write(sd,PWDN,CSI_GPIO_HIGH);
			//remember to unlock i2c adapter, so the device can access the i2c bus again
			cci_unlock(sd);
			//inactive mclk after stadby in
			vfe_set_mclk(sd,OFF);
			break;
		case CSI_SUBDEV_STBY_OFF:
			vfe_dev_dbg("CSI_SUBDEV_STBY_OFF!\n");
			cci_lock(sd);
			vfe_set_mclk_freq(sd,MCLK);
			vfe_set_mclk(sd,ON);
			usleep_range(10000,12000);
//			vfe_gpio_write(sd,PWDN,CSI_GPIO_LOW);
			usleep_range(10000,12000);
			ret = sensor_s_sw_stby(sd, CSI_GPIO_LOW);
			if(ret < 0)
				vfe_dev_err("soft stby off falied!\n");
			cci_unlock(sd);
			break;
		case CSI_SUBDEV_PWR_ON:
			vfe_dev_dbg("CSI_SUBDEV_PWR_ON!\n");
			cci_lock(sd);
						//power supply
			vfe_set_pmu_channel(sd,IOVDD,ON);
			//add by chao 
			usleep_range(1000,1200);
			vfe_set_pmu_channel(sd,AVDD,ON);	
			vfe_gpio_write(sd,POWER_EN,CSI_GPIO_HIGH);
			vfe_set_pmu_channel(sd,DVDD,ON);
			vfe_set_pmu_channel(sd,AFVDD,ON);
			//power on reset
			vfe_gpio_set_status(sd,PWDN,1);//set the gpio to output
			vfe_gpio_set_status(sd,RESET,1);//set the gpio to output
			vfe_gpio_set_status(sd,POWER_EN,1);//set the gpio to output 
			
//			vfe_gpio_write(sd,PWDN,CSI_GPIO_HIGH);
//			usleep_range(7000,8000);
//			vfe_gpio_write(sd,PWDN,CSI_GPIO_LOW);
			usleep_range(7000,8000);
			vfe_gpio_write(sd,RESET,CSI_GPIO_LOW);  
			usleep_range(7000,8000);
			
			vfe_gpio_write(sd,POWER_EN,CSI_GPIO_LOW);
			usleep_range(1000,1200);

			//add by chao
			usleep_range(7000,8000);
//			vfe_gpio_write(sd,PWDN,CSI_GPIO_HIGH); 
			//reset on io
			//active mclk before power on
			vfe_set_mclk_freq(sd,MCLK);
			vfe_set_mclk(sd,ON);
			usleep_range(10000,12000); 
			vfe_gpio_write(sd,RESET,CSI_GPIO_HIGH);
			usleep_range(10000,12000);
			cci_unlock(sd);
			break;
		case CSI_SUBDEV_PWR_OFF:
			vfe_dev_dbg("CSI_SUBDEV_PWR_OFF!\n");
			cci_lock(sd);
//			vfe_gpio_set_status(sd,PWDN,1);//set the gpio to output
			vfe_gpio_set_status(sd,RESET,1);//set the gpio to output
			vfe_gpio_write(sd,RESET,CSI_GPIO_HIGH);  
//			vfe_gpio_write(sd,PWDN,CSI_GPIO_HIGH);
			//inactive mclk before power off
			vfe_set_mclk(sd,OFF);
			//power supply off
			vfe_set_pmu_channel(sd,AFVDD,OFF);
			vfe_set_pmu_channel(sd,DVDD,OFF);
			vfe_gpio_write(sd,POWER_EN,CSI_GPIO_LOW);
			vfe_set_pmu_channel(sd,AVDD,OFF);
			vfe_set_pmu_channel(sd,IOVDD,OFF);
			//set the io to hi-z
			vfe_gpio_set_status(sd,RESET,CSI_GPIO_LOW);//set the gpio to input
//			vfe_gpio_set_status(sd,PWDN,CSI_GPIO_LOW);//set the gpio to input
			vfe_gpio_set_status(sd,POWER_EN,0);//set the gpio to input
			cci_unlock(sd);
			break;
		default:
			return -EINVAL;
	}

	return 0;

}
 
static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	switch(val)
	{
		case 0:
			vfe_gpio_write(sd,RESET,CSI_RST_OFF);
			usleep_range(10000,12000);
			break;
		case 1:
			vfe_gpio_write(sd,RESET,CSI_RST_ON);
			usleep_range(10000,12000);
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
	unsigned char rdval = 0;
	vfe_dev_dbg("sensor_detect_1--!\n");
  
	LOG_ERR_RET(sensor_read(sd, 0x00, &rdval));
	vfe_dev_dbg("sensor_detect_2--!\n");

	if(rdval != (V4L2_IDENT_SENSOR>>8))
	{
		printk(KERN_DEBUG"*********sensor error,read id is %d.\n",rdval);
		return -ENODEV;
	}

	LOG_ERR_RET(sensor_read(sd, 0x01, &rdval))
	if(rdval != (V4L2_IDENT_SENSOR&0x00ff))
	{
		printk(KERN_DEBUG"*********sensor error,read id is %d.\n",rdval);
		return -ENODEV;
	}
	else
	{
		printk(KERN_DEBUG"*********find ps1210 raw data camera sensor now.\n");
		return 0;
	}
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct sensor_info *info = to_state(sd);
   
	vfe_dev_dbg("sensor_init\n");
  
	/*Make sure it is a target sensor*/
	ret = sensor_detect(sd);
	if (ret) {
		vfe_dev_err("chip found is not an target chip.\n");
		return ret;
	}
  
	vfe_get_standby_mode(sd,&info->stby_mode);

	if((info->stby_mode == HW_STBY || info->stby_mode == SW_STBY) \
	&& info->init_first_flag == 0) {
		vfe_dev_print("stby_mode and init_first_flag = 0\n");
		return 0;
	} 
  
	info->focus_status = 0;
	info->low_speed = 0;
	info->width = HD1080_WIDTH;
	info->height = HD1080_HEIGHT;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 0;

	info->tpf.numerator = 1;            
	info->tpf.denominator = 30;    /* 30fps */    

	ret = sensor_write_array(sd, sensor_default_regs, ARRAY_SIZE(sensor_default_regs));  
	if(ret < 0) {
		vfe_dev_err("write sensor_default_regs error\n");
		return ret;
	}
  
	if(info->stby_mode == 0)
		info->init_first_flag = 0;
  
	info->preview_first_flag = 1;
     
	return 0;
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret=0;
	struct sensor_info *info = to_state(sd);

	switch(cmd) {
	case GET_CURRENT_WIN_CFG:
		if(info->current_wins != NULL)
		{
			memcpy( arg,
			        info->current_wins,
			        sizeof(struct sensor_win_size) );
			ret=0;
		}
		else
		{
			vfe_dev_err("empty wins!\n");
			ret=-1;
		}
		break;
	case SET_FPS:
		ret=0;
		break;
	case ISP_SET_EXP_GAIN:
		ret = sensor_s_exp_gain(sd, (struct sensor_exp_gain *)arg);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}


/*
 * Store information about the video data format. 
 */
static struct sensor_format_struct {
  __u8 *desc;
  //__u32 pixelformat;
  enum v4l2_mbus_pixelcode mbus_code;
  struct regval_list *regs;
  int regs_size;
  int bpp;   /* Bytes per pixel */
}sensor_formats[] = {
	{
		.desc		= "Raw RGB Bayer",
		.mbus_code	=V4L2_MBUS_FMT_SBGGR10_1X10,
		.regs 		= sensor_fmt_raw,
		.regs_size = ARRAY_SIZE(sensor_fmt_raw),
		.bpp		= 1
	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */

static struct sensor_win_size sensor_win_sizes[] = {
	/* 1080P */
	{
		.width	    = HD1080_WIDTH,
		.height 	= HD1080_HEIGHT,
		.hoffset	= 0,
		.voffset	= 0,
		.hts        = 2400,
		.vts        = 1125,
		.pclk       = 81000000,
		.fps_fixed  = 1,
		.bin_factor = 1,
		.intg_min   = 1<<4,
		.intg_max   = 1124<<4,
		.gain_min   = 1<<4,
		.gain_max   = 20<<4,
		.regs       = sensor_1080p_regs,//
		.regs_size  = ARRAY_SIZE(sensor_1080p_regs),//
		.set_size	= NULL,
	},

	/* 720P */
	{
		.width	    = HD720_WIDTH,
		.height 	= HD720_HEIGHT,
		.hoffset	= 0,
		.voffset	= 0,
		.hts		= 2400,
		.vts		= 1125,
		.pclk		= 81000000,
		.fps_fixed	= 1,
		.bin_factor = 1,
		.intg_min	= 1<<4,
		.intg_max	= 1124<<4,
		.gain_min	= 1<<4,
		.gain_max	= 20<<4,
		.width_input	  = 1920,
		.height_input	  = 1080,	
		.regs		= sensor_1080p_regs,//
		.regs_size	= ARRAY_SIZE(sensor_1080p_regs),//
		.set_size		= NULL,
	},

};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

static int sensor_enum_fmt(struct v4l2_subdev *sd, unsigned index,
                 enum v4l2_mbus_pixelcode *code)
{
	if (index >= N_FMTS)
		return -EINVAL;

	*code = sensor_formats[index].mbus_code;
	return 0;
}

static int sensor_enum_size(struct v4l2_subdev *sd,
                            struct v4l2_frmsizeenum *fsize)
{
	if(fsize->index > N_WIN_SIZES-1)
		return -EINVAL;
  
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = sensor_win_sizes[fsize->index].width;
	fsize->discrete.height = sensor_win_sizes[fsize->index].height;
  
	return 0;
}

static int sensor_try_fmt_internal(struct v4l2_subdev *sd,
    struct v4l2_mbus_framefmt *fmt,
    struct sensor_format_struct **ret_fmt,
    struct sensor_win_size **ret_wsize)
{
	int index;
	struct sensor_win_size *wsize;
	struct sensor_info *info = to_state(sd);

	for (index = 0; index < N_FMTS; index++)
		if (sensor_formats[index].mbus_code == fmt->code)
		break;

	if (index >= N_FMTS) 
		return -EINVAL;

	if (ret_fmt != NULL)
		*ret_fmt = sensor_formats + index;
    
	/*
	* Fields: the sensor devices claim to be progressive.
	*/

	fmt->field = V4L2_FIELD_NONE;

	/*
	* Round requested image size down to the nearest
	* we support, but not below the smallest.
	*/
	for (wsize = sensor_win_sizes; wsize < sensor_win_sizes + N_WIN_SIZES;
	   wsize++)
		if (fmt->width >= wsize->width && fmt->height >= wsize->height)
			break;
    
	if (wsize >= sensor_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	* Note the size we'll actually handle.
	*/
	fmt->width = wsize->width;
	fmt->height = wsize->height;
	info->current_wins = wsize;
	//pix->bytesperline = pix->width*sensor_formats[index].bpp;
	//pix->sizeimage = pix->height*pix->bytesperline;

	return 0;
}

static int sensor_try_fmt(struct v4l2_subdev *sd, 
             struct v4l2_mbus_framefmt *fmt)
{
	return sensor_try_fmt_internal(sd, fmt, NULL, NULL);
}

static int sensor_g_mbus_config(struct v4l2_subdev *sd,
           struct v4l2_mbus_config *cfg)
{
  cfg->type = V4L2_MBUS_PARALLEL;
  cfg->flags = V4L2_MBUS_MASTER | VREF_POL | HREF_POL | CLK_POL ;
  
  return 0;
}


/*
 * Set a format.
 */
static int sensor_s_fmt(struct v4l2_subdev *sd, 
             struct v4l2_mbus_framefmt *fmt)
{
	int ret;
	struct sensor_format_struct *sensor_fmt;
	struct sensor_win_size *wsize;
	struct sensor_info *info = to_state(sd);

	vfe_dev_dbg("sensor_s_fmt\n");

	//  sensor_write_array(sd, sensor_oe_disable_regs, ARRAY_SIZE(sensor_oe_disable_regs));
  
	ret = sensor_try_fmt_internal(sd, fmt, &sensor_fmt, &wsize);
	if (ret)
	return ret;
  
	if(info->capture_mode == V4L2_MODE_VIDEO)
	{
	//video
	}
	else if(info->capture_mode == V4L2_MODE_IMAGE)
	{
	//image 
	}
  
	LOG_ERR_RET(sensor_write_array(sd, sensor_fmt->regs, sensor_fmt->regs_size))
  
	ret = 0;
	if (wsize->regs)
	{
		// usleep_range(5000,6000);
		LOG_ERR_RET(sensor_write_array(sd, wsize->regs, wsize->regs_size))
	}

	if (wsize->set_size)
		LOG_ERR_RET(wsize->set_size(sd))

	info->fmt = sensor_fmt;
	info->width = wsize->width;
	info->height = wsize->height;
	ps1210_sensor_vts = wsize->vts;  
   
	vfe_dev_print("s_fmt set width = %d, height = %d\n",wsize->width,wsize->height);

	if(info->capture_mode == V4L2_MODE_VIDEO)
	{
		//video
	} else {
		//capture image
	}
	//sensor_write_array(sd, sensor_oe_enable_regs, ARRAY_SIZE(sensor_oe_enable_regs));
	
	return 0;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int sensor_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct sensor_info *info = to_state(sd);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
  
	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->capturemode = info->capture_mode;
     
	return 0;
}

static int sensor_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	//struct v4l2_fract *tpf = &cp->timeperframe;
	struct sensor_info *info = to_state(sd);
	//unsigned char div;
  
	vfe_dev_dbg("sensor_s_parm\n");
  
	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
  
	if (info->tpf.numerator == 0)
		return -EINVAL;
    
	info->capture_mode = cp->capturemode;
  
	return 0;
}


static int sensor_queryctrl(struct v4l2_subdev *sd,
    struct v4l2_queryctrl *qc)
{
	/* Fill in min, max, step and default value for these controls. */
	/* see include/linux/videodev2.h for details */
  
	switch (qc->id) {
	case V4L2_CID_GAIN:
		return v4l2_ctrl_query_fill(qc, 1*16, 16*16, 1, 16);
	case V4L2_CID_EXPOSURE:
		return v4l2_ctrl_query_fill(qc, 1, 65536*16, 1, 1);
	case V4L2_CID_FRAME_RATE:
		return v4l2_ctrl_query_fill(qc, 15, 120, 1, 30);
	}
	return -EINVAL;
}

static int sensor_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return sensor_g_gain(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE:
		return sensor_g_exp(sd, &ctrl->value);
	}
	return -EINVAL;
}

static int sensor_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct v4l2_queryctrl qc;
	int ret;
  
	qc.id = ctrl->id;
	ret = sensor_queryctrl(sd, &qc);
	if (ret < 0) {
		return ret;
	}

	// if (ctrl->value < qc.minimum || ctrl->value > qc.maximum) {
	//   return -ERANGE;
	// }
  
	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		  return sensor_s_gain(sd, ctrl->value);
		  
	case V4L2_CID_EXPOSURE:
		return sensor_s_exp(sd, ctrl->value);
	  
	//case V4L2_CID_FRAME_RATE:
	//  return sensor_s_framerate(sd, ctrl->value);
	}
	return -EINVAL;
}


static int sensor_g_chip_ident(struct v4l2_subdev *sd,
    struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_SENSOR, 0);
}


/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.g_chip_ident = sensor_g_chip_ident,
	.g_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
	.queryctrl = sensor_queryctrl,
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.enum_mbus_fmt = sensor_enum_fmt,
	.enum_framesizes = sensor_enum_size,
	.try_mbus_fmt = sensor_try_fmt,
	.s_mbus_fmt = sensor_s_fmt,
	.s_parm = sensor_s_parm,
	.g_parm = sensor_g_parm,
	.g_mbus_config = sensor_g_mbus_config,
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
};

/* ----------------------------------------------------------------------- */
static struct cci_driver cci_drv = {
	.name = SENSOR_NAME,
};

static int sensor_probe(struct i2c_client *client,
      const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;
	//  int ret;

	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
	glb_sd = sd;
	cci_dev_probe_helper(sd, client, &sensor_ops, &cci_drv);

	info->fmt = &sensor_formats[0];
	info->af_first_flag = 1;
	info->init_first_flag = 1;

	return 0;
}

static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	sd = cci_dev_remove_helper(client, &cci_drv);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
  { SENSOR_NAME, 0 },
  { }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);


static struct i2c_driver sensor_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};
static __init int init_sensor(void)
{
	return cci_dev_init_helper(&sensor_driver);
}

static __exit void exit_sensor(void)
{
	cci_dev_exit_helper(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);

