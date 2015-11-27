              /*
 * A V4L2 driver for IMX224 Raw cameras.
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
MODULE_DESCRIPTION("A low-level driver for IMX224 Raw sensors");
MODULE_LICENSE("GPL");

//for internel driver debug
#define DEV_DBG_EN      1 
#if(DEV_DBG_EN == 1)    
#define vfe_dev_dbg(x,arg...) printk("[IMX224 Raw]"x,##arg)
#else
#define vfe_dev_dbg(x,arg...) 
#endif
#define vfe_dev_err(x,arg...) printk("[IMX224 Raw]"x,##arg)
#define vfe_dev_print(x,arg...) printk("[IMX224 Raw]"x,##arg)

#define LOG_ERR_RET(x)  { \
                          int ret;  \
                          ret = x; \
                          if(ret < 0) {\
                            vfe_dev_err("error at %s\n",__func__);  \
                            return ret; \
                          } \
                        }

//define module timing
#define MCLK              (27*1000*1000)
#define VREF_POL          V4L2_MBUS_VSYNC_ACTIVE_HIGH
#define HREF_POL          V4L2_MBUS_HSYNC_ACTIVE_HIGH
#define CLK_POL           V4L2_MBUS_PCLK_SAMPLE_RISING
#define V4L2_IDENT_SENSOR  0x0224



//define the voltage level of control signal
#define CSI_STBY_ON     1
#define CSI_STBY_OFF    0
#define CSI_RST_ON      0
#define CSI_RST_OFF     1
#define CSI_PWR_ON      1
#define CSI_PWR_OFF     0
#define CSI_AF_PWR_ON   1
#define CSI_AF_PWR_OFF  0
#define regval_list reg_list_a16_d8

#define REG_TERM 0xfffe
#define VAL_TERM 0xfe
#define REG_DLY  0xffff

//#define DOL_ENABLE
//#define DOL_3FRAME

/*
 * Our nominal (default) frame rate.
 */

#define SENSOR_FRAME_RATE 30


/*
 * The IMX224 i2c address
 */
#define I2C_ADDR 0x34

//static struct delayed_work sensor_s_ae_ratio_work;
static struct v4l2_subdev *glb_sd;
#define SENSOR_NAME "imx224"

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
};

static struct regval_list sensor_960p60_12bit_regs[] = { //1080: 1920*1080@30fps
	{0x3000, 0x01},

	{0x3002, 0x00},
	{0x300F, 0x00},
	{0x3012, 0x2C},
	{0x3013, 0x01},
	{0x3016, 0x09},
	{0x301B, 0xCA},
	{0x301C, 0x08},
	{0x301D, 0xC2},
	{0x3022, 0xff},
	{0x3049, 0x0A},
	{0x305D, 0x00},
	{0x305F, 0x00},
	{0x3070, 0x02},
	{0x3071, 0x01},
	{0x309E, 0x22},
	{0x30A5, 0xFB},
	{0x30A6, 0x02},
	{0x30B3, 0xFF},
	{0x30B4, 0x01},
	{0x30B5, 0x42},
	{0x30B8, 0x10},
	{0x30C2, 0x01},

	{0x310F, 0x0F},
	{0x3110, 0x0E},
	{0x3111, 0xE7},
	{0x3112, 0x9C},
	{0x3113, 0x83},
	{0x3114, 0x10},
	{0x3115, 0x42},
	{0x3128, 0x1E},
	{0x31ED, 0x38},

	{0x320C, 0xCF},
	{0x324C, 0x40},
	{0x324D, 0x03},
	{0x3261, 0xE0},
	{0x3262, 0x02},
	{0x326E, 0x2F},
	{0x326F, 0x30},
	{0x3270, 0x03},
	{0x3298, 0x00},
	{0x329A, 0x12},
	{0x329B, 0xF1},
	{0x329C, 0x0C},

	{0x3344, 0x10},
	{0x336B, 0x37},
	{0x3381, 0x1B},
	{0x3382, 0x5F},
	{0x3383, 0x17},
	{0x3384, 0x37},
	{0x3385, 0x17},
	{0x3386, 0x17},
	{0x3387, 0x17},
	{0x3388, 0x4F},
	{0x3389, 0x27},
	{0x338D, 0x3D},
	{0x338E, 0x01},

	//DOL
	{0x300C, 0x11},

	{0x3000, 0x00},

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



static int sensor_read(struct v4l2_subdev *sd, unsigned short reg,
    unsigned char *value) //!!!!be careful of the para type!!!
{
	int ret=0;
	int cnt=0;
	
  ret = cci_read_a16_d8(sd,reg,value);
  while(ret!=0&&cnt<2)
  {
  	ret = cci_read_a16_d8(sd,reg,value);
  	cnt++;
  }
  if(cnt>0)
  	vfe_dev_dbg("sensor read retry=%d\n",cnt);
  
  return ret;
}
#if 0
static int reg_val_show(struct v4l2_subdev *sd,unsigned short reg)
{
	unsigned char tmp;
	sensor_read(sd,reg,&tmp);
	printk("0x%x value is 0x%x\n",reg,tmp);
	return 0;
}
#endif
/*static int show_regs_array (struct v4l2_subdev *sd,struct regval_list *array)
{	int i =0;
	unsigned char tmp;
	
	for(i=0;i<(sizeof(sensor_1080p_regs)/3);i++) 
	{		if((array+i)->addr == REG_DLY) 
		 {
               msleep((array+i)->data);
   	     } 
		sensor_read(sd,(array+i)->addr,&tmp);
		printk("IMX224 0x%x value is 0x%x\n",(array+i)->addr,tmp);
		
	}
	return 0;
}
*/



static int sensor_write(struct v4l2_subdev *sd, unsigned short reg,
    unsigned char value)
{
	int ret=0;
	int cnt=0;
  ret = cci_write_a16_d8(sd,reg,value);
  while(ret!=0&&cnt<2)
  {
  	ret = cci_write_a16_d8(sd,reg,value);
  	cnt++;
  }
  if(cnt>0)
  	vfe_dev_dbg("sensor write retry=%d\n",cnt);

  return ret;
}

/*
 * Write a list of register settings;
 */
static int sensor_write_array(struct v4l2_subdev *sd, struct regval_list *regs, int array_size)
{
	int i=0;
	
  if(!regs)
  	return -EINVAL;
  
  while(i<array_size)
  {
    if(regs->addr == REG_DLY) {
      msleep(regs->data);
    } 
    else {
      LOG_ERR_RET(sensor_write(sd, regs->addr, regs->data))
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

static int imx224_sensor_vts;
static int sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	unsigned char explow,expmid,exphigh;
	unsigned int exptime;
#ifdef DOL_ENABLE 
	unsigned int exp_val_m;
#endif	
#ifdef DOL_3FRAME 
	unsigned int exp_val_s;
#endif	
	struct sensor_info *info = to_state(sd);
	
	if(exp_val>0x1fffff)
		exp_val=0x1fffff;

	exptime = imx224_sensor_vts - (exp_val>>4) - 1;
	
	exphigh  = (unsigned char) ((0x0010000&exptime)>>16);
	expmid  = (unsigned char) ((0x000ff00&exptime)>>8 );
	explow  = (unsigned char) ((0x00000ff&exptime)    );
	
	sensor_write(sd, 0x3020, explow);
	sensor_write(sd, 0x3021, expmid);
	sensor_write(sd, 0x3022, exphigh);

#ifdef DOL_ENABLE 
	exp_val_m = exp_val>>2;
	exptime = imx224_sensor_vts - (exp_val_m>>4) - 1;

	exphigh	= (unsigned char) ((0x0010000&exptime)>>16);
	expmid	= (unsigned char) ((0x000ff00&exptime)>>8 );
	explow	= (unsigned char) ((0x00000ff&exptime)	  );

	sensor_write(sd, 0x3023, explow);
	sensor_write(sd, 0x3024, expmid);
	sensor_write(sd, 0x3025, exphigh);
#endif

#ifdef DOL_3FRAME 
	exp_val_s = exp_val>>4;
	exptime = imx224_sensor_vts - (exp_val_s>>4) - 1;

	exphigh	= (unsigned char) ((0x0010000&exptime)>>16);
	expmid	= (unsigned char) ((0x000ff00&exptime)>>8 );
	explow	= (unsigned char) ((0x00000ff&exptime)	  );

	sensor_write(sd, 0x3026, explow);
	sensor_write(sd, 0x3027, expmid);
	sensor_write(sd, 0x3028, exphigh);
#endif

//	printk("imx224 sensor_set_exp = %d      %d line Done!\n", exp_val , exptime);

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

unsigned char gain2db[497] = {0,2,3,5,6,8,9,11,12,13,14,15,16,17,18,19,20,21,22,23,23,
	24,25,26,27,27,28,29,29,30,31,31,32,32,33,34,34,35,35,36,36,37,37,38,38,39,39,40,
	40,41,41,41,42,42,43,43,44,44,44,45,45,45,46,46,47,47,47,48,48,48,49,49,49,50,50,
	50,51,51,51,52,52,52,52,53,53,53,54,54,54,54,55,55,55,56,56,56,56,57,57,57,57,58,
	58,58,58,59,59,59,59,60,60,60,60,60,61,61,61,61,62,62,62,62,62,63,63,63,63,63,64,
	64,64,64,64,65,65,65,65,65,66,66,66,66,66,66,67,67,67,67,67,68,68,68,68,68,68,69,
	69,69,69,69,69,70,70,70,70,70,70,71,71,71,71,71,71,71,72,72,72,72,72,72,73,73,73,
	73,73,73,73,74,74,74,74,74,74,74,75,75,75,75,75,75,75,75,76,76,76,76,76,76,76,77,
	77,77,77,77,77,77,77,78,78,78,78,78,78,78,78,79,79,79,79,79,79,79,79,79,80,80,80,
	80,80,80,80,80,80,81,81,81,81,81,81,81,81,81,82,82,82,82,82,82,82,82,82,83,83,83,
	83,83,83,83,83,83,83,84,84,84,84,84,84,84,84,84,84,85,85,85,85,85,85,85,85,85,85,
	86,86,86,86,86,86,86,86,86,86,86,87,87,87,87,87,87,87,87,87,87,87,88,88,88,88,88,
	88,88,88,88,88,88,88,89,89,89,89,89,89,89,89,89,89,89,89,90,90,90,90,90,90,90,90,
	90,90,90,90,91,91,91,91,91,91,91,91,91,91,91,91,91,92,92,92,92,92,92,92,92,92,92,
	92,92,92,93,93,93,93,93,93,93,93,93,93,93,93,93,93,94,94,94,94,94,94,94,94,94,94,
	94,94,94,94,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,96,96,96,96,96,96,96,96,
	96,96,96,96,96,96,96,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,98,98,98,98,
	98,98,98,98,98,98,98,98,98,98,98,98,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
	99,99,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
static int sensor_s_gain(struct v4l2_subdev *sd, int gain_val)
{
	struct sensor_info *info = to_state(sd);
	unsigned short gain = 0;
	
	if(gain_val<1*16)
		gain_val=16;

	if (gain_val < 512)
		gain = (gain2db[gain_val-16])*3;
	else
		gain = 300;
	
	sensor_write(sd, 0x3014, gain & 0xff);//enter group write
	sensor_write(sd, 0x3015, gain >> 8  );//enter group write
	
	//printk("imx224 sensor_set_gain = %d, Done!\n", gain_val);
	info->gain = gain_val;
	
	return 0;
}


static int sensor_s_exp_gain(struct v4l2_subdev *sd, struct sensor_exp_gain *exp_gain)
{
  int exp_val, gain_val, shutter, frame_length;  
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
  if(shutter  > imx224_sensor_vts- 4)
		frame_length = shutter + 4;
  else
		frame_length = imx224_sensor_vts;

  sensor_write(sd, 0x3018, (frame_length & 0xff));
  sensor_write(sd, 0x3019, (frame_length >> 8  ));
  sensor_write(sd, 0x301a, (frame_length >> 16 ));
  sensor_s_exp(sd,exp_val);
  sensor_s_gain(sd,gain_val);
  
  printk("imx224 sensor_set_gain exp= %d, %d Done!\n", gain_val,exp_val);

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
			//vfe_gpio_write(sd,PWDN,CSI_GPIO_HIGH);
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
			//vfe_gpio_write(sd,PWDN,CSI_GPIO_LOW);
			usleep_range(10000,12000);
			ret = sensor_s_sw_stby(sd, CSI_GPIO_LOW);
			if(ret < 0)
				vfe_dev_err("soft stby off falied!\n");
			cci_unlock(sd);
			break;
		case CSI_SUBDEV_PWR_ON:
			vfe_dev_dbg("CSI_SUBDEV_PWR_ON!\n");
			cci_lock(sd);
			//power on reset
			vfe_gpio_set_status(sd,PWDN,1);//set the gpio to output
			vfe_gpio_set_status(sd,RESET,1);//set the gpio to output
			vfe_gpio_set_status(sd,POWER_EN,1);//set the gpio to output 
			
			vfe_gpio_write(sd,RESET,CSI_GPIO_HIGH);  
			vfe_gpio_write(sd,PWDN,CSI_GPIO_HIGH);
			vfe_gpio_write(sd,POWER_EN,CSI_GPIO_LOW);
			usleep_range(1000,1200);
			//power supply
			vfe_set_pmu_channel(sd,IOVDD,ON);
			//add by chao 
			usleep_range(1000,1200);
			vfe_set_pmu_channel(sd,AVDD,ON);	
			vfe_gpio_write(sd,POWER_EN,CSI_GPIO_HIGH);
			vfe_set_pmu_channel(sd,DVDD,ON);
			vfe_set_pmu_channel(sd,AFVDD,ON);
			//add by chao
			usleep_range(7000,8000);
			vfe_gpio_write(sd,PWDN,CSI_GPIO_HIGH); 
			usleep_range(10000,12000); 
			//reset on io
			vfe_gpio_write(sd,RESET,CSI_GPIO_LOW);
			usleep_range(20000,22000);
			vfe_gpio_write(sd,RESET,CSI_GPIO_HIGH);
			//active mclk before power on
			vfe_set_mclk_freq(sd,MCLK);
			vfe_set_mclk(sd,ON);
			usleep_range(10000,12000);
			cci_unlock(sd);
			break;
		case CSI_SUBDEV_PWR_OFF:
			vfe_dev_dbg("CSI_SUBDEV_PWR_OFF!\n");
			cci_lock(sd);
			vfe_gpio_set_status(sd,PWDN,1);//set the gpio to output
			vfe_gpio_set_status(sd,RESET,1);//set the gpio to output
			vfe_gpio_write(sd,RESET,CSI_GPIO_LOW);  
			vfe_gpio_write(sd,PWDN,CSI_GPIO_LOW);
			//inactive mclk before power off
			vfe_set_mclk(sd,OFF);
			//power supply off
			vfe_set_pmu_channel(sd,AFVDD,OFF);
			vfe_set_pmu_channel(sd,DVDD,OFF);
			vfe_gpio_write(sd,POWER_EN,CSI_GPIO_LOW);
			vfe_set_pmu_channel(sd,AVDD,OFF);
			vfe_set_pmu_channel(sd,IOVDD,OFF);
			//set the io to hi-z
			vfe_gpio_set_status(sd,RESET,0);//set the gpio to input
			vfe_gpio_set_status(sd,PWDN,0);//set the gpio to input
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
  
  LOG_ERR_RET(sensor_read(sd, 0x0008, &rdval))
//	  printk("imx224!!!!!!!read id is %d.\n",rdval);
  
  return 0;
  	
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
		.desc	   = "Raw RGB Bayer",
		.mbus_code = V4L2_MBUS_FMT_SGBRG12_12X1,
		.regs 	   = sensor_fmt_raw,
		.regs_size = ARRAY_SIZE(sensor_fmt_raw),
		.bpp	   = 1
	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

  

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */


static struct sensor_win_size sensor_win_sizes[] = {
    /* 1080P */

    {
      .width	  = 1280,
      .height 	  = 960,
      .hoffset	  = 0,
      .voffset	  = 0,
      .hts        = 2250,
      .vts        = 1100,
      .pclk       = 148500000,
	  .mipi_bps   = 297*1000*1000,
      .fps_fixed  = 1,
      .bin_factor = 1,
      .intg_min   = 1,
      .intg_max   = (1100-1)<<4,
      .gain_min   = 1<<4,
      .gain_max   = 32<<4,
      .regs       = sensor_960p60_12bit_regs,//
      .regs_size  = ARRAY_SIZE(sensor_960p60_12bit_regs),//
      .set_size		= NULL,
    },
#if 0
    /* 720p */
    {
		.width		= HD720_WIDTH,
		.height 	= HD720_HEIGHT,
		.hoffset	= 108,
		.voffset	= 18,
		.hts		= 2200,
		.vts		= 1125,
		.pclk		= 74250000,//80*1000*1000,
		.fps_fixed	= 1,
		.bin_factor = 1,
		.intg_min	= 1,
		.intg_max	= 1125<<4,
		.gain_min	= 1<<4,
		.gain_max	= 32<<4,
		.width_input	  = 1920,
		.height_input	  = 1080,	
		.regs			  = sensor_1080p_regs,//
		.regs_size	= ARRAY_SIZE(sensor_1080p_regs),//
		.set_size		= NULL,
    },

  /* VGA */
    {
		.width		= VGA_WIDTH,
		.height 	= VGA_HEIGHT,
		.hoffset	= 348,
		.voffset	= 18,
		.hts		= 2200,
		.vts		= 1125,
		.pclk		= 74250000,//80*1000*1000,
		.fps_fixed	= 1,
		.bin_factor = 1,
		.intg_min	= 1,
		.intg_max	= 1125<<4,
		.gain_min	= 1<<4,
		.gain_max	= 32<<4,
	  	.width_input	  = 1440,
	  	.height_input	  = 1080,	
		.regs			  = sensor_1080p_regs,//
		.regs_size	= ARRAY_SIZE(sensor_1080p_regs),//
		.set_size		= NULL,
    },
    #endif
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
  cfg->type = V4L2_MBUS_CSI2;
  cfg->flags = 0|V4L2_MBUS_CSI2_4_LANE|V4L2_MBUS_CSI2_CHANNEL_0;
  
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
    LOG_ERR_RET(sensor_write_array(sd, wsize->regs, wsize->regs_size))
  	}
  
  if (wsize->set_size)
    LOG_ERR_RET(wsize->set_size(sd))

  info->fmt = sensor_fmt;
  info->width = wsize->width;
  info->height = wsize->height;
  imx224_sensor_vts = wsize->vts;

  vfe_dev_print("s_fmt set width = %d, height = %d\n",wsize->width,wsize->height);

  if(info->capture_mode == V4L2_MODE_VIDEO)
  {
    //video
   
  } else {
    //capture image

  }
	
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

