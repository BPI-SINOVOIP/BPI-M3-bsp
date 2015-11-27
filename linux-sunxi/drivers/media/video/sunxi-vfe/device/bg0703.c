/*
 * A V4L2 driver for BG0703 Raw cameras.
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
MODULE_DESCRIPTION("A low-level driver for BG0703 Raw sensors");
MODULE_LICENSE("GPL");

//for internel driver debug
#define DEV_DBG_EN      1 
#if(DEV_DBG_EN == 1)    
#define vfe_dev_dbg(x,arg...) printk("[BG0703 Raw]"x,##arg)
#else
#define vfe_dev_dbg(x,arg...) 
#endif
#define vfe_dev_err(x,arg...) printk("[BG0703 Raw]"x,##arg)
#define vfe_dev_print(x,arg...) printk("[BG0703 Raw]"x,##arg)

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
#define V4L2_IDENT_SENSOR  0x0703



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
#define REG_DLY  0xffff

//define the registers
#define EXP_HIGH		0xff
#define EXP_MID			0x0c
#define EXP_LOW			0x0d
#define GAIN_HIGH		0xe5
#define GAIN_LOW		0xe6
//#define FRACTION_EXP
#define ID_REG_HIGH		0xf2
#define ID_REG_LOW		0xf3
#define ID_VAL_HIGH		((V4L2_IDENT_SENSOR) >> 8)
#define ID_VAL_LOW		((V4L2_IDENT_SENSOR) & 0xff)

#define ROWTIME      (0x408)                                                   
#define HSIZE        (1288)                                                    
#define VSIZE        (728)                                                     
                                                                               
#define FRAME_RATE   (30)                                                      
                                                                               
// Local define                                                                
#define BG_I2C_ADDR  (0x64)                                                    
#define BG070X_ID    (0x0703)                                                  
#define BG0701_ID    (0x0701)                                                  
#define BG0703_ID    (0x0707)                                                  
                                                                               
#define TROW         (ROWTIME*1000000/MCLK)                                    
#define PORCH        (8)                                                       
                                                                               
#define VBLANK_Std   ((1000000/(FRAME_RATE*TROW)-(PORCH+VSIZE)))               
#define VBLANK_5fps  ((1000000/(5*TROW)-(PORCH+VSIZE)))                        
                                                                               
#define MAXINT_Std   ((VSIZE+VBLANK_Std))                                      
#define MAXINT_5fps  ((VSIZE+VBLANK_5fps))                                     


/*
 * Our nominal (default) frame rate.
 */

#define SENSOR_FRAME_RATE 30


/*
 * The bg0703 i2c address
 */
//#define I2C_ADDR 0x6c
#define BG0703_WRITE_ADDR (0x64)
#define BG0703_READ_ADDR  (0x65)

//static struct delayed_work sensor_s_ae_ratio_work;
static struct v4l2_subdev *glb_sd;
#define SENSOR_NAME "bg0703"

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

static struct regval_list sensor_720p_regs[] = { 
	#if 0
	{0xf0, 0x00}, //select page0
	{0xb9, 0x22}, //MGC
	{0x0e, 0x04},
	{0x0f, 0x08}, //row time
	{0x14, 0x03}, //TXB ON *
	{0x1E, 0x0b}, //VTH 3.8V please check the voltage
	{0x20, 0x02}, //row reverse
	{0x21, 0x00}, //Vblank
	{0x22, 0x27}, //Vblank
	{0x28, 0x00}, //RAMP1 ONLY
	{0x29, 0x18}, //RSTB =1us
	{0x2a, 0x18}, //TXB = 1us
	{0x2d, 0x01}, //
	{0x2e, 0xB0}, //ibias_cnten_gap=17u
	{0x30, 0x18}, //rstb_cmprst_gap=1u
	{0x34, 0x20}, //tx_ramp2=32 CLKIN cycle*
	{0x36, 0x01}, //read start position
	{0x38, 0x03},
	{0x3a, 0x03},
	{0x53, 0x76},
	{0x54, 0x03},
	{0x52, 0xdd},
	{0x60, 0x00}, //row refresh mode
	{0x6d, 0x01}, //pll=288M pclk=72M  (when clkin=24M)
	{0x64, 0x02}, 
	{0x65, 0x00}, //RAMP1 length=200
	{0x67, 0x05},
	{0x68, 0xff}, //RAMP1 length=5ff
	{0x1d, 0x01}, //restart
	{0xf0, 0x01}, //select page1
	
	{0x00, 0x00}, //test pattern
	
	{0xc8, 0x04},
	{0xc7, 0xaa}, 
	{0xe0, 0x01},
	{0xe1, 0x04},
	{0xe2, 0x03},
	{0xe3, 0x02},
	{0xe4, 0x01},
	{0xe5, 0x01}, //vcm_comp =2.56V
	{0xb4, 0x01}, //row noise remove on*
	{0x20, 0x00}, //blcc off
	{0x31, 0x00},
	{0x32, 0x18},
	{0x33, 0x00},
	{0x34, 0x14},
	{0x35, 0x00},
	{0x36, 0x14},
	{0x37, 0x00},
	{0x38, 0x10},
	{0x20, 0x02}, //blcc on
	{0x4e, 0x00}, 
	{0x4f, 0x00}, //digital offset
	{0xf1, 0x00}, //dpc off
	{0xf0, 0x00},
	{0x7f, 0x00}, //cmp current
	{0x81, 0x09}, //dot_en=1,vrst=vth,vtx=vth
	{0x82, 0x11}, //bandgap current & ramp current
	{0x83, 0x01}, //pixel current
	{0x84, 0x07}, //check rst voltage 
	{0x89, 0x21}, //internal vddpix off
	{0x88, 0x05}, //pclk phase
	{0x8a, 0x03}, //pclk drv
	{0x8c, 0x03}, //data drv
	{0xb0, 0x01},
	{0xb1, 0x7f},
	{0xb2, 0x01},
	{0xb3, 0x7f}, //analog gain=1X
	{0xb4, 0x11},
	{0xb5, 0x11},
	{0xb6, 0x11},
	{0xb7, 0x01},
	{0xb8, 0x00}, //digital gain=1X
	{0xbf, 0x0c},
	{0x8e, 0x00}, //OEN
	{0x8d, 0x00}, //OEN
	{0x1d, 0x02},
	#endif
	  {0xf0, 0x00},  //select page0
  {0x1c, 0x01},  //soft reset
  {0x89, 0x21},  //internal vddpix off
  {0xb9, 0x22},  //Manual Gain

  {0x06, 0xFF&(HSIZE>>8)}, 
  {0x07, 0xFF&(HSIZE)},
  {0x08, 0xFF&(VSIZE>>8)}, 
  {0x09, 0xFF&(VSIZE)},
  
  {0x0e, 0xFF&(ROWTIME>>8)}, 
  {0x0f, 0xFF&(ROWTIME)},  //row time = 0x408/Fmclk = 1032/24MHz = 43 us

  {0x14, 0x03},  //TXB ON *
  {0x1E, 0x0f},  //VTH 3.8V please check the voltage
  //{0x20, 0x02},  //mirror
  {0x20, 0x02},  //mirror

  {0x21, 0xFF&(VBLANK_Std>>8)},  
  {0x22, 0xFF&(VBLANK_Std)},

  {0x28, 0x00},  //RAMP1 ONLY
  {0x29, 0x18},  //RSTB =1us
  {0x2a, 0x18},  //TXB = 1us
  {0x2d, 0x01},  
  {0x2e, 0xB0},  //ibias_cnten_gap=17u
  //{0x2d, 0x00},  
  //{0x2e, 0x01},  //ibias_cnten_gap
  {0x30, 0x18},  //rstb_cmprst_gap=1u
  {0x34, 0x20},  //tx_ramp2=32 CLKIN cycle*

  {0x38, 0x03}, 
  {0x39, 0xfd}, 
  {0x3a, 0x03}, 
  {0x3b, 0xfa}, 

  {0x53, 0x76}, 
  //{0x53, 0x4e}, 
  {0x54, 0x03}, 
  {0x52, 0xdd}, 
  {0x60, 0x00},  //row refresh mode
  {0x6d, 0x01},  //pll=288M pclk=72M  (when clkin=24M)
  {0x64, 0x02},  
  {0x65, 0x00},  //RAMP1 length=200
  {0x67, 0x05}, 
  {0x68, 0xff},  //RAMP1 length=5ff
  {0x1d, 0x01},  //restart

  {0xf0, 0x01}, 
  {0xc8, 0x04}, 
  {0xc7, 0xaa},  // FD Gain = 1X 
  {0xe0, 0x01}, 
  {0xe1, 0x04}, 
  {0xe2, 0x03}, 
  {0xe3, 0x02}, 
  {0xe4, 0x01}, 
  {0xe5, 0x01},  //vcm_comp =2.56V
  {0xb4, 0x01},  //row noise remove on*
  {0x20, 0x00}, //blcc off
  {0x31, 0x00}, //blcc target upper high
  {0x32, 0x38}, 
  {0x33, 0x00}, //blcc target upper low
  {0x34, 0x35}, 
  {0x35, 0x00}, //blcc target lower high
  {0x36, 0x33}, 
  {0x37, 0x00}, //blcc target lower low
  {0x38, 0x30}, 
  {0x39, 0x04}, //frame count to ave
  
  {0x3E, 0x07},
  {0x3F, 0x7f}, // Upper Limit
  
  {0x40, 0xff},
  {0x41, 0x00}, // Lower Limit
  
  {0x20, 0x02},  //blcc on

  {0x4e, 0x00},  
  {0x4f, 0x00},  //digital offset

  {0xf1, 0x07},  //dpc on

  {0xf0, 0x00}, 
  {0x7f, 0x00},  //cmp current
  {0x81, 0x09},  //dot_en=1,vrst=vth,vtx=vth
  {0x82, 0x11},  //bandgap current & ramp current
  {0x83, 0x01},  //pixel current
  {0x84, 0x07},  //check rst voltage 
  {0x88, 0x05},  //pclk phase
  {0x8a, 0x01},  //pclk drv
  {0x8c, 0x01},  //data drv
  {0xb0, 0x01}, 
  {0xb1, 0x7f}, 
  {0xb2, 0x01}, 
  {0xb3, 0x7f},  //analog gain=1X
  {0xb4, 0x11}, 
  {0xb5, 0x11}, 
  {0xb6, 0x11}, 
  {0xb7, 0x01}, 
  {0xb8, 0x00},  //digital gain=1X
  {0xbf, 0x0c}, 
  {0x8e, 0x00},  //OEN
  {0x8d, 0x00},  //OEN
  {0x1d, 0x02}, 
};
#if 0 //for warning
static struct regval_list sensor_720p_60fps_regs[] = { 
	{0xf0, 0x00}, //select page0
	{0xb9, 0x22}, //MGC
	
	{0x0e, 0x02},
	{0x0f, 0x15}, //row time
	
	{0x14, 0x03}, //TXB ON *
	{0x1E, 0x0f}, //VTH 3.8V please check the voltage
	
	{0x20, 0x02}, //row reverse
	{0x21, 0x00}, //Vblank
	{0x22, 0x1f}, //Vblank
	
	{0x28, 0x00}, //RAMP1 ONLY
	{0x29, 0x08}, //RSTB =0.33us  *2
	{0x2a, 0x18}, //TXB = 1us *4
	{0x2c, 0x01}, //CNTRSTB *5
	{0x2d, 0x00}, //
	{0x2e, 0x01}, //ibias_cnten_gap=0u
	
	{0x30, 0x12}, //rstb_cmprst_gap - 3CLKIN cycle* 8
	{0x32, 0x01}, //VCM2RAMP GAP 
	{0x33, 0x01}, //RMP1 2 TX2 -2 *10
	{0x34, 0x1a}, //tx_ramp2-6 CLKIN cycle*16
	{0x36, 0x01}, //read start position
	{0x37, 0x05}, //LTCSTG2
	
	{0x38, 0x02},
	{0x39, 0x0d},
	{0x3a, 0x02},
	{0x3b, 0x0a},
	
	{0x5f, 0x02}, //IBIAS_SW_W 
	{0x63, 0x01}, //LTCSTG2RMP2
	
	
	{0x53, 0x69},
	{0x54, 0x06},
	{0x52, 0xdc},
	{0x60, 0x00}, //row refresh mode
	{0x6d, 0x01}, //pll=320M pclk=80M  (when clkin=24M)
	{0x64, 0x02}, 
	{0x65, 0x00}, //RAMP1 length=200
	{0x67, 0x05},
	{0x68, 0xff}, //RAMP1 length=5ff *CNTTIME-0.5u *28
	{0x87, 0xaf}, //votlgate of vbg-i
	
	{0x1d, 0x01}, //restart
	
	{0xf0, 0x01},
	{0xc8, 0x04},
	{0xc7, 0xaa}, 
	{0xe0, 0x01}, 
	{0xe1, 0x01},
	{0xe2, 0x01},
	{0xe3, 0x01},
	{0xe4, 0x01},
	{0xe5, 0x01}, //vcm_comp =2.56V
	
	{0xb4, 0x01}, //row noise remove on*
	{0x20, 0x00},//blcc off
	{0x31, 0x00},//blcc target upper high
	{0x32, 0x18},
	{0x33, 0x00},//blcc target upper low
	{0x34, 0x14},
	{0x35, 0x00},//blcc target lower high
	{0x36, 0x14},
	{0x37, 0x00},//blcc target lower low
	{0x38, 0x10},
	{0x20, 0x02}, //blcc on
	{0x4e, 0x00}, 
	{0x4f, 0x00}, //digital offset
	
	{0xf1, 0x00}, //dpc off
	
	{0xf0, 0x00},
	
	{0x7f, 0x00}, //cmp current
	{0x81, 0x09}, //dot_en=1,vrst=vth,vtx=vth
	{0x82, 0x11}, //bandgap current & ramp current
	{0x83, 0x01}, //pixel current
	{0x84, 0x07}, //check rst voltage 
	{0x89, 0x21}, //internal vddpix off
	{0x88, 0x05}, //pclk phase
	{0x8a, 0x03}, //pclk drv
	{0x8c, 0x03}, //data drv
	
	{0xb0, 0x01},
	{0xb1, 0x7f},
	{0xb2, 0x01},
	{0xb3, 0x7f}, //analog gain=1X
	{0xb4, 0x11},
	{0xb5, 0x11},
	{0xb6, 0x11},
	{0xb7, 0x01},
	{0xb8, 0x00}, //digital gain=1X
	{0xbf, 0x0c},
	
	{0x8e, 0x00}, //OEN
	{0x8d, 0x00}, //OEN
	{0x1d, 0x02},

};
#endif

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
    unsigned char *value) //!!!!be careful of the para type!!!
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

static int reg_val_show(struct v4l2_subdev *sd,unsigned char reg)
{
	unsigned char tmp;
	sensor_read(sd,reg,&tmp);
	printk("0x%x value is 0x%x\n",reg,tmp);
	return 0;
}

/*static int show_regs_array (struct v4l2_subdev *sd,struct regval_list *array)
{	int i =0;
	unsigned char tmp;
	
	for(i=0;i<(sizeof(sensor_1080p_regs)/3);i++) 
	{		if((array+i)->addr == REG_DLY) 
		 {
               msleep((array+i)->data);
   	     } 
		sensor_read(sd,(array+i)->addr,&tmp);
		printk("bg0703 0x%x value is 0x%x\n",(array+i)->addr,tmp);
		
	}
	return 0;
}
*/



static int sensor_write(struct v4l2_subdev *sd, unsigned char reg,
    unsigned char value)
{
	int ret=0;
	int cnt=0;
  ret = cci_write_a8_d8(sd,reg,value);
  while(ret!=0&&cnt<2)
  {
  	ret = cci_write_a8_d8(sd,reg,value);
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
		} else {
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

static int sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	unsigned char explow,expmid,exphigh;
	struct sensor_info *info = to_state(sd);
	
	if(exp_val>0x1fffff)
		exp_val=0x1fffff;
	
	sensor_write(sd, 0xf0, 0x00);

#ifdef FRACTION_EXP
		exphigh   = (unsigned char) ( (0x0f0000&exp_val)>>16);
		expmid	  = (unsigned char) ( (0x00ff00&exp_val)>>8);
		explow	  = (unsigned char) ( (0x0000ff&exp_val)   );
#else
	    exphigh = 0;
	    expmid  = (unsigned char) ( (0x0ff000&exp_val)>>12);
	    explow  = (unsigned char) ( (0x000ff0&exp_val)>>4);
#endif
	
#ifdef FRACTION_EXP
	sensor_write(sd, EXP_HIGH, exphigh);
#endif
	sensor_write(sd, EXP_MID, expmid);
	sensor_write(sd, EXP_LOW, explow);

	//printk("0703 sensor_set_exp = %d Done!\n", exp_val );

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

static int sensor_s_gain(struct v4l2_subdev *sd, int gain_val)
{		
	struct sensor_info *info = to_state(sd);
	unsigned char gainlow=0;
	unsigned char gainhigh=0;
	
	if(gain_val<1*16)
		gain_val=16;
	if(gain_val>64*16-1)
		gain_val=64*16-1;
	//vfe_dev_dbg("sensor_set_gain = %d\n", gain_val);

	gainhigh=(unsigned char)(gain_val>>4);
	gainlow =(unsigned char)((gain_val<<4)&0xff);
	
	sensor_write(sd, GAIN_LOW, gainlow);
	sensor_write(sd, GAIN_HIGH, gainhigh);

	info->gain = gain_val;
	
	return 0;
	
}

static int bg0703_sensor_vts;
static int sensor_s_exp_gain(struct v4l2_subdev *sd, struct sensor_exp_gain *exp_gain)
{
  int exp_val, gain_val,shutter,frame_length;  
//  unsigned char explow=0,expmid=0,exphigh=0,vts_diff_low,vts_diff_high;
//  unsigned char gainlow=0,gainhigh=0;  
  struct sensor_info *info = to_state(sd);

  exp_val = exp_gain->exp_val;
  gain_val = exp_gain->gain_val;

  //if((info->exp == exp_val)&&(info->gain == gain_val))
  //	return 0;
  if(gain_val<1*16)
	  gain_val=16;
  if(gain_val>64*16-1)
	  gain_val=64*16-1;
  
  if(exp_val>0xfffff)
	  exp_val=0xfffff;

  shutter = exp_val/16;
  if(shutter > bg0703_sensor_vts- 4)
		frame_length = shutter + 4;
  else
		frame_length = bg0703_sensor_vts;

//  sensor_write(sd, 0x380f, (frame_length & 0xff));
//  sensor_write(sd, 0x380e, (frame_length >> 8));
  sensor_s_exp(sd,exp_val);
  sensor_s_gain(sd,gain_val);
  sensor_write(sd,0x1d,0x02);
  
  printk("0703 sensor_set_gain gain= %d,exp= %d Done!\n", gain_val,exp_val);

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
			#if 1
			vfe_dev_dbg("CSI_SUBDEV_STBY_ON!\n");
			//software standby on
			ret = sensor_s_sw_stby(sd, CSI_GPIO_HIGH);
			if(ret < 0)
				vfe_dev_err("soft stby falied!\n");
			usleep_range(10000,12000);
			//when using i2c_lock_adpater function, the following codes must not access i2c bus before calling i2c_unlock_adapter
			cci_lock(sd);
			//standby on io
			vfe_gpio_write(sd,PWDN,CSI_GPIO_LOW);
			//remember to unlock i2c adapter, so the device can access the i2c bus again
			cci_unlock(sd);
			//inactive mclk after stadby in
			vfe_set_mclk(sd,OFF);
			#endif
			break;
		case CSI_SUBDEV_STBY_OFF:
#if 1
			vfe_dev_dbg("CSI_SUBDEV_STBY_OFF!\n");
			cci_lock(sd);
			vfe_set_mclk_freq(sd,MCLK);
			vfe_set_mclk(sd,ON);
			usleep_range(10000,12000);
			vfe_gpio_write(sd,PWDN,CSI_GPIO_HIGH);
			usleep_range(10000,12000);
			ret = sensor_s_sw_stby(sd, CSI_GPIO_LOW);
			if(ret < 0)
				vfe_dev_err("soft stby off falied!\n");
			cci_unlock(sd);
#endif
			break;
		case CSI_SUBDEV_PWR_ON:
			vfe_dev_dbg("CSI_SUBDEV_PWR_ON!\n");
			cci_lock(sd);
			//power on reset
			vfe_gpio_set_status(sd,PWDN,1);//set the gpio to output
			vfe_gpio_set_status(sd,RESET,1);//set the gpio to output
			vfe_gpio_set_status(sd,POWER_EN,1);//set the gpio to output 
			
			vfe_gpio_write(sd,RESET,CSI_GPIO_LOW);  
			vfe_gpio_write(sd,PWDN,CSI_GPIO_LOW);
			vfe_gpio_write(sd,POWER_EN,CSI_GPIO_LOW);
			usleep_range(1000,1200);
			//power supply
			vfe_set_pmu_channel(sd,IOVDD,ON);
			//add by chao 
			usleep_range(1000,1200);
			vfe_set_pmu_channel(sd,AVDD,ON);	
			vfe_set_pmu_channel(sd,DVDD,ON);
			vfe_set_pmu_channel(sd,AFVDD,ON);
			//add by chao
			usleep_range(7000,8000);
			vfe_gpio_write(sd,POWER_EN,CSI_GPIO_HIGH);
			vfe_gpio_write(sd,PWDN,CSI_GPIO_HIGH); 
			usleep_range(10000,12000); 
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
  unsigned char rdval;
  
  LOG_ERR_RET(sensor_read(sd, ID_REG_HIGH, &rdval))
//  if(rdval != ID_VAL_HIGH)
//	return -ENODEV;
 
  LOG_ERR_RET(sensor_read(sd, ID_REG_LOW, &rdval))
//  if(rdval != ID_VAL_LOW)
//	return -ENODEV;

  reg_val_show(sd,0x00);
  reg_val_show(sd,0x01);
  reg_val_show(sd,0x02);
  reg_val_show(sd,0x03);
  reg_val_show(sd,0x04);
  reg_val_show(sd,0xf1);
  reg_val_show(sd,0xf2);
  reg_val_show(sd,0xf3);
  reg_val_show(sd,0xf4);
  
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
	info->width = HD720_WIDTH;
	info->height = HD720_HEIGHT;
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
		.mbus_code	= V4L2_MBUS_FMT_SRGGB12_1X12,
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
	/* 720P */
	#if 1
	{
		.width	    = 1280,
		.height 	= 720,
		.hoffset	= 4,
		.voffset	= 4,
		.hts		= 3097,//1400,
		.vts		= 775,//1032,
		.pclk		= 72*1000*1000,
		.fps_fixed	= 1,
		.bin_factor = 1,
		.intg_min	= 1,
		.intg_max	= 767<<4,
		.gain_min	= 1<<4,
		.gain_max	= (32<<4)-1,
		.regs		= sensor_720p_regs,//
		.regs_size	= ARRAY_SIZE(sensor_720p_regs),//
		.set_size		= NULL,
	},
#else
	/* 720P 60fps*/
	{
		.width		= 1280,
		.height 	= 720,
		.hoffset	= 4,
		.voffset	= 4,
		.hts		= 1720,
		.vts		= 775,//1032,
		.pclk		= 80*1000*1000,
		.fps_fixed	= 1,
		.bin_factor = 1,
		.intg_min	= 1,
		.intg_max	= (775-1)<<4,
		.gain_min	= 1<<4,
		.gain_max	= 16<<4,
		.regs		= sensor_720p_60fps_regs,//
		.regs_size	= ARRAY_SIZE(sensor_720p_60fps_regs),//
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
	bg0703_sensor_vts = wsize->vts;  
   
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
	.addr_width = CCI_BITS_8,
	.data_width = CCI_BITS_8,
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

