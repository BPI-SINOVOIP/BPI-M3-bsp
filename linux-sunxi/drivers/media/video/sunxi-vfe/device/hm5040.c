/*
 * A V4L2 driver for HM5040 cameras.
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
MODULE_DESCRIPTION("A low-level driver for HM5040 sensors");
MODULE_LICENSE("GPL");

//for internel driver debug
#define DEV_DBG_EN      1 
#if(DEV_DBG_EN == 1)    
#define vfe_dev_dbg(x,arg...) printk("[HM5040]"x,##arg)
#else
#define vfe_dev_dbg(x,arg...) 
#endif
#define vfe_dev_err(x,arg...) printk("[HM5040]"x,##arg)
#define vfe_dev_print(x,arg...) printk("[HM5040]"x,##arg)

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
#define V4L2_IDENT_SENSOR 0x5040
int hm5040_sensor_vts;

//define the voltage level of control signal
#define CSI_STBY_ON     0
#define CSI_STBY_OFF    1
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

/*
 * Our nominal (default) frame rate.
 */

#define SENSOR_FRAME_RATE 30


/*
 * The HM5040 sits on i2c with ID 0x6c
 */
#define I2C_ADDR 0x36
#define SENSOR_NAME "hm5040"
//static struct delayed_work sensor_s_ae_ratio_work;
static struct v4l2_subdev *glb_sd;

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


static struct regval_list sensor_default_regs[] = {
	//-------------------------------------
	//	software reset
	//-------------------------------------
	//{0x0103, 0x01},//
	
	//-------------------------------------
	//	mode select
	//-------------------------------------
	{0x0100, 0x00},//
	// Register Default Updates
	{0x3002, 0x32},//
	{0x3016, 0x46},//
	{0x3017, 0x29},//
	{0x3003, 0x03},//
	{0x3045, 0x03},//
	
	// Dark Shading Coefficients
	
	// Full Frame
	{0xFBD7, 0x00},// // FF_SUB_G1 HI
	{0xFBD8, 0x00},// // FF_SUB_G1 LO
	{0xFBD9, 0x00},// // FF_DIV_G1 HI
	{0xFBDA, 0x00},// // FF_DIV_G1 LO
	{0xFBDB, 0x00},// // FF_SUB_G2 HI
	{0xFBDC, 0x00},// // FF_SUB_G2 LO
	{0xFBDD, 0x00},// // FF_DIV_G2 HI
	{0xFBDE, 0x00},// // FF_DIV_G2 LO
	{0xFBDF, 0x00},// // FF_SUB_G4 HI
	{0xFBE0, 0x00},// // FF_SUB_G4 LO
	{0xFBE1, 0x00},// // FF_DIV_G4 HI
	{0xFBE2, 0x00},// // FF_DIV_G4 LO
	{0xFBE3, 0x00},// // FF_SUB_G8 HI
	{0xFBE4, 0x00},// // FF_SUB_G8 LO
	{0xFBE5, 0x00},// // FF_DIV_G8 HI
	{0xFBE6, 0x00},// // FF_DIV_G8 LO
	{0xFBE7, 0x00},// // FF_SUB_G16 HI
	{0xFBE8, 0x00},// // FF_SUB_G16 LO
	{0xFBE9, 0x00},// // FF_DIV_G16 HI
	{0xFBEA, 0x00},// // FF_DIV_G16 LO
							  
	// Analogue Binning 2x2   
	{0xFBEB, 0x00},// // AB_SUB_G1 HI
	{0xFBEC, 0x00},// // AB_SUB_G1 LO
	{0xFBED, 0x00},// // AB_DIV_G1 HI
	{0xFBEE, 0x00},// // AB_DIV_G1 LO
	{0xFBEF, 0x00},// // AB_SUB_G2 HI
	{0xFBF0, 0x00},// // AB_SUB_G2 LO
	{0xFBF1, 0x00},// // AB_DIV_G2 HI
	{0xFBF2, 0x00},// // AB_DIV_G2 LO
	{0xFBF3, 0x00},// // AB_SUB_G4 HI
	{0xFBF4, 0x00},// // AB_SUB_G4 LO
	{0xFBF5, 0x00},// // AB_DIV_G4 HI
	{0xFBF6, 0x00},// // AB_DIV_G4 LO
	{0xFBF7, 0x00},// // AB_SUB_G8 HI
	{0xFBF8, 0x00},// // AB_SUB_G8 LO
	{0xFBF9, 0x00},// // AB_DIV_G8 HI
	{0xFBFA, 0x00},// // AB_DIV_G8 LO
	{0xFBFB, 0x00},// // AB_SUB_G16 HI
	{0xFBFC, 0x00},// // AB_SUB_G16 LO
	{0xFBFD, 0x00},// // AB_DIV_G16 HI
	{0xFBFE, 0x00},// // AB_DIV_G16 LO
	
	// Set fTweak Reg 
	{0xFB00, 0x51},//
	
	// Main Patch Code
	// Dark Visible Mismatch
	{0xF800, 0xc0},//
	{0xF801, 0x24},//
	{0xF802, 0x7c},//
	{0xF803, 0xfb},//
	{0xF804, 0x7d},//
	{0xF805, 0xc7},//
	{0xF806, 0x7b},//
	{0xF807, 0x10},//
	{0xF808, 0x7f},//
	{0xF809, 0x72},//
	{0xF80A, 0x7e},//
	{0xF80B, 0x30},//
	{0xF80C, 0x12},//
	{0xF80D, 0x09},//
	{0xF80E, 0x47},//
	{0xF80F, 0xd0},//
	{0xF810, 0x24},//
	{0xF811, 0x90},//
	{0xF812, 0x02},//
	{0xF813, 0x05},//
	{0xF814, 0xe0},//
	{0xF815, 0xf5},//
	{0xF816, 0x77},//
	{0xF817, 0xe5},//
	{0xF818, 0x77},//
	{0xF819, 0xc3},//
	{0xF81A, 0x94},//
	{0xF81B, 0x80},//
	{0xF81C, 0x50},//
	{0xF81D, 0x08},//
	{0xF81E, 0x75},//
	{0xF81F, 0x7a},//
	{0xF820, 0xfb},//
	{0xF821, 0x75},//
	{0xF822, 0x7b},//
	{0xF823, 0xd7},//
	{0xF824, 0x80},//
	{0xF825, 0x33},//
	{0xF826, 0xe5},//
	{0xF827, 0x77},//
	{0xF828, 0xc3},//
	{0xF829, 0x94},//
	{0xF82A, 0xc0},//
	{0xF82B, 0x50},//
	{0xF82C, 0x08},//
	{0xF82D, 0x75},//
	{0xF82E, 0x7a},//
	{0xF82F, 0xfb},//
	{0xF830, 0x75},//
	{0xF831, 0x7b},//
	{0xF832, 0xdb},//
	{0xF833, 0x80},//
	{0xF834, 0x24},//
	{0xF835, 0xe5},//
	{0xF836, 0x77},//
	{0xF837, 0xc3},//
	{0xF838, 0x94},//
	{0xF839, 0xe0},//
	{0xF83A, 0x50},//
	{0xF83B, 0x08},//
	{0xF83C, 0x75},//
	{0xF83D, 0x7a},//
	{0xF83E, 0xfb},//
	{0xF83F, 0x75},//
	{0xF840, 0x7b},//
	{0xF841, 0xdf},//
	{0xF842, 0x80},//
	{0xF843, 0x15},//
	{0xF844, 0xe5},//
	{0xF845, 0x77},//
	{0xF846, 0xc3},//
	{0xF847, 0x94},//
	{0xF848, 0xf0},//
	{0xF849, 0x50},//
	{0xF84A, 0x08},//
	{0xF84B, 0x75},//
	{0xF84C, 0x7a},//
	{0xF84D, 0xfb},//
	{0xF84E, 0x75},//
	{0xF84F, 0x7b},//
	{0xF850, 0xe3},//
	{0xF851, 0x80},//
	{0xF852, 0x06},//
	{0xF853, 0x75},//
	{0xF854, 0x7a},//
	{0xF855, 0xfb},//
	{0xF856, 0x75},//
	{0xF857, 0x7b},//
	{0xF858, 0xe7},//
	{0xF859, 0xe5},//
	{0xF85A, 0x55},//
	{0xF85B, 0x7f},//
	{0xF85C, 0x00},//
	{0xF85D, 0xb4},//
	{0xF85E, 0x22},//
	{0xF85F, 0x02},//
	{0xF860, 0x7f},//
	{0xF861, 0x01},//
	{0xF862, 0xe5},//
	{0xF863, 0x53},//
	{0xF864, 0x5f},//
	{0xF865, 0x60},//
	{0xF866, 0x05},//
	{0xF867, 0x74},//
	{0xF868, 0x14},//
	{0xF869, 0x12},//
	{0xF86A, 0xfa},//
	{0xF86B, 0x4c},//
	{0xF86C, 0x75},//
	{0xF86D, 0x7c},//
	{0xF86E, 0xfb},//
	{0xF86F, 0x75},//
	{0xF870, 0x7d},//
	{0xF871, 0xc7},//
	{0xF872, 0x75},//
	{0xF873, 0x7e},//
	{0xF874, 0x30},//
	{0xF875, 0x75},//
	{0xF876, 0x7f},//
	{0xF877, 0x62},//
	{0xF878, 0xe4},//
	{0xF879, 0xf5},//
	{0xF87A, 0x77},//
	{0xF87B, 0xe5},//
	{0xF87C, 0x77},//
	{0xF87D, 0xc3},//
	{0xF87E, 0x94},//
	{0xF87F, 0x08},//
	{0xF880, 0x40},//
	{0xF881, 0x03},//
	{0xF882, 0x02},//
	{0xF883, 0xf9},//
	{0xF884, 0x0e},//
	{0xF885, 0x85},//
	{0xF886, 0x7d},//
	{0xF887, 0x82},//
	{0xF888, 0x85},//
	{0xF889, 0x7c},//
	{0xF88A, 0x83},//
	{0xF88B, 0xe0},//
	{0xF88C, 0xfe},//
	{0xF88D, 0xa3},//
	{0xF88E, 0xe0},//
	{0xF88F, 0xff},//
	{0xF890, 0x12},//
	{0xF891, 0x21},//
	{0xF892, 0x22},//
	{0xF893, 0x8e},//
	{0xF894, 0x78},//
	{0xF895, 0x8f},//
	{0xF896, 0x79},//
	{0xF897, 0x12},//
	{0xF898, 0xfa},//
	{0xF899, 0x40},//
	{0xF89A, 0x12},//
	{0xF89B, 0x22},//
	{0xF89C, 0x93},//
	{0xF89D, 0x50},//
	{0xF89E, 0x07},//
	{0xF89F, 0xe4},//
	{0xF8A0, 0xf5},//
	{0xF8A1, 0x78},//
	{0xF8A2, 0xf5},//
	{0xF8A3, 0x79},//
	{0xF8A4, 0x80},//
	{0xF8A5, 0x33},//
	{0xF8A6, 0x12},//
	{0xF8A7, 0xfa},//
	{0xF8A8, 0x40},//
	{0xF8A9, 0x7b},//
	{0xF8AA, 0x01},//
	{0xF8AB, 0xaf},//
	{0xF8AC, 0x79},//
	{0xF8AD, 0xae},//
	{0xF8AE, 0x78},//
	{0xF8AF, 0x12},//
	{0xF8B0, 0x22},//
	{0xF8B1, 0x4f},//
	{0xF8B2, 0x74},//
	{0xF8B3, 0x02},//
	{0xF8B4, 0x12},//
	{0xF8B5, 0xfa},//
	{0xF8B6, 0x4c},//
	{0xF8B7, 0x85},//
	{0xF8B8, 0x7b},//
	{0xF8B9, 0x82},//
	{0xF8BA, 0xf5},//
	{0xF8BB, 0x83},//
	{0xF8BC, 0xe0},//
	{0xF8BD, 0xfe},//
	{0xF8BE, 0xa3},//
	{0xF8BF, 0xe0},//
	{0xF8C0, 0xff},//
	{0xF8C1, 0x7d},//
	{0xF8C2, 0x03},//
	{0xF8C3, 0x12},//
	{0xF8C4, 0x17},//
	{0xF8C5, 0xd8},//
	{0xF8C6, 0x12},//
	{0xF8C7, 0x1b},//
	{0xF8C8, 0x9b},//
	{0xF8C9, 0x8e},//
	{0xF8CA, 0x78},//
	{0xF8CB, 0x8f},//
	{0xF8CC, 0x79},//
	{0xF8CD, 0x74},//
	{0xF8CE, 0xfe},//
	{0xF8CF, 0x25},//
	{0xF8D0, 0x7b},//
	{0xF8D1, 0xf5},//
	{0xF8D2, 0x7b},//
	{0xF8D3, 0x74},//
	{0xF8D4, 0xff},//
	{0xF8D5, 0x35},//
	{0xF8D6, 0x7a},//
	{0xF8D7, 0xf5},//
	{0xF8D8, 0x7a},//
	{0xF8D9, 0x78},//
	{0xF8DA, 0x24},//
	{0xF8DB, 0xe6},//
	{0xF8DC, 0xff},//
	{0xF8DD, 0xc3},//
	{0xF8DE, 0x74},//
	{0xF8DF, 0x20},//
	{0xF8E0, 0x9f},//
	{0xF8E1, 0x7e},//
	{0xF8E2, 0x00},//
	{0xF8E3, 0x25},//
	{0xF8E4, 0x79},//
	{0xF8E5, 0xff},//
	{0xF8E6, 0xee},//
	{0xF8E7, 0x35},//
	{0xF8E8, 0x78},//
	{0xF8E9, 0x85},//
	{0xF8EA, 0x7f},//
	{0xF8EB, 0x82},//
	{0xF8EC, 0x85},//
	{0xF8ED, 0x7e},//
	{0xF8EE, 0x83},//
	{0xF8EF, 0xf0},//
	{0xF8F0, 0xa3},//
	{0xF8F1, 0xef},//
	{0xF8F2, 0xf0},//
	{0xF8F3, 0x05},//
	{0xF8F4, 0x77},//
	{0xF8F5, 0x74},//
	{0xF8F6, 0x02},//
	{0xF8F7, 0x25},//
	{0xF8F8, 0x7d},//
	{0xF8F9, 0xf5},//
	{0xF8FA, 0x7d},//
	{0xF8FB, 0xe4},//
	{0xF8FC, 0x35},//
	{0xF8FD, 0x7c},//
	{0xF8FE, 0xf5},//
	{0xF8FF, 0x7c},//
	{0xF900, 0x74},//
	{0xF901, 0x02},//
	{0xF902, 0x25},//
	{0xF903, 0x7f},//
	{0xF904, 0xf5},//
	{0xF905, 0x7f},//
	{0xF906, 0xe4},//
	{0xF907, 0x35},//
	{0xF908, 0x7e},//
	{0xF909, 0xf5},//
	{0xF90A, 0x7e},//
	{0xF90B, 0x02},//
	{0xF90C, 0xf8},//
	{0xF90D, 0x7b},//
	{0xF90E, 0x22},//
	{0xF90F, 0x90},//
	
	// Apply BayerAVG Config
	{0xF910, 0x30},//
	{0xF911, 0x47},//
	{0xF912, 0x74},//
	{0xF913, 0x98},//
	{0xF914, 0xf0},//
	{0xF915, 0x90},//
	{0xF916, 0x30},//
	{0xF917, 0x36},//
	{0xF918, 0x74},//
	{0xF919, 0x1e},//
	{0xF91A, 0xf0},//
	{0xF91B, 0x90},//
	{0xF91C, 0x30},//
	{0xF91D, 0x42},//
	{0xF91E, 0x74},//
	{0xF91F, 0x24},//
	{0xF920, 0xf0},//
	{0xF921, 0xe5},//
	{0xF922, 0x53},//
	{0xF923, 0x60},//
	{0xF924, 0x42},//
	{0xF925, 0x78},//
	{0xF926, 0x2b},//
	{0xF927, 0x76},//
	{0xF928, 0x01},//
	{0xF929, 0xe5},//
	{0xF92A, 0x55},//
	{0xF92B, 0xb4},//
	{0xF92C, 0x22},//
	{0xF92D, 0x17},//
	{0xF92E, 0x90},//
	{0xF92F, 0x30},//
	{0xF930, 0x36},//
	{0xF931, 0x74},//
	{0xF932, 0x46},//
	{0xF933, 0xf0},//
	{0xF934, 0x78},//
	{0xF935, 0x28},//
	{0xF936, 0x76},//
	{0xF937, 0x31},//
	{0xF938, 0x90},//
	{0xF939, 0x30},//
	{0xF93A, 0x0e},//
	{0xF93B, 0xe0},//
	{0xF93C, 0xc3},//
	{0xF93D, 0x13},//
	{0xF93E, 0x30},//
	{0xF93F, 0xe0},//
	{0xF940, 0x04},//
	{0xF941, 0x78},//
	{0xF942, 0x26},//
	{0xF943, 0x76},//
	{0xF944, 0x40},//
	{0xF945, 0xe5},//
	{0xF946, 0x55},//
	{0xF947, 0xb4},//
	{0xF948, 0x44},//
	{0xF949, 0x21},//
	{0xF94A, 0x90},//
	{0xF94B, 0x30},//
	{0xF94C, 0x47},//
	{0xF94D, 0x74},//
	{0xF94E, 0x9a},//
	{0xF94F, 0xf0},//
	{0xF950, 0x90},//
	{0xF951, 0x30},//
	{0xF952, 0x42},//
	{0xF953, 0x74},//
	{0xF954, 0x64},//
	{0xF955, 0xf0},//
	{0xF956, 0x90},//
	{0xF957, 0x30},//
	{0xF958, 0x0e},//
	{0xF959, 0xe0},//
	{0xF95A, 0x13},//
	{0xF95B, 0x13},//
	{0xF95C, 0x54},//
	{0xF95D, 0x3f},//
	{0xF95E, 0x30},//
	{0xF95F, 0xe0},//
	{0xF960, 0x0a},//
	{0xF961, 0x78},//
	{0xF962, 0x24},//
	{0xF963, 0xe4},//
	{0xF964, 0xf6},//
	{0xF965, 0x80},//
	{0xF966, 0x04},//
	{0xF967, 0x78},//
	{0xF968, 0x2b},//
	{0xF969, 0xe4},//
	{0xF96A, 0xf6},//
	{0xF96B, 0x90},//
	{0xF96C, 0x30},//
	{0xF96D, 0x88},//
	{0xF96E, 0x02},//
	{0xF96F, 0x1d},//
	{0xF970, 0x4f},//
	{0xF971, 0x22},//
	
	// Flash Strobe Trigger 1
	{0xF972, 0x90},//
	{0xF973, 0x0c},//
	{0xF974, 0x1a},//
	{0xF975, 0xe0},//
	{0xF976, 0x30},//
	{0xF977, 0xe2},//
	{0xF978, 0x18},//
	{0xF979, 0x90},//
	{0xF97A, 0x33},//
	{0xF97B, 0x68},//
	{0xF97C, 0xe0},//
	{0xF97D, 0x64},//
	{0xF97E, 0x05},//
	{0xF97F, 0x70},//
	{0xF980, 0x2f},//
	{0xF981, 0x90},//
	{0xF982, 0x30},//
	{0xF983, 0x38},//
	{0xF984, 0xe0},//
	{0xF985, 0x70},//
	{0xF986, 0x02},//
	{0xF987, 0xa3},//
	{0xF988, 0xe0},//
	{0xF989, 0xc3},//
	{0xF98A, 0x70},//
	{0xF98B, 0x01},//
	{0xF98C, 0xd3},//
	{0xF98D, 0x40},//
	{0xF98E, 0x21},//
	{0xF98F, 0x80},//
	{0xF990, 0x1b},//
	{0xF991, 0x90},//
	{0xF992, 0x33},//
	{0xF993, 0x68},//
	{0xF994, 0xe0},//
	{0xF995, 0xb4},//
	{0xF996, 0x05},//
	{0xF997, 0x18},//
	{0xF998, 0xc3},//
	{0xF999, 0x90},//
	{0xF99A, 0x30},//
	{0xF99B, 0x3b},//
	{0xF99C, 0xe0},//
	{0xF99D, 0x94},//
	{0xF99E, 0x0d},//
	{0xF99F, 0x90},//
	{0xF9A0, 0x30},//
	{0xF9A1, 0x3a},//
	{0xF9A2, 0xe0},//
	{0xF9A3, 0x94},//
	{0xF9A4, 0x00},//
	{0xF9A5, 0x50},//
	{0xF9A6, 0x02},//
	{0xF9A7, 0x80},//
	{0xF9A8, 0x01},//
	{0xF9A9, 0xc3},//
	{0xF9AA, 0x40},//
	{0xF9AB, 0x04},//
	{0xF9AC, 0x75},//
	{0xF9AD, 0x10},//
	{0xF9AE, 0x01},//
	{0xF9AF, 0x22},//
	{0xF9B0, 0x02},//
	{0xF9B1, 0x16},//
	{0xF9B2, 0xe1},//
	{0xF9B3, 0x22},//
	{0xF9B4, 0x90},//
	
	// Copy NVM Dark Shade Data
	{0xF9B5, 0xff},//
	{0xF9B6, 0x33},//
	{0xF9B7, 0xe0},//
	{0xF9B8, 0x90},//
	{0xF9B9, 0xff},//
	{0xF9BA, 0x34},//
	{0xF9BB, 0xe0},//
	{0xF9BC, 0x60},//
	{0xF9BD, 0x0d},//
	{0xF9BE, 0x7c},//
	{0xF9BF, 0xfb},//
	{0xF9C0, 0x7d},//
	{0xF9C1, 0xd7},//
	{0xF9C2, 0x7b},//
	{0xF9C3, 0x28},//
	{0xF9C4, 0x7f},//
	{0xF9C5, 0x34},//
	{0xF9C6, 0x7e},//
	{0xF9C7, 0xff},//
	{0xF9C8, 0x12},//
	{0xF9C9, 0x09},//
	{0xF9CA, 0x47},//
	{0xF9CB, 0x7f},//
	{0xF9CC, 0x20},//
	{0xF9CD, 0x7e},//
	{0xF9CE, 0x01},//
	{0xF9CF, 0x7d},//
	{0xF9D0, 0x00},//
	{0xF9D1, 0x7c},//
	{0xF9D2, 0x00},//
	{0xF9D3, 0x12},//
	{0xF9D4, 0x12},//
	{0xF9D5, 0xa4},//
	{0xF9D6, 0xe4},//
	{0xF9D7, 0x90},//
	{0xF9D8, 0x3e},//
	{0xF9D9, 0x44},//
	{0xF9DA, 0xf0},//
	{0xF9DB, 0x02},//
	{0xF9DC, 0x16},//
	{0xF9DD, 0x7e},//
	{0xF9DE, 0x22},//
	{0xF9DF, 0xe5},//
	
	// Call SLC Copy
	{0xF9E0, 0x44},//
	{0xF9E1, 0x60},//
	{0xF9E2, 0x10},//
	{0xF9E3, 0x90},//
	{0xF9E4, 0xf6},//
	{0xF9E5, 0x2c},//
	{0xF9E6, 0x74},//
	{0xF9E7, 0x04},//
	{0xF9E8, 0xf0},//
	{0xF9E9, 0x90},//
	{0xF9EA, 0xf6},//
	{0xF9EB, 0x34},//
	{0xF9EC, 0xf0},//
	{0xF9ED, 0x90},//
	{0xF9EE, 0xf6},//
	{0xF9EF, 0x3c},//
	{0xF9F0, 0xf0},//
	{0xF9F1, 0x80},//
	{0xF9F2, 0x0e},//
	{0xF9F3, 0x90},//
	{0xF9F4, 0xf5},//
	{0xF9F5, 0xc0},//
	{0xF9F6, 0x74},//
	{0xF9F7, 0x04},//
	{0xF9F8, 0xf0},//
	{0xF9F9, 0x90},//
	{0xF9FA, 0xf5},//
	{0xF9FB, 0xc8},//
	{0xF9FC, 0xf0},//
	{0xF9FD, 0x90},//
	{0xF9FE, 0xf5},//
	{0xF9FF, 0xd0},//
	{0xFA00, 0xf0},//
	{0xFA01, 0x90},//
	{0xFA02, 0xfb},//
	{0xFA03, 0x7f},//
	{0xFA04, 0x02},//
	{0xFA05, 0x19},//
	{0xFA06, 0x0b},//
	{0xFA07, 0x22},//
	{0xFA08, 0x90},//
	
	// Flash Strobe Start Streaming
	{0xFA09, 0x0c},//
	{0xFA0A, 0x1a},//
	{0xFA0B, 0xe0},//
	{0xFA0C, 0x20},//
	{0xFA0D, 0xe2},//
	{0xFA0E, 0x15},//
	{0xFA0F, 0xe4},//
	{0xFA10, 0x90},//
	{0xFA11, 0x30},//
	{0xFA12, 0xf8},//
	{0xFA13, 0xf0},//
	{0xFA14, 0xa3},//
	{0xFA15, 0xf0},//
	{0xFA16, 0x90},//
	{0xFA17, 0x30},//
	{0xFA18, 0xf1},//
	{0xFA19, 0xe0},//
	{0xFA1A, 0x44},//
	{0xFA1B, 0x08},//
	{0xFA1C, 0xf0},//
	{0xFA1D, 0x90},//
	{0xFA1E, 0x30},//
	{0xFA1F, 0xf0},//
	{0xFA20, 0xe0},//
	{0xFA21, 0x44},//
	{0xFA22, 0x08},//
	{0xFA23, 0xf0},//
	{0xFA24, 0x02},//
	{0xFA25, 0x03},//
	{0xFA26, 0xde},//
	{0xFA27, 0x22},//
	{0xFA28, 0x90},//
	
	// Flash Strobe Frame Start
	{0xFA29, 0x0c},//
	{0xFA2A, 0x1a},//
	{0xFA2B, 0xe0},//
	{0xFA2C, 0x30},//
	{0xFA2D, 0xe2},//
	{0xFA2E, 0x0d},//
	{0xFA2F, 0xe0},//
	{0xFA30, 0x20},//
	{0xFA31, 0xe0},//
	{0xFA32, 0x06},//
	{0xFA33, 0x90},//
	{0xFA34, 0xfb},//
	{0xFA35, 0x85},//
	{0xFA36, 0x74},//
	{0xFA37, 0x00},//
	{0xFA38, 0xa5},//
	{0xFA39, 0x12},//
	{0xFA3A, 0x16},//
	{0xFA3B, 0xa0},//
	{0xFA3C, 0x02},//
	{0xFA3D, 0x18},//
	{0xFA3E, 0xac},//
	{0xFA3F, 0x22},//
	{0xFA40, 0x85},//
	{0xFA41, 0x7b},//
	{0xFA42, 0x82},//
	{0xFA43, 0x85},//
	{0xFA44, 0x7a},//
	{0xFA45, 0x83},//
	{0xFA46, 0xe0},//
	{0xFA47, 0xfc},//
	{0xFA48, 0xa3},//
	{0xFA49, 0xe0},//
	{0xFA4A, 0xfd},//
	{0xFA4B, 0x22},//
	{0xFA4C, 0x25},//
	{0xFA4D, 0x7b},//
	{0xFA4E, 0xf5},//
	{0xFA4F, 0x7b},//
	{0xFA50, 0xe4},//
	{0xFA51, 0x35},//
	{0xFA52, 0x7a},//
	{0xFA53, 0xf5},//
	{0xFA54, 0x7a},//
	{0xFA55, 0x22},//
	{0xFA56, 0xc0},//
	
	// Flash Strobe ISR Timer 1
	{0xFA57, 0xd0},//
	{0xFA58, 0x90},//
	{0xFA59, 0x35},//
	{0xFA5A, 0xb5},//
	{0xFA5B, 0xe0},//
	{0xFA5C, 0x54},//
	{0xFA5D, 0xfc},//
	{0xFA5E, 0x44},//
	{0xFA5F, 0x01},//
	{0xFA60, 0xf0},//
	{0xFA61, 0x12},//
	{0xFA62, 0x1f},//
	{0xFA63, 0x5f},//
	{0xFA64, 0xd0},//
	{0xFA65, 0xd0},//
	{0xFA66, 0x02},//
	{0xFA67, 0x0a},//
	{0xFA68, 0x16},//
	{0xFA69, 0x22},//
	
	// Flash Strobe Line Count Int
	{0xFA6A,0x90},//
	{0xFA6B,0x0c},//
	{0xFA6C,0x1a},//
	{0xFA6D,0xe0},//
	{0xFA6E,0x20},//
	{0xFA6F,0xe0},//
	{0xFA70,0x06},//
	{0xFA71,0x90},//
	{0xFA72,0xfb},//
	{0xFA73,0x85},//
	{0xFA74,0x74},//
	{0xFA75,0x00},//
	{0xFA76,0xa5},//
	{0xFA77,0xe5},//
	{0xFA78,0x10},//
	{0xFA79,0x02},//
	{0xFA7A,0x1e},//
	{0xFA7B,0x8f},//
	{0xFA7C,0x22},//
	
	// Flash Strobe Trigger 3
	{0xFA7D, 0x90},//
	{0xFA7E, 0xfb},//
	{0xFA7F, 0x85},//
	{0xFA80, 0x74},//
	{0xFA81, 0x00},//
	{0xFA82, 0xa5},//
	{0xFA83, 0xe5},//
	{0xFA84, 0x1a},//
	{0xFA85, 0x60},//
	{0xFA86, 0x03},//
	{0xFA87, 0x02},//
	{0xFA88, 0x17},//
	{0xFA89, 0x47},//
	{0xFA8A, 0x22},//
	
	// Call Dark Cal Transform
	{0xFA8B, 0x90},//
	{0xFA8C, 0xfb},//
	{0xFA8D, 0x84},//
	{0xFA8E, 0x02},//
	{0xFA8F, 0x18},//
	{0xFA90, 0xd9},//
	{0xFA91, 0x22},//
	
	// Flash Strobe Stop
	{0xFA92, 0x02},//
	{0xFA93, 0x1f},//
	{0xFA94, 0xb1},//
	{0xFA95, 0x22},//
	
	// Patch Ctrl
	// Offsets
	{0x35D8, 0x01},//
	{0x35D9, 0x0F},//
	{0x35DA, 0x01},//
	{0x35DB, 0x72},//
	{0x35DC, 0x01},//
	{0x35DD, 0xB4},//
	{0x35DE, 0x01},//
	{0x35DF, 0xDF},//
	{0x35E0, 0x02},//
	{0x35E1, 0x08},//
	{0x35E2, 0x02},//
	{0x35E3, 0x28},//
	{0x35E4, 0x02},//
	{0x35E5, 0x56},//
	{0x35E6, 0x02},//
	{0x35E7, 0x6A},//
	{0x35E8, 0x02},//
	{0x35E9, 0x7D},//
	{0x35EA, 0x02},//
	{0x35EB, 0x8B},//
	{0x35EC, 0x02},//
	{0x35ED, 0x92},//
	{0x35EF, 0x22},//
	{0x35F1, 0x23},//
	{0x35F3, 0x22},//
	
	// Breakpoints
	{0x35F6, 0x19},//
	{0x35F7, 0x55},//
	{0x35F8, 0x1D},//
	{0x35F9, 0x4C},//
	{0x35FA, 0x16},//
	{0x35FB, 0xC7},//
	{0x35FC, 0x1A},//
	{0x35FD, 0xA0},//
	{0x35FE, 0x18},//
	{0x35FF, 0xD6},//
	{0x3600, 0x03},//
	{0x3601, 0xD4},//
	{0x3602, 0x18},//
	{0x3603, 0x8A},//
	{0x3604, 0x0A},//
	{0x3605, 0x0D},//
	{0x3606, 0x1E},//
	{0x3607, 0x8D},//
	{0x3608, 0x17},//
	{0x3609, 0x43},//
	{0x360A, 0x19},//
	{0x360B, 0x16},//
	{0x360C, 0x1F},//
	{0x360D, 0xAD},//
	{0x360E, 0x19},//
	{0x360F, 0x08},//
	{0x3610, 0x14},//
	{0x3611, 0x26},//
	{0x3612, 0x1A},//
	{0x3613, 0xB3},//
	
	// Jump Enables
	{0x35D2, 0x7F},//
	{0x35D3, 0xFF},//
	
	// Data Enables
	{0x35D4, 0x70},//
	
	// Patching Enable
	{0x35D0, 0x01},//
	
	// NVM1 Power Up
	{0x3E44, 0x01},//
	
	//-------------------------------------
	//	CSI mode
	//-------------------------------------
	{0x0111, 0x02},//
	{0x0114, 0x01},//
	
	{0x2136, 0x0C},//
	{0x2137, 0x00},//
	
	//-------------------------------------
	//	CSI data format
	//-------------------------------------
	{0x0112, 0x0A},//
	{0x0113, 0x0A},//
	
	{0x3016, 0x46},//
	{0x3017, 0x29},//
	{0x3003, 0x03},//
	{0x3045, 0x03},//
	{0x3047, 0x98},//
	//-------------------------------------
	//	PLL
	//-------------------------------------
	{0x0305, 0x04},//
	{0x0306, 0x01},//
	{0x0307, 0x18},//
	{0x0301, 0x0A},//
	{0x0309, 0x0A},//
 
	//-------------------------------------
	//	start_end address
	//-------------------------------------
	{0x0344, 0x00},//
	{0x0345, 0x00},//
	{0x0346, 0x00},//
	{0x0347, 0x00},//
	{0x0348, 0x0A},//
	{0x0349, 0x27},//
	{0x034A, 0x07},//
	{0x034B, 0x9F},//

	//-------------------------------------
	//	address increment for odd pixel 
	//-------------------------------------
	{0x0383, 0x01},//
	{0x0387, 0x01},//
	
	//-------------------------------------
	//	integration time
	//-------------------------------------
	{0x0202, 0x07},//
	{0x0203, 0xad},//
	
	//-------------------------------------
	//	Gain
	//-------------------------------------
	{0x0205, 0xE0},//

	//-------------------------------------
	//	mode select
	//-------------------------------------
	{0x0100, 0x01},//
};

//for capture                                                                         
static struct regval_list sensor_qsxga_regs[] = { //qsxga: 2592*1936@15fps
	//-------------------------------------
	//	mode select
	//-------------------------------------
	{0x0100, 0x00},//
	{REG_DLY,0x10},
	//-------------------------------------
	//	frame length
	//-------------------------------------
	{0x0340, 0x07},//
	{0x0341, 0xb6},//
	
	//-------------------------------------
	//	Data output size
	//-------------------------------------
	{0x034C, 0x0A},//
	{0x034D, 0x28},//  ;x_output_size -2600
	{0x034E, 0x07},//
	{0x034F, 0xA0},//  ;Y_output_size -1952
	
	
	//-------------------------------------
	//	Binning
	//-------------------------------------
	{0x0900, 0x00},//
	{0x0901, 0x00},//
	{0x0902, 0x00},//
	
	//-------------------------------------
	//	Crop image
	//-------------------------------------
	{0x0408, 0x00},//
	{0x0409, 0x00},//
	{0x040A, 0x00},//
	{0x040B, 0x00},//
	{0x040C, 0x0A},//
	{0x040D, 0x28},//
	{0x040E, 0x07},//
	{0x040F, 0xA0},//
	
	//-------------------------------------
	//	mode select
	//-------------------------------------
	{0x0100, 0x01},//
};

//for video
static struct regval_list sensor_sxga_regs[] = { //SXGA: 1280*960@30fps
	//-------------------------------------
	//	mode select
	//-------------------------------------
	{0x0100, 0x00},//
	{REG_DLY,0x90},
	//-------------------------------------
	//	frame length
	//-------------------------------------
	{0x0340, 0x07},//
	{0x0341, 0xbA},//
	
	//-------------------------------------
	//	Data output size
	//-------------------------------------
	{0x034C, 0x05},//
	{0x034D, 0x18},//  ;x_output_size -2600
	{0x034E, 0x03},//
	{0x034F, 0xD0},//  ;Y_output_size -1952
	
	
	//-------------------------------------
	//	Binning
	//-------------------------------------
	{0x0900, 0x01},//
	{0x0901, 0x22},//
	{0x0902, 0x00},//
	
	//-------------------------------------
	//	Crop image
	//-------------------------------------
	{0x0408, 0x00},//
	{0x0409, 0x0A},//
	{0x040A, 0x00},//
	{0x040B, 0x08},//
	{0x040C, 0x05},//
	{0x040D, 0x00},//
	{0x040E, 0x03},//
	{0x040F, 0xC0},//
	
	//-------------------------------------
	//	mode select
	//-------------------------------------
	{0x0100, 0x01},//
};


/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 * 
 */

static struct regval_list sensor_fmt_raw[] = {

};

/*
 * Low-level register I/O.
 *
 */


/*
 * On most platforms, we'd rather do straight i2c I/O.
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
  
//  vfe_dev_dbg("sensor write 0x%x = 0x%x\n", reg, value);
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

/* *********************************************begin of ******************************************** */
/*
static int sensor_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
  struct sensor_info *info = to_state(sd);
  unsigned char rdval;
    
  LOG_ERR_RET(sensor_read(sd, 0x3821, &rdval))
  
  rdval &= (1<<1);
  rdval >>= 1;
    
  *value = rdval;

  info->hflip = *value;
  return 0;
}

static int sensor_s_hflip(struct v4l2_subdev *sd, int value)
{
  struct sensor_info *info = to_state(sd);
  unsigned char rdval;
  
  if(info->hflip == value)
    return 0;
    
  LOG_ERR_RET(sensor_read(sd, 0x3821, &rdval))
  
  switch (value) {
    case 0:
      rdval &= 0xf9;
      break;
    case 1:
      rdval |= 0x06;
      break;
    default:
      return -EINVAL;
  }
  
  LOG_ERR_RET(sensor_write(sd, 0x3821, rdval))
  
  mdelay(10);
  info->hflip = value;
  return 0;
}

static int sensor_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
  struct sensor_info *info = to_state(sd);
  unsigned char rdval;
  
  LOG_ERR_RET(sensor_read(sd, 0x3820, &rdval))
  
  rdval &= (1<<1);  
  *value = rdval;
  rdval >>= 1;
  
  info->vflip = *value;
  return 0;
}

static int sensor_s_vflip(struct v4l2_subdev *sd, int value)
{
  struct sensor_info *info = to_state(sd);
  unsigned char rdval;
  
  if(info->vflip == value)
    return 0;
  
  LOG_ERR_RET(sensor_read(sd, 0x3820, &rdval))

  switch (value) {
    case 0:
      rdval &= 0xf9;
      break;
    case 1:
      rdval |= 0x06;
      break;
    default:
      return -EINVAL;
  }

  LOG_ERR_RET(sensor_write(sd, 0x3820, rdval))
  
  mdelay(10);
  info->vflip = value;
  return 0;
}
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
	unsigned char explow, expmid, exphigh;
	struct sensor_info *info = to_state(sd);

	vfe_dev_dbg("sensor_set_exposure = %d\n", exp_val>>4);
	if(exp_val>0xfffff)
		exp_val=0xfffff;

    exphigh  = (unsigned char) ( (0x0ff000&exp_val)>>12);
    expmid  = (unsigned char) ( (0x000ff0&exp_val)>>4);
    explow  = (unsigned char) ( (0x00000f&exp_val)<<4);
	
	sensor_write(sd, 0x0104, 0x01);
	sensor_write(sd, 0x0202, exphigh);
	sensor_write(sd, 0x0203, expmid);
	sensor_write(sd, 0x0200, explow);
	sensor_write(sd, 0x0201, 0x00);
	sensor_write(sd, 0x0104, 0x00);
	
//	printk("5040 sensor_set_exp = %d %d %d, Done!\n", exp_val, expmid, explow);
	
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
	unsigned char gaindig = 0, gainana = 0;
	
	if(gain_val<1*16)
		gain_val=16;
	if(gain_val>64*16-1)
		gain_val=64*16-1;
	vfe_dev_dbg("sensor_set_gain = %d\n", gain_val);
    //if(info->gain == gain_val)
    //    return 0;

	if (gain_val <= 16*4)
	{
		gainana = (2048 - 32768/gain_val)>>3;
	}
	else if (gain_val <= 85)//16*5.33
	{
		gainana = 0xC0;
		gaindig = gain_val*4;
	}
	else if (gain_val < 128)//16*8
	{
		gainana = 0xD0;
		gaindig = gain_val*3;
	}
	else if (gain_val <= 145)//16*9.1
	{
		gainana = 0xE0;
		gaindig = gain_val*2;
	}
	else if (gain_val <= 171)//16*10.7
	{
		gainana = 0xE4;
		gaindig = gain_val*7/4+1;
	}
	else if (gain_val <= 204)//16*12.8
	{
		gainana = 0xE8;
		gaindig = gain_val*3/2;
	}
	else if (gain_val < 256)//16*16
	{
		gainana = 0xEC;
		gaindig = gain_val*5/4;
	}
	else
	{
		gainana = 0xF0;
	}
	
	sensor_write(sd, 0x0104, 0x01);
	sensor_write(sd, 0x0205, gainana);
	sensor_write(sd, 0x020f, gaindig);
	sensor_write(sd, 0x0211, gaindig);
	sensor_write(sd, 0x0213, gaindig);
	sensor_write(sd, 0x0215, gaindig);
	sensor_write(sd, 0x0104, 0x00);
	
	printk("5040 sensor_set_gain = %d,, Done!\n", gain_val);
	info->gain = gain_val;
	
	return 0;
}

static int sensor_s_exp_gain(struct v4l2_subdev *sd, struct sensor_exp_gain *exp_gain)
{
  int exp_val, gain_val,frame_length,shutter;
  unsigned char explow=0,expmid=0,exphigh=0;
  unsigned char gaindig = 0, gainana = 0;
  struct sensor_info *info = to_state(sd);

  exp_val = exp_gain->exp_val;
  gain_val = exp_gain->gain_val;

  if((info->exp == exp_val)&&(info->gain == gain_val))
  	return 0;
  
  //return -EINVAL;
  if(gain_val<1*16)
	  gain_val=16;
  if(gain_val>64*16-1)
	  gain_val=64*16-1;
  
  if(exp_val>0xfffff)
	  exp_val=0xfffff;
  
  shutter = exp_val/16;
  if(shutter  > hm5040_sensor_vts - 4)
		frame_length = shutter + 4;
  else
		frame_length = hm5040_sensor_vts;
  
  
  exphigh  = (unsigned char) ( (0x0ff000&exp_val)>>12);
  expmid  = (unsigned char) ( (0x000ff0&exp_val)>>4);
  explow  = (unsigned char) ( (0x00000f&exp_val)<<4);
  
  if (gain_val <= 16*4)
  {
	  gainana = (2048 - 32768/gain_val)>>3;
  }
  else if (gain_val <= 85)//16*5.33
  {
	  gainana = 0xC0;
	  gaindig = gain_val*4;
  }
  else if (gain_val < 128)//16*8
  {
	  gainana = 0xD0;
	  gaindig = gain_val*3;
  }
  else if (gain_val <= 145)//16*9.1
  {
	  gainana = 0xE0;
	  gaindig = gain_val*2;
  }
  else if (gain_val <= 171)//16*10.7
  {
	  gainana = 0xE4;
	  gaindig = gain_val*7/4+1;
  }
  else if (gain_val <= 204)//16*12.8
  {
	  gainana = 0xE8;
	  gaindig = gain_val*3/2;
  }
  else if (gain_val < 256)//16*16
  {
	  gainana = 0xEC;
	  gaindig = gain_val*5/4;
  }
  else
  {
	  gainana = 0xF0;
  }
  
  sensor_write(sd, 0x0341, (frame_length & 0xff));
  sensor_write(sd, 0x0340, (frame_length >> 8));

  sensor_write(sd, 0x0104, 0x01);
  sensor_write(sd, 0x020f, gaindig);
  sensor_write(sd, 0x0211, gaindig);
  sensor_write(sd, 0x0213, gaindig);
  sensor_write(sd, 0x0215, gaindig);
  sensor_write(sd, 0x0202, exphigh);
  sensor_write(sd, 0x0203, expmid);
  sensor_write(sd, 0x0205, gainana);
  sensor_write(sd, 0x0104, 0x00);

//  printk("5040 sensor_set_gain = %d, exp %d, 0x%x    Done!\n", gain_val, exp_val, frame_length);

  info->exp = exp_val;
  info->gain = gain_val;
  return 0; 
}

static int sensor_s_sw_stby(struct v4l2_subdev *sd, int on_off)
{
	int ret;
	unsigned char rdval;
	
	ret=sensor_read(sd, 0x0100, &rdval);
	if(ret!=0)
		return ret;
	
	if(on_off==CSI_STBY_ON)//sw stby on
	{
		ret=sensor_write(sd, 0x0100, rdval&0xfe);
	}
	else//sw stby off
	{
		ret=sensor_write(sd, 0x0100, rdval|0x01);
	}
	return ret;
}

/*
 * Stuff that knows about the sensor.
 */
 
static int sensor_power(struct v4l2_subdev *sd, int on)
{
  int ret;
  
  //insure that clk_disable() and clk_enable() are called in pair 
  //when calling CSI_SUBDEV_STBY_ON/OFF and CSI_SUBDEV_PWR_ON/OFF
  ret = 0;
  switch(on)
  {
    case CSI_SUBDEV_STBY_ON:
      vfe_dev_dbg("CSI_SUBDEV_STBY_ON!\n");
      //software standby on
      ret = sensor_s_sw_stby(sd, CSI_STBY_ON);
      if(ret < 0)
        vfe_dev_err("soft stby falied!\n");
      usleep_range(10000,12000);
      //make sure that no device can access i2c bus during sensor initial or power down
      //when using i2c_lock_adpater function, the following codes must not access i2c bus before calling i2c_unlock_adapter
      cci_lock(sd);
      //standby on io
      vfe_gpio_write(sd,RESET,CSI_STBY_ON);
      //remember to unlock i2c adapter, so the device can access the i2c bus again
      cci_unlock(sd);  
      //inactive mclk after stadby in
      vfe_set_mclk(sd,OFF);
      break;
    case CSI_SUBDEV_STBY_OFF:
      vfe_dev_dbg("CSI_SUBDEV_STBY_OFF!\n");
      //make sure that no device can access i2c bus during sensor initial or power down
      //when using i2c_lock_adpater function, the following codes must not access i2c bus before calling i2c_unlock_adapter
      cci_lock(sd);    
      //active mclk before stadby out
      vfe_set_mclk_freq(sd,MCLK);
      vfe_set_mclk(sd,ON);
      usleep_range(10000,12000);
      //standby off io
      vfe_gpio_write(sd,RESET,CSI_STBY_OFF);
      usleep_range(10000,12000);
      //remember to unlock i2c adapter, so the device can access the i2c bus again
      cci_unlock(sd);        
      //software standby
      ret = sensor_s_sw_stby(sd, CSI_STBY_OFF);
      if(ret < 0)
        vfe_dev_err("soft stby off falied!\n");
      usleep_range(10000,12000);
      break;
    case CSI_SUBDEV_PWR_ON:
      vfe_dev_dbg("CSI_SUBDEV_PWR_ON!\n");
      //make sure that no device can access i2c bus during sensor initial or power down
      //when using i2c_lock_adpater function, the following codes must not access i2c bus before calling i2c_unlock_adapter
      cci_lock(sd);    
      //power on reset
      vfe_gpio_set_status(sd,PWDN,1);//set the gpio to output
//      vfe_gpio_set_status(sd,RESET,1);//set the gpio to output
      //power down io
      vfe_gpio_write(sd,PWDN,CSI_STBY_ON);
      //reset on io
//      vfe_gpio_write(sd,RESET,CSI_RST_ON);
      usleep_range(1000,1200);
      //active mclk before power on
      vfe_set_mclk_freq(sd,MCLK);
      vfe_set_mclk(sd,ON);
      usleep_range(10000,12000);
      //power supply
      vfe_gpio_write(sd,POWER_EN,CSI_PWR_ON);
      vfe_set_pmu_channel(sd,IOVDD,ON);
      vfe_set_pmu_channel(sd,AVDD,ON);
      vfe_set_pmu_channel(sd,DVDD,ON);
      vfe_set_pmu_channel(sd,AFVDD,ON);
      //standby off io
      usleep_range(30000,31000);
      vfe_gpio_write(sd,PWDN,CSI_STBY_OFF);
      usleep_range(10000,12000);
      //reset after power on
//      vfe_gpio_write(sd,RESET,CSI_RST_OFF);
      usleep_range(30000,31000);
      //remember to unlock i2c adapter, so the device can access the i2c bus again
      cci_unlock(sd);    

      break;
    case CSI_SUBDEV_PWR_OFF:
      vfe_dev_dbg("CSI_SUBDEV_PWR_OFF!\n");
      //make sure that no device can access i2c bus during sensor initial or power down
      //when using i2c_lock_adpater function, the following codes must not access i2c bus before calling i2c_unlock_adapter
      cci_lock(sd);   
      //inactive mclk before power off
      vfe_set_mclk(sd,OFF);
      //power supply off
      vfe_gpio_write(sd,POWER_EN,CSI_PWR_OFF);
      vfe_set_pmu_channel(sd,AFVDD,OFF);
      vfe_set_pmu_channel(sd,DVDD,OFF);
      vfe_set_pmu_channel(sd,AVDD,OFF);
      vfe_set_pmu_channel(sd,IOVDD,OFF);  
      //standby and reset io
      usleep_range(10000,12000);
      vfe_gpio_write(sd,POWER_EN,CSI_PWR_OFF);
//      vfe_gpio_write(sd,RESET,CSI_RST_ON);
      vfe_gpio_write(sd,PWDN,CSI_STBY_ON);
      //set the io to hi-z
//      vfe_gpio_set_status(sd,RESET,0);//set the gpio to input
      vfe_gpio_set_status(sd,PWDN,0);//set the gpio to input
      //remember to unlock i2c adapter, so the device can access the i2c bus again
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
  
	LOG_ERR_RET(sensor_read(sd, 0x0000, &rdval))
  //  if(rdval != 0x81)
  //	return -ENODEV;
  printk("sensor_detect 0x0000 = 0x%x\n", rdval);
	  
	LOG_ERR_RET(sensor_read(sd, 0x0001, &rdval))
  //  if(rdval != 0x31)
  //	return -ENODEV;
	printk("sensor_detect 0x0001 = 0x%x\n", rdval);
		  
			LOG_ERR_RET(sensor_read(sd, 0x2016, &rdval))
		  //  if(rdval != 0x81)
		  //	return -ENODEV;
		  printk("sensor_detect 0x2016 = 0x%x\n", rdval);
			  
			LOG_ERR_RET(sensor_read(sd, 0x2017, &rdval))
		  //  if(rdval != 0x31)
		  //	return -ENODEV;
			printk("sensor_detect 0x2017 = 0x%x\n", rdval);
				  
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
  info->width = QSXGA_WIDTH;
  info->height = QSXGA_HEIGHT;
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
//  vfe_dev_dbg("[]cmd=%d\n",cmd);
//  vfe_dev_dbg("[]arg=%0x\n",arg);
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
//      if((unsigned int *)arg==1)
//        ret=sensor_write(sd, 0x3036, 0x78);
//      else
//        ret=sensor_write(sd, 0x3036, 0x32);
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
		.desc				= "Raw RGB Bayer",
		.mbus_code	= V4L2_MBUS_FMT_SBGGR10_10X1,
		.regs 			= sensor_fmt_raw,
		.regs_size 	= ARRAY_SIZE(sensor_fmt_raw),
		.bpp				= 1
	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

  

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */


static struct sensor_win_size sensor_win_sizes[] = {
#if 1
	/* quxga: 2592*1936 */
	{
		.width		= QSXGA_WIDTH,
		.height 	= QSXGA_HEIGHT,
		.hoffset	= 4,
		.voffset	= 8,
		.hts		= 2750,
		.vts		= 1974,
		.pclk		= 163*1000*1000,
		.mipi_bps	= 420*1000*1000,
		.fps_fixed	= 2,
		.bin_factor = 1,
		.intg_min	= 1<<4,
		.intg_max	= (1974-4)<<4,
		.gain_min	= 1<<4,
		.gain_max	= (16<<4),
		.regs		= sensor_qsxga_regs,
		.regs_size	= ARRAY_SIZE(sensor_qsxga_regs),
		.set_size	= NULL,
	},
#endif
#if 1
    /* 1080P */
    {
      .width	  = 2560,
      .height 	  = 1440,
      .hoffset    = 16,
      .voffset    = 248,
	  .hts		  = 2750,
	  .vts		  = 1974,
	  .pclk 	  = 163*1000*1000,
	  .mipi_bps   = 420*1000*1000,
	  .fps_fixed  = 2,
	  .bin_factor = 1,
	  .intg_min   = 1<<4,
	  .intg_max   = (1974-4)<<4,
	  .gain_min   = 1<<4,
	  .gain_max   = (16<<4),
      .regs       = sensor_qsxga_regs,//
      .regs_size  = ARRAY_SIZE(sensor_qsxga_regs),//
      .set_size	  = NULL,
    },
#endif
#if 1

  	/* SXGA */
    {
      .width	  = SXGA_WIDTH,
      .height 	  = SXGA_HEIGHT,
      .hoffset	  = 0,
      .voffset	  = 0,
	  .hts		  = 2750,
	  .vts		  = 1978,
	  .pclk 	  = 163*1000*1000,
	  .mipi_bps   = 420*1000*1000,
      .fps_fixed  = 1,
      .bin_factor = 1,
      .intg_min   = 1,
      .intg_max   = 1978<<4,
      .gain_min   = 1<<4,
      .gain_max   = 12<<4,
      .regs		  = sensor_sxga_regs,
      .regs_size  = ARRAY_SIZE(sensor_sxga_regs),
      .set_size	  = NULL,
    },

    /* 720p */
    {
      .width      = HD720_WIDTH,
      .height     = HD720_HEIGHT,
      .hoffset    = 0,
      .voffset    = 120,
	  .hts		  = 2750,
	  .vts		  = 1978,
	  .pclk 	  = 163*1000*1000,
	  .mipi_bps   = 420*1000*1000,
	  .fps_fixed  = 1,
	  .bin_factor = 1,
	  .intg_min   = 1,
	  .intg_max   = 1978<<4,
      .gain_min   = 1<<4,
      .gain_max   = 12<<4,
      .regs		  = sensor_sxga_regs,//
      .regs_size  = ARRAY_SIZE(sensor_sxga_regs),//
      .set_size	  = NULL,
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
  
  info->current_wins = wsize;  
    
  /*
   * Note the size we'll actually handle.
   */
  fmt->width = wsize->width;
  fmt->height = wsize->height;
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
  cfg->flags = 0|V4L2_MBUS_CSI2_2_LANE|V4L2_MBUS_CSI2_CHANNEL_0;
  
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
  
  //sensor_write_array(sd, sensor_oe_disable_regs, ARRAY_SIZE(sensor_oe_disable_regs));
  
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

  sensor_write_array(sd, sensor_fmt->regs, sensor_fmt->regs_size);

  ret = 0;
  if (wsize->regs)
    LOG_ERR_RET(sensor_write_array(sd, wsize->regs, wsize->regs_size))
  
  if (wsize->set_size)
    LOG_ERR_RET(wsize->set_size(sd))

  info->fmt = sensor_fmt;
  info->width = wsize->width;
  info->height = wsize->height;
  hm5040_sensor_vts = wsize->vts;

  vfe_dev_print("s_fmt = %x, width = %d, height = %d\n",sensor_fmt->mbus_code,wsize->width,wsize->height);

  if(info->capture_mode == V4L2_MODE_VIDEO)
  {
    //video
   
  } else {
    //capture image

  }
	
	//sensor_write_array(sd, sensor_oe_enable_regs, ARRAY_SIZE(sensor_oe_enable_regs));
	vfe_dev_print("s_fmt end\n");
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
		return v4l2_ctrl_query_fill(qc, 1*16, 32*16, 1, 16);
	case V4L2_CID_EXPOSURE:
		return v4l2_ctrl_query_fill(qc, 0, 65535*16, 1, 0);
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

  if (ctrl->value < qc.minimum || ctrl->value > qc.maximum) {
    return -ERANGE;
  }
  
  switch (ctrl->id) {
    case V4L2_CID_GAIN:
      return sensor_s_gain(sd, ctrl->value);
    case V4L2_CID_EXPOSURE:
	  return sensor_s_exp(sd, ctrl->value);
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

