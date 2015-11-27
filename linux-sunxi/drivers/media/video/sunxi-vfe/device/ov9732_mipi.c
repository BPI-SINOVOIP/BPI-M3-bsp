/*
 * A V4L2 driver for ov9732_mipi Raw cameras.
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
MODULE_DESCRIPTION("A low-level driver for ov9732_mipi Raw sensors");
MODULE_LICENSE("GPL");

//for internel driver debug
#define DEV_DBG_EN      1 
#if(DEV_DBG_EN == 1)    
#define vfe_dev_dbg(x,arg...) printk("[ov9732_mipi Raw]"x,##arg)
#else
#define vfe_dev_dbg(x,arg...) 
#endif
#define vfe_dev_err(x,arg...) printk("[ov9732_mipi Raw]"x,##arg)
#define vfe_dev_print(x,arg...) printk("[ov9732_mipi Raw]"x,##arg)

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
#define VREF_POL          V4L2_MBUS_VSYNC_ACTIVE_LOW
#define HREF_POL          V4L2_MBUS_HSYNC_ACTIVE_HIGH
#define CLK_POL           V4L2_MBUS_PCLK_SAMPLE_RISING
#define V4L2_IDENT_SENSOR  0x9732


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

/*
 * Our nominal (default) frame rate.
 */

#define SENSOR_FRAME_RATE 30


/*
 * The ov9732_mipi i2c address
 */
#define I2C_ADDR 0x6c
#define  SENSOR_NAME "ov9732_mipi"

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

static struct regval_list sensor_default_regs[] = 
{
	{0x0103, 0x1 },//
	{0x0100, 0x0 },//
	{0x3001, 0x0 },//
	{0x3002, 0x0 },//
	{0x3007, 0x0 },//
	{0x3009, 0x2 },//
	{0x3010, 0x0 },//
	{0x3011, 0x8 },//
	{0x3014, 0x22},// 
	{0x301e, 0x15},// 
	{0x3030, 0x19},// 
	{0x3080, 0x2 },//
	{0x3081, 0x3c},// 
	{0x3082, 0x4 },//
	{0x3083, 0x0 },//
	{0x3084, 0x2 },//
	{0x3085, 0x1 },//
	{0x3086, 0x1 },//
	{0x3089, 0x1 },//
	{0x308a, 0x0 },//
	{0x3103, 0x1 },//
	{0x3600, 0xff},// 
	{0x3601, 0x72},// 
	{0x3610, 0xc },//
	{0x3611, 0xb0},// 
	{0x3612, 0x35},// 
	{0x3654, 0x10},// 
	{0x3655, 0x77},// 
	{0x3656, 0x77},// 
	{0x3657, 0x7 },//
	{0x3658, 0x22},// 
	{0x3659, 0x22},// 
	{0x365a, 0x2 },//
	{0x3700, 0x1f},// 
	{0x3701, 0x10},// 
	{0x3702, 0xc },//
	{0x3703, 0x7 },//
	{0x3704, 0x3c},// 
	{0x3705, 0x41},// 
	{0x370d, 0x10},// 
	{0x3710, 0xc },//
	{0x3783, 0x8 },//
	{0x3784, 0x5 },//
	{0x3785, 0x55},// 
	{0x37c0, 0x7 },//
	{0x3800, 0x0 },//
	{0x3801, 0x4 },//
	{0x3802, 0x0 },//
	{0x3803, 0x4 },//
	{0x3804, 0x5 },//
	{0x3805, 0xb },//
	{0x3806, 0x2 },//
	{0x3807, 0xdb},// 
	{0x3808, 0x5 },//
	{0x3809, 0x0 },//
	{0x380a, 0x2 },//
	{0x380b, 0xd0},// 
	{0x380c, 0x5 },//
	{0x380d, 0xc6},// 
	{0x380e, 0x3 },//
	{0x380f, 0x22},// 
	{0x3810, 0x0 },//
	{0x3811, 0x4 },//
	{0x3812, 0x0 },//
	{0x3813, 0x4 },//
	{0x3816, 0x0 },//
	{0x3817, 0x0 },//
	{0x3818, 0x0 },//
	{0x3819, 0x4 },//
	{0x3820, 0x10},// 
	{0x3821, 0x0 },//
	{0x382c, 0x6 },//
	{0x3500, 0x0 },//
	{0x3501, 0x31},// 
	{0x3502, 0x0 },//
	{0x3503, 0x3 },//
	{0x3504, 0x0 },//
	{0x3505, 0x0 },//
	{0x3509, 0x10},// 
	{0x350a, 0x0 },//
	{0x350b, 0x40},// 
	{0x3d00, 0x0 },//
	{0x3d01, 0x0 },//
	{0x3d02, 0x0 },//
	{0x3d03, 0x0 },//
	{0x3d04, 0x0 },//
	{0x3d05, 0x0 },//
	{0x3d06, 0x0 },//
	{0x3d07, 0x0 },//
	{0x3d08, 0x0 },//
	{0x3d09, 0x0 },//
	{0x3d0a, 0x0 },//
	{0x3d0b, 0x0 },//
	{0x3d0c, 0x0 },//
	{0x3d0d, 0x0 },//
	{0x3d0e, 0x0 },//
	{0x3d0f, 0x0 },//
	{0x3d80, 0x0 },//
	{0x3d81, 0x0 },//
	{0x3d82, 0x38},// 
	{0x3d83, 0xa4},// 
	{0x3d84, 0x0 },//
	{0x3d85, 0x0 },//
	{0x3d86, 0x1f},// 
	{0x3d87, 0x3 },//
	{0x3d8b, 0x0 },//
	{0x3d8f, 0x0 },//
	{0x4001, 0xe0},// 
	{0x4004, 0x0 },//
	{0x4005, 0x2 },//
	{0x4006, 0x1 },//
	{0x4007, 0x40},// 
	{0x4009, 0xb },//
	{0x4300, 0x3 },//
	{0x4301, 0xff},// 
	{0x4304, 0x0 },//
	{0x4305, 0x0 },//
	{0x4309, 0x0 },//
	{0x4600, 0x0 },//
	{0x4601, 0x4 },//
	{0x4800, 0x0 },//
	{0x4805, 0x0 },//
	{0x4821, 0x50},// 
	{0x4823, 0x50},// 
	{0x4837, 0x2d},// 
	{0x4a00, 0x0 },//
	{0x4f00, 0x80},// 
	{0x4f01, 0x10},// 
	{0x4f02, 0x0 },//
	{0x4f03, 0x0 },//
	{0x4f04, 0x0 },//
	{0x4f05, 0x0 },//
	{0x4f06, 0x0 },//
	{0x4f07, 0x0 },//
	{0x4f08, 0x0 },//
	{0x4f09, 0x0 },//
	{0x5000, 0x2f},// 
	{0x500c, 0x0 },//
	{0x500d, 0x0 },//
	{0x500e, 0x0 },//
	{0x500f, 0x0 },//
	{0x5010, 0x0 },//
	{0x5011, 0x0 },//
	{0x5012, 0x0 },//
	{0x5013, 0x0 },//
	{0x5014, 0x0 },//
	{0x5015, 0x0 },//
	{0x5016, 0x0 },//
	{0x5017, 0x0 },//
	{0x5080, 0x0 },//
	{0x5180, 0x1 },//
	{0x5181, 0x0 },//
	{0x5182, 0x1 },//
	{0x5183, 0x0 },//
	{0x5184, 0x1 },//
	{0x5185, 0x0 },//
	{0x5708, 0x6 },//
	{0x5781, 0x0 },//
	{0x5783, 0xf },//
	{0x0100, 0x01},//
	{0x3600, 0xf6},// 
	{0x5c80, 0x5 },//
	{0x5c81, 0x60},// 
	{0x5c82, 0x9 },//
	{0x5c83, 0x5f},// 
	{0x5c85, 0x6c},// 
	{0x5601, 0x4 },//
	{0x5602, 0x2 },//
	{0x5603, 0x1 },//
	{0x5604, 0x4 },//
	{0x5605, 0x2 },//
	{0x5606, 0x1 },//
	{0x5400, 0xff},// 
	{0x5401, 0xc8},// 
	{0x5402, 0x9f},// 
	{0x5403, 0x8b},// 
	{0x5404, 0x83},// 
	{0x5405, 0x86},// 
	{0x5406, 0x91},// 
	{0x5407, 0xa9},// 
	{0x5408, 0xe1},// 
	{0x5409, 0xff},// 
	{0x540a, 0x7e},// 
	{0x540b, 0x60},// 
	{0x540c, 0x50},// 
	{0x540d, 0x47},// 
	{0x540e, 0x43},// 
	{0x540f, 0x44},// 
	{0x5410, 0x49},// 
	{0x5411, 0x55},// 
	{0x5412, 0x65},// 
	{0x5413, 0x8c},// 
	{0x5414, 0x4f},// 
	{0x5415, 0x3c},// 
	{0x5416, 0x31},// 
	{0x5417, 0x2a},// 
	{0x5418, 0x27},// 
	{0x5419, 0x28},// 
	{0x541a, 0x2c},// 
	{0x541b, 0x33},// 
	{0x541c, 0x3f},// 
	{0x541d, 0x56},// 
	{0x541e, 0x34},// 
	{0x541f, 0x26},// 
	{0x5420, 0x1d},// 
	{0x5421, 0x18},// 
	{0x5422, 0x15},// 
	{0x5423, 0x15},// 
	{0x5424, 0x19},// 
	{0x5425, 0x1f},// 
	{0x5426, 0x29},// 
	{0x5427, 0x39},// 
	{0x5428, 0x27},// 
	{0x5429, 0x1a},// 
	{0x542a, 0x11},// 
	{0x542b, 0xa },//
	{0x542c, 0x7 },//
	{0x542d, 0x7 },//
	{0x542e, 0xb },//
	{0x542f, 0x12},// 
	{0x5430, 0x1c},// 
	{0x5431, 0x2b},// 
	{0x5432, 0x20},// 
	{0x5433, 0x14},// 
	{0x5434, 0xa },//
	{0x5435, 0x5 },//
	{0x5436, 0x2 },//
	{0x5437, 0x2 },//
	{0x5438, 0x5 },//
	{0x5439, 0xc },//
	{0x543a, 0x16},// 
	{0x543b, 0x24},// 
	{0x543c, 0x21},// 
	{0x543d, 0x14},// 
	{0x543e, 0xb },//
	{0x543f, 0x4 },//
	{0x5440, 0x2 },//
	{0x5441, 0x2 },//
	{0x5442, 0x6 },//
	{0x5443, 0xc },//
	{0x5444, 0x16},// 
	{0x5445, 0x25},// 
	{0x5446, 0x29},// 
	{0x5447, 0x1b},// 
	{0x5448, 0x12},// 
	{0x5449, 0xc },//
	{0x544a, 0x8 },//
	{0x544b, 0x9 },//
	{0x544c, 0xd },//
	{0x544d, 0x14},// 
	{0x544e, 0x1e},// 
	{0x544f, 0x2c},// 
	{0x5450, 0x36},// 
	{0x5451, 0x28},// 
	{0x5452, 0x1f},// 
	{0x5453, 0x19},// 
	{0x5454, 0x17},// 
	{0x5455, 0x17},// 
	{0x5456, 0x1b},// 
	{0x5457, 0x21},// 
	{0x5458, 0x2b},// 
	{0x5459, 0x3d},// 
	{0x545a, 0x54},// 
	{0x545b, 0x40},// 
	{0x545c, 0x34},// 
	{0x545d, 0x2e},// 
	{0x545e, 0x2b},// 
	{0x545f, 0x2b},// 
	{0x5460, 0x2f},// 
	{0x5461, 0x38},// 
	{0x5462, 0x46},// 
	{0x5463, 0x5d},// 
	{0x5464, 0x8a},// 
	{0x5465, 0x6a},// 
	{0x5466, 0x58},// 
	{0x5467, 0x4f},// 
	{0x5468, 0x4b},// 
	{0x5469, 0x4d},// 
	{0x546a, 0x52},// 
	{0x546b, 0x5e},// 
	{0x546c, 0x72},// 
	{0x546d, 0x9f},// 
	{0x546e, 0xff},// 
	{0x546f, 0xe9},// 
	{0x5470, 0xb6},// 
	{0x5471, 0x9e},// 
	{0x5472, 0x96},// 
	{0x5473, 0x96},// 
	{0x5474, 0xa4},// 
	{0x5475, 0xc6},// 
	{0x5476, 0xff},// 
	{0x5477, 0xff},// 
	{0x5478, 0x6d},// 
	{0x5479, 0x70},// 
	{0x547a, 0x71},// 
	{0x547b, 0x74},// 
	{0x547c, 0x73},// 
	{0x547d, 0x73},// 
	{0x547e, 0x74},// 
	{0x547f, 0x73},// 
	{0x5480, 0x73},// 
	{0x5481, 0x70},// 
	{0x5482, 0x70},// 
	{0x5483, 0x6f},// 
	{0x5484, 0x6f},// 
	{0x5485, 0x70},// 
	{0x5486, 0x71},// 
	{0x5487, 0x71},// 
	{0x5488, 0x71},// 
	{0x5489, 0x71},// 
	{0x548a, 0x6f},// 
	{0x548b, 0x72},// 
	{0x548c, 0x70},// 
	{0x548d, 0x70},// 
	{0x548e, 0x72},// 
	{0x548f, 0x74},// 
	{0x5490, 0x75},// 
	{0x5491, 0x76},// 
	{0x5492, 0x75},// 
	{0x5493, 0x74},// 
	{0x5494, 0x71},// 
	{0x5495, 0x72},// 
	{0x5496, 0x71},// 
	{0x5497, 0x72},// 
	{0x5498, 0x74},// 
	{0x5499, 0x77},// 
	{0x549a, 0x79},// 
	{0x549b, 0x79},// 
	{0x549c, 0x79},// 
	{0x549d, 0x77},// 
	{0x549e, 0x74},// 
	{0x549f, 0x74},// 
	{0x54a0, 0x72},// 
	{0x54a1, 0x73},// 
	{0x54a2, 0x77},// 
	{0x54a3, 0x7c},// 
	{0x54a4, 0x7f},// 
	{0x54a5, 0x80},// 
	{0x54a6, 0x7f},// 
	{0x54a7, 0x7c},// 
	{0x54a8, 0x77},// 
	{0x54a9, 0x78},// 
	{0x54aa, 0x74},// 
	{0x54ab, 0x73},// 
	{0x54ac, 0x79},// 
	{0x54ad, 0x7f},// 
	{0x54ae, 0x83},// 
	{0x54af, 0x84},// 
	{0x54b0, 0x84},// 
	{0x54b1, 0x7f},// 
	{0x54b2, 0x7a},// 
	{0x54b3, 0x79},// 
	{0x54b4, 0x72},// 
	{0x54b5, 0x73},// 
	{0x54b6, 0x78},// 
	{0x54b7, 0x7f},// 
	{0x54b8, 0x83},// 
	{0x54b9, 0x84},// 
	{0x54ba, 0x83},// 
	{0x54bb, 0x7f},// 
	{0x54bc, 0x7a},// 
	{0x54bd, 0x79},// 
	{0x54be, 0x72},// 
	{0x54bf, 0x72},// 
	{0x54c0, 0x76},// 
	{0x54c1, 0x7b},// 
	{0x54c2, 0x7e},// 
	{0x54c3, 0x7f},// 
	{0x54c4, 0x7f},// 
	{0x54c5, 0x7b},// 
	{0x54c6, 0x77},// 
	{0x54c7, 0x77},// 
	{0x54c8, 0x71},// 
	{0x54c9, 0x71},// 
	{0x54ca, 0x74},// 
	{0x54cb, 0x77},// 
	{0x54cc, 0x79},// 
	{0x54cd, 0x7a},// 
	{0x54ce, 0x7a},// 
	{0x54cf, 0x77},// 
	{0x54d0, 0x75},// 
	{0x54d1, 0x74},// 
	{0x54d2, 0x71},// 
	{0x54d3, 0x70},// 
	{0x54d4, 0x72},// 
	{0x54d5, 0x74},// 
	{0x54d6, 0x76},// 
	{0x54d7, 0x76},// 
	{0x54d8, 0x76},// 
	{0x54d9, 0x75},// 
	{0x54da, 0x71},// 
	{0x54db, 0x73},// 
	{0x54dc, 0x72},// 
	{0x54dd, 0x6f},// 
	{0x54de, 0x70},// 
	{0x54df, 0x71},// 
	{0x54e0, 0x71},// 
	{0x54e1, 0x72},// 
	{0x54e2, 0x72},// 
	{0x54e3, 0x71},// 
	{0x54e4, 0x71},// 
	{0x54e5, 0x72},// 
	{0x54e6, 0x6b},// 
	{0x54e7, 0x70},// 
	{0x54e8, 0x71},// 
	{0x54e9, 0x72},// 
	{0x54ea, 0x73},// 
	{0x54eb, 0x73},// 
	{0x54ec, 0x73},// 
	{0x54ed, 0x72},// 
	{0x54ee, 0x71},// 
	{0x54ef, 0x6c},// 
	{0x54f0, 0x8a},// 
	{0x54f1, 0x8f},// 
	{0x54f2, 0x8f},// 
	{0x54f3, 0x8e},// 
	{0x54f4, 0x90},// 
	{0x54f5, 0x8e},// 
	{0x54f6, 0x8f},// 
	{0x54f7, 0x8f},// 
	{0x54f8, 0x8e},// 
	{0x54f9, 0x90},// 
	{0x54fa, 0x8a},// 
	{0x54fb, 0x89},// 
	{0x54fc, 0x88},// 
	{0x54fd, 0x88},// 
	{0x54fe, 0x87},// 
	{0x54ff, 0x88},// 
	{0x5500, 0x89},// 
	{0x5501, 0x89},// 
	{0x5502, 0x8c},// 
	{0x5503, 0x8c},// 
	{0x5504, 0x87},// 
	{0x5505, 0x84},// 
	{0x5506, 0x83},// 
	{0x5507, 0x82},// 
	{0x5508, 0x82},// 
	{0x5509, 0x83},// 
	{0x550a, 0x83},// 
	{0x550b, 0x85},// 
	{0x550c, 0x87},// 
	{0x550d, 0x8c},// 
	{0x550e, 0x81},// 
	{0x550f, 0x80},// 
	{0x5510, 0x80},// 
	{0x5511, 0x7f},// 
	{0x5512, 0x7f},// 
	{0x5513, 0x80},// 
	{0x5514, 0x81},// 
	{0x5515, 0x82},// 
	{0x5516, 0x85},// 
	{0x5517, 0x86},// 
	{0x5518, 0x7f},// 
	{0x5519, 0x7e},// 
	{0x551a, 0x7e},// 
	{0x551b, 0x7f},// 
	{0x551c, 0x80},// 
	{0x551d, 0x81},// 
	{0x551e, 0x81},// 
	{0x551f, 0x82},// 
	{0x5520, 0x83},// 
	{0x5521, 0x86},// 
	{0x5522, 0x7d},// 
	{0x5523, 0x7c},// 
	{0x5524, 0x7d},// 
	{0x5525, 0x7f},// 
	{0x5526, 0x80},// 
	{0x5527, 0x81},// 
	{0x5528, 0x82},// 
	{0x5529, 0x82},// 
	{0x552a, 0x83},// 
	{0x552b, 0x85},// 
	{0x552c, 0x7d},// 
	{0x552d, 0x7c},// 
	{0x552e, 0x7d},// 
	{0x552f, 0x80},// 
	{0x5530, 0x81},// 
	{0x5531, 0x82},// 
	{0x5532, 0x82},// 
	{0x5533, 0x82},// 
	{0x5534, 0x82},// 
	{0x5535, 0x85},// 
	{0x5536, 0x7e},// 
	{0x5537, 0x7d},// 
	{0x5538, 0x7f},// 
	{0x5539, 0x80},// 
	{0x553a, 0x81},// 
	{0x553b, 0x81},// 
	{0x553c, 0x82},// 
	{0x553d, 0x82},// 
	{0x553e, 0x83},// 
	{0x553f, 0x84},// 
	{0x5540, 0x81},// 
	{0x5541, 0x80},// 
	{0x5542, 0x7f},// 
	{0x5543, 0x7f},// 
	{0x5544, 0x7f},// 
	{0x5545, 0x80},// 
	{0x5546, 0x81},// 
	{0x5547, 0x82},// 
	{0x5548, 0x83},// 
	{0x5549, 0x86},// 
	{0x554a, 0x87},// 
	{0x554b, 0x84},// 
	{0x554c, 0x82},// 
	{0x554d, 0x81},// 
	{0x554e, 0x81},// 
	{0x554f, 0x82},// 
	{0x5550, 0x82},// 
	{0x5551, 0x83},// 
	{0x5552, 0x85},// 
	{0x5553, 0x88},// 
	{0x5554, 0x8a},// 
	{0x5555, 0x89},// 
	{0x5556, 0x88},// 
	{0x5557, 0x87},// 
	{0x5558, 0x86},// 
	{0x5559, 0x87},// 
	{0x555a, 0x87},// 
	{0x555b, 0x88},// 
	{0x555c, 0x89},// 
	{0x555d, 0x8b},// 
	{0x555e, 0x88},// 
	{0x555f, 0x8e},// 
	{0x5560, 0x8c},// 
	{0x5561, 0x8d},// 
	{0x5562, 0x8e},// 
	{0x5563, 0x8c},// 
	{0x5564, 0x8c},// 
	{0x5565, 0x8b},// 
	{0x5566, 0x8d},// 
	{0x5567, 0x89},// 
	{0x5570, 0x54},// 
	{0x5940, 0x50},// 
	{0x5941, 0x4 },//
	{0x5942, 0x8 },//
	{0x5943, 0x10},// 
	{0x5944, 0x18},// 
	{0x5945, 0x30},// 
	{0x5946, 0x40},// 
	{0x5947, 0x80},// 
	{0x5948, 0xf0},// 
	{0x5949, 0x10},// 
	{0x594a, 0x8 },//
	{0x594b, 0x18},// 
	{0x5b80, 0x4 },//
	{0x5b81, 0xa },//
	{0x5b82, 0x18},// 
	{0x5b83, 0x20},// 
	{0x5b84, 0x30},// 
	{0x5b85, 0x44},// 
	{0x5b86, 0x10},// 
	{0x5b87, 0x12},// 
	{0x5b88, 0x14},// 
	{0x5b89, 0x15},// 
	{0x5b8a, 0x16},// 
	{0x5b8b, 0x18},// 
	{0x5b8c, 0x10},// 
	{0x5b8d, 0x12},// 
	{0x5b8e, 0x14},// 
	{0x5b8f, 0x15},// 
	{0x5b90, 0x16},// 
	{0x5b91, 0x18},// 
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

static int sensor_read(struct v4l2_subdev *sd, unsigned short reg,
    unsigned char *value)
{
	int ret=0;
	int cnt=0;
	
	ret = cci_read_a16_d8(sd,reg,value);
	while(ret!=0&&cnt<3)
	{
		ret = cci_read_a16_d8(sd,reg,value);
		cnt++;
	}
	if(cnt>0)
		vfe_dev_dbg("sensor read retry=%d\n",cnt);
  
	return ret;
}

static int reg_val_show(struct v4l2_subdev *sd,unsigned short reg)
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
		printk("0x%x value is 0x%x\n",(array+i)->addr,tmp);
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
	
	exphigh = (unsigned char) ( (0x0f0000&exp_val)>>16);
	expmid  = (unsigned char) ( (0x00ff00&exp_val)>>8);
	explow  = (unsigned char) (0x0000f0&exp_val) ;
	
	sensor_write(sd, 0x3502, explow);
	sensor_write(sd, 0x3501, expmid);
	sensor_write(sd, 0x3500, exphigh);
//	printk("9732 sensor_set_exp = %d Done!\n", exp_val );

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
		
	gainlow=(unsigned char)(gain_val&0xff);
	gainhigh=(unsigned char)((gain_val>>8)&0x3);
	
	sensor_write(sd, 0x350b, gainlow);
	sensor_write(sd, 0x350a, gainhigh);
	
	//printk("ov5647_mipi sensor_set_gain = %d, Done!\n", gain_val);
	info->gain = gain_val;
	
	return 0;
}

static int ov9732_sensor_vts;
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
  if(shutter > ov9732_sensor_vts- 4)
		frame_length = shutter + 4;
  else
		frame_length = ov9732_sensor_vts;

  sensor_write(sd,0x3212,0x00);
  sensor_write(sd, 0x380f, (frame_length & 0xff));
  sensor_write(sd, 0x380e, (frame_length >> 8));
  sensor_s_exp(sd,exp_val);
  sensor_s_gain(sd,gain_val);
  sensor_write(sd,0x3212,0x10);
  sensor_write(sd,0x3212,0xa0);
  
 // printk("9732 sensor_set_gain gain= %d,exp= %d Done!\n", gain_val,exp_val);

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
			vfe_gpio_write(sd,PWDN,CSI_GPIO_LOW);
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
			vfe_gpio_write(sd,PWDN,CSI_GPIO_HIGH);
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
			
			vfe_gpio_write(sd,RESET,CSI_GPIO_LOW);
			vfe_gpio_write(sd,PWDN,CSI_GPIO_LOW);
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
			//reset on io
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
	unsigned char rdval = 0;
	vfe_dev_dbg("sensor_detect_1--!\n");
  
	LOG_ERR_RET(sensor_read(sd, 0x300A, &rdval));
	vfe_dev_dbg("sensor_detect_2--!\n");

	if(rdval != (V4L2_IDENT_SENSOR>>8))
	{
		printk(KERN_DEBUG"*********sensor error,read id is %d.\n",rdval);
		return -ENODEV;
	}

	LOG_ERR_RET(sensor_read(sd, 0x300B, &rdval))
	if(rdval != (V4L2_IDENT_SENSOR&0x00ff))
	{
		printk(KERN_DEBUG"*********sensor error,read id is %d.\n",rdval);
		return -ENODEV;
	}
	else
	{
		printk(KERN_DEBUG"*********find ov9732 raw data camera sensor now.\n");
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
		.mbus_code	= V4L2_MBUS_FMT_SBGGR10_10X1,
		.regs 		= sensor_fmt_raw,
		.regs_size  = ARRAY_SIZE(sensor_fmt_raw),
		.bpp		= 1
	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */
static struct sensor_win_size sensor_win_sizes[] = {
    /* qsxga: 2304*1296 */  
    {
        .width      = HD720_WIDTH,
        .height     = HD720_HEIGHT,
        .hoffset    = 0,
        .voffset    = 0,
        .hts        = 1500,
        .vts        = 854,
        .pclk       = 38400000,
        .mipi_bps	= 360*1000*1000,
        .fps_fixed  = 1,
        .bin_factor = 1,
        .intg_min   = 1<<4,
        .intg_max   = (854-4)<<4,//
        .gain_min   = 1<<4,
        .gain_max   = (16<<4)-1,
        .regs       = sensor_default_regs,
        .regs_size  = ARRAY_SIZE(sensor_default_regs),
        .set_size   = NULL,
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
    cfg->type = V4L2_MBUS_CSI2;
  cfg->flags = 0|V4L2_MBUS_CSI2_1_LANE|V4L2_MBUS_CSI2_CHANNEL_0;
  
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
        LOG_ERR_RET(sensor_write_array(sd, wsize->regs, wsize->regs_size))
    if (wsize->set_size)
        LOG_ERR_RET(wsize->set_size(sd))

    info->fmt = sensor_fmt;
    info->width = wsize->width;
    info->height = wsize->height;
    ov9732_sensor_vts = wsize->vts;
    // show_regs_array(sd,sensor_1080p_regs);

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
    struct v4l2_subdev * sd;

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
