/*
 * A V4L2 driver for ov8865 cameras.
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


MODULE_AUTHOR("clarkyy");
MODULE_DESCRIPTION("A low-level driver for ov8865 sensors");
MODULE_LICENSE("GPL");

//for internel driver debug
#define DEV_DBG_EN      1
#if(DEV_DBG_EN == 1)    
#define vfe_dev_dbg(x,arg...) printk("[ov8865_4lane]"x,##arg)
#else
#define vfe_dev_dbg(x,arg...) 
#endif
#define vfe_dev_err(x,arg...) printk("[ov8865_4lane]"x,##arg)
#define vfe_dev_print(x,arg...) printk("[ov8865_4lane]"x,##arg)

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
#define V4L2_IDENT_SENSOR 0x8865
int ov8865_sensor_vts;
//#define QSXGA_12FPS

//define the voltage level of control signal
#define CSI_STBY_ON     0
#define CSI_STBY_OFF    1
#define CSI_RST_ON      0
#define CSI_RST_OFF     1
//#define CSI_PWR_ON      1
//#define CSI_PWR_OFF     0
//#define CSI_AF_PWR_ON   1
//#define CSI_AF_PWR_OFF  0

#define regval_list reg_list_a16_d8


#define REG_TERM 0xfffe
#define VAL_TERM 0xfe
#define REG_DLY  0xffff

/*
 * Our nominal (default) frame rate.
 */

#define SENSOR_FRAME_RATE 30


/*
 * The ov8865 sits on i2c with ID 0x6c
 */
#define I2C_ADDR 0x6c
#define SENSOR_NAME "ov8865_4lane"
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
    // default_4Lane_30fps_720Mbps/lane
    { 0x0103, 0x01 },  //; software reset
    { REG_DLY, 0x10 }, //; delay
    { 0x0100, 0x00 },  //; software standby
    { 0x0100, 0x00 },  
    { 0x0100, 0x00 },  
    { 0x0100, 0x00 },  
    { 0x3638, 0xff },  //; analog control
    { 0x0302, 0x1e },  //; PLL
    { 0x0303, 0x00 },  //; PLL
    { 0x0304, 0x03 },  //; PLL
    { 0x030e, 0x00 },  //; PLL
    { 0x030f, 0x09 },  //; PLL
    { 0x0312, 0x01 },  //; PLL
    { 0x031e, 0x0c },  //; PLL
    { 0x3015, 0x01 },  //; clock Div
    { 0x3018, 0x72 },  //; MIPI 4 lane
    { 0x3020, 0x93 },  //; clock normal, pclk/1
    { 0x3022, 0x01 },  //; pd_mini enable when rst_sync
    { 0x3031, 0x0a },  //; 10-bit
    { 0x3106, 0x01 },  //; PLL
    { 0x3305, 0xf1 },  
    { 0x3308, 0x00 },  
    { 0x3309, 0x28 },  
    { 0x330a, 0x00 },  
    { 0x330b, 0x20 },  
    { 0x330c, 0x00 },  
    { 0x330d, 0x00 },  
    { 0x330e, 0x00 },  
    { 0x330f, 0x40 },  
    { 0x3307, 0x04 },  
    { 0x3604, 0x04 },  //; analog control
    { 0x3602, 0x30 },  
    { 0x3605, 0x00 },  
    { 0x3607, 0x20 },  
    { 0x3608, 0x11 },  
    { 0x3609, 0x68 },  
    { 0x360a, 0x40 },  
    { 0x360c, 0xdd },  
    { 0x360e, 0x0c },  
    { 0x3610, 0x07 },  
    { 0x3612, 0x86 },  
    { 0x3613, 0x58 },  
    { 0x3614, 0x28 },  
    { 0x3617, 0x40 },  
    { 0x3618, 0x5a },  
    { 0x3619, 0x9b },  
    { 0x361c, 0x00 },  
    { 0x361d, 0x60 },  
    { 0x3631, 0x60 },  
    { 0x3633, 0x10 },  
    { 0x3634, 0x10 },  
    { 0x3635, 0x10 },  
    { 0x3636, 0x10 },  
    { 0x3641, 0x55 },  //; MIPI settings
    { 0x3646, 0x86 },  //; MIPI settings
    { 0x3647, 0x27 },  //; MIPI settings
    { 0x364a, 0x1b },  //; MIPI settings
    { 0x3500, 0x00 },  //; exposurre HH
    { 0x3501, 0x4c },  //; expouere H
    { 0x3502, 0x00 },  //; exposure L
    { 0x3503, 0x00 },  //; gain no delay, exposure no delay
    { 0x3508, 0x02 },  //; gain H
    { 0x3509, 0x00 },  //; gain L
    { 0x3700, 0x24 },  //; sensor control
    { 0x3701, 0x0c },  
    { 0x3702, 0x28 },  
    { 0x3703, 0x19 },  
    { 0x3704, 0x14 },  
    { 0x3705, 0x00 },  
    { 0x3706, 0x38 },  
    { 0x3707, 0x04 },  
    { 0x3708, 0x24 },  
    { 0x3709, 0x40 },  
    { 0x370a, 0x00 },  
    { 0x370b, 0xb8 },  
    { 0x370c, 0x04 },  
    { 0x3718, 0x12 },  
    { 0x3719, 0x31 },  
    { 0x3712, 0x42 },  
    { 0x3714, 0x12 },  
    { 0x371e, 0x19 },  
    { 0x371f, 0x40 },  
    { 0x3720, 0x05 },  
    { 0x3721, 0x05 },  
    { 0x3724, 0x02 },  
    { 0x3725, 0x02 },  
    { 0x3726, 0x06 },  
    { 0x3728, 0x05 },  
    { 0x3729, 0x02 },  
    { 0x372a, 0x03 },  
    { 0x372b, 0x53 },  
    { 0x372c, 0xa3 },  
    { 0x372d, 0x53 },  
    { 0x372e, 0x06 },  
    { 0x372f, 0x10 },  
    { 0x3730, 0x01 },  
    { 0x3731, 0x06 },  
    { 0x3732, 0x14 },  
    { 0x3733, 0x10 },  
    { 0x3734, 0x40 },  
    { 0x3736, 0x20 },  
    { 0x373a, 0x02 },  
    { 0x373b, 0x0c },  
    { 0x373c, 0x0a },  
    { 0x373e, 0x03 },  
    { 0x3755, 0x40 },  
    { 0x3758, 0x00 },  
    { 0x3759, 0x4c },  
    { 0x375a, 0x06 },  
    { 0x375b, 0x13 },  
    { 0x375c, 0x40 },  
    { 0x375d, 0x02 },  
    { 0x375e, 0x00 },  
    { 0x375f, 0x14 },  
    { 0x3767, 0x1c },  
    { 0x3768, 0x04 },  
    { 0x3769, 0x20 },  
    { 0x376c, 0xc0 },  
    { 0x376d, 0xc0 },  
    { 0x376a, 0x08 },  
    { 0x3761, 0x00 },  
    { 0x3762, 0x00 },  
    { 0x3763, 0x00 },  
    { 0x3766, 0xff },  
    { 0x376b, 0x42 },  
    { 0x3772, 0x23 },  
    { 0x3773, 0x02 },  
    { 0x3774, 0x16 },  
    { 0x3775, 0x12 },  
    { 0x3776, 0x08 },  
    { 0x37a0, 0x44 },  
    { 0x37a1, 0x3d },  
    { 0x37a2, 0x3d },  
    { 0x37a3, 0x01 },  
    { 0x37a4, 0x00 },  
    { 0x37a5, 0x08 },  
    { 0x37a6, 0x00 },  
    { 0x37a7, 0x44 },  
    { 0x37a8, 0x58 },  
    { 0x37a9, 0x58 },  
    { 0x3760, 0x00 },  
    { 0x376f, 0x01 },  
    { 0x37aa, 0x44 },  
    { 0x37ab, 0x2e },  
    { 0x37ac, 0x2e },  
    { 0x37ad, 0x33 },  
    { 0x37ae, 0x0d },  
    { 0x37af, 0x0d },  
    { 0x37b0, 0x00 },  
    { 0x37b1, 0x00 },  
    { 0x37b2, 0x00 },  
    { 0x37b3, 0x42 },  
    { 0x37b4, 0x42 },  
    { 0x37b5, 0x33 },  
    { 0x37b6, 0x00 },  
    { 0x37b7, 0x00 },  
    { 0x37b8, 0x00 },  
    { 0x37b9, 0xff },  //; sensor control
    { 0x3800, 0x00 },  //; X start H
    { 0x3801, 0x0c },  //; X start L
    { 0x3802, 0x00 },  //; Y start H
    { 0x3803, 0x0c },  //; Y start L
    { 0x3804, 0x0c },  //; X end H
    { 0x3805, 0xd3 },  //; X end L
    { 0x3806, 0x09 },  //; Y end H
    { 0x3807, 0xa3 },  //; Y end L
    { 0x3808, 0x06 },  //; X output size H
    { 0x3809, 0x60 },  //; X output size L
    { 0x380a, 0x04 },  //; Y output size H
    { 0x380b, 0xc8 },  //; Y output size L
    { 0x380c, 0x07 },  //; HTS H
    { 0x380d, 0x83 },  //; HTS L
    { 0x380e, 0x04 },  //; VTS H
    { 0x380f, 0xe0 },  //; VTS L
    { 0x3810, 0x00 },  //; ISP X win H
    { 0x3811, 0x04 },  //; ISP X win L
    { 0x3813, 0x04 },  //; ISP Y win L
    { 0x3814, 0x03 },  //; X inc odd
    { 0x3815, 0x01 },  //; X inc even
    { 0x3820, 0x00 },  //; flip off
    { 0x3821, 0x67 },  //; hsync_en_o, fst_vbin, mirror on
    { 0x382a, 0x03 },  //; Y inc odd
    { 0x382b, 0x01 },  //; Y inc even
    { 0x3830, 0x08 },  //; ablc_use_num[5:1]
    { 0x3836, 0x02 },  //; zline_use_num[5:1]
    { 0x3837, 0x18 },  //; vts_add_dis, cexp_gt_vts_offs=8
    { 0x3841, 0xff },  //; auto size
    { 0x3846, 0x88 },  //; Y/X boundary pixel numbber for auto size mode
    { 0x3d85, 0x06 },  //; OTP power up load data enable, OTP power up load setting enable
    { 0x3d8c, 0x75 },  //; OTP setting start address H
    { 0x3d8d, 0xef },  //; OTP setting start address L
    { 0x3f08, 0x0b },  
    { 0x4000, 0xf1 },  //; our range trig en, format chg en, gan chg en, exp chg en, median en
    { 0x4001, 0x14 },  //; left 32 column, final BLC offset limitation enable
    { 0x4005, 0x10 },  //; BLC target
    { 0x400b, 0x0c },  //; start line =0, offset limitation en, cut range function en
    { 0x400d, 0x10 },  //; offset trigger threshold
    { 0x401b, 0x00 },  
    { 0x401d, 0x00 },  
    { 0x4020, 0x01 },  //; anchor left start H
    { 0x4021, 0x20 },  //; anchor left start L
    { 0x4022, 0x01 },  //; anchor left end H
    { 0x4023, 0x9f },  //; anchor left end L
    { 0x4024, 0x03 },  //; anchor right start H
    { 0x4025, 0xe0 },  //; anchor right start L
    { 0x4026, 0x04 },  //; anchor right end H
    { 0x4027, 0x5f },  //; anchor right end L
    { 0x4028, 0x00 },  //; top zero line start
    { 0x4029, 0x02 },  //; top zero line number
    { 0x402a, 0x04 },  //; top black line start
    { 0x402b, 0x04 },  //; top black line number
    { 0x402c, 0x02 },  //; bottom zero line start
    { 0x402d, 0x02 },  //; bottom zero line number
    { 0x402e, 0x08 },  //; bottom black line start
    { 0x402f, 0x02 },  //; bottom black line number
    { 0x401f, 0x00 },  //; anchor one disable
    { 0x4034, 0x3f },  //; limitation BLC offset
    { 0x4300, 0xff },  //; clip max H
    { 0x4301, 0x00 },  //; clip min H
    { 0x4302, 0x0f },  //; clip min L/clip max L
    { 0x4500, 0x40 },  //; ADC sync control
    { 0x4503, 0x10 },  
    { 0x4601, 0x74 },  //; V FIFO control
    { 0x481f, 0x32 },  //; clk_prepare_min
    { 0x4837, 0x16 },  //; clock period
    { 0x4850, 0x10 },  //; lane select
    { 0x4851, 0x32 },  //; lane select
    { 0x4b00, 0x2a },  //; LVDS settings
    { 0x4b0d, 0x00 },  //; LVDS settings
    { 0x4d00, 0x04 },  //; temperature sensor
    { 0x4d01, 0x18 },  //; temperature sensor
    { 0x4d02, 0xc3 },  //; temperature sensor
    { 0x4d03, 0xff },  //; temperature sensor
    { 0x4d04, 0xff },  //; temperature sensor
    { 0x4d05, 0xff },  //; temperature sensor
    { 0x5000, 0x96 },  //; LENC on, MWB on, BPC on, WPC on
    { 0x5001, 0x01 },  //; BLC on
    { 0x5002, 0x08 },  //; vario pixel off
    { 0x5901, 0x00 },  
    { 0x5e00, 0x00 },  //; test pattern off
    { 0x5e01, 0x41 },  //; window cut enable
    { 0x0100, 0x01 },  //; wake up, streaming
    { 0x5b00, 0x02 },  //; OTP DPC start address H
    { 0x5b01, 0xd0 },  //; OTP DPC start address L
    { 0x5b02, 0x03 },  //; OTP DPC end address H
    { 0x5b03, 0xff },  //; OTP DPC end address L
    { 0x5b05, 0x6c },  //; Recover method 11, use 0x3ff to recover cluster, flip option enable
    { 0x5780, 0xfc },  //; DPC
    { 0x5781, 0xdf },  //;
    { 0x5782, 0x3f },  //;
    { 0x5783, 0x08 },  //;
    { 0x5784, 0x0c },  //;
    { 0x5786, 0x20 },  //;
    { 0x5787, 0x40 },  //;
    { 0x5788, 0x08 },  //;
    { 0x5789, 0x08 },  //;
    { 0x578a, 0x02 },  //;
    { 0x578b, 0x01 },  //;
    { 0x578c, 0x01 },  //;
    { 0x578d, 0x0c },  //;
    { 0x578e, 0x02 },  //;
    { 0x578f, 0x01 },  //;
    { 0x5790, 0x01 },  //; DPC
    { 0x5800, 0x1d },  //; lens correction
    { 0x5801, 0x0e },  
    { 0x5802, 0x0c },  
    { 0x5803, 0x0c },  
    { 0x5804, 0x0f },  
    { 0x5805, 0x22 },  
    { 0x5806, 0x0a },  
    { 0x5807, 0x06 },  
    { 0x5808, 0x05 },  
    { 0x5809, 0x05 },  
    { 0x580a, 0x07 },  
    { 0x580b, 0x0a },  
    { 0x580c, 0x06 },  
    { 0x580d, 0x02 },  
    { 0x580e, 0x00 },  
    { 0x580f, 0x00 },  
    { 0x5810, 0x03 },  
    { 0x5811, 0x07 },  
    { 0x5812, 0x06 },  
    { 0x5813, 0x02 },  
    { 0x5814, 0x00 },  
    { 0x5815, 0x00 },  
    { 0x5816, 0x03 },  
    { 0x5817, 0x07 },  
    { 0x5818, 0x09 },  
    { 0x5819, 0x06 },  
    { 0x581a, 0x04 },  
    { 0x581b, 0x04 },  
    { 0x581c, 0x06 },  
    { 0x581d, 0x0a },  
    { 0x581e, 0x19 },  
    { 0x581f, 0x0d },  
    { 0x5820, 0x0b },  
    { 0x5821, 0x0b },  
    { 0x5822, 0x0e },  
    { 0x5823, 0x22 },  
    { 0x5824, 0x23 },  
    { 0x5825, 0x28 },  
    { 0x5826, 0x29 },  
    { 0x5827, 0x27 },  
    { 0x5828, 0x13 },  
    { 0x5829, 0x26 },  
    { 0x582a, 0x33 },  
    { 0x582b, 0x32 },  
    { 0x582c, 0x33 },  
    { 0x582d, 0x16 },  
    { 0x582e, 0x14 },  
    { 0x582f, 0x30 },  
    { 0x5830, 0x31 },  
    { 0x5831, 0x30 },  
    { 0x5832, 0x15 },  
    { 0x5833, 0x26 },  
    { 0x5834, 0x23 },  
    { 0x5835, 0x21 },  
    { 0x5836, 0x23 },  
    { 0x5837, 0x05 },  
    { 0x5838, 0x36 },  
    { 0x5839, 0x27 },  
    { 0x583a, 0x28 },  
    { 0x583b, 0x26 },  
    { 0x583c, 0x24 },  
    { 0x583d, 0xdf },  //; lens correction
	{ 0x0100, 0x01 }   // stream on	
};

static struct regval_list sensor_quxga_regs[] = {
	//@@3264_2448_4lane_30fps_720Mbps/lane
    { 0x0100, 0x00 },  //; software standby
	{ REG_DLY, 0x05 }, //must delay
    { 0x030f, 0x04 },  //; PLL
    { 0x3018, 0x72 },  //
    { 0x3106, 0x01 },  //
    { 0x3501, 0x98 },  //; expouere H
    { 0x3502, 0x60 },  //; exposure L
    { 0x3700, 0x48 },  //; sensor control
    { 0x3701, 0x18 },  //
    { 0x3702, 0x50 },  //
    { 0x3703, 0x32 },  //
    { 0x3704, 0x28 },  //
    { 0x3706, 0x70 },  //
    { 0x3707, 0x08 },  //
    { 0x3708, 0x48 },  //
    { 0x3709, 0x80 },  //
    { 0x370a, 0x01 },  //
    { 0x370b, 0x70 },  //
    { 0x370c, 0x07 },  //
    { 0x3718, 0x14 },  //
    { 0x3712, 0x44 },  //
    { 0x371e, 0x31 },  //
    { 0x371f, 0x7f },  //
    { 0x3720, 0x0a },  //
    { 0x3721, 0x0a },  //
    { 0x3724, 0x04 },  //
    { 0x3725, 0x04 },  //
    { 0x3726, 0x0c },  //
    { 0x3728, 0x0a },  //
    { 0x3729, 0x03 },  //
    { 0x372a, 0x06 },  //
    { 0x372b, 0xa6 },  //
    { 0x372c, 0xa6 },  //
    { 0x372d, 0xa6 },  //
    { 0x372e, 0x0c },  //
    { 0x372f, 0x20 },  //
    { 0x3730, 0x02 },  //
    { 0x3731, 0x0c },  //
    { 0x3732, 0x28 },  //
    { 0x3736, 0x30 },  //
    { 0x373a, 0x04 },  //
    { 0x373b, 0x18 },  //
    { 0x373c, 0x14 },  //
    { 0x373e, 0x06 },  //
    { 0x375a, 0x0c },  //
    { 0x375b, 0x26 },  //
    { 0x375d, 0x04 },  //
    { 0x375f, 0x28 },  //
    { 0x3767, 0x1e },  //
    { 0x3772, 0x46 },  //
    { 0x3773, 0x04 },  //
    { 0x3774, 0x2c },  //
    { 0x3775, 0x13 },  //
    { 0x3776, 0x10 },  //
    { 0x37a0, 0x88 },  //
    { 0x37a1, 0x7a },  //
    { 0x37a2, 0x7a },  //
    { 0x37a3, 0x02 },  //
    { 0x37a5, 0x09 },  //
    { 0x37a7, 0x88 },  //
    { 0x37a8, 0xb0 },  //
    { 0x37a9, 0xb0 },  //
    { 0x37aa, 0x88 },  //
    { 0x37ab, 0x5c },  //
    { 0x37ac, 0x5c },  //
    { 0x37ad, 0x55 },  //
    { 0x37ae, 0x19 },  //
    { 0x37af, 0x19 },  //
    { 0x37b3, 0x84 },  //
    { 0x37b4, 0x84 },  //
    { 0x37b5, 0x66 },  //
    { 0x3808, 0x0c },  //; X output size H
    { 0x3809, 0xc0 },  //; X output size L
    { 0x380a, 0x09 },  //; Y output size H
    { 0x380b, 0x90 },  //; Y output size L
    { 0x380c, 0x07 },  //; HTS H
    { 0x380d, 0x98 },  //; HTS L
    { 0x380e, 0x09 },  //; VTS H
    { 0x380f, 0xa6 },  //; VTS L
    { 0x3813, 0x02 },  //; ISP Y win L
    { 0x3814, 0x01 },  //; X inc odd
    { 0x3821, 0x46 },  //; hsync_en_o, fst_vbin, mirror on
    { 0x382a, 0x01 },  //; Y inc odd
    { 0x382b, 0x01 },  //; Y inc even
    { 0x3830, 0x04 },  //; ablc_use_num[5:1]
    { 0x3836, 0x01 },  //; zline_use_num[5:1]
    { 0x3846, 0x48 },  //; Y/X boundary pixel numbber for auto size mode
    { 0x3f08, 0x16 },  //
    { 0x4000, 0xf1 },  //; our range trig en, format chg en, gan chg en, exp chg en, median en
    { 0x4001, 0x04 },  //; left 32 column, final BLC offset limitation enable
    { 0x4020, 0x02 },  //; anchor left start H
    { 0x4021, 0x40 },  //; anchor left start L
    { 0x4022, 0x03 },  //; anchor left end H
    { 0x4023, 0x3f },  //; anchor left end L
    { 0x4024, 0x07 },  //; anchor right start H
    { 0x4025, 0xc0 },  //; anchor right start L
    { 0x4026, 0x08 },  //; anchor right end H
    { 0x4027, 0xbf },  //; anchor right end L
    { 0x402a, 0x04 },  //; top black line start
    { 0x402b, 0x04 },  //; top black line number
    { 0x402c, 0x02 },  //; bottom zero line start
    { 0x402d, 0x02 },  //; bottom zero line number
    { 0x402e, 0x08 },  //; bottom black line start
    { 0x4500, 0x68 },  //; ADC sync control
    { 0x4601, 0x10 },  //; V FIFO control
    { 0x5002, 0x08 },  //; vario pixel off
    { 0x5901, 0x00 },  //
    { 0x0100, 0x01 },  //; wake up, streaming
};

static struct regval_list sensor_6M_regs[] = { 
	//@@3264_1836_4Lane_30fps_720Mbps/lane
    { 0x0100, 0x00 },  //; software standby
	{ REG_DLY, 0x05 }, //must delay
    { 0x030f, 0x04 },  //; PLL
    { 0x3018, 0x72 },  //
    { 0x3106, 0x01 },  //
    { 0x3501, 0x72 },  //; expouere H
    { 0x3502, 0x20 },  //; exposure L
    { 0x3700, 0x48 },  //; sensor control
    { 0x3701, 0x18 },  //
    { 0x3702, 0x50 },  //
    { 0x3703, 0x32 },  //
    { 0x3704, 0x28 },  //
    { 0x3706, 0x70 },  //
    { 0x3707, 0x08 },  //
    { 0x3708, 0x48 },  //
    { 0x3709, 0x80 },  //
    { 0x370a, 0x01 },  //
    { 0x370b, 0x70 },  //
    { 0x370c, 0x07 },  //
    { 0x3718, 0x14 },  //
    { 0x3712, 0x44 },  //
    { 0x371e, 0x31 },  //
    { 0x371f, 0x7f },  //
    { 0x3720, 0x0a },  //
    { 0x3721, 0x0a },  //
    { 0x3724, 0x04 },  //
    { 0x3725, 0x04 },  //
    { 0x3726, 0x0c },  //
    { 0x3728, 0x0a },  //
    { 0x3729, 0x03 },  //
    { 0x372a, 0x06 },  //
    { 0x372b, 0xa6 },  //
    { 0x372c, 0xa6 },  //
    { 0x372d, 0xa6 },  //
    { 0x372e, 0x0c },  //
    { 0x372f, 0x20 },  //
    { 0x3730, 0x02 },  //
    { 0x3731, 0x0c },  //
    { 0x3732, 0x28 },  //
    { 0x3736, 0x30 },  //
    { 0x373a, 0x04 },  //
    { 0x373b, 0x18 },  //
    { 0x373c, 0x14 },  //
    { 0x373e, 0x06 },  //
    { 0x375a, 0x0c },  //
    { 0x375b, 0x26 },  //
    { 0x375d, 0x04 },  //
    { 0x375f, 0x28 },  //
    { 0x3767, 0x1e },  //
    { 0x3772, 0x46 },  //
    { 0x3773, 0x04 },  //
    { 0x3774, 0x2c },  //
    { 0x3775, 0x13 },  //
    { 0x3776, 0x10 },  //
    { 0x37a0, 0x88 },  //
    { 0x37a1, 0x7a },  //
    { 0x37a2, 0x7a },  //
    { 0x37a3, 0x02 },  //
    { 0x37a5, 0x09 },  //
    { 0x37a7, 0x88 },  //
    { 0x37a8, 0xb0 },  //
    { 0x37a9, 0xb0 },  //
    { 0x37aa, 0x88 },  //
    { 0x37ab, 0x5c },  //
    { 0x37ac, 0x5c },  //
    { 0x37ad, 0x55 },  //
    { 0x37ae, 0x19 },  //
    { 0x37af, 0x19 },  //
    { 0x37b3, 0x84 },  //
    { 0x37b4, 0x84 },  //
    { 0x37b5, 0x66 },  //
    { 0x3808, 0x0c },  //; X output size H
    { 0x3809, 0xc0 },  //; X output size L
    { 0x380a, 0x07 },  //; Y output size H
    { 0x380b, 0x2c },  //; Y output size L
    { 0x380c, 0x0a },  //; HTS H
    { 0x380d, 0x16 },  //; HTS L
    { 0x380e, 0x07 },  //; VTS H
    { 0x380f, 0x42 },  //; VTS L
    { 0x3813, 0x02 },  //; ISP Y win L
    { 0x3814, 0x01 },  //; X inc odd
    { 0x3821, 0x46 },  //; hsync_en_o, fst_vbin, mirror on
    { 0x382a, 0x01 },  //; Y inc odd
    { 0x382b, 0x01 },  //; Y inc even
    { 0x3830, 0x04 },  //; ablc_use_num[5:1]
    { 0x3836, 0x01 },  //; zline_use_num[5:1]
    { 0x3846, 0x48 },  //; Y/X boundary pixel numbber for auto size mode
    { 0x3f08, 0x16 },  //
    { 0x4000, 0xf1 },  //; our range trig en, format chg en, gan chg en, exp chg en, median en
    { 0x4001, 0x04 },  //; left 32 column, final BLC offset limitation enable
    { 0x4020, 0x02 },  //; anchor left start H
    { 0x4021, 0x40 },  //; anchor left start L
    { 0x4022, 0x03 },  //; anchor left end H
    { 0x4023, 0x3f },  //; anchor left end L
    { 0x4024, 0x07 },  //; anchor right start H
    { 0x4025, 0xc0 },  //; anchor right start L
    { 0x4026, 0x08 },  //; anchor right end H
    { 0x4027, 0xbf },  //; anchor right end L
    { 0x402a, 0x04 },  //; top black line start
    { 0x402b, 0x04 },  //; top black line number
    { 0x402c, 0x02 },  //; bottom zero line start
    { 0x402d, 0x02 },  //; bottom zero line number
    { 0x402e, 0x08 },  //; bottom black line start
    { 0x4500, 0x68 },  //; ADC sync control
    { 0x4601, 0x10 },  //; V FIFO control
    { 0x5002, 0x08 },  //; vario pixel off
    { 0x5901, 0x00 },  //
    { 0x0100, 0x01 },  //; wake up, streaming
};


static struct regval_list sensor_uxga_regs[] = {
	//@@1632_1224_4Lane_30fps_720Mbps/lane
    { 0x0100, 0x00 },  //; software standby
	{ REG_DLY, 0x05 }, //must delay
    { 0x030f, 0x09 },  //; PLL
    { 0x3018, 0x72 },  //
    { 0x3106, 0x01 },  //
    { 0x3501, 0x4c },  //; expouere H
    { 0x3502, 0x00 },  //; exposure L
    { 0x3700, 0x24 },  //; sensor control
    { 0x3701, 0x0c },  //
    { 0x3702, 0x28 },  //
    { 0x3703, 0x19 },  //
    { 0x3704, 0x14 },  //
    { 0x3706, 0x38 },  //
    { 0x3707, 0x04 },  //
    { 0x3708, 0x24 },  //
    { 0x3709, 0x40 },  //
    { 0x370a, 0x00 },  //
    { 0x370b, 0xb8 },  //
    { 0x370c, 0x04 },  //
    { 0x3718, 0x12 },  //
    { 0x3712, 0x42 },  //
    { 0x371e, 0x19 },  //
    { 0x371f, 0x40 },  //
    { 0x3720, 0x05 },  //
    { 0x3721, 0x05 },  //
    { 0x3724, 0x02 },  //
    { 0x3725, 0x02 },  //
    { 0x3726, 0x06 },  //
    { 0x3728, 0x05 },  //
    { 0x3729, 0x02 },  //
    { 0x372a, 0x03 },  //
    { 0x372b, 0x53 },  //
    { 0x372c, 0xa3 },  //
    { 0x372d, 0x53 },  //
    { 0x372e, 0x06 },  //
    { 0x372f, 0x10 },  //
    { 0x3730, 0x01 },  //
    { 0x3731, 0x06 },  //
    { 0x3732, 0x14 },  //
    { 0x3736, 0x20 },  //
    { 0x373a, 0x02 },  //
    { 0x373b, 0x0c },  //
    { 0x373c, 0x0a },  //
    { 0x373e, 0x03 },  //
    { 0x375a, 0x06 },  //
    { 0x375b, 0x13 },  //
    { 0x375d, 0x02 },  //
    { 0x375f, 0x14 },  //
    { 0x3767, 0x1c },  //
    { 0x3772, 0x23 },  //
    { 0x3773, 0x02 },  //
    { 0x3774, 0x16 },  //
    { 0x3775, 0x12 },  //
    { 0x3776, 0x08 },  //
    { 0x37a0, 0x44 },  //
    { 0x37a1, 0x3d },  //
    { 0x37a2, 0x3d },  //
    { 0x37a3, 0x01 },  //
    { 0x37a5, 0x08 },  //
    { 0x37a7, 0x44 },  //
    { 0x37a8, 0x58 },  //
    { 0x37a9, 0x58 },  //
    { 0x37aa, 0x44 },  //
    { 0x37ab, 0x2e },  //
    { 0x37ac, 0x2e },  //
    { 0x37ad, 0x33 },  //
    { 0x37ae, 0x0d },  //
    { 0x37af, 0x0d },  //
    { 0x37b3, 0x42 },  //
    { 0x37b4, 0x42 },  //
    { 0x37b5, 0x33 },  //
    { 0x3808, 0x06 },  //; X output size H
    { 0x3809, 0x60 },  //; X output size L
    { 0x380a, 0x04 },  //; Y output size H
    { 0x380b, 0xc8 },  //; Y output size L
    { 0x380c, 0x07 },  //; HTS H
    { 0x380d, 0x83 },  //; HTS L
    { 0x380e, 0x04 },  //; VTS H
    { 0x380f, 0xe0 },  //; VTS L
    { 0x3813, 0x04 },  //; ISP Y win L
    { 0x3814, 0x03 },  //; X inc odd
    { 0x3821, 0x67 },  //; hsync_en_o, fst_vbin, mirror on
    { 0x382a, 0x03 },  //; Y inc odd
    { 0x382b, 0x01 },  //; Y inc even
    { 0x3830, 0x08 },  //; ablc_use_num[5:1]
    { 0x3836, 0x02 },  //; zline_use_num[5:1]
    { 0x3846, 0x88 },  //; Y/X boundary pixel numbber for auto size mode
    { 0x3f08, 0x0b },  //
    { 0x4000, 0xf1 },  //; our range trig en, format chg en, gan chg en, exp chg en, median en
    { 0x4001, 0x14 },  //; left 32 column, final BLC offset limitation enable
    { 0x4020, 0x01 },  //; anchor left start H
    { 0x4021, 0x20 },  //; anchor left start L
    { 0x4022, 0x01 },  //; anchor left end H
    { 0x4023, 0x9f },  //; anchor left end L
    { 0x4024, 0x03 },  //; anchor right start H
    { 0x4025, 0xe0 },  //; anchor right start L
    { 0x4026, 0x04 },  //; anchor right end H
    { 0x4027, 0x5f },  //; anchor right end L
    { 0x402a, 0x04 },  //; top black line start
    { 0x402b, 0x04 },  //; top black line number
    { 0x402c, 0x02 },  //; bottom zero line start
    { 0x402d, 0x02 },  //; bottom zero line number
    { 0x402e, 0x08 },  //; bottom black line start
    { 0x4500, 0x40 },  //; ADC sync control
    { 0x4601, 0x74 },  //; V FIFO control
    { 0x5002, 0x08 },  //; vario pixel off
    { 0x5901, 0x00 },  //
    { 0x0100, 0x01 },  //; wake up, streaming
};

static struct regval_list sensor_svga_90fps_regs[] = {
	//@@800_600_4Lane_90fps_720Mbps/lane
    { 0x0100, 0x00 },   //; software standby
	{ REG_DLY, 0x05 }, //must delay
    { 0x030f, 0x09 },   //; PLL
    { 0x3018, 0x72 },   //
    { 0x3106, 0x01 },   //
    { 0x3501, 0x26 },   //; expouere H
    { 0x3502, 0x00 },   //; exposure L
    { 0x3700, 0x24 },   //; sensor control
    { 0x3701, 0x0c },   //
    { 0x3702, 0x28 },   //
    { 0x3703, 0x19 },   //
    { 0x3704, 0x14 },   //
    { 0x3706, 0x38 },   //
    { 0x3707, 0x04 },   //
    { 0x3708, 0x24 },   //
    { 0x3709, 0x40 },   //
    { 0x370a, 0x00 },   //
    { 0x370b, 0xb8 },   //
    { 0x370c, 0x04 },   //
    { 0x3718, 0x12 },   //
    { 0x3712, 0x42 },   //
    { 0x371e, 0x19 },   //
    { 0x371f, 0x40 },   //
    { 0x3720, 0x05 },   //
    { 0x3721, 0x05 },   //
    { 0x3724, 0x02 },   //
    { 0x3725, 0x02 },   //
    { 0x3726, 0x06 },   //
    { 0x3728, 0x05 },   //
    { 0x3729, 0x02 },   //
    { 0x372a, 0x03 },   //
    { 0x372b, 0x53 },   //
    { 0x372c, 0xa3 },   //
    { 0x372d, 0x53 },   //
    { 0x372e, 0x06 },   //
    { 0x372f, 0x10 },   //
    { 0x3730, 0x01 },   //
    { 0x3731, 0x06 },   //
    { 0x3732, 0x14 },   //
    { 0x3736, 0x20 },   //
    { 0x373a, 0x02 },   //
    { 0x373b, 0x0c },   //
    { 0x373c, 0x0a },   //
    { 0x373e, 0x03 },   //
    { 0x375a, 0x06 },   //
    { 0x375b, 0x13 },   //
    { 0x375d, 0x02 },   //
    { 0x375f, 0x14 },   //
    { 0x3767, 0x18 },   //
    { 0x3772, 0x23 },   //
    { 0x3773, 0x02 },   //
    { 0x3774, 0x16 },   //
    { 0x3775, 0x12 },   //
    { 0x3776, 0x08 },   //
    { 0x37a0, 0x44 },   //
    { 0x37a1, 0x3d },   //
    { 0x37a2, 0x3d },   //
    { 0x37a3, 0x01 },   //
    { 0x37a5, 0x08 },   //
    { 0x37a7, 0x44 },   //
    { 0x37a8, 0x58 },   //
    { 0x37a9, 0x58 },   //
    { 0x37aa, 0x44 },   //
    { 0x37ab, 0x2e },   //
    { 0x37ac, 0x2e },   //
    { 0x37ad, 0x33 },   //
    { 0x37ae, 0x0d },   //
    { 0x37af, 0x0d },   //
    { 0x37b3, 0x42 },   //
    { 0x37b4, 0x42 },   //
    { 0x37b5, 0x33 },   //
    { 0x3808, 0x03 },   //; X output size H
    { 0x3809, 0x20 },   //; X output size L
    { 0x380a, 0x02 },   //; Y output size H
    { 0x380b, 0x58 },   //; Y output size L
    { 0x380c, 0x04 },   //; HTS H
    { 0x380d, 0xe2 },   //; HTS L
    { 0x380e, 0x02 },   //; VTS H
    { 0x380f, 0x80 },   //; VTS L
    { 0x3813, 0x04 },   //; ISP Y win L
    { 0x3814, 0x03 },   //; X inc odd
    { 0x3821, 0x6f },   //; hsync_en_o, fst_vbin, mirror on
    { 0x382a, 0x05 },   //; Y inc odd
    { 0x382b, 0x03 },   //; Y inc even
    { 0x3830, 0x08 },   //; ablc_use_num[5:1]
    { 0x3836, 0x02 },   //; zline_use_num[5:1]
    { 0x3846, 0x88 },   //; Y/X boundary pixel numbber for auto size mode
    { 0x3f08, 0x0b },   //
    { 0x4000, 0xf1 },   //; our range trig en, format chg en, gan chg en, exp chg en, median en
    { 0x4001, 0x14 },   //; left 32 column, final BLC offset limitation enable
    { 0x4020, 0x01 },   //; anchor left start H
    { 0x4021, 0x20 },   //; anchor left start L
    { 0x4022, 0x01 },   //; anchor left end H
    { 0x4023, 0x9f },   //; anchor left end L
    { 0x4024, 0x03 },   //; anchor right start H
    { 0x4025, 0xe0 },   //; anchor right start L
    { 0x4026, 0x04 },   //; anchor right end H
    { 0x4027, 0x5f },   //; anchor right end L
    { 0x402a, 0x02 },   //; top black line start
    { 0x402b, 0x02 },   //; top black line number
    { 0x402c, 0x00 },   //; bottom zero line start
    { 0x402d, 0x00 },   //; bottom zero line number
    { 0x402e, 0x04 },   //; bottom black line start
    { 0x4500, 0x40 },   //; ADC sync control
    { 0x4601, 0x50 },   //; V FIFO control
    { 0x5002, 0x0c },   //; vario pixel off
    { 0x5901, 0x04 },   //
    { 0x0100, 0x01 },   //; wake up, streaming
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

static int sensor_s_exp_gain(struct v4l2_subdev *sd, struct sensor_exp_gain *exp_gain)
{
	int exp_val, gain_val,frame_length,shutter;
	unsigned char explow=0,expmid=0,exphigh=0;
	unsigned char gainlow=0,gainhigh=0;  
	struct sensor_info *info = to_state(sd);

	exp_val = exp_gain->exp_val;
	gain_val = exp_gain->gain_val;

	//if((info->exp == exp_val)&&(info->gain == gain_val))
	//	return 0;
  
	//return -EINVAL;
	if(gain_val<1*16)
		gain_val=16;
	if(gain_val>64*16-1)
		gain_val=64*16-1;
  
	if(exp_val>0xfffff)
		exp_val=0xfffff;
  
	gain_val *= 8;
	gainlow=(unsigned char)(gain_val&0xff);
	gainhigh=(unsigned char)((gain_val>>8)&0x7);
	exphigh	= (unsigned char) ( (0x0f0000&exp_val)>>16);
	expmid	= (unsigned char) ( (0x00ff00&exp_val)>>8);
	explow	= (unsigned char) ( (0x0000ff&exp_val)	 );

	shutter = exp_val/16;  
	if(shutter  > ov8865_sensor_vts- 4)
		frame_length = shutter + 4;
	else 
		frame_length = ov8865_sensor_vts;
  
	sensor_write(sd, 0x3208, 0x00);//enter group write
  
	sensor_write(sd, 0x380f, (frame_length & 0xff));
	sensor_write(sd, 0x380e, (frame_length >> 8));
  
	sensor_write(sd, 0x3509, gainlow);
	sensor_write(sd, 0x3508, gainhigh);
  
	sensor_write(sd, 0x3502, explow);
	sensor_write(sd, 0x3501, expmid);
	sensor_write(sd, 0x3500, exphigh);	
	sensor_write(sd, 0x3208, 0x10);//end group write
	sensor_write(sd, 0x3208, 0xa0);//init group write
	//printk("norm exp_val = %d,gain_val = %d\n",exp_val,gain_val);

	info->exp = exp_val;
	info->gain = gain_val;
	return 0;
}

static int sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	unsigned char explow,expmid,exphigh;
	struct sensor_info *info = to_state(sd);

	vfe_dev_dbg("sensor_set_exposure = %d\n", exp_val>>4);
	if(exp_val>0xfffff)
		exp_val=0xfffff;
	
	//if(info->exp == exp_val && exp_val <= (2480)*16)
	//	return 0;
  
    exphigh = (unsigned char) ( (0x0f0000&exp_val)>>16);
    expmid  = (unsigned char) ( (0x00ff00&exp_val)>>8);
    explow  = (unsigned char) ( (0x0000ff&exp_val)   );
	
	sensor_write(sd, 0x3208, 0x00);//enter group write
	sensor_write(sd, 0x3502, explow);
	sensor_write(sd, 0x3501, expmid);
	sensor_write(sd, 0x3500, exphigh);	
	//sensor_write(sd, 0x3208, 0x10);//end group write
	//sensor_write(sd, 0x3208, 0xa0);//init group write
	//printk("8865 sensor_set_exp = %d, Done!\n", exp_val);
	
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
	vfe_dev_dbg("sensor_set_gain = %d\n", gain_val);
    //if(info->gain == gain_val)
    //    return 0;

	gain_val *= 8;

	if (gain_val < 2*16*8)
	{
	  gainhigh = 0;
	  gainlow = gain_val;
	}
	else if (2*16*8 <=gain_val && gain_val < 4*16*8)
	{
	  gainhigh = 1;
	  gainlow = gain_val/2-8;
	}
	else if (4*16*8 <= gain_val && gain_val < 8*16*8)
	{
	  gainhigh = 3;
	  gainlow = gain_val/4-12;
	}
	else 
	{
	  gainhigh = 7;
	  gainlow = gain_val/8-8;
	}
	
	sensor_write(sd, 0x3509, gainlow);
	sensor_write(sd, 0x3508, gainhigh);
	sensor_write(sd, 0x3208, 0x10);//end group write
	sensor_write(sd, 0x3208, 0xa0);//init group write
	
	//printk("8865 sensor_set_gain = %d, Done!\n", gain_val);
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
	int ret = 0;
	switch(on)
	{
		case CSI_SUBDEV_STBY_ON:
			vfe_dev_dbg("CSI_SUBDEV_STBY_ON!\n");
			ret = sensor_s_sw_stby(sd, CSI_GPIO_LOW);
			if(ret < 0)
				vfe_dev_err("soft stby falied!\n");
			usleep_range(10000,12000);
			cci_lock(sd);    
			vfe_gpio_write(sd,RESET,CSI_GPIO_LOW);
			cci_unlock(sd);    
			vfe_set_mclk(sd,OFF);
			break;
		case CSI_SUBDEV_STBY_OFF:
			vfe_dev_dbg("CSI_SUBDEV_STBY_OFF!\n");
			cci_lock(sd);    
			vfe_set_mclk_freq(sd,MCLK);
			vfe_set_mclk(sd,ON);
			usleep_range(10000,12000);
			vfe_gpio_write(sd,RESET,CSI_GPIO_HIGH);
			usleep_range(10000,12000);
			ret = sensor_s_sw_stby(sd, CSI_GPIO_HIGH);
			if(ret < 0)
				vfe_dev_err("soft stby off falied!\n");
			cci_unlock(sd);    
			break;
		case CSI_SUBDEV_PWR_ON:
			vfe_dev_dbg("CSI_SUBDEV_PWR_ON!\n");
			cci_lock(sd);    
			vfe_gpio_set_status(sd,PWDN,1);//set the gpio to output
			vfe_gpio_set_status(sd,RESET,1);//set the gpio to output
			vfe_gpio_write(sd,PWDN,CSI_GPIO_LOW);
			vfe_gpio_write(sd,RESET,CSI_GPIO_LOW);
			vfe_set_pmu_channel(sd,IOVDD, CSI_GPIO_HIGH);
			vfe_gpio_write(sd,PWDN,CSI_GPIO_HIGH);
			usleep_range(10000,12000);
			vfe_set_pmu_channel(sd,AVDD,CSI_GPIO_HIGH);
			usleep_range(5000,6000);
			vfe_set_pmu_channel(sd,DVDD,CSI_GPIO_HIGH);
			vfe_set_pmu_channel(sd,AFVDD,CSI_GPIO_HIGH);
			usleep_range(5000,6000);
			vfe_gpio_write(sd,RESET,CSI_GPIO_HIGH);
			usleep_range(10000,12000);
			vfe_set_mclk_freq(sd,MCLK);
			vfe_set_mclk(sd,ON);
			usleep_range(10000,12000);
			cci_unlock(sd);    
			break;
		case CSI_SUBDEV_PWR_OFF:
			vfe_dev_dbg("CSI_SUBDEV_PWR_OFF!\n");
			cci_lock(sd);   
			vfe_set_mclk(sd,OFF);
			usleep_range(10000,12000);
			vfe_gpio_write(sd,RESET,CSI_GPIO_LOW);
			usleep_range(10000,12000);
			vfe_set_pmu_channel(sd,AFVDD,CSI_GPIO_LOW);
			vfe_set_pmu_channel(sd,DVDD,CSI_GPIO_LOW);
			usleep_range(5000,6000);
			vfe_set_pmu_channel(sd,AVDD,CSI_GPIO_LOW);
			usleep_range(5000,6000);
			vfe_set_pmu_channel(sd,IOVDD,CSI_GPIO_LOW);  
			vfe_gpio_write(sd,PWDN,CSI_GPIO_LOW);
			usleep_range(10000,12000);
			vfe_gpio_set_status(sd,RESET,0);//set the gpio to input
			vfe_gpio_set_status(sd,PWDN,0);//set the gpio to input
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
      mdelay(10);
      break;
    case 1:
      vfe_gpio_write(sd,RESET,CSI_RST_ON);
      mdelay(10);
      break;
    default:
      return -EINVAL;
  }
    
  return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
  unsigned char rdval;
  
  LOG_ERR_RET(sensor_read(sd, 0x300a, &rdval))
  if(rdval != 0x00)
  {
      vfe_dev_err("read 0x300a: 0x%02x != 0x00\n", rdval);
	  return -ENODEV;
  }
  	
  LOG_ERR_RET(sensor_read(sd, 0x300b, &rdval))
  if(rdval != 0x88)
  {
	  vfe_dev_err("read 0x300b: 0x%02x != 0x88\n", rdval);
	  return -ENODEV;
  }
	  
  LOG_ERR_RET(sensor_read(sd, 0x300c, &rdval))
  if(rdval != 0x65)
  {
	  vfe_dev_err("read 0x300c: 0x%02x != 0x65\n", rdval);
	  return -ENODEV;
  }
  
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
  info->width = QUXGA_WIDTH;
  info->height = QUXGA_HEIGHT;
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
		.mbus_code	= V4L2_MBUS_FMT_SBGGR10_10X1,//V4L2_MBUS_FMT_SGRBG10_10X1,
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
	  /* quxga: 3264*2448 */
	  {
      .width      = QUXGA_WIDTH,
      .height     = QUXGA_HEIGHT,
      .hoffset    = 0,
      .voffset    = 0,
      .hts        = 1944,
      .vts        = 2470,
      .pclk       = 144*1000*1000,
      .mipi_bps	  = 720*1000*1000,
      .fps_fixed  = 30,
      .bin_factor = 1,
      .intg_min   = 16,
      .intg_max   = (2470-4)<<4,
      .gain_min   = 1<<4,
      .gain_max   = 15<<4,
      .regs       = sensor_quxga_regs,
      .regs_size  = ARRAY_SIZE(sensor_quxga_regs),
      .set_size   = NULL,
    },

    /* 6M */
    {
      .width	  = 3264,
      .height 	  = 1836,
      .hoffset    = 0,
      .voffset    = 0,
	  .hts		  = 2582,
	  .vts		  = 1858,
	  .pclk 	  = 144*1000*1000,
	  .mipi_bps   = 720*1000*1000,
      .fps_fixed  = 30,
      .bin_factor = 1,
      .intg_min   = 1<<4,
      .intg_max   = (1858-4)<<4,
      .gain_min   = 1<<4,
      .gain_max   = 15<<4,
      .regs       = sensor_6M_regs,//
      .regs_size  = ARRAY_SIZE(sensor_6M_regs),//
      .set_size	  = NULL,
    },
    /* 1080p */
    {
      .width	  = HD1080_WIDTH,
      .height 	  = HD1080_HEIGHT,
      .hoffset    = 0,
      .voffset    = 0,
	  .hts		  = 2582,
	  .vts		  = 1858,
	  .pclk 	  = 144*1000*1000,
	  .mipi_bps   = 720*1000*1000,
      .fps_fixed  = 30,
      .bin_factor = 1,
      .intg_min   = 1<<4,
      .intg_max   = (1858-4)<<4,
      .gain_min   = 1<<4,
      .gain_max   = 15<<4,
      .width_input	  = 3264,
	  .height_input 	  = 1836,
      .regs       = sensor_6M_regs,//
      .regs_size  = ARRAY_SIZE(sensor_6M_regs),//
      .set_size	  = NULL,
    },
	/* 720p@60fps */
    {
      .width	  = HD720_WIDTH,
      .height 	  = HD720_HEIGHT,
      .hoffset	  = 0,
      .voffset	  = 162,
      .hts        = 1923,
      .vts        = 1248,
	  .pclk 	  = 72*1000*1000,
	  .mipi_bps   = 720*1000*1000,
      .fps_fixed  = 30,
      .bin_factor = 1,
      .intg_min   = 1,
      .intg_max   = (1248-4)<<4,
      .gain_min   = 1<<4,
      .gain_max   = 15<<4,
      .width_input	  = 1632,
	  .height_input 	  = 900,
      .regs		  = sensor_uxga_regs,
      .regs_size  = ARRAY_SIZE(sensor_uxga_regs),
      .set_size	  = NULL,
    },
  	/* UXGA */
    {
      .width	  = UXGA_WIDTH,
      .height 	  = UXGA_HEIGHT,
      .hoffset	  = 16,
      .voffset	  = 12,
      .hts        = 1923,
      .vts        = 1248,
	  .pclk 	  = 72*1000*1000,
	  .mipi_bps   = 720*1000*1000,
      .fps_fixed  = 30,
      .bin_factor = 1,
      .intg_min   = 1,
      .intg_max   = (1248-4)<<4,
      .gain_min   = 1<<4,
      .gain_max   = 15<<4,
      .regs		  = sensor_uxga_regs,
      .regs_size  = ARRAY_SIZE(sensor_uxga_regs),
      .set_size	  = NULL,
    },
    /* SVGA */
    {
      .width	  = SVGA_WIDTH,
      .height 	  = SVGA_HEIGHT,
      .hoffset	  = 0,
      .voffset	  = 0,
      .hts        = 1250,
      .vts        = 640,
	  .pclk 	  = 72*1000*1000,
	  .mipi_bps   = 720*1000*1000,
      .fps_fixed  = 90,
      .bin_factor = 1,
      .intg_min   = 1,
      .intg_max   = (640-4)<<4,
      .gain_min   = 1<<4,
      .gain_max   = 15<<4,
      .regs		  = sensor_svga_90fps_regs,
      .regs_size  = ARRAY_SIZE(sensor_svga_90fps_regs),
      .set_size	  = NULL,
    },
    /* VGA */
    {
      .width	  = VGA_WIDTH,
      .height 	  = VGA_HEIGHT,
      .hoffset	  = 0,
      .voffset	  = 0,
      .hts        = 1250,
      .vts        = 640,
	  .pclk 	  = 72*1000*1000,
	  .mipi_bps   = 720*1000*1000,
      .fps_fixed  = 90,
      .bin_factor = 1,
      .intg_min   = 1,
      .intg_max   = (640-4)<<4,
      .gain_min   = 1<<4,
      .gain_max   = 15<<4,
      .width_input	= 800,
	  .height_input = 600,
      .regs		  = sensor_svga_90fps_regs,
      .regs_size  = ARRAY_SIZE(sensor_svga_90fps_regs),
      .set_size	  = NULL,
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
  int index, isMatched;
  int ratio_input, ratio_fmt;
  struct sensor_win_size *wsize, *wsize_last_ok = NULL;
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

    isMatched = 0;
    for (wsize = sensor_win_sizes; wsize < sensor_win_sizes + N_WIN_SIZES; wsize++)
    {
        if (fmt->width == wsize->width && fmt->height == wsize->height && info->tpf.denominator == wsize->fps_fixed)
     	{
     	    isMatched = 1;   // perfect!
     	    break;
    	}
    }

	if (!isMatched)
	{
		for (wsize = sensor_win_sizes; wsize < sensor_win_sizes + N_WIN_SIZES; wsize++)
		{
			ratio_fmt = 400*wsize->height / wsize->width ;
			ratio_input = 400*fmt->height/ fmt->width;
			if(ABS_SENSOR(ratio_input - ratio_fmt) <= 5)
			{
				if (fmt->width >= wsize->width && fmt->height >= wsize->height)
				{
					wsize_last_ok = wsize;
					break;
				}
				wsize_last_ok = wsize;
			}
		}	
	}	
  
	if (wsize >= sensor_win_sizes + N_WIN_SIZES)
	{
		if(NULL != wsize_last_ok){
			wsize = wsize_last_ok;
		} else {
			wsize--;   /* Take the smallest one */
		}
	}

	vfe_dev_dbg("try fmt: [%d %d %d] -> [%d %d %d]\n", 
		fmt->width, fmt->height, info->tpf.denominator,
		wsize->width, wsize->height, wsize->fps_fixed);

	
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
  ov8865_sensor_vts = wsize->vts;

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
		return v4l2_ctrl_query_fill(qc, 1*16, 64*16-1, 1, 1*16);
	case V4L2_CID_EXPOSURE:
		return v4l2_ctrl_query_fill(qc, 0, 65535*16, 1, 0);
	case V4L2_CID_FRAME_RATE:
		return v4l2_ctrl_query_fill(qc, 15, 120, 1, 120);
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

