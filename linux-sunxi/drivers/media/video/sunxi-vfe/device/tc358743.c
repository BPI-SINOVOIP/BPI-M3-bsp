/*
 * A V4L2 driver for TC358743 cameras.
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


MODULE_AUTHOR("zw");
MODULE_DESCRIPTION("A low-level driver for TC358743 sensors");
MODULE_LICENSE("GPL");

//for internel driver debug
#define DEV_DBG_EN      1
#if(DEV_DBG_EN == 1)
#define vfe_dev_dbg(x,arg...) printk("[TC358743]"x,##arg)
#else
#define vfe_dev_dbg(x,arg...)
#endif
#define vfe_dev_err(x,arg...) printk("[TC358743]"x,##arg)
#define vfe_dev_print(x,arg...) printk("[TC358743]"x,##arg)

#define LOG_ERR_RET(x)  { \
                          int ret;  \
                          ret = x; \
                          if(ret < 0) {\
                            vfe_dev_err("error at %s\n",__func__);  \
                            return ret; \
                          } \
                        }

//define module timing
#define MCLK              (26*1000*1000)
#define VREF_POL          V4L2_MBUS_VSYNC_ACTIVE_HIGH
#define HREF_POL          V4L2_MBUS_HSYNC_ACTIVE_HIGH
#define CLK_POL           V4L2_MBUS_PCLK_SAMPLE_RISING
#define V4L2_IDENT_SENSOR 0x8743
int tc358743_sensor_vts;

/*
 * Our nominal (default) frame rate.
 */
#define SENSOR_FRAME_RATE 30


/*
 * The TC358743 sits on i2c with ID 0x1e
 */
#define I2C_ADDR 0x1e
#define SENSOR_NAME "tc358743"
static struct v4l2_subdev *glb_sd;

/*
 * Information we maintain about a known sensor.
 */
struct sensor_format_struct;  /* coming later */

static inline struct sensor_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sensor_info, sd);
}

static int sensor_read8(struct v4l2_subdev *sd, unsigned short reg,unsigned char *value)
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

static int sensor_write8(struct v4l2_subdev *sd, unsigned short reg, unsigned char value)
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

static int sensor_read16(struct v4l2_subdev *sd, unsigned short reg, unsigned short *value)
{
	int ret=0;
	int cnt=0;
	unsigned short data = 0;

	ret = cci_read_a16_d16(sd,reg,&data);
	while(ret!=0&&cnt<2)
	{
		ret = cci_read_a16_d16(sd,reg,&data);
		cnt++;
	}
	if(cnt>0)
		vfe_dev_dbg("sensor read retry=%d\n",cnt);

	*value |= (data&0xff00)>>8;
	*value |= (data&0x00ff)<<8;

	return ret;
}

static int sensor_write16(struct v4l2_subdev *sd, unsigned short reg, unsigned short value)
{
	int ret=0;
	int cnt=0;
	unsigned short data = 0;
	data |= (value&0xff00)>>8;
	data |= (value&0x00ff)<<8;
	ret = cci_write_a16_d16(sd,reg,data);
	while(ret!=0&&cnt<2)
	{
		ret = cci_write_a16_d16(sd,reg,data);
		cnt++;
	}
	if(cnt>0)
		vfe_dev_dbg("sensor write retry=%d\n",cnt);

	return ret;
}

static int sensor_write32(struct v4l2_subdev *sd, unsigned short reg,unsigned int value)
{
	int ret=0;
	int cnt=0;
	unsigned int data = 0;
	data |= (value&0xff000000)>>24;
	data |= ((value&0x00ff0000)>>16)<<8;
	data |= ((value&0x0000ff00)>>8)<<16;
	data |= (value&0x000000ff)<<24;
	ret = cci_write_a16_d32(sd,reg,data);
	while(ret!=0&&cnt<2)
	{
		ret = cci_write_a16_d32(sd,reg,data);
		cnt++;
	}
	if(cnt>0)
		vfe_dev_dbg("sensor write retry=%d\n",cnt);

	return ret;
}

static int reg_show8(struct v4l2_subdev *sd, unsigned short addr)
{
	unsigned char rdval;
	rdval = 0;
	sensor_read8(sd, addr, &rdval);
	vfe_dev_print("reg 0x%x rdval = 0x%x\n",addr, rdval);
	return 0;
}

static int default_reg_init(struct v4l2_subdev *sd)
{
	// Software Reset
	sensor_write16(sd,0x0002,0x0F00); // SysCtl
	sensor_write16(sd,0x0002,0x0000); // SysCtl
	// FIFO Delay Setting
	sensor_write16(sd,0x0006,0x0154); // FIFO Ctl
	// PLL Setting
	sensor_write16(sd,0x0020,0x3057); // PLLCtl0
	sensor_write16(sd,0x0022,0x0203); // PLLCtl1
	usleep_range(1000, 1200);//Waitx1us(10);
	sensor_write16(sd,0x0022,0x0213); // PLLCtl1
	// Interrupt Control
	sensor_write16(sd,0x0014,0x0000); // IntStatus
	sensor_write16(sd,0x0016,0x05FF); // IntMask
	// CSI Lane Enable
	sensor_write32(sd,0x0140,0x00000000); // CLW_CNTRL
	sensor_write32(sd,0x0144,0x00000000); // D0W_CNTRL
	sensor_write32(sd,0x0148,0x00000000); // D1W_CNTRL
	sensor_write32(sd,0x014C,0x00000000); // D2W_CNTRL
	sensor_write32(sd,0x0150,0x00000000); // D3W_CNTRL
	// CSI Transition Timing
	sensor_write32(sd,0x0210,0x00000FA0); // LINEINITCNT
	sensor_write32(sd,0x0214,0x00000004); // LPTXTIMECNT
	sensor_write32(sd,0x0218,0x00001503); // TCLK_HEADERCNT
	sensor_write32(sd,0x021C,0x00000001); // TCLK_TRAILCNT
	sensor_write32(sd,0x0220,0x00000103); // THS_HEADERCNT
	sensor_write32(sd,0x0224,0x00003A98); // TWAKEUP
	sensor_write32(sd,0x0228,0x00000008); // TCLK_POSTCNT
	sensor_write32(sd,0x022C,0x00000002); // THS_TRAILCNT
	sensor_write32(sd,0x0230,0x00000005); // HSTXVREGCNT
	sensor_write32(sd,0x0234,0x0000001F); // HSTXVREGEN
	sensor_write32(sd,0x0238,0x00000001); // TXOPTIONACNTRL
	sensor_write32(sd,0x0204,0x00000001); // STARTCNTRL
	sensor_write32(sd,0x0518,0x00000001); // CSI_START
	sensor_write32(sd,0x0500,0xA3008086); // CSI_CONFW
	// HDMI Interrupt Mask
	sensor_write8(sd,0x8502,0x01); // SYS_INT
	sensor_write8(sd,0x8512,0xFE); // SYS_INTM
	sensor_write8(sd,0x8514,0x00); // PACKET_INTM
	sensor_write8(sd,0x8515,0x00); // AUDIO_IMNTM
	sensor_write8(sd,0x8516,0x00); // ABUF_INTM
	// HDMI Audio REFCLK
	sensor_write8(sd,0x8531,0x01); // PHY_CTL0
	sensor_write8(sd,0x8540,0x8C); // SYS_FREQ0
	sensor_write8(sd,0x8541,0x0A); // SYS_FREQ1
	sensor_write8(sd,0x8630,0xB0); // LOCKDET_REF0
	sensor_write8(sd,0x8631,0x1E); // LOCKDET_REF1
	sensor_write8(sd,0x8632,0x04); // LOCKDET_REF2
	sensor_write8(sd,0x8670,0x01); // NCO_F0_MOD
	// NCO_48F0A
	// NCO_48F0B
	// NCO_48F0C
	// NCO_48F0D
	// NCO_44F0A
	// NCO_44F0B
	// NCO_44F0C
	// NCO_44F0D
	// HDMI PHY
	sensor_write8(sd,0x8532,0x80); // PHY CTL1
	sensor_write8(sd,0x8536,0x40); // PHY_BIAS
	sensor_write8(sd,0x853F,0x0A); // PHY_CSQ
	// HDMI SYSTEM
	sensor_write8(sd,0x8543,0x32); // DDC_CTL
	sensor_write8(sd,0x8544,0x10); // HPD_CTL
	sensor_write8(sd,0x8545,0x31); // ANA_CTL
	sensor_write8(sd,0x8546,0x2D); // AVM_CTL
	// EDID
	sensor_write8(sd,0x85C7,0x01); // EDID_MODE
	sensor_write8(sd,0x85CA,0x00); // EDID_LEN1
	sensor_write8(sd,0x85CB,0x01); // EDID_LEN2
	// EDID Data
	sensor_write8(sd,0x8C00,0x00); // EDID_RAM
	sensor_write8(sd,0x8C01,0xFF); // EDID_RAM
	sensor_write8(sd,0x8C02,0xFF); // EDID_RAM
	sensor_write8(sd,0x8C03,0xFF); // EDID_RAM
	sensor_write8(sd,0x8C04,0xFF); // EDID_RAM
	sensor_write8(sd,0x8C05,0xFF); // EDID_RAM
	sensor_write8(sd,0x8C06,0xFF); // EDID_RAM
	sensor_write8(sd,0x8C07,0x00); // EDID_RAM
	sensor_write8(sd,0x8C08,0x52); // EDID_RAM
	sensor_write8(sd,0x8C09,0x62); // EDID_RAM
	sensor_write8(sd,0x8C0A,0x88); // EDID_RAM
	sensor_write8(sd,0x8C0B,0x88); // EDID_RAM
	sensor_write8(sd,0x8C0C,0x00); // EDID_RAM
	sensor_write8(sd,0x8C0D,0x88); // EDID_RAM
	sensor_write8(sd,0x8C0E,0x88); // EDID_RAM
	sensor_write8(sd,0x8C0F,0x88); // EDID_RAM
	sensor_write8(sd,0x8C10,0x1C); // EDID_RAM
	sensor_write8(sd,0x8C11,0x15); // EDID_RAM
	sensor_write8(sd,0x8C12,0x01); // EDID_RAM
	sensor_write8(sd,0x8C13,0x03); // EDID_RAM
	sensor_write8(sd,0x8C14,0x80); // EDID_RAM
	sensor_write8(sd,0x8C15,0x00); // EDID_RAM
	sensor_write8(sd,0x8C16,0x00); // EDID_RAM
	sensor_write8(sd,0x8C17,0x78); // EDID_RAM
	sensor_write8(sd,0x8C18,0x0A); // EDID_RAM
	sensor_write8(sd,0x8C19,0xDA); // EDID_RAM
	sensor_write8(sd,0x8C1A,0xFF); // EDID_RAM
	sensor_write8(sd,0x8C1B,0xA3); // EDID_RAM
	sensor_write8(sd,0x8C1C,0x58); // EDID_RAM
	sensor_write8(sd,0x8C1D,0x4A); // EDID_RAM
	sensor_write8(sd,0x8C1E,0xA2); // EDID_RAM
	sensor_write8(sd,0x8C1F,0x29); // EDID_RAM
	sensor_write8(sd,0x8C20,0x17); // EDID_RAM
	sensor_write8(sd,0x8C21,0x49); // EDID_RAM
	sensor_write8(sd,0x8C22,0x4B); // EDID_RAM
	sensor_write8(sd,0x8C23,0x00); // EDID_RAM
	sensor_write8(sd,0x8C24,0x00); // EDID_RAM
	sensor_write8(sd,0x8C25,0x00); // EDID_RAM
	sensor_write8(sd,0x8C26,0x01); // EDID_RAM
	sensor_write8(sd,0x8C27,0x01); // EDID_RAM
	sensor_write8(sd,0x8C28,0x01); // EDID_RAM
	sensor_write8(sd,0x8C29,0x01); // EDID_RAM
	sensor_write8(sd,0x8C2A,0x01); // EDID_RAM
	sensor_write8(sd,0x8C2B,0x01); // EDID_RAM
	sensor_write8(sd,0x8C2C,0x01); // EDID_RAM
	sensor_write8(sd,0x8C2D,0x01); // EDID_RAM
	sensor_write8(sd,0x8C2E,0x01); // EDID_RAM
	sensor_write8(sd,0x8C2F,0x01); // EDID_RAM
	sensor_write8(sd,0x8C30,0x01); // EDID_RAM
	sensor_write8(sd,0x8C31,0x01); // EDID_RAM
	sensor_write8(sd,0x8C32,0x01); // EDID_RAM
	sensor_write8(sd,0x8C33,0x01); // EDID_RAM
	sensor_write8(sd,0x8C34,0x01); // EDID_RAM
	sensor_write8(sd,0x8C35,0x01); // EDID_RAM
	sensor_write8(sd,0x8C36,0x02); // EDID_RAM
	sensor_write8(sd,0x8C37,0x3A); // EDID_RAM
	sensor_write8(sd,0x8C38,0x80); // EDID_RAM
	sensor_write8(sd,0x8C39,0x18); // EDID_RAM
	sensor_write8(sd,0x8C3A,0x71); // EDID_RAM
	sensor_write8(sd,0x8C3B,0x38); // EDID_RAM
	sensor_write8(sd,0x8C3C,0x2D); // EDID_RAM
	sensor_write8(sd,0x8C3D,0x40); // EDID_RAM
	sensor_write8(sd,0x8C3E,0x58); // EDID_RAM
	sensor_write8(sd,0x8C3F,0x2C); // EDID_RAM
	sensor_write8(sd,0x8C40,0x45); // EDID_RAM
	sensor_write8(sd,0x8C41,0x00); // EDID_RAM
	sensor_write8(sd,0x8C42,0xC4); // EDID_RAM
	sensor_write8(sd,0x8C43,0x8E); // EDID_RAM
	sensor_write8(sd,0x8C44,0x21); // EDID_RAM
	sensor_write8(sd,0x8C45,0x00); // EDID_RAM
	sensor_write8(sd,0x8C46,0x00); // EDID_RAM
	sensor_write8(sd,0x8C47,0x1E); // EDID_RAM
	sensor_write8(sd,0x8C48,0x01); // EDID_RAM
	sensor_write8(sd,0x8C49,0x1D); // EDID_RAM
	sensor_write8(sd,0x8C4A,0x00); // EDID_RAM
	sensor_write8(sd,0x8C4B,0x72); // EDID_RAM
	sensor_write8(sd,0x8C4C,0x51); // EDID_RAM
	sensor_write8(sd,0x8C4D,0xD0); // EDID_RAM
	sensor_write8(sd,0x8C4E,0x1E); // EDID_RAM
	sensor_write8(sd,0x8C4F,0x20); // EDID_RAM
	sensor_write8(sd,0x8C50,0x6E); // EDID_RAM
	sensor_write8(sd,0x8C51,0x28); // EDID_RAM
	sensor_write8(sd,0x8C52,0x55); // EDID_RAM
	sensor_write8(sd,0x8C53,0x00); // EDID_RAM
	sensor_write8(sd,0x8C54,0xC4); // EDID_RAM
	sensor_write8(sd,0x8C55,0x8E); // EDID_RAM
	sensor_write8(sd,0x8C56,0x21); // EDID_RAM
	sensor_write8(sd,0x8C57,0x00); // EDID_RAM
	sensor_write8(sd,0x8C58,0x00); // EDID_RAM
	sensor_write8(sd,0x8C59,0x1E); // EDID_RAM
	sensor_write8(sd,0x8C5A,0x00); // EDID_RAM
	sensor_write8(sd,0x8C5B,0x00); // EDID_RAM
	sensor_write8(sd,0x8C5C,0x00); // EDID_RAM
	sensor_write8(sd,0x8C5D,0xFC); // EDID_RAM
	sensor_write8(sd,0x8C5E,0x00); // EDID_RAM
	sensor_write8(sd,0x8C5F,0x54); // EDID_RAM
	sensor_write8(sd,0x8C60,0x6F); // EDID_RAM
	sensor_write8(sd,0x8C61,0x73); // EDID_RAM
	sensor_write8(sd,0x8C62,0x68); // EDID_RAM
	sensor_write8(sd,0x8C63,0x69); // EDID_RAM
	sensor_write8(sd,0x8C64,0x62); // EDID_RAM
	sensor_write8(sd,0x8C65,0x61); // EDID_RAM
	sensor_write8(sd,0x8C66,0x2D); // EDID_RAM
	sensor_write8(sd,0x8C67,0x48); // EDID_RAM
	sensor_write8(sd,0x8C68,0x32); // EDID_RAM
	sensor_write8(sd,0x8C69,0x43); // EDID_RAM
	sensor_write8(sd,0x8C6A,0x0A); // EDID_RAM
	sensor_write8(sd,0x8C6B,0x20); // EDID_RAM
	sensor_write8(sd,0x8C6C,0x00); // EDID_RAM
	sensor_write8(sd,0x8C6D,0x00); // EDID_RAM
	sensor_write8(sd,0x8C6E,0x00); // EDID_RAM
	sensor_write8(sd,0x8C6F,0xFD); // EDID_RAM
	sensor_write8(sd,0x8C70,0x00); // EDID_RAM
	sensor_write8(sd,0x8C71,0x17); // EDID_RAM
	sensor_write8(sd,0x8C72,0x3D); // EDID_RAM
	sensor_write8(sd,0x8C73,0x0F); // EDID_RAM
	sensor_write8(sd,0x8C74,0x8C); // EDID_RAM
	sensor_write8(sd,0x8C75,0x17); // EDID_RAM
	sensor_write8(sd,0x8C76,0x00); // EDID_RAM
	sensor_write8(sd,0x8C77,0x0A); // EDID_RAM
	sensor_write8(sd,0x8C78,0x20); // EDID_RAM
	sensor_write8(sd,0x8C79,0x20); // EDID_RAM
	sensor_write8(sd,0x8C7A,0x20); // EDID_RAM
	sensor_write8(sd,0x8C7B,0x20); // EDID_RAM
	sensor_write8(sd,0x8C7C,0x20); // EDID_RAM
	sensor_write8(sd,0x8C7D,0x20); // EDID_RAM
	sensor_write8(sd,0x8C7E,0x01); // EDID_RAM
	sensor_write8(sd,0x8C7F,0x78); // EDID_RAM
	sensor_write8(sd,0x8C80,0x02); // EDID_RAM
	sensor_write8(sd,0x8C81,0x03); // EDID_RAM
	sensor_write8(sd,0x8C82,0x17); // EDID_RAM
	sensor_write8(sd,0x8C83,0x74); // EDID_RAM
	sensor_write8(sd,0x8C84,0x47); // EDID_RAM
	sensor_write8(sd,0x8C85,0x10); // EDID_RAM
	sensor_write8(sd,0x8C86,0x04); // EDID_RAM
	sensor_write8(sd,0x8C87,0x02); // EDID_RAM
	sensor_write8(sd,0x8C88,0x01); // EDID_RAM
	sensor_write8(sd,0x8C89,0x02); // EDID_RAM
	sensor_write8(sd,0x8C8A,0x02); // EDID_RAM
	sensor_write8(sd,0x8C8B,0x02); // EDID_RAM
	sensor_write8(sd,0x8C8C,0x23); // EDID_RAM
	sensor_write8(sd,0x8C8D,0x09); // EDID_RAM
	sensor_write8(sd,0x8C8E,0x07); // EDID_RAM
	sensor_write8(sd,0x8C8F,0x01); // EDID_RAM
	sensor_write8(sd,0x8C90,0x66); // EDID_RAM
	sensor_write8(sd,0x8C91,0x03); // EDID_RAM
	sensor_write8(sd,0x8C92,0x0C); // EDID_RAM
	sensor_write8(sd,0x8C93,0x00); // EDID_RAM
	sensor_write8(sd,0x8C94,0x30); // EDID_RAM
	sensor_write8(sd,0x8C95,0x00); // EDID_RAM
	sensor_write8(sd,0x8C96,0x80); // EDID_RAM
	sensor_write8(sd,0x8C97,0x8C); // EDID_RAM
	sensor_write8(sd,0x8C98,0x0A); // EDID_RAM
	sensor_write8(sd,0x8C99,0xD0); // EDID_RAM
	sensor_write8(sd,0x8C9A,0x8C); // EDID_RAM
	sensor_write8(sd,0x8C9B,0x0A); // EDID_RAM
	sensor_write8(sd,0x8C9C,0xD0); // EDID_RAM
	sensor_write8(sd,0x8C9D,0x8A); // EDID_RAM
	sensor_write8(sd,0x8C9E,0x20); // EDID_RAM
	sensor_write8(sd,0x8C9F,0xE0); // EDID_RAM
	sensor_write8(sd,0x8CA0,0x2D); // EDID_RAM
	sensor_write8(sd,0x8CA1,0x10); // EDID_RAM
	sensor_write8(sd,0x8CA2,0x10); // EDID_RAM
	sensor_write8(sd,0x8CA3,0x3E); // EDID_RAM
	sensor_write8(sd,0x8CA4,0x96); // EDID_RAM
	sensor_write8(sd,0x8CA5,0x00); // EDID_RAM
	sensor_write8(sd,0x8CA6,0x13); // EDID_RAM
	sensor_write8(sd,0x8CA7,0x8E); // EDID_RAM
	sensor_write8(sd,0x8CA8,0x21); // EDID_RAM
	sensor_write8(sd,0x8CA9,0x00); // EDID_RAM
	sensor_write8(sd,0x8CAA,0x00); // EDID_RAM
	sensor_write8(sd,0x8CAB,0x1E); // EDID_RAM
	sensor_write8(sd,0x8CAC,0xD8); // EDID_RAM
	sensor_write8(sd,0x8CAD,0x09); // EDID_RAM
	sensor_write8(sd,0x8CAE,0x80); // EDID_RAM
	sensor_write8(sd,0x8CAF,0xA0); // EDID_RAM
	sensor_write8(sd,0x8CB0,0x20); // EDID_RAM
	sensor_write8(sd,0x8CB1,0xE0); // EDID_RAM
	sensor_write8(sd,0x8CB2,0x2D); // EDID_RAM
	sensor_write8(sd,0x8CB3,0x10); // EDID_RAM
	sensor_write8(sd,0x8CB4,0x10); // EDID_RAM
	sensor_write8(sd,0x8CB5,0x60); // EDID_RAM
	sensor_write8(sd,0x8CB6,0xA2); // EDID_RAM
	sensor_write8(sd,0x8CB7,0x00); // EDID_RAM
	sensor_write8(sd,0x8CB8,0xC4); // EDID_RAM
	sensor_write8(sd,0x8CB9,0x8E); // EDID_RAM
	sensor_write8(sd,0x8CBA,0x21); // EDID_RAM
	sensor_write8(sd,0x8CBB,0x00); // EDID_RAM
	sensor_write8(sd,0x8CBC,0x00); // EDID_RAM
	sensor_write8(sd,0x8CBD,0x1E); // EDID_RAM
	sensor_write8(sd,0x8CBE,0x8C); // EDID_RAM
	sensor_write8(sd,0x8CBF,0x0A); // EDID_RAM
	sensor_write8(sd,0x8CC0,0xD0); // EDID_RAM
	sensor_write8(sd,0x8CC1,0x8A); // EDID_RAM
	sensor_write8(sd,0x8CC2,0x20); // EDID_RAM
	sensor_write8(sd,0x8CC3,0xE0); // EDID_RAM
	sensor_write8(sd,0x8CC4,0x2D); // EDID_RAM
	sensor_write8(sd,0x8CC5,0x10); // EDID_RAM
	sensor_write8(sd,0x8CC6,0x10); // EDID_RAM
	sensor_write8(sd,0x8CC7,0x3E); // EDID_RAM
	sensor_write8(sd,0x8CC8,0x96); // EDID_RAM
	sensor_write8(sd,0x8CC9,0x00); // EDID_RAM
	sensor_write8(sd,0x8CCA,0x13); // EDID_RAM
	sensor_write8(sd,0x8CCB,0x8E); // EDID_RAM
	sensor_write8(sd,0x8CCC,0x21); // EDID_RAM
	sensor_write8(sd,0x8CCD,0x00); // EDID_RAM
	sensor_write8(sd,0x8CCE,0x00); // EDID_RAM
	sensor_write8(sd,0x8CCF,0x1E); // EDID_RAM
	sensor_write8(sd,0x8CD0,0x8C); // EDID_RAM
	sensor_write8(sd,0x8CD1,0x0A); // EDID_RAM
	sensor_write8(sd,0x8CD2,0xD0); // EDID_RAM
	sensor_write8(sd,0x8CD3,0x8A); // EDID_RAM
	sensor_write8(sd,0x8CD4,0x20); // EDID_RAM
	sensor_write8(sd,0x8CD5,0xE0); // EDID_RAM
	sensor_write8(sd,0x8CD6,0x2D); // EDID_RAM
	sensor_write8(sd,0x8CD7,0x10); // EDID_RAM
	sensor_write8(sd,0x8CD8,0x10); // EDID_RAM
	sensor_write8(sd,0x8CD9,0x3E); // EDID_RAM
	sensor_write8(sd,0x8CDA,0x96); // EDID_RAM
	sensor_write8(sd,0x8CDB,0x00); // EDID_RAM
	sensor_write8(sd,0x8CDC,0x13); // EDID_RAM
	sensor_write8(sd,0x8CDD,0x8E); // EDID_RAM
	sensor_write8(sd,0x8CDE,0x21); // EDID_RAM
	sensor_write8(sd,0x8CDF,0x00); // EDID_RAM
	sensor_write8(sd,0x8CE0,0x00); // EDID_RAM
	sensor_write8(sd,0x8CE1,0x1E); // EDID_RAM
	sensor_write8(sd,0x8CE2,0x00); // EDID_RAM
	sensor_write8(sd,0x8CE3,0x00); // EDID_RAM
	sensor_write8(sd,0x8CE4,0x00); // EDID_RAM
	sensor_write8(sd,0x8CE5,0x00); // EDID_RAM
	sensor_write8(sd,0x8CE6,0x00); // EDID_RAM
	sensor_write8(sd,0x8CE7,0x00); // EDID_RAM
	sensor_write8(sd,0x8CE8,0x00); // EDID_RAM
	sensor_write8(sd,0x8CE9,0x00); // EDID_RAM
	sensor_write8(sd,0x8CEA,0x00); // EDID_RAM
	sensor_write8(sd,0x8CEB,0x00); // EDID_RAM
	sensor_write8(sd,0x8CEC,0x00); // EDID_RAM
	sensor_write8(sd,0x8CED,0x00); // EDID_RAM
	sensor_write8(sd,0x8CEE,0x00); // EDID_RAM
	sensor_write8(sd,0x8CEF,0x00); // EDID_RAM
	sensor_write8(sd,0x8CF0,0x00); // EDID_RAM
	sensor_write8(sd,0x8CF1,0x00); // EDID_RAM
	sensor_write8(sd,0x8CF2,0x00); // EDID_RAM
	sensor_write8(sd,0x8CF3,0x00); // EDID_RAM
	sensor_write8(sd,0x8CF4,0x00); // EDID_RAM
	sensor_write8(sd,0x8CF5,0x00); // EDID_RAM
	sensor_write8(sd,0x8CF6,0x00); // EDID_RAM
	sensor_write8(sd,0x8CF7,0x00); // EDID_RAM
	sensor_write8(sd,0x8CF8,0x00); // EDID_RAM
	sensor_write8(sd,0x8CF9,0x00); // EDID_RAM
	sensor_write8(sd,0x8CFA,0x00); // EDID_RAM
	sensor_write8(sd,0x8CFB,0x00); // EDID_RAM
	sensor_write8(sd,0x8CFC,0x00); // EDID_RAM
	sensor_write8(sd,0x8CFD,0x00); // EDID_RAM
	sensor_write8(sd,0x8CFE,0x00); // EDID_RAM
	sensor_write8(sd,0x8CFF,0x99); // EDID_RAM
	// HDCP Setting
	sensor_write8(sd,0x85D1,0x01); //
	sensor_write8(sd,0x8560,0x24); // HDCP_MODE
	sensor_write8(sd,0x8563,0x11); //
	sensor_write8(sd,0x8564,0x0F); //
	// Video Setting
	sensor_write8(sd,0x8573,0x81); // VOUT_SET2
	// HDMI Audio Setting
	sensor_write8(sd,0x8600,0x00); // AUD_Auto_Mute
	sensor_write8(sd,0x8602,0xF3); // Auto_CMD0
	sensor_write8(sd,0x8603,0x02); // Auto_CMD1
	sensor_write8(sd,0x8604,0x0C); // Auto_CMD2
	sensor_write8(sd,0x8606,0x05); // BUFINIT_START
	sensor_write8(sd,0x8607,0x00); // FS_MUTE
	sensor_write8(sd,0x8620,0x22); // FS_IMODE
	sensor_write8(sd,0x8640,0x01); // ACR_MODE
	sensor_write8(sd,0x8641,0x65); // ACR_MDF0
	sensor_write8(sd,0x8642,0x07); // ACR_MDF1
	sensor_write8(sd,0x8652,0x02); // SDO_MODE1
	sensor_write8(sd,0x8665,0x10); // DIV_MODE
	// Info Frame Extraction
	sensor_write8(sd,0x8709,0xFF); // PK_INT_MODE
	sensor_write8(sd,0x870B,0x2C); // NO_PKT_LIMIT
	sensor_write8(sd,0x870C,0x53); // NO_PKT_CLR
	sensor_write8(sd,0x870D,0x01); // ERR_PK_LIMIT
	sensor_write8(sd,0x870E,0x30); // NO_PKT_LIMIT2
	sensor_write8(sd,0x9007,0x10); // NO_GDB_LIMIT
	sensor_write8(sd,0x854A,0x01); // INIT_END
	sensor_write16(sd,0x0004,0x0CD7); // ConfCtl
	vfe_dev_print("default_reg_init!!!!!!!!!!!!\n");
	return 0;
}

static int colorbar_reg_init(struct v4l2_subdev *sd)
{
	sensor_write16(sd,0x7080,0x0000);
	sensor_write16(sd,0x0002,0x0F00);
	sensor_write16(sd,0x0002,0x0000);
	sensor_write16(sd,0x0004,0x0084);
	sensor_write16(sd,0x0010,0x001E);
	sensor_write16(sd,0x0020,0x3057);
	sensor_write16(sd,0x0022,0x0203);
	sensor_write16(sd,0xffff,0x1000);
	sensor_write16(sd,0x0022,0x0213);
	sensor_write32(sd,0x0140,0x00000000);
	sensor_write32(sd,0x0144,0x00000000);
	sensor_write32(sd,0x0148,0x00000000);
	sensor_write32(sd,0x014C,0x00000000);
	sensor_write32(sd,0x0150,0x00000000);
	sensor_write32(sd,0x0210,0x00000FA0);
	sensor_write32(sd,0x0214,0x00000005);
	sensor_write32(sd,0x0218,0x00001505);
	sensor_write32(sd,0x021C,0x00000001);
	sensor_write32(sd,0x0220,0x00000305);
	sensor_write32(sd,0x0224,0x00003A98);
	sensor_write32(sd,0x0228,0x00000008);
	sensor_write32(sd,0x022C,0x00000002);
	sensor_write32(sd,0x0230,0x00000005);
	sensor_write32(sd,0x0234,0x0000001F);
	sensor_write32(sd,0x0238,0x00000001);
	sensor_write32(sd,0x0204,0x00000001);
	sensor_write32(sd,0x0518,0x00000001);
	sensor_write32(sd,0x0500,0xA3008086);
	sensor_write16(sd,0x000A,0x0A00);
	sensor_write16(sd,0x7080,0x0082);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF7F);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xFF00);
	sensor_write16(sd,0x7000,0xFFFF);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0xC0FF);
	sensor_write16(sd,0x7000,0xC000);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7F00);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x7FFF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x00FF);
	sensor_write16(sd,0x7000,0x0000);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7000,0x007F);
	sensor_write16(sd,0x7090,0x031F);
	sensor_write16(sd,0x7092,0x0779);
	sensor_write16(sd,0x7094,0x0013);
	sensor_write16(sd,0x7080,0x0083);
	vfe_dev_print("colorbar_reg_init!!!!!!!!!!!!\n");
	return 0;
}

unsigned int input_width = 0;
unsigned int input_height = 0;
//unsigned int input_get = 0;

static void sensor_g_width(struct v4l2_subdev *sd, unsigned int *value)
{
	unsigned char rdlow,rdhi;
	sensor_read8(sd, 0x8582,&rdlow);
	sensor_read8(sd, 0x8583,&rdhi);
	*value = rdlow + (rdhi<<8);
}

static void sensor_g_height(struct v4l2_subdev *sd, unsigned int *value)
{
	unsigned char rdlow,rdhi;
	sensor_read8(sd, 0x8588,&rdlow);
	sensor_read8(sd, 0x8589,&rdhi);
	*value = rdlow + (rdhi<<8);
}

static ssize_t sensor_input_width_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cci_driver *cci_drv = dev_get_drvdata(dev);
	struct v4l2_subdev *sd = cci_drv->sd;
	input_width = 0;
	sensor_g_width(sd, &input_width);
	return sprintf(buf, "%d\n", input_width);
}

static ssize_t sensor_input_height_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	struct cci_driver *cci_drv = dev_get_drvdata(dev);
	struct v4l2_subdev *sd = cci_drv->sd;
	input_height = 0;
	sensor_g_height(sd, &input_height);
	return sprintf(buf, "%d\n", input_height);
}


//static ssize_t sensor_input_get_show(struct device *dev, struct device_attribute *attr, char *buf)
//{	
//	return sprintf(buf, "%d\n", input_get);
//}

//static ssize_t sensor_input_get_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
//{
//	struct cci_driver *cci_drv = dev_get_drvdata(dev);
//	struct v4l2_subdev *sd = cci_drv->sd;
//	input_get = simple_strtoul(buf, NULL, 16);
//	vfe_dev_print("read_flag = %d\n", input_get);

//	if(1 == input_get)
//	{
//		input_width = input_height = 0;
//		sensor_g_width(sd, &input_width);
//		sensor_g_height(sd, &input_height);
//	}
//	return count;
//}

static struct device_attribute sensor_device_attrs[] = {
	__ATTR(input_width, S_IRUGO, sensor_input_width_show, NULL),
	__ATTR(input_height, S_IRUGO, sensor_input_height_show, NULL),
//	__ATTR(input_get, S_IWUSR | S_IRUGO, sensor_input_get_show, sensor_input_get_store),
};

static int sensor_create_read_node(struct cci_driver *drv_data)
{
	int i, ret;
	/* sysfs entries */
	for (i = 0; i < ARRAY_SIZE(sensor_device_attrs); i++)
	{
		ret = device_create_file(&drv_data->cci_device, &sensor_device_attrs[i]);
		if (ret)
		{
			vfe_dev_err("device_create_file error\n");
			device_remove_file(&drv_data->cci_device, &drv_data->dev_attr_cci);
		}
	}
	return 0;
}
static int sensor_remove_read_node(struct cci_driver *drv_data)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(sensor_device_attrs); i++)
		device_remove_file(&drv_data->cci_device, &sensor_device_attrs[i]);
	return 0;
}


/*
 * Code for dealing with controls.
 * fill with different sensor module
 * different sensor module has different settings here
 * if not support the follow function ,retrun -EINVAL
 */

/* *********************************************begin of ******************************************** */

static int sensor_s_sw_stby(struct v4l2_subdev *sd, int on_off)
{
	if(on_off)
		vfe_gpio_write(sd,RESET,CSI_GPIO_LOW);
	else
		vfe_gpio_write(sd,RESET,CSI_GPIO_HIGH);
	return 0;
}

/*
 * Stuff that knows about the sensor.
 */
static int sensor_power(struct v4l2_subdev *sd, int on)
{
	switch(on)
	{
		case CSI_SUBDEV_STBY_ON:
			vfe_dev_dbg("CSI_SUBDEV_STBY_ON!\n");
			sensor_s_sw_stby(sd, ON);
			break;
		case CSI_SUBDEV_STBY_OFF:
			vfe_dev_dbg("CSI_SUBDEV_STBY_OFF!\n");
			sensor_s_sw_stby(sd, OFF);
			break;
		case CSI_SUBDEV_PWR_ON:
			vfe_dev_dbg("CSI_SUBDEV_PWR_ON!\n");
			cci_lock(sd);
			vfe_gpio_set_status(sd,RESET,1);//set the gpio to output
			vfe_gpio_write(sd,RESET,CSI_GPIO_LOW);
			usleep_range(5000,6000);
			vfe_set_pmu_channel(sd,DVDD,ON);
			usleep_range(5000,6000);
			vfe_set_pmu_channel(sd,AVDD,ON);
			usleep_range(1000,1200);
			vfe_set_pmu_channel(sd,IOVDD,ON);
			usleep_range(1000,1200);
			vfe_set_mclk_freq(sd,MCLK);
			vfe_set_mclk(sd,ON);
			usleep_range(1000,1200);
			vfe_gpio_write(sd,RESET,CSI_GPIO_HIGH);
			cci_unlock(sd);
			break;
		case CSI_SUBDEV_PWR_OFF:
			vfe_dev_dbg("CSI_SUBDEV_PWR_OFF!\n");
			cci_lock(sd);
			vfe_set_mclk(sd,OFF);
			usleep_range(10000,12000);
			vfe_set_pmu_channel(sd,IOVDD,OFF);
			usleep_range(5000,6000);
			vfe_set_pmu_channel(sd,AVDD,OFF);
			usleep_range(5000,6000);
			vfe_set_pmu_channel(sd,DVDD,OFF);
			usleep_range(10000,12000);
			vfe_gpio_write(sd,RESET,CSI_GPIO_LOW);
			vfe_gpio_set_status(sd,RESET,0);//set the gpio to input
			cci_unlock(sd);
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	vfe_gpio_write(sd,RESET,CSI_GPIO_LOW);
	usleep_range(5000,6000);
	vfe_gpio_write(sd,RESET,CSI_GPIO_HIGH);
	usleep_range(5000,6000);
	return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
	unsigned char rdval;

	rdval = 0;
	LOG_ERR_RET(sensor_read8(sd, 0x8534, &rdval))
	vfe_dev_print("reg 0x8534 rdval = 0x%x\n",rdval);
	if((rdval != 0x3f)&&(rdval != 0x3e))
		return -ENODEV;
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
	info->width = VGA_WIDTH;
	info->height = VGA_HEIGHT;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 0;

	info->tpf.numerator = 1;
	info->tpf.denominator = 30;    /* 30fps */

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
			break;
		case ISP_SET_EXP_GAIN:
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
		.desc   = "MIPI YUV422",
		.mbus_code  = V4L2_MBUS_FMT_UYVY8_16X1,
		.regs     = NULL,
		.regs_size = 0,
		.bpp	  = 2,
	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)


/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */

static struct sensor_win_size sensor_win_sizes[] = {
	/* 1080 */
	{
		.width      = 1920,
		.height     = 1080,
		.hoffset    = 0,
		.voffset    = 0,
		.regs       = NULL,
		.regs_size  = 0,
		.set_size   = default_reg_init,
		//.set_size   = colorbar_reg_init,
	},
	/* 720 */
	{
		.width      = 1280,
		.height     = 720,
		.hoffset    = 0,
		.voffset    = 0,
		.regs       = NULL,
		.regs_size  = 0,
		//.set_size   = colorbar_reg_init,
		.set_size   = default_reg_init,
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

	for (wsize = sensor_win_sizes; wsize < sensor_win_sizes + N_WIN_SIZES;wsize++)
	{
		if (fmt->width >= wsize->width && fmt->height >= wsize->height)
		{
			wsize_last_ok = wsize;
			break;
		}
		wsize_last_ok = wsize;
	}

	if (wsize >= sensor_win_sizes + N_WIN_SIZES)
	{
		if(NULL != wsize_last_ok){
			wsize = wsize_last_ok;
		} else {
			wsize--;   /* Take the smallest one */
		}
	}
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	info->current_wins = wsize;

	/*
	* Note the size we'll actually handle.
	*/
	fmt->width = wsize->width;
	fmt->height = wsize->height;
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

	sensor_reset(sd, 0);

	if (wsize->set_size)
		ret = wsize->set_size(sd);

	//ret = default_reg_init(sd);
	//ret = colorbar_reg_init(sd);

	if(ret < 0) {
		vfe_dev_err("write default_reg_init error\n");
		return ret;
	}
#if 1
	reg_show8(sd, 0x852e);
	reg_show8(sd, 0x852f);

	reg_show8(sd, 0x858a);
	reg_show8(sd, 0x858b);
	reg_show8(sd, 0x8580);
	reg_show8(sd, 0x8581);
	reg_show8(sd, 0x8582);
	reg_show8(sd, 0x8583);

	reg_show8(sd, 0x858c);
	reg_show8(sd, 0x858d);
	reg_show8(sd, 0x8584);
	reg_show8(sd, 0x8585);
	reg_show8(sd, 0x8586);
	reg_show8(sd, 0x8587);
	reg_show8(sd, 0x8588);
	reg_show8(sd, 0x8589);

	reg_show8(sd, 0x8526);
#endif

	info->fmt = sensor_fmt;
	info->width = wsize->width;
	info->height = wsize->height;

	vfe_dev_print("s_fmt = %x, width = %d, height = %d\n",sensor_fmt->mbus_code,wsize->width,wsize->height);

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
	struct sensor_info *info = to_state(sd);

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
	switch (qc->id) {
		case V4L2_CID_GAIN:
			return v4l2_ctrl_query_fill(qc, 0, 10000*10000, 1, 16);
		case V4L2_CID_EXPOSURE:
			return v4l2_ctrl_query_fill(qc, 0, 10000*10000, 1, 16);
	}
	return 0;
}

static int sensor_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return 0;
}

static int sensor_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return 0;
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
	.addr_width = CCI_BITS_16,
	.data_width = CCI_BITS_8,
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
	sensor_create_read_node(&cci_drv);

	info->fmt = &sensor_formats[0];
	info->af_first_flag = 1;
	info->init_first_flag = 1;
	return 0;
}


static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd;

	sensor_remove_read_node(&cci_drv);
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

