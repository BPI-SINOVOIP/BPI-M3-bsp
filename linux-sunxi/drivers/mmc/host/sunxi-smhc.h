/*
 * drivers/mmc/host/sunxi-mci.h
 * (C) Copyright 2007-2011
 * Reuuimlla Technology Co., Ltd. <www.reuuimllatech.com>
 * Aaron.Maoye <leafy.myeh@reuuimllatech.com>
 *
 * description for this code
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */
#ifndef _SUNXI_HOST_OP_H_
#define _SUNXI_HOST_OP_H_ "host_op.h"


#if defined CONFIG_FPGA_V4_PLATFORM || defined CONFIG_FPGA_V7_PLATFORM
#define MMC_FPGA
#endif


#define DRIVER_NAME "sunxi-smhc"
#define DRIVER_RIVISION "v1.91 2015-07-31 16:00"
#define DRIVER_VERSION "SD/MMC/SDIO Host Controller Driver smhc(" DRIVER_RIVISION ")" \
			" Compiled in " __DATE__ " at " __TIME__""

/*========== platform define ==========*/
#define SMC_IRQNO(x)	(SUNXI_IRQ_MMC0 + (x))


#define MMC_SRCCLK_HOSC   "hosc"
#define MMC_SRCCLK_PLL   "pll_periph1"

#define	SUNXI_CCM_BASE	 	0x01c20000
#define SUNXI_PIO_BASE		0x01c20800

#define MMC_MODCLK_PREFIX "sdmmc"

#ifdef MMC_FPGA
#undef SMC_IRQNO
#define SMC_IRQNO(x)	((x) & 2 ? SUNXI_IRQ_MMC2 : SUNXI_IRQ_MMC0)
#define SMC_FPGA_MMC_PREUSED(x)	((x) == 2 || (x) == 0)
#endif


#define SMHC_DEVICE_ID			(3) //number of id in multi device
//max blk count and size is limited by the SMHC_BLK_CFG register's max value
#define MAX_BLK_COUNT		(0xFFFF)
#define MAX_BLK_SIZE		(0x800)

#define MAX_DES_SIZE		PAGE_SIZE


//-------------------------------------------------------------------------------------
/* registers define */
#define SMC0_BASE          0x01C0f000
#define SMC_BASE_OS        0x1000
#define SMHC_BASE_ADDR     (SMC0_BASE + SMC_BASE_OS*SMHC_DEVICE_ID)
//--------------------------------------------------------------------------------------
#define SMHC_CMD_ARG2          (0x00)
#define SMHC_BLK_CFG           (0x04)
#define SMHC_CMD_ARG1          (0x08)
#define SMHC_CMD               (0x0C)
#define SMHC_RESP0             (0x10)
#define SMHC_RESP1             (0x14)
#define SMHC_RESP2             (0x18)
#define SMHC_RESP3             (0x1C)
#define SMHC_BUFF              (0x20)
#define SMHC_STA               (0x24)
#define SMHC_CTRL1             (0x28)
#define SMHC_RST_CLK_CTRL      (0x2C)
#define SMHC_INT_STA           (0x30)
#define SMHC_INT_STA_EN        (0x34)
#define SMHC_INT_SIG_EN        (0x38)
#define SMHC_ACMD_ERR_CTRL2    (0x3C)
#define SMHC_SET_ERR           (0x50) 
#define SMHC_ADMA_ERR          (0x54)
#define SMHC_ADMA_ADDR         (0x58)

#define SMHC_CTRL3             (0x200)
#define SMHC_CMD_ATTR          (0x204)
#define SMHC_TO_CTRL2          (0x208)
#define SMHC_ATC               (0x210) 
#define SMHC_RTC               (0x214)
#define SMHC_DITC0             (0x218)
#define SMHC_DITC1             (0x21C)
#define SMHC_TP0               (0x220)
#define SMHC_TP1               (0x224)

#define SMHC_CRC_STA           (0x240)
#define SMHC_TBC0              (0x244)
#define SMHC_TBC1              (0x248)
#define SMHC_BL                (0x24C)
#define SMHC_CEDBN             (0x250)
//--------------------------------------------------------------------------------------

#define smhc_readl(host, reg) \
	__raw_readl((host)->reg_base + reg)
#define smhc_writel(host, reg, value) \
	__raw_writel((value), (host)->reg_base + reg)
#define smhc_readw(host, reg) \
	__raw_readw((host)->reg_base + reg)
#define smhc_writew(host, reg, value) \
	__raw_writew((value), (host)->reg_base + reg)
#define smhc_clr_bit(host, reg, bitmap) \
	do{									\
		u32 tmp = smhc_readl(host,reg);	\
		tmp &= ~(bitmap);				\
		smhc_writel(host,reg,tmp);		\
	}while(0)

#define smhc_set_bit(host, reg, bitmap) \
	do{ 								\
		u32 tmp = smhc_readl(host,reg); \
		tmp |= (bitmap);				\
		smhc_writel(host,reg,tmp);		\
	}while(0)



/* control register bit field */
/*0x2c*/
#define ResetAll            (0x1U<<24)
#define ResetCmd            (0x1U<<25)
#define ResetDat            (0x1U<<26)
#define SdclkEn	            (0x1U<<2)

/*0x200*/
#define CPUAcessBuffEn      (0x1U<<31)
#define StopReadClkAtBlkGap (0x1U<<8)
#define SWDebounceMode      (0x1U<<5)
#define DebounceEnb         (0x1U<<4)
#define CdDat3En            (0x1U<<3)
#define SdclkIdleCtrl       (0x1U<<2)


/* Struct for SMC Commands */
/*0x18*/
#define CMDType         (0x3U<<22)
#define DataExp         (0x1U<<21)
#define CheckRspIdx     (0x1U<<20)
#define CheckRspCRC     (0x1U<<19)
#define NoRsp           (0x0U<<16)
#define Rsp136          (0x1U<<16)
#define Rsp48           (0x2U<<16)
#define Rsp48b          (0x3U<<16)
#define SingleBlkTrans  (0x0U<<5)
#define MultiBlkTrans   (0x1U<<5)
#define Read            (0x1U<<4)
#define Write           (0x0U<<4)
#define AutoCmd12       (0x1U<<2)
#define AutoCmd23       (0x2U<<2)
#define BlkCntEn        (0x1U<<1)
#define DMAEn           (0x1U<<0)

/*0x204*/
#define SendInitSeq     (0x1U<<4)
#define DisableBoot     (0x1U<<3)
#define BootACKExp      (0x1U<<2)
#define AltBootMode     (0x2U<<0)
#define MandBootMode    (0x1U<<0)

/*0x03C*/
#define Switch3v3To1v8  (0x1U<<19)
#define DdrType         (0x7<<16)
#define DDR_SHIFT       (16)

/*0x24*/
#define CmdLineSta      (0x1U<<24)
#define Dat3LineSta     (0x1U<<23)
#define Dat2LineSta     (0x1U<<22)
#define Dat1LineSta     (0x1U<<21)
#define Dat0LineSta     (0x1U<<20)
#define WpPinSta        (0x1U<<19)
#define CdPinInvSta     (0x1U<<18)
#define CardStable      (0x1U<<17)
#define CardInsert      (0x1U<<16)
#define BuffRDEn        (0x1U<<11)
#define BuffWREn        (0x1U<<10)
#define RDTransActive   (0x1U<<9)
#define WRTransActive   (0x1U<<8)
#define DatLineActive   (0x1U<<2)
#define CmdInhibitDat   (0x1U<<1)
#define CmdInhibitCmd   (0x1U<<0)


#define BootDataStart     (0x1U<<29)
#define BootAckRcv        (0x1U<<28)
//
#define DSFOInt		        (0x1U<<30)
#define DmaErrInt         (0x1U<<25)
#define AcmdErrInt        (0x1U<<24)
#define DatEndBitErrInt   (0x1U<<22)
#define DatCRCErrInt      (0x1U<<21)
#define DatTimeoutErrInt  (0x1U<<20)
#define CmdIdxErrInt      (0x1U<<19)
#define CmdEndBitErrInt   (0x1U<<18)
#define CmdCRCErrInt      (0x1U<<17)
#define CmdTimeoutErrInt  (0x1U<<16)
#define ErrInt            (0x1U<<15)
#define CardInt           (0x1U<<8)
#define CardRemoveInt     (0x1U<<7)
#define CardInsertInt     (0x1U<<6)
#define BuffRDRdyInt      (0x1U<<5)
#define BuffWRRdyInt      (0x1U<<4)
#define DmaInt            (0x1U<<3)
//#define BlkGapEvtInt      (0x1U<<2)
#define TransOverInt      (0x1U<<1)
#define CmdOverInt        (0x1U<<0)
#define TxDatIntBit       ( DmaInt | TransOverInt | DmaErrInt | ErrInt)
#define RxDatIntBit       ( DmaInt | TransOverInt | DmaErrInt | ErrInt)
#define DmaIntBit         (DmaInt | DmaErrInt)
#define ErrIntBit         (0x437F0000)//(0x1ff<<16)

//0x28 SMHC_CTRL1 bit field
#define Dma32BitSel       (0x3<<3)
#define DmaSel            (0x3<<3)
#define BusWidth          (0x1<<1)
#define ExtBusWidth       (0x1<<5)


/*0x3C Auto CMD Error Status */
#define NoAcmd12          (0x1U<<7)
#define AcmdIdxErr        (0x1U<<4)
#define AcmdEndBitErr     (0x1U<<3)
#define AcmdCRCErr        (0x1U<<2)
#define AcmdTimeoutErr    (0x1U<<1)
#define NotIssueAcmd      (0x0<<0)


//#define SMHC_DES_NUM_SHIFT	(16)
#define SMHC_DES_NUM_SHIFT	(15)
#define SMHC_DES_BUFFER_MAX_LEN	(1U << SMHC_DES_NUM_SHIFT)


enum
{
	ACT_NOP = 0,
	ACT_RSV,
	ACT_TRANS,
	ACT_LINK,
};

enum
{
	MAN_BOOT_MD = 0,
	ALT_BOOT_MD,
};


struct sdhc_idma_des{
	u32 valid           :1, //=1: indicates this line of descriptor is effective. =0: generate ADMA Error interrupt and stop ADMA to prevent runaway.
		end             :1, //=1: indicates end of descriptor. The Transfer Complete Interrupt is generated when the operation of the descriptor line is completed.
		int_en          :1, //=1: generates DMA Interrupt when the operation of the descriptor line is completed.
		                :1,
		act             :2, //00b: Nop, No Operation, Do not execute current line and go to next line.
		                    //01b: rsv, reserved, (Same as Nop. Do not execute current line and go to next line.)
		                    //10b: Tran, transfer data, Transfer data of one descriptor line Transfer data of one descriptor line
		                    //11b: Link, Link Descriptor, Link to another descriptor
		                :10,
		length          :16;
	u32 addr;
};


struct sunxi_mmc_ctrl_regs {

};



/* UHS-I Operation Modes
 * DS		25MHz	12.5MB/s	3.3V
 * HS		50MHz	25MB/s		3.3V
 * SDR12	25MHz	12.5MB/s	1.8V
 * SDR25	50MHz	25MB/s		1.8V
 * SDR50	100MHz	50MB/s		1.8V
 * SDR104	208MHz	104MB/s		1.8V
 * DDR50	50MHz	50MB/s		1.8V
 * MMC Operation Modes
 * DS		26MHz	26MB/s		3/1.8/1.2V
 * HS		52MHz	52MB/s		3/1.8/1.2V
 * HSDDR	52MHz	104MB/s		3/1.8/1.2V
 * HS200	200MHz	200MB/s		1.8/1.2V
 *
 * Spec. Timing
 * SD3.0
 * Fcclk    Tcclk   Fsclk   Tsclk   Tis     Tih     odly  RTis     RTih
 * 400K     2.5us   24M     41ns    5ns     5ns     1     2209ns   41ns
 * 25M      40ns    600M    1.67ns  5ns     5ns     3     14.99ns  5.01ns
 * 50M      20ns    600M    1.67ns  6ns     2ns     3     14.99ns  5.01ns
 * 50MDDR   20ns    600M    1.67ns  6ns     0.8ns   2     6.67ns   3.33ns
 * 104M     9.6ns   600M    1.67ns  3ns     0.8ns   1     7.93ns   1.67ns
 * 208M     4.8ns   600M    1.67ns  1.4ns   0.8ns   1     3.33ns   1.67ns

 * 25M      40ns    300M    3.33ns  5ns     5ns     2     13.34ns   6.66ns
 * 50M      20ns    300M    3.33ns  6ns     2ns     2     13.34ns   6.66ns
 * 50MDDR   20ns    300M    3.33ns  6ns     0.8ns   1     6.67ns    3.33ns
 * 104M     9.6ns   300M    3.33ns  3ns     0.8ns   0     7.93ns    1.67ns
 * 208M     4.8ns   300M    3.33ns  1.4ns   0.8ns   0     3.13ns    1.67ns

 * eMMC4.5
 * 400K     2.5us   24M     41ns    3ns     3ns     1     2209ns    41ns
 * 25M      40ns    600M    1.67ns  3ns     3ns     3     14.99ns   5.01ns
 * 50M      20ns    600M    1.67ns  3ns     3ns     3     14.99ns   5.01ns
 * 50MDDR   20ns    600M    1.67ns  2.5ns   2.5ns   2     6.67ns    3.33ns
 * 200M     5ns     600M    1.67ns  1.4ns   0.8ns   1     3.33ns    1.67ns
 */
#define MMC_CLK_400K					0
#define MMC_CLK_25M						1
#define MMC_CLK_50M						2
#define MMC_CLK_50MDDR				3
#define MMC_CLK_50MDDR_8BIT		4
#define MMC_CLK_100M					5
#define MMC_CLK_200M					6
#define MMC_CLK_MOD_NUM				7

struct sunxi_mmc_clk_dly {
	u32 mode;
	u32 oclk_dly;
	u32 sclk_dly;
};

struct sunxi_mmc_platform_data {
	/* predefine information */
	u32 ocr_avail;
	u32 caps;
	u32 caps2;
	u32 f_min;
	u32 f_max;
	u32 f_ddr_max;
	u32 dma_tl;
	char* regulator;
	char* power_supply;

	u32  used_ddrmode;
	/* sys config information */
	u32 used:8,
	    cdmode:8,
	    width:4,
	    wpmode:4,
	    has_hwrst:4,
	    isiodev:4;

	struct gpio_config mmcio[10];
	struct gpio_config hwrst;
	struct gpio_config cd;
	struct gpio_config wp;
	struct pinctrl *pinctrl;

	/*sample delay and output deley setting*/
	struct sunxi_mmc_clk_dly mmc_clk_dly[MMC_CLK_MOD_NUM];
};

struct sunxi_mmc_host {

	struct platform_device  *pdev;
	struct mmc_host *mmc;
	struct sunxi_mmc_platform_data  *pdata;

	/* IO mapping base */
	void __iomem 	*reg_base;

	spinlock_t 	lock;
	struct tasklet_struct tasklet;
	//for dat3 detect
	struct tasklet_struct d3_det_tasklet;

	/* clock management */
	struct clk 	*hclk;
	struct clk 	*mclk;

	/* ios information */
	u32 		mod_clk;
	u32 		card_clk;
	u32 		oclk_dly;
	u32 		sclk_dly;
	u32 		bus_width;
	u32 		ddr;
	u32 		voltage;
#define SDC_WOLTAGE_3V3 (0)
#define SDC_WOLTAGE_1V8 (1)
#define SDC_WOLTAGE_1V2 (2)
#define SDC_WOLTAGE_OFF (3)
#define SDC_WOLTAGE_ON  (4)
	u32 		voltage_switching;
	struct regulator *regulator;
	u32         regulator_voltage:8; //record the initial io voltage
	u32 		present;

	/* irq */
	int 		irq;
	volatile u32	int_sum;

	volatile u32 	trans_done:1;
	volatile u32 	dma_done:1;
	dma_addr_t	sg_dma;
	void		*sg_cpu;

	struct mmc_request *mrq;
	volatile u32	error;
	volatile u32	ferror;
	volatile u32	wait;
#define SDC_WAIT_NONE		(1<<0)
#define SDC_WAIT_CMD_DONE	(1<<1)
#define SDC_WAIT_DATA_OVER	(1<<2)
#define SDC_WAIT_AUTOCMD_DONE	(1<<3)
#define SDC_WAIT_DMA_DONE	(1<<4)
#define SDC_WAIT_RXDATA_OVER	(SDC_WAIT_DATA_OVER|SDC_WAIT_DMA_DONE)
#define SDC_WAIT_RXAUTOCMD_DONE	(SDC_WAIT_AUTOCMD_DONE|SDC_WAIT_DMA_DONE)
#define SDC_WAIT_ERROR		(1<<6)
#define SDC_WAIT_SWITCH1V8	(1<<7)
#define SDC_WAIT_FINALIZE	(1<<8)
	volatile u32	state;
#define SDC_STATE_IDLE		(0)
#define SDC_STATE_SENDCMD	(1)
#define SDC_STATE_CMDDONE	(2)

	struct timer_list cd_timer;
	u32 pio_hdle;
	s32 cd_hdle;
	s32 cd_mode;
#define CARD_DETECT_BY_GPIO_POLL (1)	/* mmc detected by gpio check */
#define CARD_DETECT_BY_GPIO_IRQ  (2)	/* mmc detected by gpio irq */
#define CARD_ALWAYS_PRESENT      (3)	/* mmc always present, without detect pin */
#define CARD_DETECT_BY_FS        (4)	/* mmc insert/remove by fs, /proc/sunxi-mmc.x/insert node */
#define CARD_DETECT_BY_D3				 (5)	/* mmc detected by data3*/

	u32 power_on:8;
	u32 read_only:8;
	u32 io_flag:8;
	u32 suspend:8;

	u32 power_for_card;/*used for card power on flag*/

	u32 debuglevel;
	u32 dump_ctl;
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry *proc_root;
	struct proc_dir_entry *proc_drvver;
	struct proc_dir_entry *proc_hostinfo;
	struct proc_dir_entry *proc_dbglevel;
	struct proc_dir_entry *proc_regs;
	struct proc_dir_entry *proc_insert;
	struct proc_dir_entry *proc_cdmode;
	struct proc_dir_entry *proc_iodrive;
#endif

	/* backup register structrue */
	struct sunxi_mmc_ctrl_regs bak_regs;
};

#define SMC_MSG(d, ...)    do { printk("[mmc]: "__VA_ARGS__); } while(0)
#define SMC_ERR(d, ...)    do { printk("[mmc]: *** %s(L%d): ", __FUNCTION__, __LINE__); \
					printk(__VA_ARGS__);} while(0)

#define SMC_DEBUG_INFO	BIT(0)
#define SMC_DEBUG_DBG	BIT(1)
#ifdef CONFIG_MMC_DEBUG_SUNXI
#define SMC_INFO(d, ...)   do {if ((d)->debuglevel & SMC_DEBUG_INFO) 	\
				SMC_MSG(d, __VA_ARGS__); } while(0)
#define SMC_DBG(d, ...)    do {if ((d)->debuglevel & SMC_DEBUG_DBG) 	\
				SMC_MSG(d, __VA_ARGS__); } while(0)
#else
#define SMC_INFO(d, ...)
#define SMC_DBG(d, ...)
#endif

#ifndef	__io_address
#define __io_address(n)     IOMEM(IO_ADDRESS(n))
#endif

#endif
