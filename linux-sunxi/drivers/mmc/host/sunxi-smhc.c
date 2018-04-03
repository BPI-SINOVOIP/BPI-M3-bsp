/*
 * drivers/mmc/host/sunxi-mci.c
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-sunxi.h>
#include <linux/device.h>


#include <linux/mmc/host.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>

#include <asm/cacheflush.h>
#include <asm/uaccess.h>


#include <mach/hardware.h>
#include <mach/platform.h>
#include <mach/sys_config.h>
#include <mach/gpio.h>
#include <mach/sunxi-chip.h>
#include <linux/clk/sunxi.h>
#include <linux/regulator/consumer.h>

#include "sunxi-smhc.h"

static struct sunxi_mmc_host* sunxi_host[] = {NULL, NULL, NULL, NULL};


#if 0
static void uart_put(char c)
{
	void __iomem *base = __io_address(0x01c28000);
	while (readl(base + 0x80) > 48);
	writel(c, base);
}

static void uart_puts(char* s)
{
	u32 i = strlen(s);
	while (i--)
		uart_put(*s++);
}

static void uart_puts_u32hex(u32 c)
{
	char ch = 0;
	uart_put('0');
	uart_put('x');
	ch = (c >> 28) & 0xf;
	ch = ch > 9 ? 'a' + ch - 10 : '0' + ch;
	uart_put(ch);
	ch = (c >> 24) & 0xf;
	ch = ch > 9 ? 'a' + ch - 10 : '0' + ch;
	uart_put(ch);
	ch = (c >> 20) & 0xf;
	ch = ch > 9 ? 'a' + ch - 10 : '0' + ch;
	uart_put(ch);
	ch = (c >> 16) & 0xf;
	ch = ch > 9 ? 'a' + ch - 10 : '0' + ch;
	uart_put(ch);
	ch = (c >> 12) & 0xf;
	ch = ch > 9 ? 'a' + ch - 10 : '0' + ch;
	uart_put(ch);
	ch = (c >> 8) & 0xf;
	ch = ch > 9 ? 'a' + ch - 10 : '0' + ch;
	uart_put(ch);
	ch = (c >> 4) & 0xf;
	ch = ch > 9 ? 'a' + ch - 10 : '0' + ch;
	uart_put(ch);
	ch = (c >> 0) & 0xf;
	ch = ch > 9 ? 'a' + ch - 10 : '0' + ch;
	uart_put(ch);
}

static void hexdump(struct sunxi_mmc_host* smc_host, char* name, char* base, int len)
{
	u32 i;

	if (smc_host->dump_ctl)
	{
		printk("%s :", name);
		for (i=0; i<len; i++) {
			if (!(i&0x1f))
				printk("\n0x%p : ", base + i);
			if (!(i&0xf))
				printk(" ");
			printk("%02x ", readb(base + i));
		}
		printk("\n");
	}
}

#endif

#if 1
static void dumphex32(struct sunxi_mmc_host* smc_host, char* name, char* base, int len)
{
	u32 i;

//	if (smc_host->dump_ctl)
//	{
		printk("dump %s registers:", name);
		for (i=0; i<len; i+=4) {
			if (!(i&0xf))
				printk("\n0x%p : ", base + i);
			printk("0x%08x ", readl(base + i));
		}
		printk("\n");
//	}
}


#define isascii(c)	(((unsigned char)(c))<=0x7f)
#define isprint(c)	((((unsigned char)(c))>=32 && ((unsigned char)(c))<=126) \
			|| (((unsigned char)(c))>=160 && ((unsigned char)(c))<=255))

#define DBG_DUMP_BUF_LEN 	(128)
#define MAX_DUMP_PER_LINE	(16)
#define MAX_DUMP_PER_LINE_HALF	(MAX_DUMP_PER_LINE >> 1)
void hexdump(char *prompt, char *buf, int len)
{
	int i, j;
	int head = 0;

	printk("Dump (%s): len %d\n", prompt, len);
	for (i = 0; i < len;) {
		if (i % MAX_DUMP_PER_LINE == 0) {
			printk("%08x  ", i);
			head = i;
		}
		printk("%02x ", buf[i]&0xff);
		if (i % MAX_DUMP_PER_LINE == MAX_DUMP_PER_LINE_HALF-1)
			printk(" ");
		if (i % MAX_DUMP_PER_LINE == MAX_DUMP_PER_LINE-1
			|| i==len-1) {
			for (j=i-head+1; j<MAX_DUMP_PER_LINE; j++)
				printk("   ");
			printk(" |");
			for (j=head; j<=i; j++) {
				if (isascii(buf[j]) && isprint(buf[j]))
					printk("%c", buf[j]);
				else
					printk(".");
			}
			printk("|\n");
		}

		i++;
	}

	printk("\n");
}

#else

#define dumphex32(fmt...)

#endif


static int sunxi_wait_bit_clr(struct sunxi_mmc_host* smc_host,u32 reg_add,u32 bit_map,s8 *reg_add_str,s8 *bit_map_str,u32 timeout_ms)
{
	unsigned long expire = 0;
	u32 tmp = 0;
	s32 ret = 0;

	expire = jiffies + msecs_to_jiffies(timeout_ms); // 1ms timeout
	do {
		//SMC_DBG(smc_host,"Wait reg %s(%x),bit %s(%x) \n",
		//		reg_add_str,reg_add,bit_map_str,bit_map);
		tmp = smhc_readl(smc_host, reg_add);
	} while (time_before(jiffies, expire) && (tmp & bit_map));

	tmp = smhc_readl(smc_host, reg_add);
	if (tmp & bit_map) {
		SMC_ERR(smc_host,"Wait reg %s(%x),bit %s(%x) %d ms timeout\n",\
				reg_add_str,reg_add, bit_map_str, bit_map, timeout_ms);
		ret = 1;
	} else {
		ret = 0;
	}

	return ret;
}

static s32 sunxi_mci_init_host(struct sunxi_mmc_host* smc_host)
{
	u32 tmp;
	SMC_DBG(smc_host, "MMC Driver init host %d\n", smc_host->pdev->id);

	//reset control
	tmp = smhc_readl(smc_host, SMHC_RST_CLK_CTRL);
	smhc_writel(smc_host, SMHC_RST_CLK_CTRL,tmp|ResetAll);
	//wait reset done
	if (sunxi_wait_bit_clr(smc_host,	\
		SMHC_RST_CLK_CTRL,ResetAll,	\
		"SMHC_RST_CLK_CTRL","ResetAll",1)){
		return -1;
	}

	smhc_writel(smc_host,SMHC_INT_STA_EN, 0xffffffff);
	smhc_writel(smc_host,SMHC_INT_STA, 0xffffffff);
	SMC_DBG(smc_host,"int sta %x\n",smhc_readl(smc_host,SMHC_INT_STA));

	//Set Data & Response Timeout Value
	#define  SMC_DATA_TIMEOUT     0xffffffU
	#define  SMC_RESP_TIMEOUT     0xff
	smhc_writel(smc_host, SMHC_TO_CTRL2, (SMC_DATA_TIMEOUT<<8)|SMC_RESP_TIMEOUT);

	//stop clock at block gap, bit8
	tmp = smhc_readl(smc_host,SMHC_CTRL3);
	smhc_writel(smc_host, SMHC_CTRL3, tmp|StopReadClkAtBlkGap);

	//disable dat3 int
	if (smc_host->cd_mode == CARD_DETECT_BY_D3) {
		//enable data3 detect
		smhc_clr_bit(smc_host,SMHC_CTRL3,SWDebounceMode);
		smhc_set_bit(smc_host,SMHC_CTRL3,CdDat3En);

		//enable data3 detect int
		smhc_set_bit(smc_host,SMHC_INT_STA_EN,CardRemoveInt|CardInsertInt);
		smhc_set_bit(smc_host,SMHC_INT_SIG_EN,CardRemoveInt|CardInsertInt);
	} else {
		smhc_clr_bit(smc_host,SMHC_INT_STA_EN,CardRemoveInt|CardInsertInt);
		smhc_clr_bit(smc_host,SMHC_INT_SIG_EN,CardRemoveInt|CardInsertInt);
	}
	
	dumphex32(smc_host, "mmc", IOMEM(IO_ADDRESS(SMHC_BASE_ADDR)), 0x100);
	dumphex32(smc_host, "gpio", IO_ADDRESS(SUNXI_PIO_BASE), 0x120);

	return 0;
}

static s32 sunxi_mci_exit_host(struct sunxi_mmc_host* smc_host)
{
	u32 tmp;
	SMC_DBG(smc_host, "MMC Driver exit host %d\n", smc_host->pdev->id);
	//reset control
	tmp = smhc_readl(smc_host, SMHC_RST_CLK_CTRL);
	smhc_writel(smc_host, SMHC_RST_CLK_CTRL, tmp|ResetAll);
	//wait reset done
	if (sunxi_wait_bit_clr(smc_host,	\
		SMHC_RST_CLK_CTRL,ResetAll,	\
		"SMHC_RST_CLK_CTRL","ResetAll",1)){
		return -1;
	}
	return 0;
}

static s32 sunxi_mci_set_vddio(struct sunxi_mmc_host* smc_host, u32 vdd)
{
	char* vddstr[] = {"3.3V", "1.8V", "1.2V", "OFF", "ON"};
	static u32 on[4] = {0};
	u32 id = smc_host->pdev->id;
	s32 ret = 0;

	if (smc_host->regulator == NULL)
		return 0;
	BUG_ON(vdd > SDC_WOLTAGE_ON);
	switch (vdd) {
		case SDC_WOLTAGE_3V3:
			//regulator_set_voltage(smc_host->regulator, 3300000, 3300000);
			ret = regulator_set_voltage(smc_host->regulator, 3000000, 3000000);
			if (IS_ERR(smc_host->regulator)) {
				SMC_ERR(smc_host,"sdc%d regulator set voltage 3.0V failed!\n", id);
				return ret;
			}
			if (!on[id]) {
				SMC_ERR(smc_host, "regulator on\n");
				ret = regulator_enable(smc_host->regulator);
				if (IS_ERR(smc_host->regulator)) {
					SMC_ERR(smc_host,"sdc%d regulator enable failed 3.0V!\n", id);
					return ret;
				} else {
					SMC_DBG(smc_host, "sdc%d regulator enable successful 3.0V\n", id);
					on[id] = 1;
				}
			}
			break;
		case SDC_WOLTAGE_1V8:
			ret = regulator_set_voltage(smc_host->regulator, 1800000, 1800000);
			if (IS_ERR(smc_host->regulator)) {
				SMC_ERR(smc_host,"sdc%d regulator set voltage 1.8V failed!\n", id);
				return ret;
			}
			if (!on[id]) {
				SMC_DBG(smc_host, "regulator on\n");
				ret = regulator_enable(smc_host->regulator);
				if (IS_ERR(smc_host->regulator)) {
					SMC_ERR(smc_host,"sdc%d regulator enable failed! 1.8V\n", id);
					return ret;
				} else {
					SMC_DBG(smc_host, "sdc%d regulator enable successful 1.8V\n", id);
					on[id] = 1;
				}
			}
			break;
		case SDC_WOLTAGE_1V2:
			ret = regulator_set_voltage(smc_host->regulator, 1200000, 1200000);
			if (IS_ERR(smc_host->regulator)) {
				SMC_ERR(smc_host,"sdc%d regulator set voltage 1.2V failed!\n", id);
				return ret;
			}
			if (!on[id]) {
				SMC_DBG(smc_host, "regulator on\n");
				ret = regulator_enable(smc_host->regulator);
				if (IS_ERR(smc_host->regulator)) {
					SMC_ERR(smc_host,"sdc%d regulator enable failed! 1.2V\n", id);
					return ret;
				} else {
					SMC_DBG(smc_host, "sdc%d regulator enable successful 1.2V\n", id);
					on[id] = 1;
				}
			}
			break;
		case SDC_WOLTAGE_OFF:
			if (on[id]) {
				SMC_DBG(smc_host, "regulator off\n");
				ret = regulator_disable(smc_host->regulator);
				if (IS_ERR(smc_host->regulator)) {
					SMC_ERR(smc_host,"sdc%d regulator off failed!\n", id);
					return ret;
				} else {
					SMC_DBG(smc_host,"sdc%d regulator off successful!\n", id);
				}
				on[id] = 0;
			}
			break;
		case SDC_WOLTAGE_ON:
			if (!on[id]) {
				SMC_DBG(smc_host, "regulator on\n");
				ret = regulator_enable(smc_host->regulator);
				if (IS_ERR(smc_host->regulator)) {
					SMC_ERR(smc_host,"sdc%d regulator enable failed! 1.2V\n", id);
					return ret;
				} else {
					SMC_DBG(smc_host, "sdc%d regulator enable successful 1.2V\n", id);
					on[id] = 1;
				}
			}
			break;

	}

	SMC_MSG(smc_host, "sdc%d switch io voltage to %s\n", smc_host->pdev->id, vddstr[vdd]);
	return 0;
}

static s32 sunxi_mci_get_vddio(struct sunxi_mmc_host* smc_host)
{
	int voltage;
	char *vol_str[4] = {"3.3V","1.8V","1.2V","0V"};

	if (smc_host->regulator == NULL)
		return 0;
	voltage = regulator_get_voltage(smc_host->regulator);
	if (IS_ERR(smc_host->regulator)) {
		SMC_ERR(smc_host, "Get vddio voltage fail !\n");
		return -1;
	}

	if (voltage > 1700000 && voltage < 1950000)
		smc_host->regulator_voltage = SDC_WOLTAGE_1V8;
	if (voltage > 2700000 && voltage < 3600000)
		smc_host->regulator_voltage = SDC_WOLTAGE_3V3;

	SMC_DBG(smc_host, "%s:card io voltage mode:%s\n", __FUNCTION__, vol_str[smc_host->regulator_voltage]);
	return 0 ;
}

static int card_power_on(struct sunxi_mmc_host* smc_host)
{
	int voltage;
	struct regulator *regu;
	int ret = 0;

	SMC_MSG(NULL, "sdc%d card_power_on start...\n", smc_host->pdev->id);

	if (smc_host->pdata->power_supply == NULL) {
		SMC_MSG(NULL, "sdc%d power_supply is null\n", smc_host->pdev->id);
		return ret;
	}

	regu = regulator_get(NULL, smc_host->pdata->power_supply);
	if(IS_ERR(regu)) {
		SMC_MSG(NULL, "sdc%d fail to get regulator\n", smc_host->pdev->id);
		return -1;
	}

	//set output voltage to 3V
	voltage = regulator_get_voltage(regu);
	if (IS_ERR(smc_host->regulator)) {
		SMC_ERR(smc_host, "get power supply voltage fail\n");
		return -1;
	}
	if (!(voltage > 2700000 && voltage < 3600000)) {
		ret = regulator_set_voltage(regu, 3000000, 3300000);
		if(ret) {
			SMC_MSG(NULL, "sdc%d fail to set regulator voltage 2.7~3.6V \n", smc_host->pdev->id);
			return ret;
		}
	}

	//enable regulator
	ret = regulator_enable(regu);
	if(ret) {
		SMC_MSG(NULL, "sdc%d fail to enable regulator\n", smc_host->pdev->id);
		return ret;
	}

	//put regulator when module exit
	regulator_put(regu);

	SMC_MSG(NULL, "sdc%d card_power_on end ...ok\n", smc_host->pdev->id);
	return ret;
}

static int card_power_off(struct sunxi_mmc_host* smc_host)
{
	struct regulator *regu;
	int ret = 0;

	SMC_MSG(NULL,"sdc%d card_power_off start...\n", smc_host->pdev->id);

	if(smc_host->pdata->power_supply == NULL) {
		SMC_MSG(NULL, "sdc%d power_supply is null\n", smc_host->pdev->id);
		return ret;
	}

	regu = regulator_get(NULL,smc_host->pdata->power_supply);
	if(IS_ERR(regu)) {
		SMC_MSG(NULL, "sdc%d fail to get regulator\n", smc_host->pdev->id);
		return -1;
	}

	//diable regulator
	ret = regulator_disable(regu);
	if(ret) {
		SMC_MSG(NULL, "sdc%d fail to disable regulator\n", smc_host->pdev->id);
		return ret;
	}

	//put regulator when module exit
	regulator_put(regu);

	SMC_MSG(NULL, "sdc%d card_power_off end ...ok\n", smc_host->pdev->id);
	return ret;
}

static s32 sunxi_mci_oclk_onoff(struct sunxi_mmc_host* smc_host, u32 oclk_en, u32 pwr_save)
{
	u32 tmp = 0;

	tmp = smhc_readl(smc_host,SMHC_RST_CLK_CTRL);
	if (oclk_en) {
		tmp |= SdclkEn;
	} else {
		tmp &= ~SdclkEn;
	}
	smhc_writel(smc_host,SMHC_RST_CLK_CTRL,tmp);

	tmp = smhc_readl(smc_host,SMHC_CTRL3);
	if (pwr_save && !smc_host->io_flag) {
		tmp |= SdclkIdleCtrl;
	} else {
		tmp &= ~SdclkIdleCtrl;
	}
	smhc_writel(smc_host,SMHC_CTRL3,tmp);

	return 0;
}

static void sunxi_mci_send_cmd(struct sunxi_mmc_host* smc_host, struct mmc_command* cmd)
{
	u32 cmd_val 	= 0;
	u32 wait 		= SDC_WAIT_NONE;
	u32 int_sg_en 	= 0;
	u32 int_st_en	= 0;
	u32 cmd_attr 	= 0;
	u32 tmp			= 0;
	
	/**************arg*******************/
	smhc_writel(smc_host,SMHC_CMD_ARG1,cmd->arg);
	SMC_DBG(smc_host,"stop %x,sbc %x\n", (u32)(cmd->mrq->stop), (u32)(cmd->mrq->sbc));
	//if sbc is set,use cmd23 instead of cmd12 ,regardless if stop is set
	if (cmd->mrq->sbc) {
		smhc_writel(smc_host, SMHC_CMD_ARG2, cmd->mrq->sbc->arg);
	} else if(cmd->mrq->stop) {
		smhc_writel(smc_host, SMHC_CMD_ARG2, cmd->mrq->stop->arg);
	}

	/*************cmd val***************/
	//response
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			cmd_val |= Rsp136;
		} else if(cmd->flags & MMC_RSP_BUSY) {
			cmd_val |= Rsp48b;
		} else {
			cmd_val |= Rsp48;
		}
		if (cmd->flags & MMC_RSP_CRC) {
			cmd_val |= CheckRspCRC;
		}
		if (cmd->flags & MMC_RSP_OPCODE) {
			cmd_val |= CheckRspIdx;
		}
	}

	//data
	if ((cmd->flags & MMC_CMD_MASK) == MMC_CMD_ADTC) {
		cmd_val |= DataExp;
		if (cmd->data->blocks > 1) {
			cmd_val |= MultiBlkTrans|BlkCntEn;
		}
		if (cmd->data->flags & MMC_DATA_READ) {
			cmd_val |= Read;
		}
		//if sbc is set,use cmd23 instead of cmd12 ,regardless if stop is set
		if (cmd->mrq->sbc != NULL) {
			cmd_val |= AutoCmd23;
		} else if(cmd->mrq->stop != NULL) {
			cmd_val |= AutoCmd12;
		}
		cmd_val |= DMAEn;
	}
	cmd_val |= (cmd->opcode & 0x3f)<<24;

	/******************Send Initialization******************/
	if (cmd->opcode == MMC_GO_IDLE_STATE) {
		cmd_attr |= SendInitSeq;
	}
	tmp = smhc_readl(smc_host, SMHC_CMD_ATTR);
	tmp &=~SendInitSeq;//clear Initialization first
	smhc_writel(smc_host, SMHC_CMD_ATTR, cmd_attr|tmp);

	/***************irq setting and the wait in irq *********************/
	if ((cmd->flags & MMC_CMD_MASK) == MMC_CMD_ADTC) {
		wait = SDC_WAIT_DATA_OVER;
		int_sg_en = TransOverInt;
		if (cmd->data->flags & MMC_DATA_READ) {
			wait |= SDC_WAIT_DMA_DONE;
			int_sg_en |= DmaInt;
		}
//		wait |= SDC_WAIT_DMA_DONE;
//		int_sg_en |= DmaInt;
	} else {
		if (cmd->flags & MMC_RSP_BUSY) {
			wait = SDC_WAIT_DATA_OVER;
			int_sg_en = TransOverInt;
		} else {
			wait = SDC_WAIT_CMD_DONE;
			int_sg_en = CmdOverInt;
		}
	}

	smc_host->wait = wait;
	//enble all int state
	smhc_writel(smc_host, SMHC_INT_STA_EN, 0xffffffff);
	int_st_en = smhc_readl(smc_host, SMHC_INT_STA_EN);
	int_sg_en |= ErrIntBit;
	int_sg_en |= (smhc_readl(smc_host, SMHC_INT_SIG_EN)) & (CardInt|CardRemoveInt|CardInsertInt);
	smhc_writel(smc_host, SMHC_INT_SIG_EN, int_sg_en);

	SMC_MSG(smc_host, "smc %d cmd %d(%08x) cmd_val %x in_sg_en 0x%08x int_st_en 0x%08x wt %x len %d\n",
		smc_host->pdev->id,cmd->opcode&0x3f, cmd->arg, cmd_val, int_sg_en,int_st_en, wait,
		smc_host->mrq->data ? smc_host->mrq->data->blksz * smc_host->mrq->data->blocks : 0);

	//dumphex32(smc_host, "mmc----cmd", IOMEM(IO_ADDRESS(SMHC_BASE_ADDR)), 0x280);

	/******************exe cmd******************/
	smhc_writel(smc_host,SMHC_CMD,cmd_val);
	//SMC_DBG(smc_host,"fun %s,line %d\n",__FUNCTION__,__LINE__);

	//dumphex32(smc_host, "mmc--- io -- timing", IOMEM(IO_ADDRESS(SMHC_BASE_ADDR+0x200)), 0x60);
	smp_wmb();
}

static void sunxi_mci_init_idma_des(struct sunxi_mmc_host* smc_host, struct mmc_data* data)
{
	struct sdhc_idma_des* pdes = (struct sdhc_idma_des*)smc_host->sg_cpu;
//	struct sdhc_idma_des* pdes_pa = (struct sdhc_idma_des*)smc_host->sg_dma;
	struct scatterlist *sg = NULL;
	u32 i	= 0;

	for_each_sg(data->sg, sg, data->sg_len, i) {
		memset((void*)(&pdes[i]), 0, sizeof(struct sdhc_idma_des));
		pdes[i].valid = 1;
		pdes[i].end = 0;
		pdes[i].int_en = 0;
		pdes[i].act = ACT_TRANS;
		pdes[i].length = sg->length;
		pdes[i].addr = sg_dma_address(sg);
	}

	pdes[i-1].end = 1;
	pdes[i-1].int_en = 1;
	SMC_DBG(smc_host,"sg len %d end des index %d\n",data->sg_len, i-1);

	for_each_sg(data->sg, sg, data->sg_len, i) {
		SMC_DBG(smc_host, "sg %d, des[%d](%08x): [0] = %08x, [1] = %08x\n",
					i, i, (u32)&pdes[i],(u32)((u32*)&pdes[i])[0], (u32)((u32*)&pdes[i])[1]);
	}

	smp_wmb();
	return;
}

static int sunxi_mci_prepare_dma(struct sunxi_mmc_host* smc_host, struct mmc_data* data)
{
	u32 dma_len;
	u32 i;
	u32 tmp = 0;
	struct scatterlist *sg = NULL;

	if (smc_host->sg_cpu == NULL)
		return -ENOMEM;

	if(data->sg_len > (MAX_DES_SIZE/sizeof(struct sdhc_idma_des))){
		SMC_ERR(smc_host, "sg_len is over max sg length\n");
		return -ENOMEM;
	}

	dma_len = dma_map_sg(mmc_dev(smc_host->mmc), data->sg, data->sg_len,
			(data->flags & MMC_DATA_WRITE) ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	if (dma_len == 0) {
		SMC_ERR(smc_host, "no dma map memory\n");
		return -ENOMEM;
	}

	for_each_sg(data->sg, sg, data->sg_len, i) {
		if (sg->offset & 3
			|| sg->length & 3
			|| (sg->length > SMHC_DES_BUFFER_MAX_LEN)
			) {
			SMC_ERR(smc_host, "unaligned scatterlist: os %x length %d\n",
				sg->offset, sg->length);
			return -EINVAL;
		}
	}

	sunxi_mci_init_idma_des(smc_host, data);
	//dma access
	tmp = smhc_readl(smc_host,SMHC_CTRL3);
	tmp &= ~CPUAcessBuffEn;
	smhc_writel(smc_host,SMHC_CTRL3,tmp);

	//dma select
	tmp = smhc_readl(smc_host,SMHC_CTRL1);
	tmp &= ~DmaSel;
	tmp |= Dma32BitSel;
	smhc_writel(smc_host,SMHC_CTRL1,tmp);
	smhc_writel(smc_host,SMHC_ADMA_ADDR,smc_host->sg_dma);
	return 0;
}

/*
int sunxi_mci_send_manual_stop(struct sunxi_mmc_host* smc_host, struct mmc_request* req)
{

}
*/
static void sunxi_mci_dump_errinfo(struct sunxi_mmc_host* smc_host)
{
	void __iomem *mclk_base = NULL;

	SMC_ERR(smc_host, "smc %d err, cmd %d, %s%s%s%s%s%s%s%s%s%s !!\n",
		smc_host->pdev->id, smc_host->mrq->cmd ? smc_host->mrq->cmd->opcode : -1,
		smc_host->int_sum & DSFOInt     	? "SBE "    : "",
		smc_host->int_sum & DmaErrInt     	? "DME "    : "",
		smc_host->int_sum & AcmdErrInt  	? "ATE "    : "",
		smc_host->int_sum & DatEndBitErrInt ? "DEBE "    : "",
		smc_host->int_sum & DatCRCErrInt 	? "DCE "    : "",
		smc_host->int_sum & DatTimeoutErrInt? "DTO "    : "",
		smc_host->int_sum & CmdIdxErrInt  	? "RIE "    : "",
		smc_host->int_sum & CmdEndBitErrInt ? "REBE "    : "",
		smc_host->int_sum & CmdCRCErrInt 	? "RCE "    : "",
		smc_host->int_sum & CmdTimeoutErrInt? "RTO "    : ""
		);

		if (smc_host->int_sum & DatCRCErrInt)
		{
			dumphex32(smc_host, "mmc-off 0x0", IOMEM(IO_ADDRESS(SMHC_BASE_ADDR)), 0x60);
			dumphex32(smc_host, "mmc-off 0x200", IOMEM(IO_ADDRESS(SMHC_BASE_ADDR+0x200)), 0x90);
			mclk_base = __io_address(SUNXI_CCM_BASE + 0x88 + 0x4 * smc_host->pdev->id);
			dumphex32(smc_host, "ccmu", mclk_base, 0x4);
			dumphex32(smc_host, "gpio", IO_ADDRESS(SUNXI_PIO_BASE), 0x120);
		}
}

static s32 sunxi_mci_request_done(struct sunxi_mmc_host* smc_host)
{
	struct mmc_request* req = smc_host->mrq;
	u32 tmp;
	s32 ret = 0;

	if (smc_host->int_sum & ErrIntBit) {
		sunxi_mci_dump_errinfo(smc_host);
		if (req->data)
			SMC_ERR(smc_host, "In data %s operation\n",
				req->data->flags & MMC_DATA_WRITE ? "write" : "read");
		ret = -1;
		goto out;
	}

	if (req->cmd) {
		if (req->cmd->flags & MMC_RSP_136) {
			req->cmd->resp[0] = (smhc_readl(smc_host,SMHC_RESP3)&0xffffff)<<8;
			req->cmd->resp[0] |= (smhc_readl(smc_host,SMHC_RESP2)>>24)&0xff;
			req->cmd->resp[1] = (smhc_readl(smc_host,SMHC_RESP2)&0xffffff)<<8;
			req->cmd->resp[1] |= (smhc_readl(smc_host,SMHC_RESP1)>>24)&0xff;			
			req->cmd->resp[2] = (smhc_readl(smc_host,SMHC_RESP1)&0xffffff)<<8;
			req->cmd->resp[2] |= (smhc_readl(smc_host,SMHC_RESP0)>>24)&0xff;
			req->cmd->resp[3] =  (smhc_readl(smc_host,SMHC_RESP0)&0xffffff)<<8;
		} else {
			req->cmd->resp[0] = smhc_readl(smc_host,SMHC_RESP0);
		}
	}

out:
	if (req->data) {
		struct mmc_data* data = req->data;
		//recover to cpu access
		tmp = smhc_readl(smc_host, SMHC_CTRL3);
		tmp |= CPUAcessBuffEn;
		smhc_writel(smc_host, SMHC_CTRL3, tmp);

		// recover dma select
		tmp = smhc_readl(smc_host, SMHC_CTRL1);
		tmp &= ~DmaSel;
		smhc_writel(smc_host, SMHC_CTRL1, tmp);
		dma_unmap_sg(mmc_dev(smc_host->mmc), data->sg, data->sg_len,
                                data->flags & MMC_DATA_WRITE ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	}

/*
	//clear int status
	tmp = smhc_readl(smc_host,SMHC_INT_SIG_EN);
	tmp &= ~(CardInt|CardRemoveInt|CardInsertInt);
	smhc_writel(smc_host,SMHC_INT_STA,tmp);

	//clear  int signal enable
	tmp = smhc_readl(smc_host,SMHC_INT_SIG_EN);
	tmp &= (CardInt|CardRemoveInt|CardInsertInt);
	smhc_writel(smc_host,SMHC_INT_SIG_EN,tmp);
*/

	//clear int status
	smhc_writel(smc_host,SMHC_INT_STA,0xffffffff&~(CardInt|CardRemoveInt|CardInsertInt));
	//clear  int signal enable
	tmp = smhc_readl(smc_host,SMHC_INT_SIG_EN);
	tmp &= (CardInt|CardRemoveInt|CardInsertInt);
	smhc_writel(smc_host,SMHC_INT_SIG_EN,tmp);

	SMC_DBG(smc_host,"final sta %x\n",smhc_readl(smc_host,SMHC_INT_STA));

	SMC_DBG(smc_host, "smc %d done, resp %08x %08x %08x %08x\n", smc_host->pdev->id,
		req->cmd->resp[0], req->cmd->resp[1], req->cmd->resp[2], req->cmd->resp[3]);

	return ret;
}

/* static s32 sunxi_mci_set_clk(struct sunxi_mmc_host* smc_host, u32 clk);
 * set clock and the phase of output/input clock incording on
 * the different timing condition
 */
static int sunxi_mci_set_clk(struct sunxi_mmc_host* smc_host)
{
	struct clk *sclk = NULL;
	u32 mod_clk = 0;
	u32 src_clk = 0;
	s32 err;
	u32 rate;
	void __iomem *mclk_base = __io_address(SUNXI_CCM_BASE + 0x88 + 0x4 * smc_host->pdev->id);
	u32 clk = 0;

	mod_clk = smc_host->mod_clk;
	if (smc_host->card_clk <= 400000) {
		sclk = clk_get(&smc_host->pdev->dev, MMC_SRCCLK_HOSC);
	} else {
		sclk = clk_get(&smc_host->pdev->dev, MMC_SRCCLK_PLL);
	}
	if (NULL==sclk || IS_ERR(sclk)) {
		SMC_ERR(smc_host, "Error to get source clock for clk %dHz\n", clk);
		return -1;
	}

	sunxi_mci_oclk_onoff(smc_host, 0, 0);

	err = clk_set_parent(smc_host->mclk, sclk);
	if (err) {
		SMC_ERR(smc_host, "sdc%d set mclk parent error\n", smc_host->pdev->id);
		clk_put(sclk);
		return -1;
	}
	rate = clk_round_rate(smc_host->mclk, mod_clk);

	SMC_DBG(smc_host, "sdc %d before set round clock %d\n", smc_host->pdev->id, rate);
	SMC_DBG(smc_host, "mclk 0x%08x 0x%08x\n", (u32)mclk_base, readl(mclk_base));

	clk_disable_unprepare(smc_host->mclk);
	err = clk_set_rate(smc_host->mclk, rate);
	if (err) {
		SMC_ERR(smc_host, "sdc%d set mclk rate error, rate %dHz\n",
						smc_host->pdev->id, rate);
		clk_put(sclk);
		return -1;
	}

	clk_prepare_enable(smc_host->mclk);
	src_clk = clk_get_rate(sclk);
	clk_put(sclk);

	/* update clock info in smc_host */
	clk = smc_host->mod_clk  = rate;
	clk = smc_host->card_clk = smc_host->ddr?(rate/8):(rate/4);

	SMC_DBG(smc_host, "sdc%d set round clock %d, soure clk is %d\n", smc_host->pdev->id, rate, src_clk);
	SMC_MSG(smc_host,"mclk 0x%08x 0x%08x\n", (u32)mclk_base, readl(mclk_base));

	sunxi_mci_oclk_onoff(smc_host, 1, 1);

	return 0;
}

static int sunxi_mci_request_mmcio(struct sunxi_mmc_host *smc_host)
{
	s32 ret = 0;
	struct sunxi_mmc_platform_data *pdata = smc_host->pdata;
	u32 smc_no = smc_host->pdev->id;
	struct gpio_config *gpio;
	u32 i;
	char               pin_name[SUNXI_PIN_NAME_MAX_LEN] = {0};
	unsigned long      config;

	SMC_DBG(smc_host,"device [%s] probe enter\n", dev_name(&smc_host->pdev->dev));

	/* signal io */
	for (i=0; i<pdata->width+2; i++) {
		gpio = &pdata->mmcio[i];
		sunxi_gpio_to_name(gpio->gpio, pin_name);

		SMC_DBG(smc_host,"pin name %s,pull%d,drv_level%d,muxsel%d,gpio index %d\n",\
							pin_name,gpio->pull,gpio->drv_level,gpio->mul_sel,gpio->gpio);

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_PUD, gpio->pull);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set mmcio%d pull failed,ret = %d\n", smc_no, i,ret);
			return -1;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DRV, gpio->drv_level);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set mmcio%d drvlel failed,ret = %d\n", smc_no, i,ret);
			return -1;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, gpio->mul_sel);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set mmcio%d mulsel failed,ret = %d\n", smc_no, i,ret);
			return -1;
		}
	}

	/* emmc hwrst */
	if (pdata->has_hwrst) {
		gpio = &pdata->hwrst;
		sunxi_gpio_to_name(gpio->gpio, pin_name);

		SMC_DBG(smc_host,"pin name %s,pull%d,drv_level%d,muxsel%d,gpio index %d\n",\
							pin_name,gpio->pull,gpio->drv_level,gpio->mul_sel,gpio->gpio);

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_PUD, gpio->pull);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set io(emmc-rst) pull failed,ret = %d\n", smc_no,ret);
			return -1;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DRV, gpio->drv_level);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set io(emmc-rst) drvlel failed\n", smc_no);
			return -1;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, gpio->mul_sel);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set io(emmc-rst) mulsel failed\n", smc_no);
			return -1;
		}
	}

	/* write protect */
	if (pdata->wpmode) {
		gpio = &pdata->wp;

		sunxi_gpio_to_name(gpio->gpio, pin_name);

		SMC_DBG(smc_host,"pin name %s,pull%d,drv_level%d,muxsel%d,gpio index %d\n",\
							pin_name,gpio->pull,gpio->drv_level,gpio->mul_sel,gpio->gpio);

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_PUD, gpio->pull);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set wp pull failed\n", smc_no);
			return -1;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DRV, gpio->drv_level);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set wp drvlel failed\n", smc_no);
			return -1;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, gpio->mul_sel);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set wp mulsel failed\n", smc_no);
			return -1;
		}
	}

	/* cd mode == gpio polling */
	if ((pdata->cdmode == CARD_DETECT_BY_GPIO_POLL) || (pdata->cdmode == CARD_DETECT_BY_GPIO_IRQ)) {
		gpio_request(smc_host->pdata->cd.gpio, "cd");
		gpio = &pdata->cd;

		sunxi_gpio_to_name(gpio->gpio, pin_name);

		SMC_DBG(smc_host,"pin name %s,pull%d,drv_level%d,muxsel%d,gpio index %d\n",\
							pin_name,gpio->pull,gpio->drv_level,gpio->mul_sel,gpio->gpio);

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_PUD, gpio->pull);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d request io(cd) failed\n", smc_no);
			return -1;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DRV, gpio->drv_level);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set cd pull failed\n", smc_no);
			return -1;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, gpio->mul_sel);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set cd to input failed\n", smc_no);
			return -1;
		}
	}

	return 0;
}

static int sunxi_mci_free_mmcio(struct sunxi_mmc_host *smc_host)
{
	u32 smc_no = smc_host->pdev->id;
	struct sunxi_mmc_platform_data *pdata = smc_host->pdata;
	struct gpio_config *gpio;
	u32 i;
	s32 ret = 0;
	char               pin_name[SUNXI_PIN_NAME_MAX_LEN];
  unsigned long      config;


	for (i=0; i<pdata->width+2; i++) {
		gpio = &pdata->mmcio[i];
		sunxi_gpio_to_name(gpio->gpio, pin_name);

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, 7);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set mmcio%d mulsel failed\n", smc_no, i);
			return -1;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DRV, 1);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set mmcio%d drvlel failed\n", smc_no, i);
			return -1;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_PUD, 0);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set mmcio%d pull failed\n", smc_no, i);
			return -1;
		}
	}

	/* emmc hwrst */
	if (pdata->has_hwrst) {
		gpio = &pdata->hwrst;
		sunxi_gpio_to_name(gpio->gpio, pin_name);

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, 7);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set io(emmc-rst) mulsel failed\n", smc_no);
			return -1;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DRV, 1);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set io(emmc-rst) drvlel failed\n", smc_no);
			return -1;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_PUD, 0);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set io(emmc-rst) pull failed\n", smc_no);
			return -1;
		}
	}

	/* write protect */
	if (pdata->wpmode) {
		gpio = &pdata->wp;

		sunxi_gpio_to_name(gpio->gpio, pin_name);

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, 7);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set wp mulsel failed\n", smc_no);
			return -1;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DRV, 1);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set wp drvlel failed\n", smc_no);
			return -1;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_PUD, 0);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set wp pull failed\n", smc_no);
			return -1;
		}
	}

	/* cd mode == gpio polling */
	if (pdata->cdmode == CARD_DETECT_BY_GPIO_POLL) {
		gpio = &pdata->cd;

		sunxi_gpio_to_name(gpio->gpio, pin_name);

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, 7);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set cd mulsel failed\n", smc_no);
			return -1;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DRV, 1);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set cd drvlel failed\n", smc_no);
			return -1;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_PUD, 0);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set cd pull failed\n", smc_no);
			return -1;
		}
	}

	return 0;
}

static u32 sunxi_gpio_setdrvlevel(u32	gpio,s32 set_driving)
{
		char	pin_name[SUNXI_PIN_NAME_MAX_LEN];
		unsigned long config;
		s32 ret = 0;
	/* get gpio name */
	sunxi_gpio_to_name(gpio, pin_name);
	config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DRV, set_driving);//ma?
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc set mmcio drvlel failed\n");
			return  __LINE__;
		}
		return 0;
}

static void sunxi_mci_update_io_driving(struct sunxi_mmc_host *smc_host, s32 driving)
{
	u32 smc_no = smc_host->pdev->id;
	struct sunxi_mmc_platform_data *pdata = smc_host->pdata;
	struct gpio_config *gpio = NULL;
	u32 set_driving;
	u32 i;
	s32 ret;

	for (i=0; i<pdata->width+2; i++) {
		gpio = &pdata->mmcio[i];
		set_driving = driving == -1 ? gpio->drv_level : driving;
		ret = sunxi_gpio_setdrvlevel(gpio->gpio, set_driving);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d set mmcio%d drvlel failed\n", smc_no, i);
			return;
		}
	}

	SMC_DBG(smc_host, "sdc%d set mmcio driving to %d\n", smc_no, driving);
}


static int sunxi_mci_resource_request(struct sunxi_mmc_host *smc_host)
{
	struct platform_device *pdev = smc_host->pdev;
	u32 smc_no = pdev->id;
	char mclk_name[8] = {0};
	struct resource* res = NULL;
	s32 ret;

	/* request IO */
#ifdef MMC_FPGA
	{
		int tmp = 0;
		void __iomem *gpio_base = IO_ADDRESS(SUNXI_PIO_BASE);
		writel(0x77777777,(u32)gpio_base+0xb4+0);
		writel(0x77777777,(u32)gpio_base+0xb4+4);
		writel(0x77777777,(u32)gpio_base+0xb4+8);
		writel(0x77777777,(u32)gpio_base+0xb4+12);
		smp_wmb();
		writel(0x77333333,(u32)gpio_base+0xb4+0);//0-7
		dumphex32(smc_host, "gpio", IO_ADDRESS(SUNXI_PIO_BASE), 0x120);

		tmp = readl((u32)gpio_base+0xfc+0);
		tmp &=~(7<<16);
		tmp |=(1<<16);
		writel(tmp,(u32)gpio_base+0xfc+0);
		writel(0xffffffff,(u32)gpio_base+0x10c+0);
		smp_wmb();
		dumphex32(smc_host, "gpio", IO_ADDRESS(SUNXI_PIO_BASE), 0x120);
	}
#else
	ret = sunxi_mci_request_mmcio(smc_host);
	if (ret) {
		SMC_ERR(smc_host, "sdc%d request mmcio failed\n", pdev->id);
		goto release_pin;
	}
#endif

	/* io mapping */
	res = request_mem_region(SMHC_BASE_ADDR, SMC_BASE_OS, pdev->name);
	if (!res) {
		SMC_ERR(smc_host, "Failed to request io memory region.\n");
		ret = -ENOENT;
		goto release_pin;
	}
	smc_host->reg_base = ioremap(res->start, SMC_BASE_OS);
	if (!smc_host->reg_base) {
		SMC_ERR(smc_host, "Failed to ioremap() io memory region.\n");
		ret = -EINVAL;
		goto free_mem_region;
	}

	/* mclk */
	sprintf(mclk_name, MMC_MODCLK_PREFIX"%d", smc_no);
	SMC_DBG(smc_host,"mclk_name %s",mclk_name);
	smc_host->mclk = clk_get(&pdev->dev, mclk_name);
	if (NULL==smc_host->mclk||IS_ERR(smc_host->mclk)) {
		ret = PTR_ERR(smc_host->mclk);
		SMC_ERR(smc_host, "Error to get clk for %s\n", mclk_name);
		goto iounmap;
	}

	/* alloc idma descriptor structure */
	smc_host->sg_cpu = dma_alloc_writecombine(NULL, MAX_DES_SIZE,
					&smc_host->sg_dma, GFP_KERNEL);
	if (smc_host->sg_cpu == NULL) {
		SMC_ERR(smc_host, "alloc dma des failed\n");
		goto free_mclk;
	}

	/* get power regulator */
	if (smc_host->pdata->regulator != NULL) {
		smc_host->regulator = regulator_get(NULL, smc_host->pdata->regulator);
		if (IS_ERR(smc_host->regulator )) {
			SMC_ERR(smc_host, "Get regulator %s failed\n", smc_host->pdata->regulator);
			goto free_sgbuff;
		} else {
			SMC_INFO(smc_host,"%s: Get regulator suceessful\n",__FUNCTION__);

			/*
			    the aims of calling sunxi_mci_set_vddio() here are as follows:
			    1. although the default state of the regulator is power-on after power cycle, we still call
			        sunxi_mci_set_vddio() -> regulator_enable() to enable regulator.
			        it can increase regulator's "use_count" to make "use_count" be valid to
			        call regulator_disable() if there is no card during initial process.
			    2. set inner static variable on[id] in sunxi_mci_set_vddio() to 1, and avoid to enable
			        regulator again when set voltage at first time.
			*/
			if (sunxi_mci_set_vddio(smc_host, SDC_WOLTAGE_ON)) {
				SMC_ERR(smc_host, "1st enable regulator failed!\n");
				goto free_sgbuff;
			}

			sunxi_mci_get_vddio(smc_host);
		}
	} else {
		smc_host->regulator_voltage = SDC_WOLTAGE_3V3;// default 3.3V
	}

	/* if SD/MMC is 1, do not decrease frequency. maybe the device is a sdio v3.0 wifi.
		we should can control sdio wifi's freqiency through " ????? " in [mmc1_para] in sys_config.fex */
	if ((smc_host->regulator_voltage == SDC_WOLTAGE_3V3) && (pdev->id != 1)) {
		//hs200 no use in 3.3v, so disable hs200 when use 3.3v
		smc_host->pdata->caps2 &= ~MMC_CAP2_HS200_1_8V_SDR;

		if (smc_host->pdata->f_max > 50000000)
			smc_host->pdata->f_max = 50000000;

		SMC_MSG(smc_host, "io voltage is not 1.8v, set max freq to %d\n", smc_host->pdata->f_max);
	}

	if (smc_host->pdata->used_ddrmode == 0) {
		smc_host->pdata->caps &= ~MMC_CAP_UHS_DDR50;
		smc_host->pdata->caps &= ~MMC_CAP_1_8V_DDR;
		SMC_DBG(smc_host, "card%d disable host ddr capability: 0x%x\n", smc_host->pdev->id, smc_host->pdata->caps);
	} else
		SMC_DBG(smc_host, "card%d keep host ddr capability: 0x%x\n", smc_host->pdev->id, smc_host->pdata->caps);

	return 0;
free_sgbuff:
	dma_free_coherent(NULL, PAGE_SIZE, smc_host->sg_cpu, smc_host->sg_dma);
	smc_host->sg_cpu = NULL;
	smc_host->sg_dma = 0;
free_mclk:
	clk_put(smc_host->mclk);
	smc_host->mclk = NULL;
iounmap:
	iounmap(smc_host->reg_base);
free_mem_region:
	release_mem_region(SMHC_BASE_ADDR, SMC_BASE_OS);
release_pin:
	sunxi_mci_free_mmcio(smc_host);

	return -1;
}

static int sunxi_mci_resource_release(struct sunxi_mmc_host *smc_host)
{
	/* free power regulator */
	if (smc_host->regulator) {
		regulator_put(smc_host->regulator);
		SMC_MSG(smc_host, "release regulator\n");
		smc_host->regulator = NULL;
	}
	/* free idma descriptor structrue */
	if (smc_host->sg_cpu) {
		dma_free_coherent(NULL, PAGE_SIZE,
				  smc_host->sg_cpu, smc_host->sg_dma);
		smc_host->sg_cpu = NULL;
		smc_host->sg_dma = 0;
	}

	clk_put(smc_host->mclk);
	smc_host->mclk = NULL;

	iounmap(smc_host->reg_base);
	release_mem_region(SMHC_BASE_ADDR, SMC_BASE_OS);

	sunxi_mci_free_mmcio(smc_host);

	return 0;
}

static void sunxi_mci_hold_io(struct sunxi_mmc_host* smc_host)
{
	int ret = 0;
	u32 i;
	struct sunxi_mmc_platform_data *pdata = smc_host->pdata;
	struct gpio_config *gpio;

	for (i=0; i<pdata->width+2; i++) {
		char pin_name[SUNXI_PIN_NAME_MAX_LEN];
		unsigned long config;
		gpio = &pdata->mmcio[i];
		sunxi_gpio_to_name(gpio->gpio, pin_name);
		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, 7);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d hold mmcio%d failed\n",
						smc_host->pdev->id, i);
			return;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_PUD, 0);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d hold mmcio%d failed\n",
						smc_host->pdev->id, i);
			return;
		}
	}
	SMC_DBG(smc_host, "mmc %d suspend pins\n", smc_host->pdev->id);
	return;
}

static void sunxi_mci_restore_io(struct sunxi_mmc_host* smc_host)
{
	int ret;
	u32 i;
	struct sunxi_mmc_platform_data *pdata = smc_host->pdata;
	struct gpio_config *gpio;

	for (i=0; i<pdata->width+2; i++) {
		char pin_name[SUNXI_PIN_NAME_MAX_LEN];
		unsigned long config;
		gpio = &pdata->mmcio[i];
		sunxi_gpio_to_name(gpio->gpio, pin_name);

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_PUD,gpio->pull);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d restore mmcio%d failed\n",
						smc_host->pdev->id, i);
			return;
		}

		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, gpio->mul_sel);
		ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (ret) {
			SMC_ERR(smc_host, "sdc%d restore mmcio%d mulsel failed\n",
						smc_host->pdev->id, i);
			return;
		}
	}

	SMC_DBG(smc_host, "mmc %d resume pins\n", smc_host->pdev->id);
}


static void sunxi_mci_finalize_request(struct sunxi_mmc_host *smc_host)
{
	struct mmc_request* mrq = smc_host->mrq;
	unsigned long iflags;

	//SMC_DBG(smc_host,"fun %s,line %d\n",__FUNCTION__,__LINE__);

	spin_lock_irqsave(&smc_host->lock, iflags);
	if (smc_host->wait != SDC_WAIT_FINALIZE) {
		spin_unlock_irqrestore(&smc_host->lock, iflags);
		SMC_ERR(smc_host, "nothing finalize, wt %x, st %d\n",
				smc_host->wait, smc_host->state);
		return;
	}
	smc_host->wait = SDC_WAIT_NONE;
	smc_host->state = SDC_STATE_IDLE;
	smc_host->trans_done = 0;
	smc_host->dma_done = 0;

	sunxi_mci_request_done(smc_host);
	if (smc_host->error) {
		mrq->cmd->error = -ETIMEDOUT;
		if (mrq->data)
			mrq->data->error = -ETIMEDOUT;
		if (mrq->stop)
			mrq->stop->error = -ETIMEDOUT;
	} else {
		if (mrq->data)
			mrq->data->bytes_xfered = (mrq->data->blocks * mrq->data->blksz);
	}

	smc_host->mrq = NULL;
	smc_host->error = 0;
	smc_host->int_sum = 0;
	smp_wmb();
	spin_unlock_irqrestore(&smc_host->lock, iflags);
	mmc_request_done(smc_host->mmc, mrq);

	return;
}

static s32 sunxi_mci_get_ro(struct mmc_host *mmc)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	struct sunxi_mmc_platform_data *pdata = smc_host->pdata;
	u32 wp_val;

	if (pdata->wpmode) {
		struct gpio_config *wp = &pdata->wp;
		wp_val = __gpio_get_value(wp->gpio);
		SMC_DBG(smc_host, "sdc fetch card wp pin status: %d \n", wp_val);
		if (!wp_val) {
			smc_host->read_only = 0;
			return 0;
		} else {
			SMC_MSG(smc_host, "Card is write-protected\n");
			smc_host->read_only = 1;
			return 1;
		}
	} else {
		smc_host->read_only = 0;
		return 0;
	}

	return 0;
}

static void sunxi_mci_cd_cb(unsigned long data)
{
	struct sunxi_mmc_host *smc_host = (struct sunxi_mmc_host *)data;
	struct sunxi_mmc_platform_data *pdata = smc_host->pdata;
	struct gpio_config *cd = &pdata->cd;
	u32 gpio_val = 0;
	u32 present;
	u32 i = 0;

	SMC_DBG(smc_host, "***in cd***\n");

	for (i=0; i<5; i++) {
		gpio_val += __gpio_get_value(cd->gpio);
		mdelay(1);
	}
	if (gpio_val==5) {
		present = 0;
	} else if (gpio_val==0)
		present = 1;
	else
		goto modtimer;
	SMC_DBG(smc_host, "cd %d, host present %d, cur present %d\n",
			gpio_val, smc_host->present, present);

	if (smc_host->present ^ present) {
		SMC_MSG(smc_host, "mmc %d detect change, present %d\n",
				smc_host->pdev->id, present);
		smc_host->present = present;
		smp_wmb();
		if (smc_host->present)
			mmc_detect_change(smc_host->mmc, msecs_to_jiffies(500));
		else
			mmc_detect_change(smc_host->mmc, msecs_to_jiffies(50));
	}

modtimer:
	if (smc_host->cd_mode == CARD_DETECT_BY_GPIO_POLL)
		mod_timer(&smc_host->cd_timer, jiffies + msecs_to_jiffies(300));

	return;
}

static u32 sunxi_mci_cd_irq(int irq,void *data)
{
	sunxi_mci_cd_cb((unsigned long)data);
	return 0;
}

static void sunxi_mci_dat3_det(unsigned long data)
{
	struct sunxi_mmc_host *smc_host = (struct sunxi_mmc_host *)data;

	SMC_DBG(smc_host, "***dat3 det***\n");
	SMC_DBG(smc_host, "host present %d\n",smc_host->present);

	if (smc_host->present)
		mmc_detect_change(smc_host->mmc, msecs_to_jiffies(500));
	else
		mmc_detect_change(smc_host->mmc, msecs_to_jiffies(50));
}

static int sunxi_mci_card_present(struct mmc_host *mmc)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	return smc_host->present;
}

static irqreturn_t sunxi_mci_irq(int irq, void *dev_id)
{
	struct sunxi_mmc_host *smc_host = dev_id;
	u32 sdio_int 	= 0;
	//u32 raw_int;
	u32 msk_int;
	u32 int_sta 	= 0;
	u32 int_sig_en 	= 0;
	u32 int_sta_en	= 0;
	u32 tmp 		= 0;

	spin_lock(&smc_host->lock);

	int_sta = smhc_readl(smc_host,SMHC_INT_STA);
	int_sig_en = smhc_readl(smc_host,SMHC_INT_SIG_EN);
	int_sta_en = smhc_readl(smc_host,SMHC_INT_STA_EN);
	msk_int = int_sta & int_sig_en;
	if (!msk_int) {
		//{
		//	#define SUNXI_PIO_BASE	 0x01c20800
		//	void __iomem *gpio_base = IO_ADDRESS(SUNXI_PIO_BASE);
		//	//writel(0xffffffff,(u32)gpio_base+0x10c+0);
		//	writel(0,(u32)gpio_base+0x10c+0);
		//	mdelay(1);
		//	writel(0xffffffff,(u32)gpio_base+0x10c+0);
		//	printk("triger %x\n",readl((u32)gpio_base+0x10c+0));
		//}

		//{
		//	struct scatterlist *sg;
		//	int i = 0;
		//	struct mmc_request *mrq = smc_host->mrq;
		//	for_each_sg(mrq->data->sg, sg, mrq->data->sg_len, i) {
		//	s32 bytes = sg->length;
		//	u32 *pointer = sg_virt(sg);
		//	//dumphex32(smc_host,"data rev",pointer,bytes);
		//	hexdump("data rec",pointer,bytes);
		//	}
		//}

		SMC_MSG(smc_host, "sdc%d nop irq: sta %08x sta_en %08x sig_en %08x\n",
			smc_host->pdev->id, int_sta,int_sta_en,int_sig_en);
		SMC_MSG(smc_host, "wait %x,state %x\n",smc_host->wait,smc_host->state);
		dumphex32(smc_host, "mmc0-100", IOMEM(IO_ADDRESS(SMHC_BASE_ADDR)), 0x60);
		dumphex32(smc_host, "mmc1-200", IOMEM(IO_ADDRESS(SMHC_BASE_ADDR+0x200)), 0x90);
		spin_unlock(&smc_host->lock);
		return IRQ_HANDLED;
	}

	smc_host->int_sum |= int_sta;
	SMC_INFO(smc_host, "smc %d irq, sta %08x(%08x) sta_en %08x sig_en %08x,msk_int %x\n",
		smc_host->pdev->id, int_sta, smc_host->int_sum,int_sta_en, int_sig_en,msk_int);

	if (msk_int & CardInt) {
		sdio_int = 1;
		//SMC_MSG(smc_host,"sdio int\n");
		//SMC_MSG(smc_host,"si\n");
		smhc_writel(smc_host,SMHC_INT_STA,CardInt);

		//clear  int sta enable
		//tmp = smhc_readl(smc_host,SMHC_INT_STA_EN);
		//tmp &= ~(CardInt);
		//smhc_writel(smc_host,SMHC_INT_STA_EN,tmp);
		//SMC_MSG(smc_host,"sta_en %x\n",smhc_readl(smc_host,SMHC_INT_STA_EN));
		goto card_int_out;
	}

	if(smc_host->cd_mode == CARD_DETECT_BY_D3)
	{
		if (msk_int & CardInsertInt) {
			smhc_writel(smc_host,SMHC_INT_STA,CardInsertInt);
			smc_host->present = 1;
			tasklet_schedule(&smc_host->d3_det_tasklet);
			goto card_int_out;
		}

		if (msk_int  & CardRemoveInt) {
			smhc_writel(smc_host,SMHC_INT_STA,CardRemoveInt);
			smc_host->present = 0;
			tasklet_schedule(&smc_host->d3_det_tasklet);
			goto card_int_out;
		}
	}

	if (smc_host->wait == SDC_WAIT_NONE && !sdio_int) {
		SMC_ERR(smc_host, "smc %x, nothing to complete, sta %08x, "
			"sta_en %08x, sig_en %08x\n", smc_host->pdev->id, int_sta, int_sta_en, int_sig_en);
		goto irq_out;
	}

	if (int_sta & ErrIntBit) {

		//{
		//	#define SUNXI_PIO_BASE 	 0x01c20800
		//	void __iomem *gpio_base = IO_ADDRESS(SUNXI_PIO_BASE);
		//	//writel(0xffffffff,(u32)gpio_base+0x10c+0);
		//	writel(0,(u32)gpio_base+0x10c+0);
		//	mdelay(1);
		//	writel(0xffffffff,(u32)gpio_base+0x10c+0);
		//	printk("triger %x\n",readl((u32)gpio_base+0x10c+0));
		//}

		//if(!(int_sta & CmdTimeoutErrInt)){
		//	#define SUNXI_PIO_BASE	 0x01c20800
		//	void __iomem *gpio_base = IO_ADDRESS(SUNXI_PIO_BASE);
		//	//writel(0xffffffff,(u32)gpio_base+0x10c+0);
		//	writel(0,(u32)gpio_base+0x10c+0);
		//	mdelay(1);
		//	writel(0xffffffff,(u32)gpio_base+0x10c+0);
		//	printk("triger %x\n",readl((u32)gpio_base+0x10c+0));
		//}

		smc_host->error = int_sta & ErrIntBit;
		smc_host->wait = SDC_WAIT_FINALIZE;
		smc_host->state = SDC_STATE_CMDDONE;
		goto irq_out;
	}

	if ((smc_host->wait == SDC_WAIT_CMD_DONE)
		&& (msk_int & CmdOverInt)) {
		smc_host->wait = SDC_WAIT_FINALIZE;
		smc_host->state = SDC_STATE_CMDDONE;
	}

	if ((smc_host->wait == SDC_WAIT_DATA_OVER)
		&& (msk_int & TransOverInt)) {
		smc_host->wait = SDC_WAIT_FINALIZE;
		smc_host->state = SDC_STATE_CMDDONE;
	}

	if (smc_host->wait == (SDC_WAIT_DATA_OVER|SDC_WAIT_DMA_DONE)) {

		if(msk_int & TransOverInt){
			smc_host->trans_done = 1;
		}
		if(msk_int & DmaInt){
			smc_host->dma_done = 1;
		}
		if(smc_host->trans_done && smc_host->dma_done){
			smc_host->wait = SDC_WAIT_FINALIZE;
			smc_host->state = SDC_STATE_CMDDONE;
		}

	}

irq_out:
	smhc_writel(smc_host,SMHC_INT_STA,msk_int);

	if (smc_host->wait == SDC_WAIT_FINALIZE) {
		//clear  int signal enable
		tmp = smhc_readl(smc_host,SMHC_INT_SIG_EN);
		tmp &= (CardInt|CardRemoveInt|CardInsertInt);
		smhc_writel(smc_host,SMHC_INT_SIG_EN,tmp);
		smp_wmb();
		tasklet_schedule(&smc_host->tasklet);
	}

card_int_out:
	spin_unlock(&smc_host->lock);

	if (sdio_int)
		mmc_signal_sdio_irq(smc_host->mmc);

	return IRQ_HANDLED;
}

static void sunxi_mci_tasklet(unsigned long data)
{
	struct sunxi_mmc_host *smc_host = (struct sunxi_mmc_host *) data;
	sunxi_mci_finalize_request(smc_host);
}

static void sunxi_mci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	char* bus_mode[] = {"", "OD", "PP"};
	char* pwr_mode[] = {"OFF", "UP", "ON"};
	char* vdd[] = {"3.3V", "1.8V", "1.2V"};
	char* timing[] = {"LEGACY(SDR12)", "MMC-HS(SDR26)", "SD-HS(SDR25)",
			"UHS-SDR50", "UHS-SDR104", "UHS-DDR50", "MMC-HS200"};
	char* drv_type[] = {"B", "A", "C", "D"};
	static u32 last_clock[4] = {0};
	u32 id = smc_host->pdev->id;
	u32 tmp;
	s32 err;

	BUG_ON(ios->bus_mode >= sizeof(bus_mode)/sizeof(bus_mode[0]));
	BUG_ON(ios->power_mode >= sizeof(pwr_mode)/sizeof(pwr_mode[0]));
	BUG_ON(ios->signal_voltage >= sizeof(vdd)/sizeof(vdd[0]));
	BUG_ON(ios->timing >= sizeof(timing)/sizeof(timing[0]));
	SMC_MSG(smc_host, "sdc%d set ios: clk %dHz bm %s pm %s vdd %d width %d timing %s dt %s\n",
			smc_host->pdev->id, ios->clock, bus_mode[ios->bus_mode],
			pwr_mode[ios->power_mode], ios->vdd,1 << ios->bus_width,
			timing[ios->timing], drv_type[ios->drv_type]);

	/* Set the power state */
	switch (ios->power_mode) {
		case MMC_POWER_ON:
			break;
		case MMC_POWER_UP:
			if (!smc_host->power_on) {
				SMC_DBG(smc_host, "sdc%d power on !!\n", smc_host->pdev->id);

				sunxi_mci_set_vddio(smc_host, smc_host->regulator_voltage);
				usleep_range(1000, 1500);
				smc_host->voltage = smc_host->regulator_voltage;

				if (card_power_on(smc_host)) {
					SMC_MSG(smc_host, "sdc%d card_power_on fail\n", smc_host->pdev->id);
				} else {
					SMC_MSG(smc_host, "sdc%d card_power_on ok\n", smc_host->pdev->id);
				}

				sunxi_mci_restore_io(smc_host);

				sunxi_periph_reset_deassert(smc_host->mclk);
				err = clk_prepare_enable(smc_host->mclk);
				if (err) {
					SMC_ERR(smc_host, "Failed to enable sdc%d mclk\n",
								smc_host->pdev->id);
				}

				mdelay(1);
				sunxi_mci_init_host(smc_host);
				enable_irq(smc_host->irq);
				smc_host->power_on = 1;
			}
			break;
		case MMC_POWER_OFF:
			if (smc_host->power_on) {
				SMC_DBG(smc_host, "sdc%d power off !!\n", smc_host->pdev->id);

				//if use data3 detect,do not power off host
				if(smc_host->cd_mode != CARD_DETECT_BY_D3){
					disable_irq(smc_host->irq);
					sunxi_mci_exit_host(smc_host);

					clk_disable_unprepare(smc_host->mclk);
					//sunxi_periph_reset_assert(smc_host->mclk);
					sunxi_mci_hold_io(smc_host);
					if (card_power_off(smc_host)) {
						SMC_MSG(smc_host, "sdc%d card_power_off fail\n", smc_host->pdev->id);
					} else {
						SMC_MSG(smc_host, "sdc%d card_power_off ok\n", smc_host->pdev->id);
					}

					sunxi_mci_set_vddio(smc_host, SDC_WOLTAGE_OFF);
					usleep_range(1000, 1500);
					smc_host->voltage = SDC_WOLTAGE_OFF;
					smc_host->power_on = 0;
				}

				smc_host->ferror = 0;
				last_clock[id] = 0;
			}
			break;
	}

	/* set bus width */
	switch (ios->bus_width) {
		case MMC_BUS_WIDTH_1:
			tmp = smhc_readl(smc_host,SMHC_CTRL1);
			tmp &= ~(BusWidth|ExtBusWidth);
			smhc_writel(smc_host,SMHC_CTRL1,tmp);
			smc_host->bus_width = 1;
			break;
		case MMC_BUS_WIDTH_4:
			tmp = smhc_readl(smc_host,SMHC_CTRL1);
			tmp &= ~(BusWidth|ExtBusWidth);
			tmp |= BusWidth;//4bit bus
			smhc_writel(smc_host,SMHC_CTRL1,tmp);
			smc_host->bus_width = 4;
			break;
		case MMC_BUS_WIDTH_8:
			tmp = smhc_readl(smc_host,SMHC_CTRL1);
			tmp &= ~(BusWidth|ExtBusWidth);
			tmp |= ExtBusWidth;//8bit bus
			smhc_writel(smc_host,SMHC_CTRL1,tmp);
			smc_host->bus_width = 8;
			break;
	}

	/* set ddr mode */
	if (ios->timing == MMC_TIMING_UHS_DDR50) {
		SMC_MSG(smc_host,"fun %s,line %d val%x\n",__FUNCTION__,__LINE__,smhc_readl(smc_host,SMHC_ACMD_ERR_CTRL2));
		tmp = smhc_readl(smc_host,SMHC_ACMD_ERR_CTRL2);
		tmp &= ~DdrType;
		tmp |= (0x4<<DDR_SHIFT);
		smhc_writel(smc_host,SMHC_ACMD_ERR_CTRL2, tmp);

		smhc_writel(smc_host,SMHC_ATC, 0x50310000);

		//smhc_writel(smc_host, SMHC_RTC,   (1U<<3)|(0));
		//smhc_writel(smc_host, SMHC_DITC0, (((1U<<3)|0)<<24) | (((1U<<3)|0)<<16) | (((1U<<3)|0)<<8) | (((1U<<3)|0)<<0) );
		//smhc_writel(smc_host, SMHC_DITC1, (((1U<<3)|0)<<24) | (((1U<<3)|0)<<16) | (((1U<<3)|0)<<8) | (((1U<<3)|0)<<0) );

		smc_host->ddr = 1;
		SMC_MSG(smc_host,"fun %s,line %d val%x %x\n",__FUNCTION__,__LINE__,smhc_readl(smc_host,SMHC_ACMD_ERR_CTRL2), smhc_readl(smc_host, SMHC_ATC));

		/* change io max driving */
		sunxi_mci_update_io_driving(smc_host, -1);
	} else {
		tmp = smhc_readl(smc_host,SMHC_ACMD_ERR_CTRL2);
		tmp &= ~DdrType;
		smc_host->ddr = 0;
		smhc_writel(smc_host,SMHC_ACMD_ERR_CTRL2,tmp);

		smhc_writel(smc_host,SMHC_ATC, 0x30330000);

		//smhc_writel(smc_host, SMHC_RTC,   (1U<<3)|(3));
		//smhc_writel(smc_host, SMHC_DITC0, (((1U<<3)|3)<<24) | (((1U<<3)|3)<<16) | (((1U<<3)|3)<<8) | (((1U<<3)|3)<<0) );
		//smhc_writel(smc_host, SMHC_DITC1, (((1U<<3)|3)<<24) | (((1U<<3)|3)<<16) | (((1U<<3)|3)<<8) | (((1U<<3)|3)<<0) );

		SMC_MSG(smc_host,"fun %s,line %d val%x %x\n",__FUNCTION__,__LINE__,smhc_readl(smc_host,SMHC_ACMD_ERR_CTRL2), smhc_readl(smc_host, SMHC_ATC));
		sunxi_mci_update_io_driving(smc_host, -1);
	}

	/* set up clock */
	if (ios->clock) {
		if (smc_host->ddr)
			ios->clock = smc_host->pdata->f_ddr_max;

		smc_host->mod_clk = ios->clock*4;
		if (smc_host->ddr)
			smc_host->mod_clk*=2;
		smc_host->card_clk = ios->clock;
		sunxi_mci_set_clk(smc_host);
		last_clock[id] = ios->clock;
		usleep_range(50000, 55000);
	} else if (!ios->clock && last_clock[id]) {
		last_clock[id] = 0;
		sunxi_mci_oclk_onoff(smc_host, 0, 0);
	}
}

static void sunxi_mci_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	unsigned long flags;
	//u32 imask;
	u32 int_sig_en = 0;
	u32 int_sta_en = 0;

	//SMC_MSG(smc_host,"%s %d,enble %d\n",__FUNCTION__,__LINE__,enable);
	//dumphex32(smc_host, "mmc", IOMEM(IO_ADDRESS(SMHC_BASE_ADDR)), 0x100);
	spin_lock_irqsave(&smc_host->lock, flags);
	//SMC_MSG(smc_host,"0en %d,sie %x,sae %x ,sta%x\n",enable,smhc_readl(smc_host,SMHC_INT_SIG_EN),smhc_readl(smc_host,SMHC_INT_STA_EN),smhc_readl(smc_host,SMHC_INT_STA));
	//SMC_MSG(smc_host,"0en %d,sie %x,sta%x\n",enable,smhc_readl(smc_host,SMHC_INT_SIG_EN),smhc_readl(smc_host,SMHC_INT_STA));
	//SMC_MSG(smc_host,"0en %d,sie %x,sae %x\n",enable,smhc_readl(smc_host,SMHC_INT_SIG_EN),smhc_readl(smc_host,SMHC_INT_STA_EN));
	int_sig_en = smhc_readl(smc_host,SMHC_INT_SIG_EN);
	int_sta_en = smhc_readl(smc_host,SMHC_INT_STA_EN);
	if(enable){
		smhc_writel(smc_host,SMHC_INT_STA_EN,int_sta_en|CardInt);
		smhc_writel(smc_host,SMHC_INT_SIG_EN,int_sig_en|CardInt);
	}else{
		smhc_writel(smc_host,SMHC_INT_STA_EN,int_sta_en & (~CardInt));
		smhc_writel(smc_host,SMHC_INT_SIG_EN,int_sig_en & (~CardInt));
	}

	//dumphex32(smc_host, "mmc", IOMEM(IO_ADDRESS(SMHC_BASE_ADDR)), 0x60);
	//SMC_MSG(smc_host,"1en %d,sie %x,sae %x\n",enable,smhc_readl(smc_host,SMHC_INT_SIG_EN),smhc_readl(smc_host,SMHC_INT_STA_EN));
	spin_unlock_irqrestore(&smc_host->lock, flags);
	//dumphex32(smc_host, "mmc", IOMEM(IO_ADDRESS(SMHC_BASE_ADDR)), 0x60);
}

static void sunxi_mci_hw_reset(struct mmc_host *mmc)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	u32 id = smc_host->pdev->id;

	if (id == 2 || id == 3) {
		udelay(10);
		udelay(300);
	}
}

static void sunxi_mci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	struct mmc_command* cmd = mrq->cmd;
	struct mmc_data* data = mrq->data;
	int ret 		=0;
	unsigned long iflags = 0;

	spin_lock_irqsave(&smc_host->lock, iflags);

	if (sunxi_mci_card_present(mmc) == 0 || smc_host->ferror
			|| smc_host->suspend || !smc_host->power_on) {
		SMC_ERR(smc_host, "no medium present, ferr %d, suspend %d pwd %d\n",
			    smc_host->ferror, smc_host->suspend, smc_host->power_on);
		mrq->cmd->error = -ENOMEDIUM;
		spin_unlock_irqrestore(&smc_host->lock, iflags);
		mmc_request_done(mmc, mrq);
		return;
	}

	//Wait for cmd line free
	if(sunxi_wait_bit_clr(smc_host,			\
					SMHC_STA,CmdInhibitCmd, \
					"SMHC_STA","CmdInhibitCmd",1)){
		SMC_ERR(smc_host,"Wait cmd free timeout\n");
		mrq->cmd->error = -EINVAL;
		spin_unlock_irqrestore(&smc_host->lock, iflags);
		mmc_request_done(mmc, mrq);
		return;
	}

	smc_host->mrq = mrq;
	if (data) {
		//Wait for data line free
		if(sunxi_wait_bit_clr(smc_host, 		\
						SMHC_STA,CmdInhibitDat, \
						"SMHC_STA","CmdInhibitDat",1)){
			SMC_ERR(smc_host,"Wait data free timeout\n");
			mrq->cmd->error = -EINVAL;
			spin_unlock_irqrestore(&smc_host->lock, iflags);
			mmc_request_done(mmc, mrq);
			return;
		}
		ret = sunxi_mci_prepare_dma(smc_host, data);
		if (ret < 0) {
			SMC_ERR(smc_host, "smc %d prepare DMA failed\n", smc_host->pdev->id);
			cmd->error = ret;
			cmd->data->error = ret;
			spin_unlock_irqrestore(&smc_host->lock, iflags);
			smp_wmb();
			mmc_request_done(smc_host->mmc, mrq);
			return;
		}
		smhc_writel(smc_host,SMHC_BLK_CFG,(data->blksz&0xFFF)|((data->blocks&0xFFFF)<<16));
	}
	sunxi_mci_send_cmd(smc_host, cmd);
	spin_unlock_irqrestore(&smc_host->lock, iflags);
}

static int sunxi_mci_do_voltage_switch(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	SMC_MSG(smc_host,"%s in\n",__FUNCTION__);
	SMC_MSG(smc_host,"%s %d,vol %x,sig vol %x\n",__FUNCTION__,__LINE__,smc_host->voltage ,ios->signal_voltage );
	if (smc_host->voltage != SDC_WOLTAGE_3V3 &&
			ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
		SMC_MSG(smc_host,"%s %d\n",__FUNCTION__,__LINE__);
		/* during initial process, ios->signal_voltage may be equal to MMC_SIGNAL_VOLTAGE_330,
		if emmc's io voltage is 1.8V, don't switch */
		if (smc_host->cd_mode == CARD_ALWAYS_PRESENT ) {
			SMC_INFO(smc_host,"%s mmc %d return, no need to switch voltage to 3v3: \n",
				         __FUNCTION__,smc_host->pdev->id);
			return 0;
		}

		sunxi_mci_set_vddio(smc_host, SDC_WOLTAGE_3V3);
		/* wait for 5ms */
		usleep_range(1000, 1500);
		smc_host->voltage = SDC_WOLTAGE_3V3;
		return 0;
	} else if (smc_host->voltage != SDC_WOLTAGE_1V8 &&
			(ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180)) {
				
			//SMC_MSG(smc_host,"%s Try to switch vol\n",__FUNCTION__);
			if((smhc_readl(smc_host,SMHC_STA)>>20)&0x1f){
				SMC_ERR(smc_host,"card not driver data low,swith vol failed\n");
				//to do ,recover
				return -EAGAIN;
			}
			//to do ,swith vol to 1v8
			sunxi_mci_oclk_onoff(smc_host,0,0);
			usleep_range(5000, 5500);
			sunxi_mci_oclk_onoff(smc_host,1,0);
			usleep_range(1000, 1500);
			if((smhc_readl(smc_host,SMHC_STA)>>20)&0x1f){
				SMC_MSG(smc_host,"vol switch ok\n");
				return 0;
			}
			SMC_ERR(smc_host,"card not driver data high,swith vol failed\n");
			//to do ,recover
			return -EAGAIN;

	} else{
		SMC_MSG(smc_host,"%s %d,vol %x,sig vol %d\n",__FUNCTION__,__LINE__,smc_host->voltage ,ios->signal_voltage );
		return 0;
	}
}

/*
 * Here we execute a tuning operation to find the sample window of MMC host.
 * Then we select the best sampling point in the host for DDR50, SDR50, and
 * SDR104 modes.
 */
 #if 0
static int sunxi_mci_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	static const char tuning_blk_4b[] = {
		0xff, 0x0f, 0xff, 0x00, 0xff, 0xcc, 0xc3, 0xcc,
		0xc3, 0x3c, 0xcc, 0xff, 0xfe, 0xff, 0xfe, 0xef,
		0xff, 0xdf, 0xff, 0xdd, 0xff, 0xfb, 0xff, 0xfb,
		0xbf, 0xff, 0x7f, 0xff, 0x77, 0xf7, 0xbd, 0xef,
		0xff, 0xf0, 0xff, 0xf0, 0x0f, 0xfc, 0xcc, 0x3c,
		0xcc, 0x33, 0xcc, 0xcf, 0xff, 0xef, 0xff, 0xee,
		0xff, 0xfd, 0xff, 0xfd, 0xdf, 0xff, 0xbf, 0xff,
		0xbb, 0xff, 0xf7, 0xff, 0xf7, 0x7f, 0x7b, 0xde
	};
	static const char tuning_blk_8b[] = {
		0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00,
		0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc, 0xcc,
		0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff, 0xff,
		0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee, 0xff,
		0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd, 0xdd,
		0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff, 0xbb,
		0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff, 0xff,
		0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee, 0xff,
		0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00,
		0x00, 0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc,
		0xcc, 0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff,
		0xff, 0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee,
		0xff, 0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd,
		0xdd, 0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff,
		0xbb, 0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff,
		0xff, 0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee
	};
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	u32 sample_min = 1;
	u32 sample_max = 0;
	u32 sample_bak = smc_host->sclk_dly;
	u32 sample_dly = 1; //0;
	u32 sample_win = 0;
	u32 loops = 64;
	u32 tuning_done = 0;
	char* rcv_pattern = (char*)kmalloc(128, GFP_KERNEL|GFP_DMA);
	char* std_pattern = NULL;
	int err = 0;

	if (!rcv_pattern) {
		SMC_ERR(smc_host, "sdc%d malloc tuning pattern buffer failed\n",
				smc_host->pdev->id);
		return -EIO;
	}
	SMC_MSG(smc_host, "sdc%d executes tuning operation\n", smc_host->pdev->id);
	/*
	 * The Host Controller needs tuning only in case of SDR104 mode
	 * and for SDR50 mode. Issue CMD19 repeatedly till get all of the
	 * sample points or the number of loops reaches 40 times or a
	 * timeout of 150ms occurs.
	 */
	do {
		struct mmc_command cmd = {0};
		struct mmc_data data = {0};
		struct mmc_request mrq = {0};
		struct scatterlist sg;

		sunxi_mci_set_clk_dly(smc_host, smc_host->oclk_dly, sample_dly);
		cmd.opcode = opcode;
		cmd.arg = 0;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
		if (opcode == MMC_SEND_TUNING_BLOCK_HS200) {
			if (mmc->ios.bus_width == MMC_BUS_WIDTH_8) {
				sg.length = 128;
				data.blksz = 128;
				std_pattern = (char*)tuning_blk_8b;
			} else if (mmc->ios.bus_width == MMC_BUS_WIDTH_4) {
				sg.length = 64;
				data.blksz = 64;
				std_pattern = (char*)tuning_blk_4b;
			}
		} else {
			sg.length = 64;
			data.blksz = 64;
			std_pattern = (char*)tuning_blk_4b;
		}
		data.blocks = 1;
		data.flags = MMC_DATA_READ;
		data.sg = &sg;
		data.sg_len = 1;
		sg_init_one(&sg, rcv_pattern, sg.length);

		mrq.cmd = &cmd;
		mrq.data = &data;

		mmc_wait_for_req(mmc, &mrq);
		/*
		 * If no error happened in the transmission, compare data with
		 * the tuning pattern. If there is no error, record the minimal
		 * and the maximal value of the sampling clock delay to find
		 * the best sampling point in the sampling window.
		 */
		if (!cmd.error && !data.error) {
			if (!memcmp(rcv_pattern, std_pattern, data.blksz)) {
				SMC_MSG(smc_host, "sdc%d tuning ok, sclk_dly %d\n",
					smc_host->pdev->id, sample_dly);
				if (!sample_win)
					sample_min = sample_dly;
				sample_win++;
				if (sample_dly == 7) {
					SMC_MSG(smc_host, "sdc%d tuning reach to max sclk_dly 7\n",
						smc_host->pdev->id);
					tuning_done = 1;
					sample_max = sample_dly;
					break;
				}
			} else if (sample_win) {
				SMC_MSG(smc_host, "sdc%d tuning data failed, sclk_dly %d\n",
					smc_host->pdev->id, sample_dly);
				tuning_done = 1;
				sample_max = sample_dly-1;
				break;
			}
		} else if (sample_win) {
			SMC_MSG(smc_host, "sdc%d tuning trans fail, sclk_dly %d\n",
				smc_host->pdev->id, sample_dly);
			tuning_done = 1;
			sample_max = sample_dly-1;
			break;
		}
		sample_dly++;
		/* if sclk_dly reach to 7(maximum), down the clock and tuning again */
		if (sample_dly == 8 && loops)
			break;
	} while (!tuning_done && loops--);

	/* select the best sampling point from the sampling window */
	if (sample_win) {
		sample_dly = sample_min + sample_win/2;
		SMC_MSG(smc_host, "sdc%d sample_window:[%d, %d], sample_point %d\n",
				smc_host->pdev->id, sample_min, sample_max, sample_dly);
		sunxi_mci_set_clk_dly(smc_host, smc_host->oclk_dly, sample_dly);
		err = 0;
	} else {
		SMC_ERR(smc_host, "sdc%d cannot find a sample point\n", smc_host->pdev->id);
		sunxi_mci_set_clk_dly(smc_host, smc_host->oclk_dly, sample_bak);
		mmc->ios.bus_width = MMC_BUS_WIDTH_1;
		mmc->ios.timing = MMC_TIMING_LEGACY;
		err = -EIO;
	}

	kfree(rcv_pattern);
	return err;
}
#endif


#if 0
/*
 * Here provide a function to scan card, for some SDIO cards that
 * may stay in busy status after writing operations. MMC host does
 * not wait for ready itself. So the driver of this kind of cards
 * should call this function to check the real status of the card.
 */

void sunxi_mci_rescan_card(unsigned id, unsigned insert)
{
	struct sunxi_mmc_host *smc_host = NULL;

	BUG_ON(id > 3);
	BUG_ON(sunxi_host[id] == NULL);
	if (sunxi_host[id] == NULL)
		return;
	smc_host = sunxi_host[id];
	smc_host->present = insert ? 1 : 0;
	mmc_detect_change(smc_host->mmc, 0);
	return;
}
EXPORT_SYMBOL_GPL(sunxi_mci_rescan_card);

int sunxi_mci_check_r1_ready(struct mmc_host* mmc, unsigned ms)
{
		return 0;
}
EXPORT_SYMBOL_GPL(sunxi_mci_check_r1_ready);

#endif 


static struct mmc_host_ops sunxi_mci_ops = {
	.request	= sunxi_mci_request,
	.set_ios	= sunxi_mci_set_ios,
	.get_ro		= sunxi_mci_get_ro,
	.get_cd		= sunxi_mci_card_present,
	.enable_sdio_irq= sunxi_mci_enable_sdio_irq,
	.hw_reset	= sunxi_mci_hw_reset,
	.start_signal_voltage_switch = sunxi_mci_do_voltage_switch,
	//.execute_tuning = sunxi_mci_execute_tuning,
};

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
static int sunxi_mci_proc_drvversion(char *page, char **start, off_t off,
					int count, int *eof, void *data)
{
	char *p = page;

	p += sprintf(p, "%s\n", DRIVER_VERSION);
	return p - page;
}

static int sunxi_mci_proc_hostinfo(char *page, char **start, off_t off,
					int count, int *eof, void *data)
{
	char *p = page;
	struct sunxi_mmc_host *smc_host = (struct sunxi_mmc_host *)data;
	struct device* dev = &smc_host->pdev->dev;
	char* cd_mode[] = {"None", "GPIO Check", "GPIO IRQ", "Always In", "Manual", "DATA3"};
	char* state[] = {"Idle", "Sending CMD", "CMD Done"};
	char* vol[] = {"3.3V", "1.8V", "1.2V", "off"};
	u32 Fmclk_MHz = (smc_host->mod_clk == 24000000 ? 24000000 : 600000000)/1000000;
	u32 Tmclk_ns = Fmclk_MHz ? 10000/Fmclk_MHz : 0;
	u32 odly = smc_host->oclk_dly ? Tmclk_ns*smc_host->oclk_dly : Tmclk_ns >> 1;
	u32 sdly = smc_host->sclk_dly ? Tmclk_ns*smc_host->sclk_dly : Tmclk_ns >> 1;

	p += sprintf(p, " %s Host Info:\n", dev_name(dev));
	p += sprintf(p, " REG Base  : %p\n", smc_host->reg_base);
	p += sprintf(p, " DMA Desp  : %p(%08x)\n", smc_host->sg_cpu, smc_host->sg_dma);
	p += sprintf(p, " Mod Clock : %d\n", smc_host->mod_clk);
	p += sprintf(p, " Card Clock: %d\n", smc_host->card_clk);
	p += sprintf(p, " Oclk Delay: %d(%d.%dns)\n", smc_host->oclk_dly, odly/10, odly%10);
	p += sprintf(p, " Sclk Delay: %d(%d.%dns)\n", smc_host->sclk_dly, sdly/10, odly%10);
	p += sprintf(p, " Bus Width : %d\n", smc_host->bus_width);
	p += sprintf(p, " DDR Mode  : %d\n", smc_host->ddr);
#if defined(CONFIG_ARCH_SUN8IW5P1) || defined(CONFIG_ARCH_SUN8IW6P1) \
		|| defined(CONFIG_ARCH_SUN8IW8P1) || defined(CONFIG_ARCH_SUN8IW7P1) \
		|| defined(CONFIG_ARCH_SUN8IW9P1)
	p += sprintf(p, " 2xCLK Mode: %d\n", smc_host->pdata->used_ddrmode);
#endif
	p += sprintf(p, " Voltage   : %s\n", vol[smc_host->voltage]);
	p += sprintf(p, " Present   : %d\n", smc_host->present);
	p += sprintf(p, " CD Mode   : %s\n", cd_mode[smc_host->cd_mode]);
	p += sprintf(p, " Read Only : %d\n", smc_host->read_only);
	p += sprintf(p, " State     : %s\n", state[smc_host->state]);
	p += sprintf(p, " Regulator : %s\n", smc_host->pdata->regulator);

	return p - page;
}

static int sunxi_mci_proc_read_regs(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	char *p = page;
	struct sunxi_mmc_host *smc_host = (struct sunxi_mmc_host *)data;
	u32 i;

	p += sprintf(p, "Dump smc regs:\n");
	for (i=0; i<0x100; i+=4) {
		if (!(i&0xf))
			p += sprintf(p, "\n0x%08x : ", (u32)(smc_host->reg_base + i));
		p += sprintf(p, "%08x ", readl(smc_host->reg_base + i));
	}
	p += sprintf(p, "\n");

	p += sprintf(p, "Dump ccmu regs:\n");
	for (i=0; i<0x170; i+=4) {
		if (!(i&0xf))
			p += sprintf(p, "\n0x%08x : ", (unsigned int)(IO_ADDRESS(SUNXI_CCM_BASE) + i));
		p += sprintf(p, "%08x ", readl(IO_ADDRESS(SUNXI_CCM_BASE) + i));
	}
	p += sprintf(p, "\n");

	p += sprintf(p, "Dump gpio regs:\n");
	for (i=0; i<0x120; i+=4) {
		if (!(i&0xf))
			p += sprintf(p, "\n0x%08x : ", (unsigned int)(IO_ADDRESS(SUNXI_PIO_BASE) + i));
		p += sprintf(p, "%08x ", readl(IO_ADDRESS(SUNXI_PIO_BASE)+ i));
	}
	p += sprintf(p, "\n");

	p += sprintf(p, "Dump gpio irqc:\n");
	for (i=0x200; i<0x300; i+=4) {
		if (!(i&0xf))
			p += sprintf(p, "\n0x%08x : ", (unsigned int)(IO_ADDRESS(SUNXI_PIO_BASE) + i));
		p += sprintf(p, "%08x ", readl(IO_ADDRESS(SUNXI_PIO_BASE)+ i));
	}
	p += sprintf(p, "\n");

	return p - page;
}

static int sunxi_mci_proc_read_dbglevel(char *page, char **start, off_t off,
					int count, int *eof, void *data)
{
	char *p = page;
	struct sunxi_mmc_host *smc_host = (struct sunxi_mmc_host *)data;

	p += sprintf(p, "Debug-Level : 0- msg&err, 1- +info, 2- +dbg, 3- all\n");
	p += sprintf(p, "current debug-level : %d\n", smc_host->debuglevel);
	return p - page;
}

static int sunxi_mci_proc_write_dbglevel(struct file *file, const char __user *buffer,
					unsigned long count, void *data)
{
	u32 smc_debug;
	struct sunxi_mmc_host *smc_host = (struct sunxi_mmc_host *)data;
	smc_debug = simple_strtoul(buffer, NULL, 10);

	smc_host->debuglevel = smc_debug;
	return sizeof(smc_debug);
}

static int sunxi_mci_proc_read_cdmode(char *page, char **start, off_t off,
					int count, int *eof, void *data)
{
	char *p = page;
	struct sunxi_mmc_host *smc_host = (struct sunxi_mmc_host *)data;

	p += sprintf(p, "card detect mode: %d\n", smc_host->cd_mode);
	return p - page;
}

static int sunxi_mci_proc_write_cdmode(struct file *file, const char __user *buffer,
					unsigned long count, void *data)
{
	u32 cdmode;
	struct sunxi_mmc_host *smc_host = (struct sunxi_mmc_host *)data;
	cdmode = simple_strtoul(buffer, NULL, 10);

	smc_host->cd_mode = cdmode;
	return sizeof(cdmode);
}

static int sunxi_mci_proc_read_insert_status(char *page, char **start, off_t off,
					int coutn, int *eof, void *data)
{
	char *p = page;
	struct sunxi_mmc_host *smc_host = (struct sunxi_mmc_host *)data;

	p += sprintf(p, "Usage: \"echo 1 > insert\" to scan card and "
			"\"echo 0 > insert\" to remove card\n");
	if (smc_host->cd_mode != CARD_DETECT_BY_FS)
		p += sprintf(p, "Sorry, this node if only for manual "
				"attach mode(cd mode 4)\n");

	p += sprintf(p, "card attach status: %s\n",
		smc_host->present ? "inserted" : "removed");


	return p - page;
}

static int sunxi_mci_proc_card_insert_ctrl(struct file *file, const char __user *buffer,
					unsigned long count, void *data)
{
	u32 insert = simple_strtoul(buffer, NULL, 10);
	struct sunxi_mmc_host *smc_host = (struct sunxi_mmc_host *)data;
	u32 present = insert ? 1 : 0;

	SMC_DBG(smc_host, "sdc%d,Change insert\n", smc_host->pdev->id);
	if (smc_host->present ^ present) {
		smc_host->present = present;
		SMC_DBG(smc_host, "sdc%d,re detect card\n", smc_host->pdev->id);
		mmc_detect_change(smc_host->mmc, msecs_to_jiffies(300));
	}

	SMC_DBG(smc_host, "Insert status %d\n", smc_host->present);
	return sizeof(insert);
}

static int sunxi_mci_proc_get_iodriving(char *page, char **start, off_t off,
						int coutn, int *eof, void *data)
{
	char *p = page;
	struct sunxi_mmc_host *smc_host = (struct sunxi_mmc_host *)data;
	struct sunxi_mmc_platform_data *pdata = smc_host->pdata;
	char* mmc_para_io[10] = {"sdc_clk", "sdc_cmd", "sdc_d0", "sdc_d1", "sdc_d2",
				"sdc_d3", "sdc_d4", "sdc_d5", "sdc_d6", "sdc_d7"};
	struct gpio_config *gpio;
	u32 i;

	p += sprintf(p, "current io driving:\n");
	for (i=0; i<pdata->width; i++) {
		char	pin_name[SUNXI_PIN_NAME_MAX_LEN];
		unsigned long config;
		gpio = &pdata->mmcio[i];
	/* get gpio name */
	sunxi_gpio_to_name(gpio->gpio, pin_name);
	config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DRV, 0xFFFF);
	pin_config_get(SUNXI_PINCTRL, pin_name, &config);
		p += sprintf(p, "%s : %d\n", mmc_para_io[i], (int)(SUNXI_PINCFG_UNPACK_VALUE(config)));

	}
	return p - page;
}


static int sunxi_mci_proc_set_iodriving(struct file *file, const char __user *buffer,
					unsigned long count, void *data)
{
	unsigned long driving = simple_strtoul(buffer, NULL, 16);
	struct sunxi_mmc_host *smc_host = (struct sunxi_mmc_host *)data;
	struct sunxi_mmc_platform_data *pdata = smc_host->pdata;
	u32 clk_drv, cmd_drv, d0_drv, d1_drv, d2_drv, d3_drv, d4_drv, d5_drv, d6_drv, d7_drv;

	clk_drv = 0xf & (driving >> 0);
	cmd_drv = 0xf & (driving >> 4);
	d0_drv = 0xf & (driving >> 8);
	d1_drv = 0xf & (driving >> 12);
	d2_drv = 0xf & (driving >> 16);
	d3_drv = 0xf & (driving >> 20);
	d4_drv = 0xf & (driving >> 8);
	d5_drv = 0xf & (driving >> 12);
	d6_drv = 0xf & (driving >> 16);
	d7_drv = 0xf & (driving >> 20);

	printk("set io driving, clk %x, cmd %x, d0 %x, d1 %x, d2, %x, d3 %x\n",
		clk_drv, cmd_drv, d0_drv, d1_drv, d2_drv, d3_drv);
	if (clk_drv > 0 && clk_drv < 4)
		sunxi_gpio_setdrvlevel(pdata->mmcio[0].gpio, clk_drv);
	if (cmd_drv > 0 && cmd_drv < 4)
		sunxi_gpio_setdrvlevel(pdata->mmcio[1].gpio, clk_drv);
	if (d0_drv > 0 && d0_drv < 4)
		sunxi_gpio_setdrvlevel(pdata->mmcio[2].gpio, clk_drv);
	if (pdata->width == 4 || pdata->width == 8) {
		if (d1_drv > 0 && d1_drv < 4)
			sunxi_gpio_setdrvlevel(pdata->mmcio[3].gpio, clk_drv);
		if (d2_drv > 0 && d2_drv < 4)
			sunxi_gpio_setdrvlevel(pdata->mmcio[4].gpio, clk_drv);
		if (d3_drv > 0 && d3_drv < 4)
			sunxi_gpio_setdrvlevel(pdata->mmcio[5].gpio, clk_drv);
		if (pdata->width == 8) {
			if (d4_drv > 0 && d4_drv < 4)
				sunxi_gpio_setdrvlevel(pdata->mmcio[6].gpio, clk_drv);
			if (d5_drv > 0 && d5_drv < 4)
				sunxi_gpio_setdrvlevel(pdata->mmcio[7].gpio, clk_drv);
			if (d6_drv > 0 && d6_drv < 4)
				sunxi_gpio_setdrvlevel(pdata->mmcio[8].gpio, clk_drv);
			if (d7_drv > 0 && d7_drv < 4)
				sunxi_gpio_setdrvlevel(pdata->mmcio[9].gpio, clk_drv);
		}
	}

	return count;
}

static void sunxi_mci_procfs_attach(struct sunxi_mmc_host *smc_host)
{
	struct device *dev = &smc_host->pdev->dev;
	char sunxi_mci_proc_rootname[32] = {0};

	//make mmc dir in proc fs path
	snprintf(sunxi_mci_proc_rootname, sizeof(sunxi_mci_proc_rootname),
			"driver/%s", dev_name(dev));
	smc_host->proc_root = proc_mkdir(sunxi_mci_proc_rootname, NULL);
	if (IS_ERR(smc_host->proc_root))
		SMC_MSG(smc_host, "%s: failed to create procfs \"driver/mmc\".\n", dev_name(dev));

	smc_host->proc_drvver = create_proc_read_entry("drv-version", 0444,
				smc_host->proc_root, sunxi_mci_proc_drvversion, NULL);
	if (IS_ERR(smc_host->proc_root))
		SMC_MSG(smc_host, "%s: failed to create procfs \"drv-version\".\n", dev_name(dev));

	smc_host->proc_hostinfo = create_proc_read_entry("hostinfo", 0444,
				smc_host->proc_root, sunxi_mci_proc_hostinfo, smc_host);
	if (IS_ERR(smc_host->proc_hostinfo))
		SMC_MSG(smc_host, "%s: failed to create procfs \"hostinfo\".\n", dev_name(dev));

	smc_host->proc_regs = create_proc_read_entry("register", 0444,
				smc_host->proc_root, sunxi_mci_proc_read_regs, smc_host);
	if (IS_ERR(smc_host->proc_regs))
		SMC_MSG(smc_host, "%s: failed to create procfs \"hostinfo\".\n", dev_name(dev));

	smc_host->proc_dbglevel = create_proc_entry("debug-level", 0644, smc_host->proc_root);
	if (IS_ERR(smc_host->proc_dbglevel))
		SMC_MSG(smc_host, "%s: failed to create procfs \"debug-level\".\n", dev_name(dev));

	smc_host->proc_dbglevel->data = smc_host;
	smc_host->proc_dbglevel->read_proc = sunxi_mci_proc_read_dbglevel;
	smc_host->proc_dbglevel->write_proc = sunxi_mci_proc_write_dbglevel;

	smc_host->proc_cdmode = create_proc_entry("cdmode", 0644, smc_host->proc_root);
	if (IS_ERR(smc_host->proc_cdmode))
		SMC_MSG(smc_host, "%s: failed to create procfs \"cdmode\".\n", dev_name(dev));

	smc_host->proc_cdmode->data = smc_host;
	smc_host->proc_cdmode->read_proc = sunxi_mci_proc_read_cdmode;
	smc_host->proc_cdmode->write_proc = sunxi_mci_proc_write_cdmode;

	smc_host->proc_insert = create_proc_entry("insert", 0644, smc_host->proc_root);
	if (IS_ERR(smc_host->proc_insert))
		SMC_MSG(smc_host, "%s: failed to create procfs \"insert\".\n", dev_name(dev));

	smc_host->proc_insert->data = smc_host;
	smc_host->proc_insert->read_proc = sunxi_mci_proc_read_insert_status;
	smc_host->proc_insert->write_proc = sunxi_mci_proc_card_insert_ctrl;

	smc_host->proc_iodrive = create_proc_entry("io-drive", 0644, smc_host->proc_root);
	if (IS_ERR(smc_host->proc_iodrive))
	{
		SMC_MSG(smc_host, "%s: failed to create procfs \"io-drive\".\n", dev_name(dev));
	}
	smc_host->proc_iodrive->data = smc_host;
	smc_host->proc_iodrive->read_proc = sunxi_mci_proc_get_iodriving;
	smc_host->proc_iodrive->write_proc = sunxi_mci_proc_set_iodriving;

}

static void sunxi_mci_procfs_remove(struct sunxi_mmc_host *smc_host)
{
	struct device *dev = &smc_host->pdev->dev;
	char sunxi_mci_proc_rootname[32] = {0};

	snprintf(sunxi_mci_proc_rootname, sizeof(sunxi_mci_proc_rootname),
		"driver/%s", dev_name(dev));
	remove_proc_entry("io-drive", smc_host->proc_root);
	remove_proc_entry("insert", smc_host->proc_root);
	remove_proc_entry("cdmode", smc_host->proc_root);
	remove_proc_entry("debug-level", smc_host->proc_root);
	remove_proc_entry("register", smc_host->proc_root);
	remove_proc_entry("hostinfo", smc_host->proc_root);
	remove_proc_entry("drv-version", smc_host->proc_root);
	remove_proc_entry(sunxi_mci_proc_rootname, NULL);
}

#else

void sunxi_mci_procfs_attach(struct sunxi_mmc_host *smc_host) { }
void sunxi_mci_procfs_remove(struct sunxi_mmc_host *smc_host) { }

#endif	//PROC_FS

static int sunxi_mci_probe(struct platform_device *pdev)
{
	struct sunxi_mmc_host *smc_host = NULL;
	struct mmc_host	*mmc = NULL;
	int ret = 0;

	mmc = mmc_alloc_host(sizeof(struct sunxi_mmc_host), &pdev->dev);
	if (!mmc) {
		SMC_ERR(smc_host, "mmc alloc host failed\n");
		ret = -ENOMEM;
		goto probe_out;
	}

	smc_host = mmc_priv(mmc);
	memset((void*)smc_host, 0, sizeof(smc_host));
	smc_host->mmc	= mmc;
	smc_host->pdev	= pdev;
	smc_host->pdata	= pdev->dev.platform_data;
	smc_host->cd_mode = smc_host->pdata->cdmode;
	smc_host->io_flag = smc_host->pdata->isiodev ? 1 : 0;
	smc_host->debuglevel = CONFIG_MMC_PRE_DBGLVL_SUNXI;

	spin_lock_init(&smc_host->lock);
	tasklet_init(&smc_host->tasklet, sunxi_mci_tasklet, (unsigned long) smc_host);

	if (sunxi_mci_resource_request(smc_host)) {
		SMC_ERR(smc_host, "%s: Failed to get resouce.\n", dev_name(&pdev->dev));
		goto probe_free_host;
	}

	sunxi_mci_procfs_attach(smc_host);

	//enable clk to clear card int to avoid card int after irq int
	sunxi_periph_reset_deassert(smc_host->mclk);
	SMC_DBG(smc_host,"try to enalble mclk\n");
	ret = clk_prepare_enable(smc_host->mclk);
	if (ret){
		SMC_ERR(smc_host, "Failed to enable sdc%d mclk\n", smc_host->pdev->id);
		goto probe_free_resource;
	}
	SMC_DBG(smc_host,"enalble mclk ok\n");
	//enable data3 detect int
	smhc_clr_bit(smc_host,SMHC_INT_STA_EN,CardRemoveInt|CardInsertInt);
	smhc_clr_bit(smc_host,SMHC_INT_SIG_EN,CardRemoveInt|CardInsertInt);

	smc_host->irq = SMC_IRQNO(pdev->id);
	if (request_irq(smc_host->irq, sunxi_mci_irq, 0, DRIVER_NAME, smc_host)) {
		SMC_ERR(smc_host, "Failed to request smc card interrupt.\n");
		ret = -ENOENT;
		goto probe_free_resource;
	}
	disable_irq(smc_host->irq);

	clk_disable_unprepare(smc_host->mclk);
	sunxi_periph_reset_assert(smc_host->mclk);

	if (smc_host->cd_mode == CARD_ALWAYS_PRESENT) {
		smc_host->present = 1;
	} else if (smc_host->cd_mode == CARD_DETECT_BY_GPIO_IRQ) {
		int virq = 0;
		unsigned debounce = (2U << 4) | 1;
		virq = gpio_to_irq(smc_host->pdata->cd.gpio);
		if (IS_ERR_VALUE(virq)) {
			SMC_ERR(smc_host,"map gpio [%d] to virq failed, errno = %d\n",
			smc_host->pdata->cd.gpio, virq);
		}

		SMC_DBG(smc_host,"gpio [%d] map to virq [%d] ok\n",smc_host->pdata->cd.gpio, virq);
		/* request virq */
		ret = devm_request_irq(&pdev->dev, virq, sunxi_mci_cd_irq,
			IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, DRIVER_NAME, smc_host);

		if (IS_ERR_VALUE(ret)) {
			SMC_ERR(smc_host, "Failed to get gpio irq for card detection\n");
		}
		smc_host->cd_hdle = virq;
		/* set debounce clock */
		gpio_set_debounce(smc_host->pdata->cd.gpio, debounce);
		smc_host->present = !__gpio_get_value(smc_host->pdata->cd.gpio);
	} else if (smc_host->cd_mode == CARD_DETECT_BY_GPIO_POLL) {
		init_timer(&smc_host->cd_timer);
		smc_host->cd_timer.expires = jiffies + 1*HZ;
		smc_host->cd_timer.function = &sunxi_mci_cd_cb;
		smc_host->cd_timer.data = (unsigned long)smc_host;
		add_timer(&smc_host->cd_timer);
		smc_host->present = 0;
	}else if(smc_host->cd_mode == CARD_DETECT_BY_D3){
		u32 rval = 0;
		int i = 0;
		tasklet_init(&smc_host->d3_det_tasklet, sunxi_mci_dat3_det, (unsigned long) smc_host);

		sunxi_periph_reset_deassert(smc_host->mclk);
		ret = clk_prepare_enable(smc_host->mclk);
		if (ret)
				SMC_ERR(smc_host, "Failed to enable sdc%d mclk\n", smc_host->pdev->id);
		mdelay(1);

		smc_host->present = 1;
		//check card in (because dat3 int can not detect when card is in/out when driver on)
		for(i=0;i<5;i++){
			rval = smhc_readl(smc_host, SMHC_STA);
			if(!(rval&Dat3LineSta)){
				smc_host->present = 0;
				SMC_MSG(smc_host,"card is not in when use dat3 detect on driver start\n");
			}
			mdelay(1);
		}

		//use to balance gating
		clk_disable_unprepare(smc_host->mclk);
		sunxi_periph_reset_assert(smc_host->mclk);
	}

	mmc->ops        = &sunxi_mci_ops;
	mmc->ocr_avail	= smc_host->pdata->ocr_avail;
	mmc->caps		= smc_host->pdata->caps;
	mmc->caps2		= smc_host->pdata->caps2;
	//mmc->pm_caps	= MMC_PM_KEEP_POWER|MMC_PM_WAKE_SDIO_IRQ;
	mmc->pm_caps	= MMC_PM_KEEP_POWER;
	mmc->f_min		= smc_host->pdata->f_min;
	mmc->f_max      = smc_host->pdata->f_max;
	mmc->max_blk_count	= MAX_BLK_COUNT;
	mmc->max_blk_size	= MAX_BLK_SIZE;
	mmc->max_req_size	= mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size	= SMHC_DES_BUFFER_MAX_LEN;
	mmc->max_segs	    = MAX_DES_SIZE/sizeof(struct sdhc_idma_des);

	if (smc_host->io_flag)
		mmc->pm_flags = MMC_PM_IGNORE_PM_NOTIFY;

	ret = mmc_add_host(mmc);
	if (ret) {
		SMC_ERR(smc_host, "Failed to add mmc host.\n");
		goto probe_free_irq;
	}
	platform_set_drvdata(pdev, mmc);
	sunxi_host[pdev->id] = smc_host;

	SMC_MSG(smc_host, "sdc%d Probe: base:0x%p irq:%u sg_cpu:%p(%x) ret %d.\n",
		pdev->id, smc_host->reg_base, smc_host->irq,
		smc_host->sg_cpu, smc_host->sg_dma, ret);

	if (smc_host->cd_mode == CARD_DETECT_BY_D3 && smc_host->present == 0) {
		//in case that if card is no in when driver in,open gating power and reset for dat3 detection
		if (!smc_host->power_on) {
			int err = 0;
			SMC_DBG(smc_host, "sdc%d power on !!\n", smc_host->pdev->id);

			sunxi_mci_set_vddio(smc_host, smc_host->regulator_voltage);
			usleep_range(1000, 1500);
			smc_host->voltage = smc_host->regulator_voltage;

			if (card_power_on(smc_host)) {
				SMC_MSG(smc_host, "sdc%d card_power_on fail\n", smc_host->pdev->id);
			} else {
				SMC_MSG(smc_host, "sdc%d card_power_on ok\n", smc_host->pdev->id);
			}

			sunxi_mci_restore_io(smc_host);

			sunxi_periph_reset_deassert(smc_host->mclk);
			err = clk_prepare_enable(smc_host->mclk);
			if (err) {
				SMC_ERR(smc_host, "Failed to enable sdc%d mclk\n",
							smc_host->pdev->id);
			}

			mdelay(1);
			sunxi_mci_init_host(smc_host);
			enable_irq(smc_host->irq);
			smc_host->power_on = 1;
		}

	} else if (smc_host->present == 0) {
		/* if card is not present and the card detect mode is not CARD_DETECT_BY_D3,
		we shutdown io voltage to save power. */
		if (smc_host->power_on) {
			SMC_DBG(smc_host, "sdc%d power off !!\n", smc_host->pdev->id);
			//if use data3 detect,do not power off host
			if(smc_host->cd_mode != CARD_DETECT_BY_D3){
				disable_irq(smc_host->irq);
				sunxi_mci_exit_host(smc_host);

				clk_disable_unprepare(smc_host->mclk);
				//sunxi_periph_reset_assert(smc_host->mclk);
				sunxi_mci_hold_io(smc_host);
				if (card_power_off(smc_host)) {
					SMC_MSG(smc_host, "sdc%d card_power_off fail\n", smc_host->pdev->id);
				} else {
					SMC_MSG(smc_host, "sdc%d card_power_off ok\n", smc_host->pdev->id);
				}

				sunxi_mci_set_vddio(smc_host, SDC_WOLTAGE_OFF);
				usleep_range(1000, 1500);
				smc_host->voltage = SDC_WOLTAGE_OFF;
				smc_host->power_on = 0;
			}

			smc_host->ferror = 0;
		}
	}
	goto probe_out;

probe_free_irq:
	if (smc_host->irq)
		free_irq(smc_host->irq, smc_host);
probe_free_resource:
	sunxi_mci_resource_release(smc_host);
probe_free_host:
	mmc_free_host(mmc);
probe_out:
	return ret;
}

static int sunxi_mci_remove(struct platform_device *pdev)
{
	struct mmc_host    	*mmc  = platform_get_drvdata(pdev);
	struct sunxi_mmc_host	*smc_host = mmc_priv(mmc);

	SMC_MSG(smc_host, "%s: Remove.\n", dev_name(&pdev->dev));

	sunxi_mci_exit_host(smc_host);

	sunxi_mci_procfs_remove(smc_host);
	mmc_remove_host(mmc);

	tasklet_disable(&smc_host->tasklet);
	free_irq(smc_host->irq, smc_host);
	if (smc_host->cd_mode == CARD_DETECT_BY_GPIO_POLL)
		del_timer(&smc_host->cd_timer);
	else if (smc_host->cd_mode == CARD_DETECT_BY_GPIO_IRQ)
		devm_free_irq(&pdev->dev,smc_host->cd_hdle,smc_host);
	else if((smc_host->cd_mode == CARD_DETECT_BY_D3))
		tasklet_disable(&smc_host->d3_det_tasklet);

	sunxi_mci_resource_release(smc_host);

	mmc_free_host(mmc);
	sunxi_host[pdev->id] = NULL;

	return 0;
}

#ifdef CONFIG_PM

static void sunxi_mci_regs_save(struct sunxi_mmc_host* smc_host)
{
	//struct sunxi_mmc_ctrl_regs* bak_regs = &smc_host->bak_regs;
	SMC_MSG(smc_host,"%s not imple\n",__FUNCTION__);
}

static void sunxi_mci_regs_restore(struct sunxi_mmc_host* smc_host)
{
	//struct sunxi_mmc_ctrl_regs* bak_regs = &smc_host->bak_regs;
	SMC_MSG(smc_host,"%s not imple\n",__FUNCTION__);
}

static int sunxi_mci_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	int ret = 0;

	if (mmc) {
		struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
		ret = mmc_suspend_host(mmc);
		smc_host->suspend = ret ? 0 : 1;
		if (!ret && mmc_card_keep_power(mmc)) {
			sunxi_mci_regs_save(smc_host);

			/* gate clock for lower power */
			clk_disable_unprepare(smc_host->mclk);
			sunxi_periph_reset_assert(smc_host->mclk);
		}

		if(ret == 0 && smc_host->cd_mode == CARD_DETECT_BY_D3){
			sunxi_mci_exit_host(smc_host);

			clk_disable_unprepare(smc_host->mclk);
			sunxi_periph_reset_assert(smc_host->mclk);
			sunxi_mci_hold_io(smc_host);

			if (smc_host->pdev->id == 0)
			{
			#ifndef KEEP_CARD0_POWER_SUPPLY
				if (card_power_off(smc_host)) {
					SMC_MSG(smc_host, "sdc%d card_power_off fail\n", pdev->id);
				} else {
					SMC_MSG(smc_host, "sdc%d card_power_off ok\n", pdev->id);
				}
			#endif
			}
			else
			{
				if (card_power_off(smc_host)) {
					SMC_MSG(smc_host, "sdc%d card_power_off fail\n", pdev->id);
				} else {
					SMC_MSG(smc_host, "sdc%d card_power_off ok\n", pdev->id);
				}
			}
			sunxi_mci_set_vddio(smc_host, SDC_WOLTAGE_OFF);
			usleep_range(1000, 1500);
			smc_host->voltage = SDC_WOLTAGE_OFF;
			smc_host->power_on = 0;
		}

		SMC_MSG(NULL, "smc %d suspend\n", pdev->id);
	}

	return ret;
}

static int sunxi_mci_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	int ret = 0;

	if (mmc) {
		struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
		smc_host->suspend = 0;
		if (mmc_card_keep_power(mmc)) {
			/* enable clock for resotre */
			sunxi_periph_reset_deassert(smc_host->mclk);
			clk_prepare_enable(smc_host->mclk);
			sunxi_mci_regs_restore(smc_host);
			//sunxi_mci_update_clk(smc_host);  //WJQ
		}
		if (smc_host->cd_mode == CARD_DETECT_BY_GPIO_IRQ)
			sunxi_mci_cd_cb((unsigned long)smc_host);

		if (smc_host->cd_mode == CARD_DETECT_BY_D3)
		{
			//u32 rval = 0;

			sunxi_mci_set_vddio(smc_host, smc_host->regulator_voltage);
			usleep_range(1000, 1500);
			smc_host->voltage = smc_host->regulator_voltage;
			smc_host->power_on = 1;

			if (smc_host->pdev->id == 0)
			{
			#ifdef KEEP_CARD0_POWER_SUPPLY
				if (!smc_host->power_for_card) {
					if (card_power_on(smc_host)) {
						SMC_MSG(smc_host, "sdc%d card_power_on fail\n", pdev->id);
					} else {
						SMC_MSG(smc_host, "sdc%d card_power_on ok\n", pdev->id);
						smc_host->power_for_card = 1;
					}
				} else {
					SMC_MSG(smc_host, "sdc%d card power suppy is already on\n", pdev->id);
				}
			#else
				if (card_power_on(smc_host)) {
					SMC_MSG(smc_host, "sdc%d card_power_on fail\n", pdev->id);
				} else {
					SMC_MSG(smc_host, "sdc%d card_power_on ok\n", pdev->id);
				}
			#endif
			} else {
				if (card_power_on(smc_host)) {
					SMC_MSG(smc_host, "sdc%d card_power_on fail\n", pdev->id);
				} else {
					SMC_MSG(smc_host, "sdc%d card_power_on ok\n", pdev->id);
				}
			}

			sunxi_mci_restore_io(smc_host);
			sunxi_periph_reset_deassert(smc_host->mclk);
			ret = clk_prepare_enable(smc_host->mclk);
			if (ret) {
				SMC_ERR(smc_host, "Failed to enable sdc%d mclk\n",
							smc_host->pdev->id);
			}

			mdelay(1);
			//rval = smhc_readl(smc_host, REG_RINTR);      //comment by WJQ
			//SMC_MSG(smc_host, ">> REG_RINTR=0x%x\n", rval);
			sunxi_mci_init_host(smc_host);
		}

		ret = mmc_resume_host(mmc);
		smc_host->suspend = ret ? 1 : 0;
		SMC_MSG(NULL, "smc %d resume\n", pdev->id);
	}

	return ret;
}

static const struct dev_pm_ops sunxi_mci_pm = {
	.suspend	= sunxi_mci_suspend,
	.resume		= sunxi_mci_resume,
};
#define sunxi_mci_pm_ops &sunxi_mci_pm

#else /* CONFIG_PM */

#define sunxi_mci_pm_ops NULL

#endif /* CONFIG_PM */

extern int mmc_go_idle(struct mmc_host *host);
extern int mmc_send_op_cond(struct mmc_host *host, u32 ocr, u32 *rocr);
extern int mmc_send_status(struct mmc_card *card, u32 *status);
extern void mmc_set_clock(struct mmc_host *host, unsigned int hz);
static void shutdown_mmc(struct platform_device * pdev)
{
	u32 ocr = 0;
	u32 err = 0;
	struct mmc_host *mmc = NULL;
	struct sunxi_mmc_host *host_sunxi = NULL;
	u32 status = 0;

 	SMC_MSG(NULL, "%s: card %d shut down mmc\n", __FUNCTION__, pdev->id);

	if (pdev->id != 2) {
		SMC_MSG(NULL, "%s: is not card 2, return\n", __FUNCTION__);
		goto out;
	}

	mmc = platform_get_drvdata(pdev);
	if (mmc == NULL) {
		SMC_ERR(NULL, "%s: mmc is NULL\n", __FUNCTION__);
		goto out;
	}

	host_sunxi = mmc_priv(mmc);
	if (host_sunxi == NULL) {
		SMC_ERR(NULL, "%s: host_sunxi is NULL\n", __FUNCTION__);
		goto out;
	}

	//claim host to not allow androd read/write during shutdown
	SMC_MSG(host_sunxi, "%s: claim host\n", __FUNCTION__);
	mmc_claim_host(mmc);

	do {
		if (mmc_send_status(mmc->card, &status) != 0) {
			SMC_ERR(host_sunxi,"%s:set status failed\n", __FUNCTION__);
			goto err_out;
		}
	} while(status != 0x00000900);

	mmc_set_clock(mmc, 400000);
	err = mmc_go_idle(mmc);
	if (err) {
		SMC_ERR(host_sunxi, "%s: mmc_go_idle err\n", __FUNCTION__);
		goto err_out;
	}

	if (mmc->card->type != MMC_TYPE_MMC) {//sd can support cmd1,so not send cmd1
		goto out;//not release host to not allow android to read/write after shutdown
	}

	//SMC_MSG(host_sunxi,"%s mmc_send_op_cond\n",__FUNCTION__);
	err = mmc_send_op_cond(mmc, 0, &ocr);
	if (err) {
		SMC_ERR(host_sunxi, "%s: first mmc_send_op_cond err\n", __FUNCTION__);
		goto err_out;
	}

	err = mmc_send_op_cond(mmc, ocr | (1 << 30), &ocr);
	if (err) {
		SMC_ERR(host_sunxi, "%s: mmc_send_op_cond err\n", __FUNCTION__);
		goto err_out;
	}

	//do not release host to not allow android to read/write after shutdown
	goto out;

err_out:
	SMC_MSG(NULL, "%s: release host\n", __FUNCTION__);
	mmc_release_host(mmc);
out:
	SMC_MSG(NULL, "%s: mmc shutdown exit\n", __FUNCTION__);

	return ;
}

static struct sunxi_mmc_platform_data sunxi_mci_pdata[] = {
	[0] = {
		.ocr_avail = MMC_VDD_28_29 | MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32
				| MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195,
		.caps = MMC_CAP_4_BIT_DATA
#ifndef MMC_FPGA
			| MMC_CAP_NONREMOVABLE
#endif
			| MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED
			| MMC_CAP_1_8V_DDR /* HS-DDR mode */
			| MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 | MMC_CAP_UHS_SDR50
			| MMC_CAP_UHS_DDR50
			| MMC_CAP_8_BIT_DATA
			| MMC_CAP_SDIO_IRQ
			| MMC_CAP_SET_XPC_330 | MMC_CAP_DRIVER_TYPE_A
			| MMC_CAP_WAIT_WHILE_BUSY
			| MMC_CAP_CMD23,
		.caps2 = MMC_CAP2_HS200_1_8V_SDR,
		.f_min = 150000,
		.f_max = 80000000,
		.f_ddr_max = 40000000,
		.regulator=NULL,
		.mmc_clk_dly[MMC_CLK_400K]				= {MMC_CLK_400K,				0,0},
		.mmc_clk_dly[MMC_CLK_25M]				= {MMC_CLK_25M,					0,5},
		.mmc_clk_dly[MMC_CLK_50M]				= {MMC_CLK_50M,					3,4},
		.mmc_clk_dly[MMC_CLK_50MDDR]			= {MMC_CLK_50MDDR,				2,4},
		.mmc_clk_dly[MMC_CLK_50MDDR_8BIT]	= {MMC_CLK_50MDDR_8BIT,				2,4},
		.mmc_clk_dly[MMC_CLK_100M]				= {MMC_CLK_100M,				1,4},
		.mmc_clk_dly[MMC_CLK_200M]				= {MMC_CLK_200M,				1,4},
	},
};
static struct platform_device sunxi_mci_device[] = {
	[0] = {.name = DRIVER_NAME, .id = SMHC_DEVICE_ID, .dev.platform_data = &sunxi_mci_pdata[0]},
};

#define SUNXI_MCI_DEVNUM (sizeof(sunxi_mci_device)/sizeof(sunxi_mci_device[0]))

static struct platform_driver sunxi_mci_driver = {
	.driver.name    = DRIVER_NAME,
	.driver.owner   = THIS_MODULE,
	.driver.pm	= sunxi_mci_pm_ops,
	.probe          = sunxi_mci_probe,
	.remove         = sunxi_mci_remove,
	.shutdown	= shutdown_mmc,
};

static int sunxi_mci_get_devinfo(void)
{
	u32 i, j ,k;
	char mmc_para[16] = {0};
	struct sunxi_mmc_platform_data* mmcinfo;
	char* mmc_para_io[10] = {"sdc_clk", "sdc_cmd", "sdc_d0", "sdc_d1", "sdc_d2",
				"sdc_d3", "sdc_d4", "sdc_d5", "sdc_d6", "sdc_d7"};
	script_item_u val;
	script_item_value_type_e type;
	char *speed_mod[MMC_CLK_MOD_NUM] = {"400K","25M","50M","50MDDR","50MDDR_8BIT","100M","200M"};

	i = SMHC_DEVICE_ID;
	do {
		mmcinfo = &sunxi_mci_pdata[0];
		sprintf(mmc_para, "mmc%d_para", i);
		/* get used information */
		type = script_get_item(mmc_para, "sdc_used", &val);
		if (type != SCIRPT_ITEM_VALUE_TYPE_INT) {
			SMC_MSG(NULL, "get mmc%d's usedcfg failed\n", i);
			goto fail;
		}
		mmcinfo->used = val.val;
		if (!mmcinfo->used)
			continue;
		/* get cdmode information */
		type = script_get_item(mmc_para, "sdc_detmode", &val);
		if (type != SCIRPT_ITEM_VALUE_TYPE_INT) {
			SMC_MSG(NULL, "get mmc%d's detmode failed\n", i);
			goto fail;
		}
		mmcinfo->cdmode = val.val;
		if (mmcinfo->cdmode == CARD_DETECT_BY_GPIO_POLL ||
			mmcinfo->cdmode == CARD_DETECT_BY_GPIO_IRQ) {
			type = script_get_item(mmc_para, "sdc_det", &val);
			if (type != SCIRPT_ITEM_VALUE_TYPE_PIO) {
				SMC_MSG(NULL, "get mmc%d's IO(sdc_cd) failed\n", i);
			} else {
				mmcinfo->cd = val.gpio;
			}
		}
		/* get buswidth information */
		type = script_get_item(mmc_para, "sdc_buswidth", &val);
		if (type != SCIRPT_ITEM_VALUE_TYPE_INT) {
			SMC_MSG(NULL, "get mmc%d's buswidth failed\n", i);
			goto fail;
		}
		mmcinfo->width = val.val;
		/* get mmc IOs information */
		for (j=0; j<mmcinfo->width+2; j++) {
			type = script_get_item(mmc_para, mmc_para_io[j], &val);
			if (type != SCIRPT_ITEM_VALUE_TYPE_PIO) {
				SMC_MSG(NULL, "get mmc%d's IO(%s) failed\n", j, mmc_para_io[j]);
				goto fail;
			}
			mmcinfo->mmcio[j] = val.gpio;
		}
		/* get wpmode information */
		type = script_get_item(mmc_para, "sdc_use_wp", &val);
		if (type != SCIRPT_ITEM_VALUE_TYPE_INT) {
			SMC_MSG(NULL, "get mmc%d's wpmode failed\n", i);
			goto fail;
		}
		mmcinfo->wpmode = val.val;
		if (mmcinfo->wpmode) {
			/* if wpmode==1 but cann't get the wp IO, we assume there is no
			   write protect detection */
			type = script_get_item(mmc_para, "sdc_wp", &val);
			if (type != SCIRPT_ITEM_VALUE_TYPE_PIO) {
				SMC_MSG(NULL, "get mmc%d's IO(sdc_wp) failed\n", i);
				mmcinfo->wpmode = 0;
			} else {
				mmcinfo->wp = val.gpio;
			}
		}
		/* get emmc-rst information */
		type = script_get_item(mmc_para, "emmc_rst", &val);
		if (type != SCIRPT_ITEM_VALUE_TYPE_PIO) {
			mmcinfo->has_hwrst = 0;
		} else {
			mmcinfo->has_hwrst = 1;
			mmcinfo->hwrst = val.gpio;
		}
		/* get sdio information */
		type = script_get_item(mmc_para, "sdc_isio", &val);
		if (type != SCIRPT_ITEM_VALUE_TYPE_INT) {
			SMC_MSG(NULL, "get mmc%d's isio? failed\n", i);
			goto fail;
		}
		mmcinfo->isiodev = val.val;

		/* get regulator information */
		type = script_get_item(mmc_para, "sdc_regulator", &val);
		if (type != SCIRPT_ITEM_VALUE_TYPE_STR) {
			SMC_MSG(NULL, "get mmc%d's sdc_regulator failed\n", i);
			mmcinfo->regulator = NULL;
		} else {
			if (!strcmp(val.str, "none")) {
				mmcinfo->regulator = NULL;
				/* If here found that no regulator can be used for this card,
				   we clear all of the UHS features support.
				   But there is a excetpion. SD/MMC controller 1 is used for sdio wifi,
				   If there is a sdio v3.0 wifi, we should make wifi io voltage power
				   up with 1.8V and not configure "sdc_regulator" in [mmc1_para].
				   During initial process, the driver do not switch io voltage and switch
				   speed mode according to device capabilites. */
				//if (i != 1) {
				//	mmcinfo->caps &= ~(MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25
				//			| MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_DDR50);
				//}
			} else
				mmcinfo->regulator = val.str;
		}

		/* get power information */
		type = script_get_item(mmc_para, "sdc_power_supply", &val);
		if (type != SCIRPT_ITEM_VALUE_TYPE_STR)
		{
			SMC_MSG(NULL, "get mmc%d's sdc_power failed\n", i);
			mmcinfo->power_supply = NULL;
		}
		else
		{
			if (!strcmp(val.str, "none"))
			{
				mmcinfo->power_supply = NULL;
			}
			else
			{
				mmcinfo->power_supply = val.str;
			}
			SMC_MSG(NULL, "get mmc%d's power supply '%s' ok\n", i,mmcinfo->power_supply);
		}

		/* get ddrmode information */
		type = script_get_item(mmc_para, "sdc_ddrmode", &val);
		if (type != SCIRPT_ITEM_VALUE_TYPE_INT) {
			SMC_MSG(NULL, "get mmc%d's ddrmode fail\n", i);
			mmcinfo->used_ddrmode = 0;
		} else {
			mmcinfo->used_ddrmode = val.val;
			SMC_MSG(NULL, "get mmc%d's ddrmode ok, val = %d\n", i, mmcinfo->used_ddrmode);
		}

		/* get f_max */
		type = script_get_item(mmc_para, "sdc_f_max", &val);
		if (type != SCIRPT_ITEM_VALUE_TYPE_INT) {
			SMC_MSG(NULL, "get mmc%d's sdc_f_max failed,use default sdc_f_max %d\n",i, mmcinfo->f_max);
		}else{
			if ((val.val>mmcinfo->f_max)||(val.val<mmcinfo->f_min)){
				SMC_MSG(NULL, "mmc%d's input sdc_f_max wrong,use default sdc_f_max %d\n", i,  mmcinfo->f_max);
			}else{
				mmcinfo->f_max = val.val;
				SMC_MSG(NULL, "get mmc%d's sdc_f_max ok,sdc_f_max %d\n", i, mmcinfo->f_max);
			}
		}

		/* get f_ddr_max */
		type = script_get_item(mmc_para, "sdc_f_ddr_max", &val);
		if (type != SCIRPT_ITEM_VALUE_TYPE_INT) {
			SMC_MSG(NULL, "get mmc%d's sdc_f_ddr_max failed,use default sdc_f_ddr_max %d\n",i, mmcinfo->f_ddr_max);
		}else{
			if((val.val>mmcinfo->f_ddr_max)||(val.val<mmcinfo->f_min)){
				SMC_MSG(NULL, "mmc%d's input sdc_f_ddr_max wrong,use default sdc_f_ddr_max %d\n", i,  mmcinfo->f_ddr_max);
			}else{
				mmcinfo->f_ddr_max = val.val;
				SMC_MSG(NULL, "get mmc%d's sdc_f_ddr_max ok,sdc_f_ddr_max %d\n", i, mmcinfo->f_ddr_max);
			}
		}

		/* get ex_dly_used information */
		type = script_get_item(mmc_para, "sdc_ex_dly_used", &val);
		if (type != SCIRPT_ITEM_VALUE_TYPE_INT) {
			SMC_MSG(NULL, "get mmc%d's sdc_ex_dly_used failed, use default dly\n", i);
			continue;
		}
		/* 1, manual mode: delay parameters are from sys_config.fex directly;
		   2, auto mode  : delay parameters are from sys_config.fex, but it maybe changed during update firmware. */
		if ((val.val != 1)&&(val.val != 2)) {
			SMC_MSG(NULL, "mmc%d's not use sdc_ex_dly, use default dly\n", i);
			continue;
		}

		/* get odly and sdly information */
		for(k = 0;k< MMC_CLK_MOD_NUM; k++) {
			char str_buf[30] = {0};

			sprintf(str_buf,"sdc_odly_%s",speed_mod[k]);
			type = script_get_item(mmc_para, str_buf, &val);
			if (type != SCIRPT_ITEM_VALUE_TYPE_INT) {
				SMC_MSG(NULL, "get mmc%d's %s failed,use default dly %d\n",i, str_buf, mmcinfo->mmc_clk_dly[k].oclk_dly);
			}else{
				if((val.val>7)||(val.val<0)){
					SMC_MSG(NULL, "mmc%d's input %s wrong,use default dly %d\n", i, str_buf, mmcinfo->mmc_clk_dly[k].oclk_dly);
				}else{
					mmcinfo->mmc_clk_dly[k].oclk_dly = val.val;
					SMC_MSG(NULL, "get mmc%d's %s ok,oclk_dly %d\n", i, str_buf,mmcinfo->mmc_clk_dly[k].oclk_dly);
				}
			}

			sprintf(str_buf,"sdc_sdly_%s",speed_mod[k]);
			type = script_get_item(mmc_para, str_buf, &val);
			if (type != SCIRPT_ITEM_VALUE_TYPE_INT) {
				SMC_MSG(NULL, "get mmc%d's %s failed,use default dly %d\n", i, str_buf, mmcinfo->mmc_clk_dly[k].sclk_dly);
			}else{
				if((val.val>7)||(val.val<0)){
					SMC_MSG(NULL, "mmc%d's input %s wrong,use default dly %d\n", i, str_buf, mmcinfo->mmc_clk_dly[k].sclk_dly);
				}else{
					mmcinfo->mmc_clk_dly[k].sclk_dly = val.val;
					SMC_MSG(NULL, "get mmc%d's %s ok,sclk_dly %d\n", i, str_buf, mmcinfo->mmc_clk_dly[k].sclk_dly);
				}
			}
		}
	}while(0);


	return 0;
fail:
	return -1;
}

static int __init sunxi_mci_init(void)
{
	int i;
	int sdc_used = 0;
	int boot_card = 0;
	int io_used = 0;
	struct sunxi_mmc_platform_data* mmcinfo;

	SMC_MSG(NULL, "%s\n", DRIVER_VERSION);

	SMC_MSG(NULL, "sunxi_mci_init\n");
	/* get devices information from sys_config1.fex */
	if (sunxi_mci_get_devinfo()) {
		SMC_MSG(NULL, "parse sys_cofnig.fex info failed\n");
		return 0;
	}

	/*
	 * Here we check whether there is a boot card. If the boot card exists,
	 * we register it firstly to make it be associatiated with the device
	 * node 'mmcblk0'. Then the applicantions of Android can fix the boot,
	 * system, data patitions on mmcblk0p1, mmcblk0p2... etc.
	 */
	for (i=0; i<SUNXI_MCI_DEVNUM; i++) {
		mmcinfo = &sunxi_mci_pdata[i];
		if (mmcinfo->used) {
			sdc_used |= 1 << i;
			if (mmcinfo->cdmode == CARD_ALWAYS_PRESENT)
				boot_card |= 1 << i;
			if (mmcinfo->isiodev)
				io_used |= 1 << i;
		}
	}

	SMC_MSG(NULL, "MMC host used card: 0x%x, boot card: 0x%x, io_card %d\n",
					sdc_used, boot_card, io_used);
	/* register boot card firstly */
	for (i=0; i<SUNXI_MCI_DEVNUM; i++) {
		if (boot_card & (1 << i)) {
			platform_device_register(&sunxi_mci_device[i]);
			device_enable_async_suspend(&sunxi_mci_device[i].dev);
		}
	}
	/* register other cards */
	for (i=0; i<SUNXI_MCI_DEVNUM; i++) {
		if (boot_card & (1 << i))
			continue;
		if (sdc_used & (1 << i)) {
			platform_device_register(&sunxi_mci_device[i]);
			device_enable_async_suspend(&sunxi_mci_device[i].dev);
		}
	}

	return platform_driver_register(&sunxi_mci_driver);
}

static void __exit sunxi_mci_exit(void)
{
	SMC_MSG(NULL, "sunxi_mci_exit\n");
	platform_driver_unregister(&sunxi_mci_driver);
}

module_init(sunxi_mci_init);
module_exit(sunxi_mci_exit);

MODULE_DESCRIPTION("Winner's SD/MMC Card Controller Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aaron.maoye<leafy.myeh@reuuimllatech.com>");
MODULE_ALIAS("platform:sunxi-mmc");
