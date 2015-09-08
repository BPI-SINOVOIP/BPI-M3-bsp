/*
 * Copyright (c) 2013 Espressif System.
 *
 *  sdio stub code for allwinner
 */

extern void sunxi_mci_rescan_card(unsigned id, unsigned insert);
extern int wifi_pm_gpio_ctrl(char* name, int level);
extern int rf_pm_gpio_ctrl(char *name, int level);

#define SDIO_ID 1

void sif_platform_rescan_card(unsigned insert)
{
	printk("%s id %d %u\n", __func__, SDIO_ID, insert);
	sunxi_mci_rescan_card(SDIO_ID, insert);
}

void sif_platform_reset_target(void)
{
        rf_pm_gpio_ctrl("chip_en", 0);
        mdelay(100);
        rf_pm_gpio_ctrl("chip_en", 1);
	
	mdelay(100);
}

void sif_platform_target_poweroff(void)
{
        rf_pm_gpio_ctrl("wl_reg_on", 0);
	mdelay(100);
        wifi_pm_gpio_ctrl("chip_en", 0);
	mdelay(100);
	
	//wifi_pm_gpio_ctrl("esp_wl_pw",0);
	//mdelay(10);
}

void sif_platform_target_poweron(void)
{
//	int num = 3;
//	while(num--)
//	{
//		if(!wifi_pm_gpio_ctrl("esp_wl_pw",1))
//			break;
//		mdelay(10);
//	}
//	mdelay(120);
	rf_pm_gpio_ctrl("chip_en",1);
	mdelay(100);

        wifi_pm_gpio_ctrl("wl_reg_on",1);
	mdelay(100);
        }

void sif_platform_target_speed(int high_speed)
{
        wifi_pm_gpio_ctrl("wl_reg_on", high_speed);
}

#ifdef MMC_NO_CHANGE
extern int sunxi_mci_check_r1_ready(struct mmc_host* mmc, unsigned ms);
void sif_platform_check_r1_ready(struct esp_pub *epub)
{
        struct esp_sdio_ctrl *sctrl = NULL;
        struct sdio_func *func = NULL;
	int err;

        ASSERT(epub != NULL);
        sctrl = (struct esp_sdio_ctrl *)epub->sif;
        func = sctrl->func;
        ASSERT(func != NULL);
	err = sunxi_mci_check_r1_ready(func->card->host, 1000);
        if (err != 0)
          printk("%s data timeout.\n", __func__);
}
#else
void sif_platform_check_r1_ready(struct esp_pub *epub)
{
}
#endif

#ifdef ESP_ACK_INTERRUPT
extern void sdmmc_ack_interrupt(struct mmc_host *mmc);
void sif_platform_ack_interrupt(struct esp_pub *epub)
{
        struct esp_sdio_ctrl *sctrl = NULL;
        struct sdio_func *func = NULL;

        ASSERT(epub != NULL);
        sctrl = (struct esp_sdio_ctrl *)epub->sif;
        func = sctrl->func;
        ASSERT(func != NULL);

        sdmmc_ack_interrupt(func->card->host);
}
#endif //ESP_ACK_INTERRUPT

module_init(esp_sdio_init);
module_exit(esp_sdio_exit);
