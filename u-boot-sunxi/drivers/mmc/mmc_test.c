/*
 * MMC driver for allwinner sunxi platform.
 *
 */

#include <config.h>
#include <common.h>
#include <command.h>
#include <errno.h>
#include <mmc.h>
#include <part.h>
#include <malloc.h>
#include "mmc_def.h"
#include "mmc_test.h"

#ifdef MMC_INTERNAL_TEST

/*
	write, read and compare.
*/
int mmc_t_rwc(struct mmc *mmc, ulong start, ulong blkcnt)
{
	ulong blk;
	u32 *src=NULL, *dst=NULL;
	int i, ret=0;

	MMCINFO("%s: start test: start %d, len %d\n", __FUNCTION__, start, blkcnt);

	src = (unsigned int *)malloc(512*blkcnt);
	if (src == NULL) {
		MMCINFO("%s: malloc error!\n", __FUNCTION__);
		return -1;
	}
	else
		MMCINFO("%s: src 0x%x!\n", __FUNCTION__, src);

	dst = (unsigned int *)malloc(512*blkcnt);
	if (src == NULL) {
		MMCINFO("%s: malloc error!\n", __FUNCTION__);
		ret = -1;
		goto ERR_RET1;
	}
	else
		MMCINFO("%s: dst 0x%x!\n", __FUNCTION__, dst);

	for (i=0; i<blkcnt*512>>2; i++)
	{
		src[i] = i;
		dst[i] = 0;
	}

	if ((mmc->block_dev.block_write == NULL)
		|| (mmc->block_dev.block_read == NULL)) {
		MMCINFO("%s: write and/or read func is null\n", __FUNCTION__);
	}

	blk = mmc->block_dev.block_write(mmc->block_dev.dev, start, blkcnt, src);
	if (blk != blkcnt) {
		MMCINFO("%s: write block fail\n", __FUNCTION__);
		ret = -1;
		goto ERR_RET;
	}

	blk = mmc->block_dev.block_read(mmc->block_dev.dev, start, blkcnt, dst);
	if (blk != blkcnt) {
		MMCINFO("%s: read block fail\n", __FUNCTION__);
		ret = -1;
		goto ERR_RET;
	}

	if (memcmp(src, dst, blkcnt*512) != 0) {
		MMCINFO("%s: memory compare fail\n", __FUNCTION__);
		ret = -1;
	}

	for (i=0; i<100; i+=10)
	{
		MMCINFO("%d: %d %d\n", i, src[i], dst[i]);
	}

ERR_RET:
	free(dst);

ERR_RET1:
	free(src);

	MMCINFO("%s: test end......%s\n", __FUNCTION__, ret?"FAIL!":"OK");
	return ret;
}


/*
	write, read and compare.
*/
int mmc_t_seq_write_speed_test(struct mmc *mmc, ulong start, ulong blkcnt_tatal, ulong blkcnt_eachcmd)
{
	ulong blk, blkcnt = blkcnt_eachcmd;
	u32 *src=NULL, *dst=NULL;
	int i, ret=0;
	int loop = blkcnt_tatal/blkcnt_eachcmd;
	ulong begin_time, time_elapse;
	

	MMCINFO("%s: start test: start %d, len %d, each %d\n", __FUNCTION__, start, blkcnt_tatal, blkcnt_eachcmd);

	src = (unsigned int *)malloc(512*blkcnt);
	if (src == NULL) {
		MMCINFO("%s: malloc error!\n", __FUNCTION__);
		return -1;
	}
	else
		MMCINFO("%s: src 0x%x!\n", __FUNCTION__, src);

	dst = (unsigned int *)malloc(512*blkcnt);
	if (src == NULL) {
		MMCINFO("%s: malloc error!\n", __FUNCTION__);
		ret = -1;
		goto ERR_RET1;
	}
	else
		MMCINFO("%s: dst 0x%x!\n", __FUNCTION__, dst);

	for (i=0; i<blkcnt*512>>2; i++)
	{
		src[i] = i;
		dst[i] = 0;
	}

	if ((mmc->block_dev.block_write == NULL)
		|| (mmc->block_dev.block_read == NULL)) {
		MMCINFO("%s: write and/or read func is null\n", __FUNCTION__);
	}

	tick_printf("wrie loop %d\n", loop);
	tick_printf("--- start write ...\n");
	begin_time = get_timer(0);
	for (i=0; i<loop; i++)
	{
		blk = mmc->block_dev.block_write(mmc->block_dev.dev, start+i*blkcnt, blkcnt, src);
		if (blk != blkcnt) {
			MMCINFO("%s: write block fail\n", __FUNCTION__);
			ret = -1;
			goto ERR_RET;
		}
	}
	time_elapse = get_timer(begin_time);
	tick_printf("--- end write\n");

	tick_printf("time: %d s, data size: %d KB\n", time_elapse, blkcnt_tatal>>1);
	tick_printf("write speed: %d KB/s \n", (blkcnt_tatal>>1)/time_elapse);


ERR_RET:
	free(dst);

ERR_RET1:
	free(src);

	MMCINFO("%s: test end......%s\n", __FUNCTION__, ret?"FAIL!":"OK");
	return ret;
}

int mmc_t_memcpy(void)
{
	u32 blkcnt = 128;
	u32 *src=NULL, *dst=NULL;
	int i, ret=0;
	int loop = 0;
	ulong begin_time, time_elapse;
	

	MMCINFO("%s: start test .......\n", __FUNCTION__);

	src = (unsigned int *)malloc(512*blkcnt);
	if (src == NULL) {
		MMCINFO("%s: malloc error!\n", __FUNCTION__);
		return -1;
	}
	else
		MMCINFO("%s: src 0x%x!\n", __FUNCTION__, src);

	dst = (unsigned int *)malloc(512*blkcnt);
	if (src == NULL) {
		MMCINFO("%s: malloc error!\n", __FUNCTION__);
		ret = -1;
		goto ERR_RET1;
	}
	else
		MMCINFO("%s: dst 0x%x!\n", __FUNCTION__, dst);		
	
	loop = 1024;
	tick_printf("loop %d\n", loop);
	tick_printf("--- start ...\n");
	begin_time = get_timer(0);
	for (i=0; i<loop; i++)
	{
		memcpy(src, dst, 512*blkcnt);
	}
	time_elapse = get_timer(begin_time);
	tick_printf("--- end \n");

	tick_printf("time: %d s, data size: %d KB\n", time_elapse, (loop*blkcnt)>>1);
	tick_printf("write speed: %d KB/s \n", ((loop*blkcnt)>>1)/time_elapse);


	free(dst);

ERR_RET1:
	free(src);

	MMCINFO("%s: test end......%s\n", __FUNCTION__, ret?"FAIL!":"OK");
	return ret;		
}
	
int mmc_t_emmc_erase(struct mmc *mmc, ulong start, ulong blkcnt)
{
	return 0;
}

int mmc_t_emmc_trim(struct mmc *mmc, ulong start, ulong blkcnt)
{
	return 0;
}

int mmc_t_emmc_discard(struct mmc *mmc, ulong start, ulong blkcnt)
{
	return 0;
}

int mmc_t_emmc_secure_erase(struct mmc *mmc, ulong start, ulong blkcnt)
{
	return 0;
}

int mmc_t_emmc_secure_trim(struct mmc *mmc, ulong start, ulong blkcnt)
{
	return 0;
}

int mmc_t_emmc_sanitize(struct mmc *mmc)
{
	return 0;
}


#endif /*MMC_INTERNAL_TEST*/

