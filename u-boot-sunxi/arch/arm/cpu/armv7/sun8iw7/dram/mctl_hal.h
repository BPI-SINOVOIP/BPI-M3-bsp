#ifndef   _MCTL_HAL_H
#define   _MCTL_HAL_H
//only for dram simulation
#define   DRAM_TYPE_DDR3
//#define   DRAM_TYPE_DDR2
//#define   DRAM_TYPE_LPDDR2
//#define   DRAM_TYPE_LPDDR3
#include "common.h"

#define __REG(x)    					(*(volatile unsigned int   *)(x))
//-------------------------------------------------------------------------------------------------------------
//#define USE_CMP
//#define DRAMC_ALL_TRAINING_TEST
//#define DRAMC_REGISTER_TEST
//#define DRAMC_ADDRESS_MAP0_TEST   //ok
//#define DRAMC_ADDRESS_MAP1_TEST//ok
//#define DRAMC_MASTER_TEST
//#define DRAMC_PHY_BIST_TEST
//#define DRAMC_PHY_PAD_HOLD_TEST
//#define DRAMC_SUPER_STANDBY_TEST
//#define DRAMC_SUPER_STANDBY_ZQ_CALIBRATION_TEST
//#define DRAMC_BANDWIDTH_CNT_TEST
//#define DRAMC_HALF_DQ_TEST
//#define DRAMC_ALL_TRAINING_TEST
//#define DRAMC_2T_1T_TEST
//#define DRAMC_USE_MPR_TEST
//#define DRAMC_FRE_800M
//#define DRAMC_RAM_BIST
//#define DRAMC_SOFT_ADJUST_FRE
//#define DRAMC_AUTO_DQS_GATE_PD_MODE_TEST
//#define DRAMC_AUTO_DQS_GATE_PU_MODE_TEST
//#define DRAMC_PHY_ZQ_TEST
//#define DRAMC_TIMING_TEST
//#define DRAMC_NET_PATTERN_TEST
//#define DRAMC_MDFS_CFS_TEST
//#define DRAMC_MDFS_DFS_TEST

//-------------------------------------------------------------------------------------------------------------

//#define USE_LPDDR
//#include  "bsp.h"
//#include  "dram_for_debug.h"
//#define rand32() ((unsigned int) rand16() | ( (unsigned int) rand16() << 16))
//#define rand_ul() rand32()
#ifdef DRAM_PRINK_ENABLE
#  define dram_dbg(fmt,args...)	UART_printf2(fmt ,##args)
#else
#  define dram_dbg(fmt,args...)
#endif

typedef struct __DRAM_PARA
{
	//normal configuration
	unsigned int        dram_clk;
	unsigned int        dram_type;		//dram_type			DDR2: 2				DDR3: 3		LPDDR2: 6	LPDDR3: 7	DDR3L: 31
	//unsigned int        lpddr2_type;	//LPDDR2 type		S4:0	S2:1 	NVM:2
    unsigned int        dram_zq;		//do not need
    unsigned int		dram_odt_en;

	//control configuration
	unsigned int		dram_para1;
    unsigned int		dram_para2;

	//timing configuration
	unsigned int		dram_mr0;
    unsigned int		dram_mr1;
    unsigned int		dram_mr2;
    unsigned int		dram_mr3;
    unsigned int		dram_tpr0;	//DRAMTMG0
    unsigned int		dram_tpr1;	//DRAMTMG1
    unsigned int		dram_tpr2;	//DRAMTMG2
    unsigned int		dram_tpr3;	//DRAMTMG3
    unsigned int		dram_tpr4;	//DRAMTMG4
    unsigned int		dram_tpr5;	//DRAMTMG5
   	unsigned int		dram_tpr6;	//DRAMTMG8

    //reserved for future use
    unsigned int		dram_tpr7;
    unsigned int		dram_tpr8;
    unsigned int		dram_tpr9;
    unsigned int		dram_tpr10;
    unsigned int		dram_tpr11;
    unsigned int		dram_tpr12;
    unsigned int		dram_tpr13;


}__dram_para_t;
extern unsigned int compare_regions(unsigned long *bufa, unsigned long *bufb, long count);
//extern int16 rand16(void);
extern unsigned int test_random_value(unsigned long *bufa, unsigned long *bufb, long count);
extern unsigned int test_stuck_address(unsigned long *bufa, long count);
extern unsigned int dram_mem_test(void);
extern unsigned int test_basic_data(u32 len);
extern void local_delay (unsigned int n);
extern unsigned int mctl_init(void);
extern unsigned int mctl_init_dram(void);
extern unsigned int mctl_sys_init(__dram_para_t *para);
extern void auto_set_timing_para(__dram_para_t *para);
extern void auto_set_dram_para(__dram_para_t *para);
extern signed int init_DRAM(int type, __dram_para_t *para);
extern void mctl_com_init(__dram_para_t *para);
extern unsigned int mctl_soft_training(void);
extern unsigned int mdfs_dfs(unsigned int freq_jump,__dram_para_t *para);
extern unsigned int mdfs_cfs(unsigned int freq_jump,__dram_para_t *para);
extern void mctl_self_refresh_exit(unsigned int ch_index);
#endif  //_MCTL_HAL_H
