#ifndef _A6_INNO_
#define _A6_INNO_

#include "mcompat_drv.h"
#include "mcompat_lib.h"
#include "A6_inno_cmd.h"
#include "A6_inno_fan.h"
#include "A6_inno_clock.h"
#include "A6_inno_gpio.h"

#define WEAK_CHIP_THRESHOLD 5
#define BROKEN_CHIP_THRESHOLD 5
#define LOCK_USER 0

#define INNO_MINER_TYPE_FILE            "/tmp/type"
#define INNO_HARDWARE_VERSION_FILE      "/tmp/hwver"
#define MAX_CHIP_NUM                    (72)
#define MAX_CORES                      (72*9)
#define SPI_SPEED_RUN			(SPI_SPEED_6250K)
const int g_pll_list[PLL_LV_NUM];
const uint8_t g_pll_regs[PLL_LV_NUM][REG_LENGTH];
#define CHAIN_DEAD_TIME              (5*60)




typedef enum{
HARDWARE_VERSION_NONE = 0x00,
HARDWARE_VERSION_G9 = 0x09,
HARDWARE_VERSION_G19 = 0x13,

}hardware_version_e;

/*
typedef enum{
MINER_TYPE_NONE = 0x00,
MINER_TYPE_T0,
MINER_TYPE_T1,
MINER_TYPE_T2,
MINER_TYPE_T3,
MINER_TYPE_T4,
MINER_TYPE_T5,
MINER_TYPE_SUM,

}miner_type_e;
*/

typedef struct{
   double highest_vol[ASIC_CHAIN_NUM];    /* chip temp bits */;
   double lowest_vol[ASIC_CHAIN_NUM];    /* chip temp bits */;
   double avarge_vol[ASIC_CHAIN_NUM];    /* chip temp bits */; 
   int stat_val[ASIC_CHAIN_NUM][ASIC_CHIP_NUM]; 
   int stat_cnt[ASIC_CHAIN_NUM][ASIC_CHIP_NUM];
}inno_reg_ctrl_t;

bool inno_check_voltage(struct A1_chain *a1, int chip_id, inno_reg_ctrl_t *s_reg_ctrl);
void inno_configure_tvsensor(struct A1_chain *a1, int chip_id,bool is_tsensor);
int inno_get_voltage_stats(struct A1_chain *a1, inno_reg_ctrl_t *s_reg_ctrl);

bool check_chip(struct A1_chain *a1, int i);
void prechain_detect(struct A1_chain *a1, int idxpll);
bool prechain_detect_yex(struct A1_chain *a1, int idxpll, int lastidx);

int chain_detect(struct A1_chain *a1);
bool abort_work(struct A1_chain *a1);

int get_current_ms(void);
bool is_chip_disabled(struct A1_chain *a1, uint8_t chip_id);
void disable_chip(struct A1_chain *a1, uint8_t chip_id);

bool get_nonce(struct A1_chain *a1, uint8_t *nonce, uint8_t *chip_id, uint8_t *job_id);
bool set_work(struct A1_chain *a1, uint8_t chip_id, struct work *work, uint8_t queue_states);
void check_disabled_chips(struct A1_chain *a1, int pllnum);
uint8_t *create_job(uint8_t chip_id, uint8_t job_id, struct work *work);
void test_bench_pll_config(struct A1_chain *a1,uint32_t uiPll);

hardware_version_e inno_get_hwver(void);
inno_type_e inno_get_miner_type(void);
void chain_all_exit(void);

#endif

