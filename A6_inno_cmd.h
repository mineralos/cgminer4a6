#ifndef _A6_INNO_CMD_
#define _A6_INNO_CMD_

#include <stdint.h>
#include <pthread.h>
#include <sys/time.h>
#include "elist.h"


#if  0   //add by lzl 20180531
#define CMD_BIST_START      0x01
#define CMD_BIST_COLLECT    0x0b
#define CMD_BIST_FIX        0x03
#define CMD_RESET           0x04
#define CMD_RESETBC         0x05
#define CMD_WRITE_JOB       0x07
#define CMD_READ_RESULT     0x08
#define CMD_WRITE_REG       0x09
#define CMD_READ_REG        0x0a
#define CMD_READ_REG_RESP   0x1a
#define CMD_POWER_ON        0x02
#define CMD_POWER_OFF       0x06
#define CMD_POWER_RESET     0x0c
#define CMD_READ_SEC_REG    0x0d
#endif


#define ADDR_BROADCAST      0x00

#define LEN_BIST_START      6
#define LEN_BIST_COLLECT    4
#define LEN_BIST_FIX        4
#define LEN_RESET           6
#define LEN_WRITE_JOB       94
#define LEN_READ_RESULT     4
#define LEN_WRITE_REG       18
#define LEN_READ_REG        4
#define NONCE_LEN           (6)


#define SPI_REC_DATA_LOOP   10
#define SPI_REC_DATA_DELAY  1

//#define ASIC_REGISTER_NUM 12
#define ASIC_RESULT_LEN     6
#define READ_RESULT_LEN     (ASIC_RESULT_LEN + 2)

//#define REG_LENGTH      14

#define JOB_LENGTH      92

#define MAX_CHAIN_LENGTH    64
#define MAX_CMD_LENGTH      (JOB_LENGTH + MAX_CHAIN_LENGTH * 2 * 2)

#define WORK_BUSY 0
#define WORK_FREE 1

#define BIN1_CORE_THR  620
#define BIN2_CORE_THR  550

#define TYPE_A4  0
#define TYPE_A4R 1


struct work_ent {
    struct work *work;
    struct list_head head;
};

struct work_queue {
    int num_elems;
    struct list_head head;
};

struct A1_chip {
    uint8_t reg[12];
    int num_cores;
    int last_queued_id;
    struct work *work[4];
    /* stats */
    int hw_errors;
    int stales;
    int nonces_found;
    int nonce_ranges_done;

    /* systime in ms when chip was disabled */
    int cooldown_begin;
    /* number of consecutive failures to access the chip */
    int fail_count;
    int fail_reset;
    /* mark chip disabled, do not try to re-enable it */
    bool disabled;

    /* temp */
    int temp;
    int nVol;

	#if 1   //add by lzl 20180515
	int tunedir; // Tune direction, +/- 1
	int pll;
	int cycles;
	double product; // Hashrate product of cycles / time
	bool pllOptimal; // We've stopped tuning frequency
	#endif
};

struct A1_chain {
    int chain_id;
    struct cgpu_info *cgpu;
    struct mcp4x *trimpot;
    int num_chips;
    int num_cores;
    int num_active_chips;
    int chain_skew;
    uint8_t spi_tx[MAX_CMD_LENGTH];
    uint8_t spi_rx[MAX_CMD_LENGTH];
    struct spi_ctx *spi_ctx;
    struct A1_chip *chips;
    pthread_mutex_t lock;

    struct work_queue active_wq;

	#if 1  //add by lzl 20180515
	bool throttle; /* Needs throttling */
	int cycles; /* Cycles used for iVid tuning */
	int tunedir; // Tune direction, -1..+1
	int pll; /* Current chain speed */
	int base_pll; /* Initial chain speed */

	int vid; /* Current actual iVid */
	double product; // Hashrate product of cycles / time
	bool VidOptimal; // We've stopped tuning voltage
	bool pllOptimal; // We've stopped tuning frequency
	bool voltagebalanced; // We've balanced voltage b/w chips
	#endif

    /* mark chain disabled, do not try to re-enable it */
    bool disabled;
    uint8_t temp;
    int last_temp_time;
    int pre_heat;

    struct timeval tvScryptLast;
    struct timeval tvScryptCurr;
    struct timeval tvScryptDiff;
    int work_start_delay;
    time_t lastshare;
};

struct Test_bench {
    uint32_t uiPll; 
    int uiVol;
    uint32_t uiScore;
    uint32_t uiCoreNum;
};


unsigned short CRC16_2(unsigned char* pchMsg, unsigned short wDataLen);

extern bool inno_cmd_reset(struct A1_chain *pChain, uint8_t chip_id);
extern bool inno_cmd_resetbist(struct A1_chain *pChain, uint8_t chip_id);
extern bool inno_cmd_resetjob(struct A1_chain *pChain, uint8_t chip_id);

extern bool inno_cmd_bist_start(struct A1_chain *pChain, uint8_t chip_id, uint8_t *num);

extern bool inno_cmd_bist_collect(struct A1_chain *pChain, uint8_t chip_id);

extern bool inno_cmd_bist_fix(struct A1_chain *pChain, uint8_t chip_id);

extern bool inno_cmd_write_reg(struct A1_chain *pChain, uint8_t chip_id, uint8_t *reg);

extern bool inno_cmd_read_reg(struct A1_chain *pChain, uint8_t chip_id, uint8_t *reg);

extern bool inno_cmd_read_result(struct A1_chain *pChain, uint8_t chip_id, uint8_t *res);

extern bool inno_cmd_write_job(struct A1_chain *pChain, uint8_t chip_id, uint8_t *job);

extern uint8_t inno_cmd_isBusy(struct A1_chain *pChain, uint8_t chip_id);

extern uint32_t inno_cmd_test_chip(struct A1_chain *pChain);

void A6_flush_spi(struct A1_chain *pChain);
void hexdump_error(char *prefix, uint8_t *buff, int len);
void hexdump(char *prefix, uint8_t *buff, int len);




#endif
