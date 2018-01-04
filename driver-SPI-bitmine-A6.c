/*
 * cgminer SPI driver for Bitmine.ch A1 devices
 *
 * Copyright 2013, 2014 Zefir Kurtisi <zefir.kurtisi@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>

#include "spi-context.h"
#include "logging.h"
#include "miner.h"
#include "util.h"

#include "A1-board-selector.h"
#include "A1-trimpot-mcp4x.h"

#include "A6_inno.h"
#include "A6_inno_clock.h"
#include "A6_inno_cmd.h"
#include "A6_inno_gpio.h"

#include "A6_inno_fan.h"

struct spi_config cfg[ASIC_CHAIN_NUM];
struct spi_ctx *spi[ASIC_CHAIN_NUM];
struct A1_chain *chain[ASIC_CHAIN_NUM];

#define TEMP_UPDATE_INT_MS	10000
#define VOLTAGE_UPDATE_INT  120
#define WRITE_CONFG_TIME  3
#define CHECK_DISABLE_TIME  30

/*
struct Test_bench Test_bench_Array[6]={
    {1260,   14,    0,  0}, //default
    {1260,   15,    0,  0}, 
    {1296,   14,    0,  0},
    {1296,   15,    0,  0}, 
    {1332,   14,    0,  0},
    {1332,   15,    0,  0}, 
};
*/
//pll/vid/good core/ score
struct Test_bench Test_bench_Array[5]={
    {1332,  0,  0,  0},
    {1332,  0,  0,  0},
    {1332,  0,  0,  0},
    {1332,  0,  0,  0},
    {1332,  0,  0,  0},
};


uint8_t A1Pll1=A5_PLL_CLOCK_800MHz;
uint8_t A1Pll2=A5_PLL_CLOCK_800MHz;
uint8_t A1Pll3=A5_PLL_CLOCK_800MHz;
uint8_t A1Pll4=A5_PLL_CLOCK_800MHz;
uint8_t A1Pll5=A5_PLL_CLOCK_800MHz;
uint8_t A1Pll6=A5_PLL_CLOCK_800MHz;
uint8_t A1Pll7=A5_PLL_CLOCK_800MHz;
uint8_t A1Pll8=A5_PLL_CLOCK_800MHz;


/* FAN CTRL */
inno_fan_temp_s g_fan_ctrl;
static uint32_t show_log[ASIC_CHAIN_NUM];
static uint32_t update_cnt[ASIC_CHAIN_NUM];
static uint32_t write_flag[ASIC_CHAIN_NUM];
static uint32_t check_disbale_flag[ASIC_CHAIN_NUM];
static uint32_t first_flag[ASIC_CHAIN_NUM] = {0};
static inno_reg_ctrl_t s_reg_ctrl;

#define STD_V          0.84
int spi_plug_status[ASIC_CHAIN_NUM] = {0};
int fan_level[8]={30, 40, 50, 60, 70, 80, 90, 100};
hardware_version_e g_hwver;
inno_type_e g_type;
int g_reset_delay = 0xffff;
int miner_type;

/********** work queue */
static bool wq_enqueue(struct work_queue *wq, struct work *work)
{
    if (work == NULL)
        return false;
    struct work_ent *we = malloc(sizeof(*we));
    assert(we != NULL);

    we->work = work;
    INIT_LIST_HEAD(&we->head);
    list_add_tail(&we->head, &wq->head);
    wq->num_elems++;
    return true;
}

static struct work *wq_dequeue(struct work_queue *wq)
{
    if (wq == NULL)
        return NULL;
    if (wq->num_elems == 0)
        return NULL;
    struct work_ent *we;
    we = list_entry(wq->head.next, struct work_ent, head);
    struct work *work = we->work;

    list_del(&we->head);
    free(we);
    wq->num_elems--;
    return work;
}

/*
 * for now, we have one global config, defaulting values:
 * - ref_clk 16MHz / sys_clk 800MHz
 * - 2000 kHz SPI clock
 */
struct A1_config_options A1_config_options = {
    .ref_clk_khz = 16000, .sys_clk_khz = 800000, .spi_clk_khz = 2000,
};

/* override values with --bitmine-a1-options ref:sys:spi: - use 0 for default */
static struct A1_config_options *parsed_config_options;

/********** driver interface */
void exit_A1_chain(struct A1_chain *a1)
{
    if (a1 == NULL)
        return;
    free(a1->chips);
    asic_gpio_write(a1->spi_ctx->led, 1);
    asic_gpio_write(a1->spi_ctx->power_en, 0);
    a1->chips = NULL;
    a1->spi_ctx = NULL;
    free(a1);
}

struct A1_chain *pre_init_A1_chain(struct spi_ctx *ctx, int chain_id)
{
    int i;
    struct A1_chain *a1 = malloc(sizeof(struct A1_chain));
    if (a1 == NULL){
        goto failure;
    }

    applog(LOG_INFO, "pre %d: A1 init chain", chain_id);
    
    memset(a1, 0, sizeof(struct A1_chain));
    a1->spi_ctx = ctx;
    a1->chain_id = chain_id;

    inno_cmd_reset(a1, ADDR_BROADCAST);
	sleep(1);
	
    a1->num_chips =  chain_detect(a1);
    usleep(10000);
    
    if (a1->num_chips <= 0)
        goto failure;

    applog(LOG_WARNING, "spidev%d.%d: %d: Found %d A1 chips",
           a1->spi_ctx->config.bus, a1->spi_ctx->config.cs_line,
           a1->chain_id, a1->num_chips);

    /* override max number of active chips if requested */
    a1->num_active_chips = a1->num_chips;
    if (A1_config_options.override_chip_num > 0 &&
        a1->num_chips > A1_config_options.override_chip_num) 
    {
        a1->num_active_chips = A1_config_options.override_chip_num;
        applog(LOG_WARNING, "%d: limiting chain to %d chips",
               a1->chain_id, a1->num_active_chips);
    }

    a1->chips = calloc(a1->num_active_chips, sizeof(struct A1_chip));
    if (a1->chips == NULL){
        goto failure;
    }

    return a1;

failure:
    exit_A1_chain(a1);
    return NULL;
}

static bool init_A1_chain(struct A1_chain *a1)
{
    int i;
    uint8_t src_reg[128];
    uint8_t reg[128];
	int chain_id = a1->chain_id;

    applog(LOG_INFO, "%d: A1 init chain", chain_id);

	inno_cmd_resetbist(a1, ADDR_BROADCAST);
	sleep(1);

	//bist mask
	inno_cmd_read_reg(a1, 0x01, reg);
    memset(src_reg, 0, sizeof(src_reg));
    memcpy(src_reg,reg,REG_LENGTH-2);
	src_reg[7] = src_reg[7] | 0x10;
    inno_cmd_write_reg(a1,ADDR_BROADCAST,src_reg);
    usleep(200);
	
    a1->num_chips =  chain_detect(a1);
    usleep(10000);
    
    if (a1->num_chips <= 0)
        goto failure;

    applog(LOG_WARNING, "spidev%d.%d: %d: Found %d A1 chips",
           a1->spi_ctx->config.bus, a1->spi_ctx->config.cs_line,
           a1->chain_id, a1->num_chips);
	
    /* override max number of active chips if requested */
    a1->num_active_chips = a1->num_chips;
    if (A1_config_options.override_chip_num > 0 &&
        a1->num_chips > A1_config_options.override_chip_num) 
    {
        a1->num_active_chips = A1_config_options.override_chip_num;
        applog(LOG_WARNING, "%d: limiting chain to %d chips",
               a1->chain_id, a1->num_active_chips);
    }

    a1->chips = calloc(a1->num_active_chips, sizeof(struct A1_chip));
    if (a1->chips == NULL){
        goto failure;
    }

    if (!inno_cmd_bist_fix(a1, ADDR_BROADCAST))
        goto failure;

    usleep(200);
	//configure for vsensor
	inno_configure_tvsensor(a1,ADDR_BROADCAST,0);

	for (i = 0; i < a1->num_active_chips; i++)
    {
		inno_check_voltage(a1, i+1, &s_reg_ctrl);
    }
	
	//configure for tsensor
	inno_configure_tvsensor(a1,ADDR_BROADCAST,1);

    for (i = 0; i < a1->num_active_chips; i++)
    {
        check_chip(a1, i);
        inno_fan_temp_add(&g_fan_ctrl, chain_id, i+1, a1->chips[i].temp);
    }
    chain_temp_update(&g_fan_ctrl, chain_id, g_type);
       
    applog(LOG_WARNING, "[chain_ID:%d]: Found %d Chips With Total %d Active Cores",a1->chain_id, a1->num_active_chips, a1->num_cores);
    applog(LOG_WARNING, "[chain_ID]: Temp:%d",g_fan_ctrl.temp_highest[chain_id]);
   
#if 1
       if(g_fan_ctrl.temp_highest[chain_id] < DANGEROUS_TMP){
           //asic_gpio_write(spi[a1->chain_id]->power_en, 0);
           //loop_blink_led(spi[a1->chain_id]->led, 10);
           goto failure;
           //early_quit(1,"Notice Chain %d temp:%d Maybe Has Some Problem in Temperate\n",a1->chain_id,s_fan_ctrl.temp_highest[chain_id]);
       }
#endif
   
   
    mutex_init(&a1->lock);
    INIT_LIST_HEAD(&a1->active_wq.head);

	return true;

failure:
    exit_A1_chain(a1);
    return false;
}

//add  0928
int  cfg_tsadc_divider(struct A1_chain *a1,uint32_t pll_clk)
{
    uint32_t tsadc_divider_tmp;
    uint8_t  tsadc_divider;
    
    //cmd0d(0x0d00, 0x0250, 0xa006 | (BYPASS_AUXPLL<<6), 0x2800 | tsadc_divider, 0x0300, 0x0000, 0x0000, 0x0000)
    uint8_t    buffer[64] = {0x02,0x50,0xa0,0x06,0x28,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    #ifdef MPW
        tsadc_divider_tmp = (pll_clk/2)*1000/256/650;
    #else
        tsadc_divider_tmp = (pll_clk/2)*1000/16/650;
    #endif
    tsadc_divider = tsadc_divider_tmp & 0xff;

    buffer[5] = 0x00 | tsadc_divider;

    if(!inno_cmd_write_sec_reg(a1,ADDR_BROADCAST,buffer)){
        applog(LOG_WARNING, "#####Write t/v sensor Value Failed!");
    }
	else
	{
	    applog(LOG_WARNING, "#####Write t/v sensor Value Success!");
	}
}

static void prepll_chip_temp(struct A1_chain *a1, int cid)
{
    int i,temp;
    uint8_t reg[64];

	applog(LOG_ERR,"start to read temp");

    memset(reg,0,sizeof(reg));
    for (i = a1->num_active_chips; i > 0; i--)
    {
        if (!inno_cmd_read_reg(a1, i, reg))
        {
            applog(LOG_ERR, "%d: Failed to read temperature sensor register for chip %d ", a1->chain_id, i);
            continue;
        }

        temp = 0x000003ff & ((reg[7] << 8) | reg[8]);
		//applog(LOG_ERR,"cid %d,chip %d,temp %d",cid, i, temp);
        inno_fan_temp_add(&g_fan_ctrl, cid, i, temp);
    }

    chain_temp_update(&g_fan_ctrl,cid,g_type);

}

#if 1
static void inc_pll(void)
{
    int i = 0,j = 0;
	int rep_cnt = 0;
    int ret[ASIC_CHAIN_NUM];
	int disable_chain[ASIC_CHAIN_NUM];
    static uint32_t last_pll;
  
    for(i = PLL_Clk_12Mhz[0].speedMHz; i<(opt_A1Pll1+200); i=i+200)
    {
    	if(i >= opt_A1Pll1) i = opt_A1Pll1;
		
    	applog(LOG_INFO,"start to configure all chain from %d to %d", last_pll, i);
        for(j=0; j<ASIC_CHAIN_NUM; j++)
	    {
	        if(spi[j]->disable == 1 || chain[j] == NULL)
			{
	            applog(LOG_INFO, "chain %d has been disable", j);
	            continue;
	        }

			prepll_chip_temp(chain[j], j);
			usleep(200000);
			rep_cnt = 0;
			applog(LOG_INFO,"start to configure chain %d", j);
			while(!prechain_detect_yex(chain[j], A1_ConfigA1PLLClock(i),A1_ConfigA1PLLClock(last_pll)))
			{
				applog(LOG_INFO, "Fail in prechian_detect_yex");
				sleep(10);
				if((g_fan_ctrl.temp_highest[j] > DANGEROUS_TMP) && (g_fan_ctrl.temp_lowest[j] < START_FAN_TH))
				{
					rep_cnt++;
				}
				else
				{
					spi[j]->disable = 1;
				    goto failure;
				}
				
				if(rep_cnt > 5)
				{
					spi[j]->disable = 1;
					goto failure;
				}
			}	
	    }
		
failure:
	    inno_fan_speed_update(&g_fan_ctrl);
        last_pll = i;
    }
}
#endif

int chain_flag[ASIC_CHAIN_NUM] = {0};
static bool detect_A1_chain(void)
{
	int i,cnt = 0;
	int type_score = 0;

	applog(LOG_WARNING, "A1: checking A1 chain");

	for(i = 0; i < ASIC_CHAIN_NUM; i++)
	{
		cfg[i].bus     = i;
		cfg[i].cs_line = 0;
		cfg[i].mode    = SPI_MODE_1;
		cfg[i].speed   = DEFAULT_SPI_SPEED;
		cfg[i].bits    = DEFAULT_SPI_BITS_PER_WORD;
		cfg[i].delay   = DEFAULT_SPI_DELAY_USECS;

		spi[i] = spi_init(&cfg[i]);
		if(spi[i] == NULL)
		{
			applog(LOG_ERR, "spi init fail");
			return false;
		}

		spi[i]->power_en = SPI_PIN_POWER_EN[i];		
		spi[i]->start_en = SPI_PIN_START_EN[i];		
		spi[i]->reset = SPI_PIN_RESET[i];
		spi[i]->plug  = SPI_PIN_PLUG[i];
		spi[i]->led   = SPI_PIN_LED[i];
		spi[i]->disable = false;
		

		asic_gpio_init(spi[i]->power_en, 0);
		asic_gpio_init(spi[i]->start_en, 0);
		asic_gpio_init(spi[i]->reset, 0);
		asic_gpio_init(spi[i]->plug, 1);
		asic_gpio_init(spi[i]->led, 0);

	    sleep(1);
		asic_gpio_write(spi[i]->power_en, 0);
		sleep(1);
		asic_gpio_write(spi[i]->reset, 0);
		asic_gpio_write(spi[i]->start_en, 0);

		show_log[i] = 0;
		update_cnt[i] = 0;
		write_flag[i] = 0;
		check_disbale_flag[i] = 0;
	}

    sleep(5);

	for(i = 0; i < ASIC_CHAIN_NUM; i++)
	{
		g_fan_ctrl.valid_chain[i] = asic_gpio_read(spi[i]->plug);
		applog(LOG_ERR, "Plug Status[%d] = %d",i,g_fan_ctrl.valid_chain[i]);

		if(asic_gpio_read(spi[i]->plug) != 0)
		{
			applog(LOG_ERR, "chain:%d the plat is not inserted", i);
			spi[i]->disable = true;
			continue;
		}

		asic_gpio_write(spi[i]->power_en, 1);
		sleep(5);
		asic_gpio_write(spi[i]->reset, 1);
		sleep(1);
		asic_gpio_write(spi[i]->start_en, 1);	
	}

	//init spi hardware
    asic_spi_init();
    set_spi_speed(1500000);

	for(i = 0; i < ASIC_CHAIN_NUM; i++)
	{
		int iVid;
		
		if(spi[i]->disable == true)
			continue;
		
		switch(i)
		{
			 case 0: iVid = opt_voltage1; break;
			 case 1: iVid = opt_voltage2; break;
			 case 2: iVid = opt_voltage3; break;
			 case 3: iVid = opt_voltage4; break;
			 case 4: iVid = opt_voltage5; break;
			 case 5: iVid = opt_voltage6; break;
			 case 6: iVid = opt_voltage7; break;
			 case 7: iVid = opt_voltage8; break;
			 default:break;
		}
		set_vid_value(iVid,i);
		
		chain[i] = pre_init_A1_chain(spi[i], i);
		if (chain[i] == NULL){
			applog(LOG_ERR, "init %d A1 chain fail", i);
			spi[i]->disable = true;
			continue;
		}
        else
        {
			chain_flag[i] = 1;
			applog(LOG_WARNING, "Detected the %d A1 chain with %d chips", i, chain[i]->num_active_chips);
		}
	}

	//for pre-heat
    inc_pll();

	for(i = 0; i < ASIC_CHAIN_NUM; i++)
	{
		if(spi[i]->disable == true || chain[i] == NULL)
			continue;
		
		if(!init_A1_chain(chain[i]))
		{
			applog(LOG_ERR, "init %d A1 chain fail", i);
			spi[i]->disable = true;
			continue;
		}
        else
        {
			cnt++;
			chain_flag[i] = 1;
			applog(LOG_WARNING, "Detected the %d A1 chain with %d chips", i, chain[i]->num_active_chips);
		}

		struct cgpu_info *cgpu = malloc(sizeof(*cgpu));
		assert(cgpu != NULL);
	
		memset(cgpu, 0, sizeof(*cgpu));
		cgpu->drv = &bitmineA1_drv;
		cgpu->name = "BitmineA1.SingleChain";
		cgpu->threads = 1;
		cgpu->chainNum = i;

		cgpu->device_data = chain[i];

		chain[i]->cgpu = cgpu;
		add_cgpu(cgpu);

		asic_gpio_write(chain[i]->spi_ctx->led, 0);

        if(chain[i]->num_cores > BIN1_CORE_THR)
        {
            type_score++;
        }

		applog(LOG_WARNING, "Detected the %d A1 chain with %d chips / %d cores",
		       i, chain[i]->num_active_chips, chain[i]->num_cores);
	}

    if(type_score == 0) miner_type = TYPE_A4R;
    else miner_type = TYPE_A4;

	set_spi_speed(3250000);
	inno_fan_speed_update(&g_fan_ctrl);

    return (cnt == 0) ? false : true;
}


/* Probe SPI channel and register chip chain */
void A1_detect(bool hotplug)
{
    /* no hotplug support for SPI */
    if (hotplug)
        return;
        
    struct timeval test_tv;
    int j = 0;
    /* parse bimine-a1-options */
    if (opt_bitmine_a1_options != NULL && parsed_config_options == NULL) {
        int ref_clk = 0;
        int sys_clk = 0;
        int spi_clk = 0;
        int override_chip_num = 0;
        int wiper = 0;

        sscanf(opt_bitmine_a1_options, "%d:%d:%d:%d:%d",
               &ref_clk, &sys_clk, &spi_clk,  &override_chip_num,
               &wiper);
        if (ref_clk != 0)
            A1_config_options.ref_clk_khz = ref_clk;
        if (sys_clk != 0) {
            if (sys_clk < 100000)
                quit(1, "system clock must be above 100MHz");
            A1_config_options.sys_clk_khz = sys_clk;
        }
        if (spi_clk != 0)
            A1_config_options.spi_clk_khz = spi_clk;
        if (override_chip_num != 0)
            A1_config_options.override_chip_num = override_chip_num;
        if (wiper != 0)
            A1_config_options.wiper = wiper;

        /* config options are global, scan them once */
        parsed_config_options = &A1_config_options;
    }
    applog(LOG_DEBUG, "A1 detect");
  //  memset(&s_reg_ctrl,0,sizeof(s_reg_ctrl));

    g_hwver = inno_get_hwver();
    g_type = inno_get_miner_type();
    
    memset(&s_reg_ctrl,0,sizeof(s_reg_ctrl));
    memset(&g_fan_ctrl,0,sizeof(g_fan_ctrl));
    
    inno_fan_temp_init(&g_fan_ctrl, fan_level);

     // update time
    for(j = 0; j < 100; j++)
    {
         cgtime(&test_tv);
         if(test_tv.tv_sec > 1000000000)
         {
             break;
         }
    
         usleep(500000);
    }
      
    A1Pll1 = A1_ConfigA1PLLClock(opt_A1Pll1);
    A1Pll2 = A1_ConfigA1PLLClock(opt_A1Pll2);
    A1Pll3 = A1_ConfigA1PLLClock(opt_A1Pll3);
    A1Pll4 = A1_ConfigA1PLLClock(opt_A1Pll4);
    A1Pll5 = A1_ConfigA1PLLClock(opt_A1Pll5);
    A1Pll6 = A1_ConfigA1PLLClock(opt_A1Pll6);
    A1Pll7 = A1_ConfigA1PLLClock(opt_A1Pll7);
    A1Pll8 = A1_ConfigA1PLLClock(opt_A1Pll8);

#if 0
    /* detect and register supported products */
    if (detect_coincraft_desk())
        return;
    if (detect_coincraft_rig_v3())
        return;
#endif

    if(detect_A1_chain())
    {
        return;
    }

    applog(LOG_WARNING, "A1 dectect finish");

    int i = 0;
    /* release SPI context if no A1 products found */
    for(i = 0; i < ASIC_CHAIN_NUM; i++)
    {
        spi_exit(spi[i]);
    }   
}

char szShowLog[ASIC_CHAIN_NUM][ASIC_CHIP_NUM][256] = {{0}};
#define  LOG_FILE_PREFIX "/tmp/log/analys"

uint8_t cLevelError1[3] = "!";
uint8_t cLevelError2[3] = "#";
uint8_t cLevelError3[3] = "$";
uint8_t cLevelError4[3] = "%";
uint8_t cLevelError5[3] = "*";
uint8_t cLevelNormal[3] = "+";

void Inno_Log_Save(struct A1_chip *chip,int nChip,int nChain)
{
    uint8_t szInNormal[8] = {0};
    memset(szInNormal,0, sizeof(szInNormal));
    if(chip->hw_errors > 0){
        strcat(szInNormal,cLevelError1);
    }
    if(chip->stales > 0){
        strcat(szInNormal,cLevelError2);
    }
    if((chip->temp > 564) || (chip->temp < 445)){
        strcat(szInNormal,cLevelError3);
    }
    if(chip->num_cores < 9){
        strcat(szInNormal,cLevelError4);
    }
    if((chip->nVol > 440) || (chip->nVol < 360)){
        strcat(szInNormal,cLevelError5);
    }

    if((chip->hw_errors == 0) && (chip->stales == 0) && ((chip->temp < 564) && (chip->temp > 445)) &&((chip->nVol < 440) && (chip->nVol > 360)) && (chip->num_cores == 32)){
        strcat(szInNormal,cLevelNormal);
    }
    
    sprintf(szShowLog[nChain][nChip], "\n%-8s|%32d|%8d|%8d|%8d|%8d|%8d|%8d|%8d",szInNormal,chip->nonces_found,
        chip->hw_errors, chip->stales,chip->temp,chip->nVol,chip->num_cores,nChip,nChain);
}

void inno_log_print(int cid, void* log, int len)
{
    FILE* fd;
    char fileName[128] = {0};
    
    sprintf(fileName, "%s%d.log", LOG_FILE_PREFIX, cid);
    fd = fopen(fileName, "w+"); 
    if(fd == NULL){             
        applog(LOG_ERR, "Open log File%d Failed!", cid);
        return;
    }

    fwrite(log, len, 1, fd);
    fflush(fd);
    fclose(fd);
}

static int64_t  A1_scanwork(struct thr_info *thr)
{
    int i;
    int32_t A1Pll = 1000;
    struct cgpu_info *cgpu = thr->cgpu;
    struct A1_chain *a1 = cgpu->device_data;
    int32_t nonce_ranges_processed = 0;

    if (a1->num_cores == 0) {
        cgpu->deven = DEV_DISABLED;
        return 0;
    }

    uint32_t nonce;
    uint8_t chip_id;
    uint8_t job_id;
    bool work_updated = false;
    uint8_t reg[REG_LENGTH];

    mutex_lock(&a1->lock);
    int cid = a1->chain_id;

    if (first_flag[cid] != 1)
    {
        applog(LOG_ERR, "%d: A1_scanwork first in set all parameter!", a1->chain_id);
        first_flag[cid]++;
        for (i = a1->num_active_chips; i > 0; i--) 
        {       
            if (!inno_cmd_read_reg(a1, i, reg)) 
            {
                applog(LOG_ERR, "%d: Failed to read temperature sensor register for chip %d ", a1->chain_id, i);
                continue;
            }
            /* update temp database */
            uint32_t temp = 0;

            temp = 0x000003ff & ((reg[7] << 8) | reg[8]);
            inno_fan_temp_add(&g_fan_ctrl, cid, i, temp);
        }    

        //inno_fan_temp_update(&g_fan_ctrl, cid, g_type,fan_level);
        chain_temp_update(&g_fan_ctrl, cid, g_type);
        cgpu->temp = g_fan_ctrl.temp2float[cid][1];
        cgpu->temp_max = g_fan_ctrl.temp2float[cid][0];
        cgpu->temp_min = g_fan_ctrl.temp2float[cid][2];
        cgpu->fan_duty = g_fan_ctrl.speed;
                
        cgpu->chip_num = a1->num_active_chips;
        cgpu->core_num = a1->num_cores; 
    }

    if (a1->last_temp_time + TEMP_UPDATE_INT_MS < get_current_ms())
    {
        update_cnt[cid]++;
        show_log[cid]++;
        write_flag[cid]++;
        check_disbale_flag[cid]++;

        if (write_flag[cid] > WRITE_CONFG_TIME)
        {
            inno_log_print(cid, szShowLog[cid], sizeof(szShowLog[0]));          
            write_flag[cid] = 0;
        }
        
        for (i = a1->num_active_chips; i > 0; i--) 
        {       
            uint8_t c=i;
            
            if(is_chip_disabled(a1,c))
                continue;
            if (!inno_cmd_read_reg(a1, c, reg)) 
            {
                disable_chip(a1,c);
                applog(LOG_ERR, "%d: Failed to read temperature sensor register for chip %d ", a1->chain_id, i);
                continue;
            }
            /* update temp database */
            uint32_t temp = 0;

            temp = 0x000003ff & ((reg[7] << 8) | reg[8]);
            inno_fan_temp_add(&g_fan_ctrl, cid, i, temp);
			    
		}

		//inno_fan_temp_update(&g_fan_ctrl, cid, g_type,fan_level);
		chain_temp_update(&g_fan_ctrl, cid, g_type);
		cgpu->temp = g_fan_ctrl.temp2float[cid][1];
		cgpu->temp_max = g_fan_ctrl.temp2float[cid][0];
		cgpu->temp_min = g_fan_ctrl.temp2float[cid][2];
		cgpu->fan_duty = g_fan_ctrl.speed;
		
		cgpu->pre_heat = a1->pre_heat;
		//printf("g_fan_ctrl: cid %d,chip %d, chip %d,hi %d\n",g_fan_ctrl.pre_warn[0],g_fan_ctrl.pre_warn[1],g_fan_ctrl.pre_warn[2],g_fan_ctrl.pre_warn[3]);
		memcpy(cgpu->temp_prewarn,g_fan_ctrl.pre_warn, 4*sizeof(int));
		//printf("cgpu: cid %d,chip %d, chip %d,hi %d\n",cgpu->temp_prewarn[0],cgpu->temp_prewarn[1],cgpu->temp_prewarn[2],cgpu->temp_prewarn[3]);

				
		cgpu->chip_num = a1->num_active_chips;
		cgpu->core_num = a1->num_cores; 
		//a1->temp = board_selector->get_temp(0);
		a1->last_temp_time = get_current_ms();
		applog(LOG_ERR, "%s n cid %d:arv:%5.2f, lest:%5.2f, hest:%d witt", __func__,cid, cgpu->temp, cgpu->temp_min, g_fan_ctrl.temp_highest[a1->chain_id]);
		if(g_fan_ctrl.temp_highest[a1->chain_id] < DANGEROUS_TMP)
		{
			applog(LOG_ERR, "disable chain %d", a1->chain_id);
	   		asic_gpio_write(spi[a1->chain_id]->power_en, 0);
			loop_blink_led(spi[a1->chain_id]->led, 10);
	   		//early_quit(1,"Notice chain %d maybe has some promble in temperate\n",a1->chain_id);
		}
	}		

	/* poll queued results */
	while (true)
	{
		if (!get_nonce(a1, (uint8_t*)&nonce, &chip_id, &job_id))
			break;

		//nonce = bswap_32(nonce);   //modify for A6

		work_updated = true;
		if (chip_id < 1 || chip_id > a1->num_active_chips) 
		{
			applog(LOG_WARNING, "%d: wrong chip_id %d", cid, chip_id);
			continue;
		}
		if (job_id < 1 && job_id > 4) 
		{
			applog(LOG_WARNING, "%d: chip %d: result has wrong ""job_id %d", cid, chip_id, job_id);
			flush_spi(a1);
			continue;
		}

		struct A1_chip *chip = &a1->chips[chip_id - 1];
		struct work *work = chip->work[job_id - 1];
		if (work == NULL) 
		{
			/* already been flushed => stale */
			applog(LOG_WARNING, "%d: chip %d: stale nonce 0x%08x", cid, chip_id, nonce);
			chip->stales++;
			continue;
		}
		if (!submit_nonce(thr, work, nonce)) 
		{
			applog(LOG_WARNING, "%d: chip %d: invalid nonce 0x%08x", cid, chip_id, nonce);
			chip->hw_errors++;
			/* add a penalty of a full nonce range on HW errors */
			nonce_ranges_processed--;
			continue;
		}
		applog(LOG_INFO, "YEAH: %d: chip %d / job_id %d: nonce 0x%08x", cid, chip_id, job_id, nonce);
		chip->nonces_found++;
	}

	/* check for completed works */
	if(a1->work_start_delay > 0)
	{
		applog(LOG_INFO, "wait for pll stable");
		a1->work_start_delay--;
	}
	else
	{
		if (inno_cmd_read_reg(a1, 25, reg)) 
		{
			uint8_t qstate = reg[9] & 0x01;

			if (qstate != 0x01)
			{
				work_updated = true;
				for (i = a1->num_active_chips; i > 0; i--) 
				{
					uint8_t c=i;
					struct A1_chip *chip = &a1->chips[i - 1];
					struct work *work = wq_dequeue(&a1->active_wq);
					assert(work != NULL);

					if (set_work(a1, c, work, 0))
					{
						nonce_ranges_processed++;
						chip->nonce_ranges_done++;
					}

					if(show_log[cid] > 1)					
					{												
						//applog(LOG_INFO, "%d: chip:%d ,core:%d ,job done: %d/%d/%d/%d/%d/%5.2f",
						//	   cid, c, chip->num_cores,chip->nonce_ranges_done, chip->nonces_found,
						//	   chip->hw_errors, chip->stales,chip->temp, inno_fan_temp_to_float(&g_fan_ctrl,chip->temp));
						Inno_Log_Save(chip,c-1,cid);
						if(i==1) show_log[cid] = 0;

					}
				}
			}
		}
	}

	if(check_disbale_flag[cid] > CHECK_DISABLE_TIME)
	{
		applog(LOG_INFO, "start to check disable chips");
		switch(cid){
			case 0:check_disabled_chips(a1, A1Pll1);;break;
			case 1:check_disabled_chips(a1, A1Pll2);;break;
			case 2:check_disabled_chips(a1, A1Pll3);;break;
			case 3:check_disabled_chips(a1, A1Pll4);;break;
			case 4:check_disabled_chips(a1, A1Pll5);;break;
			case 5:check_disabled_chips(a1, A1Pll6);;break;
			case 6:check_disabled_chips(a1, A1Pll7);;break;
			case 7:check_disabled_chips(a1, A1Pll8);;break;
			default:;
		}
		check_disbale_flag[cid] = 0;
	}

	mutex_unlock(&a1->lock);

	if (nonce_ranges_processed < 0)
	{
		nonce_ranges_processed = 0;
	}

	/* in case of no progress, prevent busy looping */
	if (!work_updated)
		cgsleep_ms(15);

	cgtime(&a1->tvScryptCurr);
	timersub(&a1->tvScryptCurr, &a1->tvScryptLast, &a1->tvScryptDiff);
	cgtime(&a1->tvScryptLast);


	switch(cgpu->device_id){
		case 0:A1Pll = PLL_Clk_12Mhz[A1Pll1].speedMHz;break;
		case 1:A1Pll = PLL_Clk_12Mhz[A1Pll2].speedMHz;break;
		case 2:A1Pll = PLL_Clk_12Mhz[A1Pll3].speedMHz;break;
		case 3:A1Pll = PLL_Clk_12Mhz[A1Pll4].speedMHz;break;
		case 4:A1Pll = PLL_Clk_12Mhz[A1Pll5].speedMHz;break;
		case 5:A1Pll = PLL_Clk_12Mhz[A1Pll6].speedMHz;break;
		case 6:A1Pll = PLL_Clk_12Mhz[A1Pll7].speedMHz;break;
		case 7:A1Pll = PLL_Clk_12Mhz[A1Pll8].speedMHz;break;
		default:;	
	}

    if(miner_type == TYPE_A4)
        return (int64_t)(2214663.87 * opt_A1Pll1/ 1000 * (621/9.0) * (a1->tvScryptDiff.tv_usec / 1000000.0));
	else
        return (int64_t)(2214663.87 * opt_A1Pll1 / 1000 * (576/9.0) * (a1->tvScryptDiff.tv_usec / 1000000.0));

}


/* queue two work items per chip in chain */
static bool A1_queue_full(struct cgpu_info *cgpu)
{
    struct A1_chain *a1 = cgpu->device_data;
    int queue_full = false;

    mutex_lock(&a1->lock);
    //applog(LOG_DEBUG, "%d, A1 running queue_full: %d/%d",
    //       a1->chain_id, a1->active_wq.num_elems, a1->num_active_chips);

    if (a1->active_wq.num_elems >= a1->num_active_chips * 2)
        queue_full = true;
    else
        wq_enqueue(&a1->active_wq, get_queued(cgpu));

    mutex_unlock(&a1->lock);

    return queue_full;
}

static void A1_flush_work(struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
	int i;

	mutex_lock(&a1->lock);

	for (i = 0; i < a1->num_active_chips; i++) 
	{
		int j;
		struct A1_chip *chip = &a1->chips[i];
		for (j = 0; j < 2; j++) 
		{
			struct work *work = chip->work[j];
			if (work == NULL)
				continue;

			work_completed(cgpu, work);
			chip->work[j] = NULL;
		}
		
		chip->last_queued_id = 0;

		//applog(LOG_INFO, "chip :%d flushing queued work success", i);
	}

	if(!inno_cmd_resetjob(a1, ADDR_BROADCAST))
	{
		applog(LOG_WARNING, "chip clear work false");
	}
		
	/* flush queued work */
	//applog(LOG_INFO, "%d: flushing queued work...", cid);
	while (a1->active_wq.num_elems > 0) 
	{
		struct work *work = wq_dequeue(&a1->active_wq);
		assert(work != NULL);
		work_completed(cgpu, work);
	}
	mutex_unlock(&a1->lock);

}

static void A1_get_statline_before(char *buf, size_t len, struct cgpu_info *cgpu)
{
    struct A1_chain *a1 = cgpu->device_data;
    char temp[10];
    if (a1->temp != 0)
        snprintf(temp, 9, "%2dC", a1->temp);
    tailsprintf(buf, len, " %2d:%2d/%3d %s",
            a1->chain_id, a1->num_active_chips, a1->num_cores,
            a1->temp == 0 ? "   " : temp);
}

struct device_drv bitmineA1_drv = {
    .drv_id = DRIVER_bitmineA1,
    .dname = "BitmineA1",
    .name = "BA1",
    .drv_detect = A1_detect,

    .hash_work = hash_queued_work,
    .scanwork = A1_scanwork,
    .queue_full = A1_queue_full,
    .flush_work = A1_flush_work,
    .get_statline_before = A1_get_statline_before,
};
