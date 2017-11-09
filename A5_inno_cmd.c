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

#include "A5_inno.h"
#include "A5_inno_cmd.h"
#include "A5_inno_clock.h"



const unsigned short wCRCTalbeAbs[] =
{
	0x0000, 0xCC01, 0xD801, 0x1400, 
	0xF001, 0x3C00, 0x2800, 0xE401, 
	0xA001, 0x6C00, 0x7800, 0xB401, 
	0x5000, 0x9C01, 0x8801, 0x4400,
};


unsigned short CRC16_2(unsigned char* pchMsg, unsigned short wDataLen)
{
	volatile unsigned short wCRC = 0xFFFF;
	unsigned short i;
	unsigned char chChar;

	for (i = 0; i < wDataLen; i++)
	{
		chChar = *pchMsg++;
		wCRC = wCRCTalbeAbs[(chChar ^ wCRC) & 15] ^ (wCRC >> 4);
		wCRC = wCRCTalbeAbs[((chChar >> 4) ^ wCRC) & 15] ^ (wCRC >> 4);
	}

	return wCRC;
}



static void applog_hexdump(char *prefix, uint8_t *buff, int len, int level)
{
	static char line[512];
	char *pos = line;
	int i;
	if (len < 1)
	{
		return;
	}

	pos += sprintf(pos, "%s: %d bytes:", prefix, len);
	for (i = 0; i < len; i++) 
	{
		if (i > 0 && (i % 32) == 0) 
		{
			applog(LOG_INFO, "%s", line);
			pos = line;
			pos += sprintf(pos, "\t");
		}
		pos += sprintf(pos, "%.2X ", buff[i]);
	}
	applog(level, "%s", line);
}

void hexdump(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_WARNING);
}

void hexdump_error(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_ERR);
}

void flush_spi(struct A1_chain *pChain)
{
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	struct spi_ctx *ctx = pChain->spi_ctx;

	memset(spi_tx, 0, sizeof(spi_tx));
	memset(spi_rx, 0, sizeof(spi_rx));

	spi_write_data(ctx, spi_tx, 64);
}

bool spi_send_zero(struct spi_ctx *ctx, uint8_t *txbuf, int len)
{
	bool ret;
	int index = 0;
	uint8_t spi_tx[256];
	uint8_t spi_rx[256];

	memset(spi_tx, 0, sizeof(spi_tx));
	memcpy(spi_tx, txbuf, len);
	
	do{
		memset(spi_rx, 0, sizeof(spi_rx));
		ret = spi_write_data(ctx, spi_tx + index, 2);
		if(!ret)
		{
			return false;
		}		
		
		index = index + 2;
	}while(index < len);
	
	return true;
}


bool spi_send_data(struct spi_ctx *ctx, uint8_t *txbuf, int len)
{
	bool ret;
	int index = 0;
	uint8_t spi_tx[256];
	uint8_t spi_rx[256];

	memset(spi_tx, 0, sizeof(spi_tx));
	memcpy(spi_tx, txbuf, len);
	
	do{
		ret = spi_write_data(ctx, spi_tx + index, 2);
		if(!ret)
		{
			return false;
		}		
		
		index = index + 2;
	}while(index < len);
	
	return true;
}


bool spi_send_command(struct A1_chain *pChain, uint8_t cmd, uint8_t chip_id, uint8_t *buff, int len)
{
	int tx_len;
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	struct spi_ctx *ctx = pChain->spi_ctx;
	
	assert(buff != NULL);

	memset(spi_tx, 0, sizeof(spi_tx));
	memset(spi_rx, 0, sizeof(spi_rx));
	
	spi_tx[0] = cmd;
	spi_tx[1] = chip_id;
	
	if(len > 0)
	{
		memcpy(spi_tx + 2, buff, len);
	}
	
	tx_len = (2 + len + 1) & ~1;
	//hexdump("send: TX", spi_tx, tx_len);

	if(spi_send_data(ctx, spi_tx, tx_len))
	{
		return true;
	}
	else
	{
		applog(LOG_WARNING, "send command fail !");
		return false;
	}

}


bool spi_poll_result(struct A1_chain *pChain, uint8_t cmd, uint8_t chip_id, uint8_t *buff, int len)
{
	int ret1, ret2;
	int tx_len;
	int tmp_len;
	int index,ret;
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	struct spi_ctx *ctx = pChain->spi_ctx;
	
	memset(spi_tx, 0, sizeof(spi_tx));
	memset(spi_rx, 0, sizeof(spi_rx));
	
	tx_len = ASIC_CHIP_NUM*4;

	for(tmp_len = 0; tmp_len < tx_len; tmp_len += 2)
	{
		if(!spi_read_data(ctx, spi_rx, 2))
		//if(!spi_transfer(ctx, spi_tx, spi_rx, 2))
		{
			applog(LOG_WARNING, "poll result: transfer fail !");
			return false;
		}
		//hexdump("poll: RX", spi_rx, 2);
		if(spi_rx[0] == cmd)
		{
			index = 0;	
			do{
				ret = spi_read_data(ctx, spi_rx + 2 + index, 2);
				if(!ret)
				{
					return false;
				}					
				index = index + 2;
			}while(index < len);

			//hexdump("poll: RX", spi_rx + 2, len);
			memcpy(buff, spi_rx, len);
			return true;
		}
	}
	
	return false;
}

bool inno_cmd_reset(struct A1_chain *pChain, uint8_t chip_id)
{
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	
	printf("send command [reset] \n");

	memset(spi_tx, 0, sizeof(spi_tx));
	if(!spi_send_command(pChain, CMD_RESET, chip_id, spi_tx, 2))
	{
		applog(LOG_WARNING, "cmd reset: send fail !");
		return false;
	}

	memset(spi_rx, 0, sizeof(spi_rx));
	if(!spi_poll_result(pChain, CMD_RESET, chip_id, spi_rx, 4))
	{
		applog(LOG_WARNING, "cmd reset: poll fail !");
		return false;
	}

	return true;
}

bool inno_cmd_resetjob(struct A1_chain *pChain, uint8_t chip_id)
{
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	uint8_t i,tx_len;
	uint8_t buffer[64];	

	memset(spi_tx, 0, sizeof(spi_tx));
	memset(spi_rx, 0, sizeof(spi_rx));

	spi_tx[0] = CMD_RESET;
	spi_tx[1] = chip_id;
	spi_tx[2] = 0xed;
	spi_tx[3] = 0xed;

	//tx_len = (2 + len + 1) & ~1;
	//hexdump("send: TX", spi_tx, tx_len);

	if(!spi_write_data(pChain->spi_ctx, spi_tx, 6))
	{
		applog(LOG_WARNING, "send command fail !");
		return false;
	}

	memset(spi_rx, 0, sizeof(spi_rx));
/*
	if(!spi_poll_result(pChain, CMD_RESET, chip_id, spi_rx, 4))
	{
		applog(LOG_WARNING, "cmd reset: poll fail !");
		return false;
	}
*/
	spi_poll_result(pChain, CMD_RESET, chip_id, spi_rx, 4);

	if(inno_cmd_isBusy(pChain, chip_id) != WORK_FREE)
	{	
		return false;
	}

	return true;
}

bool inno_cmd_resetbist(struct A1_chain *pChain, uint8_t chip_id)
{
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	uint8_t i,tx_len;
	uint8_t buffer[64];	

	memset(spi_tx, 0, sizeof(spi_tx));
	memset(spi_rx, 0, sizeof(spi_rx));

	spi_tx[0] = CMD_RESET;
	spi_tx[1] = chip_id;
	spi_tx[2] = 0xfb;
	spi_tx[3] = 0xfb;

	if(!spi_write_data(pChain->spi_ctx, spi_tx, 6))
	{
		applog(LOG_WARNING, "send command fail !");
		return false;
	}

	memset(spi_rx, 0, sizeof(spi_rx));
	
	if(!spi_poll_result(pChain, CMD_RESET, chip_id, spi_rx, 4))
	{
		applog(LOG_WARNING, "cmd reset: poll fail !");
		return false;
	}

	return true;
}


bool inno_cmd_bist_start(struct A1_chain *pChain, uint8_t chip_id, uint8_t *num)
{
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	
	printf("send command [bist_start] \n");

	memset(spi_tx, 0, sizeof(spi_tx));
	if(!spi_send_command(pChain, CMD_BIST_START, chip_id, spi_tx, 2))
	{
		applog(LOG_WARNING, "cmd bist start: send fail !");
		return false;
	}

	memset(spi_rx, 0, sizeof(spi_rx));
	if(!spi_poll_result(pChain, CMD_BIST_START, chip_id, num, 4))
	{
		applog(LOG_WARNING, "cmd bist start: poll fail !");
		return false;
	}

	return true;
}

bool inno_cmd_bist_collect(struct A1_chain *pChain, uint8_t chip_id)
{
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	
	printf("send command [bist_collect] \n");

	memset(spi_tx, 0, sizeof(spi_tx));
	if(!spi_send_command(pChain, CMD_BIST_COLLECT, chip_id, spi_tx, 2))
	{
		return false;
	}

	memset(spi_rx, 0, sizeof(spi_rx));
	if(!spi_poll_result(pChain, CMD_BIST_COLLECT, chip_id, spi_rx, 4))
	{
		return false;
	}

	return true;
}


bool inno_cmd_bist_fix(struct A1_chain *pChain, uint8_t chip_id)
{
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	
	printf("send command [bist_fix] \n");

	memset(spi_tx, 0, sizeof(spi_tx));
	if(!spi_send_command(pChain, CMD_BIST_FIX, chip_id, spi_tx, 2))
	{
		return false;
	}

	memset(spi_rx, 0, sizeof(spi_rx));
	if(!spi_poll_result(pChain, CMD_BIST_FIX, chip_id, spi_rx, 4))
	{
		return false;
	}

	return true;
}

//add  0929
bool inno_cmd_write_sec_reg(struct A1_chain *pChain, uint8_t chip_id, uint8_t *reg)
{
	int tx_len;
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	uint8_t tmp_buf[MAX_CMD_LENGTH];
	uint16_t clc_crc;
	uint8_t j;
		
	applog(LOG_INFO,"send command [write_reg] \n");
	assert(reg != NULL);
	
	memset(spi_tx, 0, sizeof(spi_tx));
		
	spi_tx[0] = CMD_READ_SEC_REG;
	spi_tx[1] = chip_id;
	memcpy(spi_tx+2, reg, REG_LENGTH-2);
	memset(tmp_buf, 0, sizeof(tmp_buf));

	clc_crc = CRC16_2(tmp_buf, REG_LENGTH);
	
	spi_tx[REG_LENGTH+0] = (uint8_t)(clc_crc >> 8);
	spi_tx[REG_LENGTH+1] = (uint8_t)(clc_crc);
	
	//hexdump("write reg", spi_tx, REG_LENGTH+2);
	if(!spi_write_data(pChain->spi_ctx, spi_tx, 16))
	{
		applog(LOG_WARNING, "send command fail !");
		return false;
	}
	//printf("reg:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",spi_tx[0],spi_tx[1],spi_tx[2],spi_tx[3],spi_tx[4],spi_tx[5],spi_tx[6],spi_tx[7],spi_tx[8],spi_tx[9],spi_tx[10],spi_tx[11],spi_tx[12],spi_tx[13],spi_tx[14],spi_tx[15],spi_tx[16],spi_tx[17]);
	memset(spi_rx, 0, sizeof(spi_rx));
	if(!spi_poll_result(pChain, CMD_READ_SEC_REG, chip_id, spi_rx, REG_LENGTH+4))
	{
		applog(LOG_WARNING, "cmd write reg: poll fail !");
		return false;
	}
	
	return true;
}

bool inno_cmd_write_reg(struct A1_chain *pChain, uint8_t chip_id, uint8_t *reg)
{
	int tx_len;
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	uint8_t tmp_buf[MAX_CMD_LENGTH];
	uint16_t clc_crc;
	uint8_t j;
	
	//printf("send command [write_reg] \n");
	assert(reg != NULL);

	memset(spi_tx, 0, sizeof(spi_tx));
	
 	spi_tx[0] = CMD_WRITE_REG;
 	spi_tx[1] = chip_id;
 	memcpy(spi_tx+2, reg, REG_LENGTH-2);

	memset(tmp_buf, 0, sizeof(tmp_buf));
	for(j = 0; j < REG_LENGTH; j = j + 2)
	{
		tmp_buf[j + 0] = spi_tx[j + 1];
		tmp_buf[j + 1] = spi_tx[j + 0]; 	
	}
	clc_crc = CRC16_2(tmp_buf, REG_LENGTH);

	spi_tx[REG_LENGTH+0] = (uint8_t)(clc_crc >> 8);
	spi_tx[REG_LENGTH+1] = (uint8_t)(clc_crc);

	//hexdump("write reg", spi_tx, REG_LENGTH+2);
	if(!spi_write_data(pChain->spi_ctx, spi_tx, 16))
	{
		applog(LOG_WARNING, "send command fail !");
		return false;
	}

	memset(spi_rx, 0, sizeof(spi_rx));
	if(!spi_poll_result(pChain, CMD_WRITE_REG, chip_id, spi_rx, REG_LENGTH))
	{
		applog(LOG_WARNING, "cmd write reg: poll fail !");
		return false;
	}

	return true;
}


bool inno_cmd_read_reg(struct A1_chain *pChain, uint8_t chip_id, uint8_t *reg)
{
	int i,j;
	int tx_len;
	int ret,index; 
	uint16_t clc_crc; 
	uint16_t res_crc;
	uint8_t tmp_buf[64];
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	struct spi_ctx *ctx = pChain->spi_ctx;
	
	//printf("send command [read_reg] \r\n");
	//assert(reg != NULL);

	memset(spi_tx, 0, sizeof(spi_tx));
	spi_tx[0] = CMD_READ_REG;
	spi_tx[1] = chip_id;
	
	if(!spi_write_data(ctx, spi_tx, 2))
	{
		return false;
	}

	tx_len = ASIC_CHIP_NUM*4;
	memset(spi_rx, 0, sizeof(spi_rx));
	for(i = 0; i < tx_len; i = i + 2)
	{
		if(!spi_read_data(ctx, spi_rx, 2))
		{
			applog(LOG_WARNING, "poll result: transfer fail !");
			return false;
		}
		//hexdump("poll: RX", spi_rx, 2);
		if(spi_rx[0] == CMD_READ_REG_RESP)
		{		
			index = 0;	
			do{
				ret = spi_read_data(ctx, spi_rx + 2 + index, 2);
				if(!ret)
				{
					return false;
				}					
				index = index + 2;
			}while(index < REG_LENGTH);

			//hexdump("poll: RX", spi_rx + 2, REG_LENGTH);
			memset(tmp_buf, 0, sizeof(tmp_buf));
			for(j = 0; j < REG_LENGTH + 2; j = j + 2)
			{
				tmp_buf[j + 0] = spi_rx[j + 1];
				tmp_buf[j + 1] = spi_rx[j + 0];
			}
			clc_crc = CRC16_2(tmp_buf, REG_LENGTH);
			//printf("clc_crc:0x%x.", clc_crc);
			res_crc = (spi_rx[REG_LENGTH] << 8) + (spi_rx[REG_LENGTH + 1] << 0);

			//hexdump("result: RX", spi_rx, READ_RESULT_LEN);
			if(clc_crc == res_crc)
			{
				memcpy(reg, spi_rx + 2, REG_LENGTH);
				return true;
			}
			else
			{
				applog(LOG_INFO, "inno_cmd_read_reg crc error clc=0x%4x, res=0x%4x \r\n", clc_crc, res_crc);
				return false;
			}				

			return true;
		}
	}
	
	return false;
}

bool inno_cmd_read_result(struct A1_chain *pChain, uint8_t chip_id, uint8_t *res)
{
	int i,j;
	int tx_len,index,ret;		
	uint16_t clc_crc; 
	uint16_t res_crc;
	uint8_t tmp_buf[64];
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	struct spi_ctx *ctx = pChain->spi_ctx;

	//printf("send command [read_result] \r\n");
	assert(res != NULL);

	memset(spi_tx, 0, sizeof(spi_tx));
	spi_tx[0] = CMD_READ_RESULT;
	spi_tx[1] = chip_id;
	
	if(!spi_write_data(ctx, spi_tx, 2))
	{
		return false;
	}

	tx_len = 4 * ASIC_CHIP_NUM;
	memset(spi_rx, 0, sizeof(spi_rx));
	for(i = 0; i < tx_len; i += 2)
	{
		if(!spi_read_data(ctx, spi_rx, 2))		
		{
			return false;
		}

		if(((spi_rx[0] & 0x0f) == CMD_READ_RESULT) && (spi_rx[1] != 0))
		{
			//applog(LOG_INFO, "GET GOOD RESULT");
			index = 0;	
			do{
				ret = spi_read_data(ctx, spi_rx + 2 + index, 2);
				if(!ret)
				{
					return false;
				}					
				index = index + 2;
			}while(index < ASIC_RESULT_LEN);

			memset(tmp_buf, 0, sizeof(tmp_buf));
			for(j = 0; j < READ_RESULT_LEN; j = j + 2)
			{
				tmp_buf[j + 0] = spi_rx[j + 1];
				tmp_buf[j + 1] = spi_rx[j + 0];
			}
			clc_crc = CRC16_2(tmp_buf, ASIC_RESULT_LEN);
			res_crc = (spi_rx[ASIC_RESULT_LEN] << 8) + (spi_rx[ASIC_RESULT_LEN+1] << 0);

			//hexdump("result: RX", spi_rx, READ_RESULT_LEN);
			if(clc_crc == res_crc)
			{
				memcpy(res, spi_rx, READ_RESULT_LEN);
				return true;
			}
			else
			{
				applog(LOG_INFO, "crc error clc=0x%4x, res=0x%4x \r\n", clc_crc, res_crc);
				return false;
			}				
		}
	}
	return false;

}

uint8_t inno_cmd_isBusy(struct A1_chain *pChain, uint8_t chip_id)
{
	uint8_t buffer[REG_LENGTH];

	  
	if(!inno_cmd_read_reg(pChain, chip_id, buffer))
	{
		applog(LOG_WARNING, "read chip %d busy status error", chip_id);
		return -1;
	}
	//printf("[check busy] \r\n");
	//hexdump("reg:", buffer, REG_LENGTH);

	if((buffer[9] & 0x01) == 1)
	{
		//applog(LOG_WARNING, "chip %d is busy now", chip_id);
		return WORK_BUSY;
	}
	else
	{
		//applog(LOG_WARNING, "chip %d is free now", chip_id);
		return WORK_FREE;
	}

}



bool inno_cmd_write_job(struct A1_chain *pChain, uint8_t chip_id, uint8_t *job)
{
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	struct spi_ctx *ctx = pChain->spi_ctx;
	
	memset(spi_tx, 0, sizeof(spi_tx));
	memcpy(spi_tx, job, JOB_LENGTH);

	if(!spi_write_data(ctx, spi_tx, JOB_LENGTH + 4))
	{
		return false;
	}
	//printf("[write job] \r\n");
	//hexdump("job:", spi_tx, JOB_LENGTH);

	//usleep(10);

	//if(inno_cmd_isBusy(pChain, chip_id) != WORK_BUSY)
	//{
	//	return false;
	//}

	return true;

}

uint32_t inno_cmd_test_chip(struct A1_chain *pChain)
{
	int i, j, k;
	struct work work1;
	struct work work2;
	uint32_t nonce;
	uint8_t chip_id;
	uint8_t job_id;
	uint16_t micro_job_id;
	uint8_t c;
	int bad_chip_num = 0;
	uint32_t uiScore = 0;
	uint32_t rightNonce = 0xd5067d1f;
	uint32_t rightNonce2 = 0xce1f5e74;
	uint32_t chip_valid[ASIC_CHIP_NUM] = {0};
	uint8_t tmp_buf[JOB_LENGTH];
	uint16_t crc;
	
	uint8_t job1[JOB_LENGTH] = {
		0x0C, 0x00, 0xF3, 0x4E, 0xFE, 0x37, 0x2A, 0x22, 
		0x6F, 0xE3, 0xE7, 0xB5, 0x55, 0xAD, 0x80, 0x35, 
		0xDD, 0x33, 0x54, 0x9C, 0xB4, 0x15, 0x04, 0xC9, 
		0x62, 0xD3, 0x22, 0xFD, 0x9A, 0x5D, 0x85, 0x7F,                     
		0xBD, 0x8D, 0x98, 0xEC, 0x90, 0xE3, 0x5A, 0xCD, 
		0x37, 0x04, 0xA7, 0xE0, 0xE2, 0xFD, 0x94, 0xA3, 
		0xE1, 0xB8, 0x0E, 0x98, 0x8B, 0xA1, 0x70, 0xB7, 
		0x6B, 0xC2, 0xEE, 0x99, 0x22, 0xC6, 0xC6, 0x18,                     
		0x13, 0x78, 0xED, 0x82, 0xEF, 0x83, 0xE2, 0x87, 
		0xF2, 0xB0, 0x66, 0xC6, 0x1D, 0x63, 0x98, 0x66, 
		0x15, 0x31, 0x1E, 0x8F, 0xB9, 0xEF, 0xD2, 0xDC, 
		0xD0, 0x60, 0xD2, 0x48, 0x4C, 0xD5, 0x0A, 0x90,                     
		0xAE, 0xD3, 0xEC, 0xA5, 0x90, 0xE3, 0x10, 0xC2, 
		0x6A, 0x22, 0x45, 0x7C, 0x5B, 0xA8, 0x34, 0x6C, 
		0x9E, 0xDB, 0xA9, 0x60, 0x82, 0xD9, 0x42, 0xD7, 
		0xB6, 0x28, 0x63, 0x3F, 0x2D, 0x61, 0x08, 0x9A,                     
		0x3A, 0xBA, 0x66, 0x7C, 0x46, 0x56, 0x9A, 0x1A, 
		0xC2, 0x59, 0x18, 0xFF, 0x00, 0x18, 0x00, 0x00, 
		0x00, 0x00, 0xFF, 0xFF, 0x3F, 0x1B, 0xFF, 0xFF, 
		0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x89, 0x39,                     
		0x00, 0x00, 
	};

	uint8_t job2[JOB_LENGTH] = {
		0x0C, 0x00, 0x6A, 0x63, 0x80, 0xFD, 0x1C, 0x33,
		0x48, 0xF8, 0x2C, 0xF2, 0x68, 0x0B, 0x07, 0x16, 
		0x3B, 0xF8, 0x18, 0x48, 0xF3, 0x8B, 0x9B, 0x6F, 
		0xDD, 0x93, 0x41, 0xBC, 0xA4, 0x99, 0x4D, 0x69,                     
		0xD3, 0xBF, 0x4F, 0x41, 0xAB, 0x6C, 0x0A, 0x36, 
		0x9A, 0x31, 0xEA, 0xCC, 0x33, 0x1B, 0x39, 0xFE, 
		0xAD, 0xA8, 0x4C, 0x20, 0x44, 0x1D, 0x26, 0xB3, 
		0x95, 0x79, 0xDA, 0xF2, 0x76, 0xBB, 0xA7, 0x6B,                     
		0x4E, 0x7A, 0x93, 0x07, 0xD1, 0x5B, 0x78, 0xA5, 
		0x1A, 0xA6, 0xB2, 0x34, 0xD4, 0xF5, 0x16, 0x4C, 
		0xCF, 0x81, 0xAA, 0xF4, 0x03, 0xD9, 0xC8, 0xBD, 
		0x33, 0x5A, 0x91, 0xB9, 0xA1, 0x26, 0x58, 0xFA,                     
		0xC4, 0x86, 0xF3, 0x89, 0x34, 0x8E, 0x7C, 0xA1, 
		0xFD, 0x7C, 0x7C, 0xD2, 0x14, 0xB2, 0xE4, 0x0C, 
		0xF1, 0x42, 0xBB, 0xF3, 0x78, 0xEA, 0x9C, 0x84, 
		0x07, 0xDF, 0xDB, 0x77, 0x91, 0x23, 0x6F, 0x2F,                     
		0x90, 0xA1, 0x43, 0x2A, 0xB7, 0x17, 0x96, 0xBC, 
		0xC9, 0x59, 0x18, 0xFF, 0x00, 0x18, 0x00, 0x00, 
		0x00, 0x00, 0xFF, 0xFF, 0x1F, 0x1B, 0xFF, 0xFF, 
		0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xF7, 0x7D, 
		0x00, 0x00, 
	};
	applog(LOG_INFO, "ChipNum:%d. \n", pChain->num_active_chips);
	for (k = 0; k < 3; k++){
		for (i = pChain->num_active_chips; i > 0; i--) 
		{
			c = i;
			job1[1] = i;
			memset(tmp_buf, 0, sizeof(tmp_buf));
    		for(j = 0; j < 79; j++)
    		{
        		tmp_buf[(2 * j) + 1] = job1[(2 * j) + 0];
        		tmp_buf[(2 * j) + 0] = job1[(2 * j) + 1];
    		}
    		crc = CRC16_2(tmp_buf, 158);
    		job1[158] = (uint8_t)((crc >> 8) & 0xff);
    		job1[159] = (uint8_t)((crc >> 0) & 0xff);
			
			if (!inno_cmd_write_job(pChain, c, job1)) 
			{
				applog(LOG_ERR, "failed to write job for chip %d. \n", c);
			} 			
		}

		usleep(300000);
		while (true) 
		{
			nonce = 0;
			chip_id = 0;
			job_id = 0;
			micro_job_id = 0;
			usleep(10000);
			if (!get_nonce(pChain, (uint8_t*)&nonce, &chip_id, &job_id, (uint8_t*)&micro_job_id))
				break;
			nonce = bswap_32(nonce);
			if(nonce == rightNonce){
				++chip_valid[chip_id - 1];
				//applog(LOG_ERR, "chip:%d is good, nonce:0x%x. \n", c, nonce);
			}else{
				applog(LOG_ERR, "bad nonce error, chip_id:%d, nonce:0x%x. \n", chip_id, nonce);
			}
		}
		
		for (i = pChain->num_active_chips; i > 0; i--) 
		{
			c = i;
			job2[1] = c;
			memset(tmp_buf, 0, sizeof(tmp_buf));
    		for(j = 0; j < 79; j++)
    		{
        		tmp_buf[(2 * j) + 1] = job2[(2 * j) + 0];
        		tmp_buf[(2 * j) + 0] = job2[(2 * j) + 1];
    		}
    		crc = CRC16_2(tmp_buf, 158);
    		job2[158] = (uint8_t)((crc >> 8) & 0xff);
    		job2[159] = (uint8_t)((crc >> 0) & 0xff);
		
			if (!inno_cmd_write_job(pChain, c, job2)) 
			{
				applog(LOG_ERR, "failed to write job for chip %d. \n", c);
			} 			
		}

		usleep(600000);
		while (true) 
		{
			nonce = 0;
			chip_id = 0;
			job_id = 0;
			micro_job_id = 0;
			usleep(10000);
			if (!get_nonce(pChain, (uint8_t*)&nonce, &chip_id, &job_id, (uint8_t*)&micro_job_id))
				break;
			nonce = bswap_32(nonce);
			if(nonce == rightNonce2){
				++chip_valid[chip_id - 1];
				//applog(LOG_ERR, "chip:%d is good, nonce:0x%x. \n", c, nonce);
			}else{
				applog(LOG_ERR, "bad nonce error, chip_id:%d, nonce:0x%x. \n", chip_id, nonce);
			}
		}
			
	}
	
	for (i = 1; i <= pChain->num_active_chips; i++){
		uiScore += chip_valid[i-1];
		/* printf("chip_valid[%d]=%d . \n", i-1, chip_valid[i-1]);
		if(chip_valid[i-1] >= 4){
			applog(LOG_ERR, "inno_cmd_test_chip chip %d is good. \n", i);
		}else{
			bad_chip_num++;
		}
		*/
		if(chip_valid[i-1] < 4){
			bad_chip_num++;
		}
	} 
		
	applog(LOG_ERR, "inno_cmd_test_chip bad chip num is %d. \n", bad_chip_num);
	return uiScore;
}

