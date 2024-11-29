/////////////////////////////////////////////////////////////////////////////
// File Name    : sem1217s_fw.c
// Function        : Various function for OIS control
//
// Copyright(c)    Rohm Co.,Ltd. All rights reserved
//
/***** ROHM Confidential ***************************************************/
//#define    _USE_MATH_DEFINES                            //

#ifndef SEM1217S_FW_C
#define SEM1217S_FW_C
#endif
#include <linux/firmware.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/slab.h>

#include "sem1217s_fw.h"
#include "sem1217s_flash_data.h"

extern uint8_t I2C_FW_8bit__read(uint32_t addr);
extern uint16_t I2C_FM_16bit__read(uint32_t addr);
extern uint32_t I2C_FM_32bit__read(uint32_t addr);

extern int I2C_FM_8bit_write(uint32_t addr, uint8_t data);
extern int I2C_FM_16bit_write(uint32_t addr, uint16_t data);
extern void I2C_FM_block_write(void *register_data, int size);

extern uint8_t I2C_OIS_8bit__read(uint32_t addr);
extern uint16_t I2C_OIS_16bit__read(uint32_t addr);
extern uint32_t I2C_OIS_32bit__read(uint32_t addr);

extern int I2C_OIS_8bit_write(uint32_t addr, uint8_t data);
extern int I2C_OIS_16bit_write(uint32_t addr, uint16_t data);
extern int I2C_OIS_block_write(void *register_data, int size);
extern void Wait(int us);

struct SEM1217S_FACT_ADJ FADJ_CAL = { 0x0000, 0x0000 };
struct SEM1217S_FACT_ADJ FADJ_PL = { 0x0000, 0x0000 };

int SEM1217S_Store_OIS_Cal_Data (void)
{
	uint8_t txdata[SEM1217S_TX_BUFFER_SIZE];
	uint8_t rxdata[SEM1217S_RX_BUFFER_SIZE];
	uint16_t repeatedCnt = 1000;
	int32_t data_error = 0xFFFF;

	rxdata[0] = I2C_OIS_8bit__read(SEM1217S_REG_OIS_STS);
	CAM_ERR(CAM_OIS, "SEM1217S_REG_OIS_STS: %u", rxdata[0]);

	if (rxdata[0] != SEM1217S_STATE_READY)
	{
		txdata[0] = SEM1217S_OIS_OFF;
		I2C_OIS_8bit_write(SEM1217S_REG_OIS_CTRL, txdata[0]);
	}

	txdata[0] = SEM1217S_OIS_INFO_EN; /* Set OIS_INFO_EN */
	/* Write 1 Byte to REG_INFO_BLK_UP_CTRL */
	I2C_OIS_8bit_write(SEM1217S_REGINFO_BLK_UP_CTRL, txdata[0]);
	CAM_ERR(CAM_OIS, "write SEM1217S_REGINFO_BLK_UP_CTRL: %u", txdata[0]);
	//I2C_Write_Data(REG_INFO_BLK_UP_CTRL, 1, txdata);
	Wait(100); /* Delay 100 ms */

	do
	{
		if (repeatedCnt == 0)
		{
			/* Abnormal Termination Error. */
			CAM_ERR(CAM_OIS, "REG_INFO_BLK_UP_CTRL failed: %u", rxdata[0]);
			return 0;
		}
		Wait(50); /* Delay 50 ms */
		rxdata[0]=I2C_OIS_8bit__read(SEM1217S_REGINFO_BLK_UP_CTRL);
		repeatedCnt--;
	} while ((rxdata[0] & SEM1217S_OIS_INFO_EN) == SEM1217S_OIS_INFO_EN);

	rxdata[0] = I2C_OIS_8bit__read(SEM1217S_REG_OIS_ERR);
	rxdata[1] = I2C_OIS_8bit__read(SEM1217S_REG_OIS_ERR + 1);
	data_error = (uint16_t)((rxdata[0]) | (rxdata[1] << 8 ));

	if ((data_error & SEM1217S_ERR_ODI) != SEM1217S_NO_ERROR)
	{
		/* Different INFORWRITE data on flash */
		CAM_ERR(CAM_OIS, "SEM1217S_ERR_ODI error %d", data_error);
		return 0;
	}

	return 1;
	/* INFORWRITE data on flash Success Process */
}

struct SEM1217S_FACT_ADJ SEM1217S_Gyro_offset_cal(void)
{
	uint8_t txdata[SEM1217S_TX_BUFFER_SIZE];
	uint8_t rxdata[SEM1217S_RX_BUFFER_SIZE];
	uint16_t repeatedCnt = 1000;
	uint16_t data_error = 0xFFFF;
	int rc = 0;

	struct SEM1217S_FACT_ADJ SEM1217S_FADJ_CAL = { 0x0000, 0x0000};

	rxdata[0] = I2C_OIS_8bit__read(SEM1217S_REG_OIS_STS);
	CAM_ERR(CAM_OIS, "SEM1217S_REG_OIS_STS: %u", rxdata[0]);

	if (rxdata[0] != SEM1217S_STATE_READY)
	{
		txdata[0] = SEM1217S_OIS_OFF;
		I2C_OIS_8bit_write(SEM1217S_REG_OIS_CTRL, txdata[0]);
	}

	rxdata[0] = I2C_OIS_8bit__read(SEM1217S_REG_AF_STS);
	CAM_ERR(CAM_OIS, "SEM1217S_REG_AF_STS: %u", rxdata[0]);

	if (rxdata[0] != SEM1217S_STATE_READY)
	{
		txdata[0] = SEM1217S_AF_OFF;
		I2C_OIS_8bit_write(SEM1217S_REG_AF_CTRL, txdata[0]);
	}

	txdata[0] = SEM1217S_G_OFFSET_EN;
	I2C_OIS_8bit_write(SEM1217S_REG_GCAL_CTRL, txdata[0]);
	Wait(50);

	do
	{
		if (repeatedCnt == 0)
		{
			return SEM1217S_FADJ_CAL;
		}
		Wait(50);
		rxdata[0] = I2C_OIS_8bit__read(SEM1217S_REG_GCAL_CTRL);
		CAM_ERR(CAM_OIS, "SEM1217S_REG_GCAL_CTRL: %u", rxdata[0]);
		repeatedCnt--;
	} while ((rxdata[0] & SEM1217S_G_OFFSET_EN) == SEM1217S_G_OFFSET_EN);

	rxdata[0] = I2C_OIS_8bit__read(SEM1217S_REG_OIS_ERR);
	rxdata[1] = I2C_OIS_8bit__read(SEM1217S_REG_OIS_ERR + 1);
	data_error = (uint16_t)((rxdata[0]) | (rxdata[1] << 8 ));

	if ((data_error & (SEM1217S_ERR_GCALX | SEM1217S_ERR_GCALY)) != SEM1217S_NO_ERROR)
	{
		CAM_ERR(CAM_OIS, "SEM1217S offset cal failed, data_error: 0x%x", data_error);
		SEM1217S_FADJ_CAL.gl_GX_OFS = 0;
		SEM1217S_FADJ_CAL.gl_GY_OFS = 0;
		return SEM1217S_FADJ_CAL;
	}

	rxdata[0] = I2C_OIS_8bit__read(SEM1217S_REG_GX_OFFSET);
	rxdata[1] = I2C_OIS_8bit__read(SEM1217S_REG_GX_OFFSET + 1);
	SEM1217S_FADJ_CAL.gl_GX_OFS = (uint16_t)((rxdata[0]) | (rxdata[1] << 8 ));

	rxdata[0] = I2C_OIS_8bit__read(SEM1217S_REG_GY_OFFSET);
	rxdata[1] = I2C_OIS_8bit__read(SEM1217S_REG_GY_OFFSET + 1);
	SEM1217S_FADJ_CAL.gl_GY_OFS = (uint16_t)((rxdata[0]) | (rxdata[1] << 8 ));
	CAM_ERR(CAM_OIS, "sem1217s x-offset: %u, y-offset: %u", SEM1217S_FADJ_CAL.gl_GX_OFS, SEM1217S_FADJ_CAL.gl_GY_OFS);

	rc = SEM1217S_Store_OIS_Cal_Data();

	return SEM1217S_FADJ_CAL;
}

int do_ois_cali(unsigned short *gyro_offset_x, unsigned short *gyro_offset_y)
{
	struct SEM1217S_FACT_ADJ data;
	data = SEM1217S_Gyro_offset_cal();
	*gyro_offset_x = data.gl_GX_OFS;
	*gyro_offset_y = data.gl_GY_OFS;
	return 0;
}

void GyroRead(uint32_t address)
{
	unsigned char rxdata[4];
	rxdata[0] = I2C_OIS_8bit__read(address);
	rxdata[1] = I2C_OIS_8bit__read(address+1);
	rxdata[2] = I2C_OIS_8bit__read(address+2);
	rxdata[3] = I2C_OIS_8bit__read(address+3);
	CAM_ERR(CAM_OIS, "[GyroRead] address= 0x%x, read = 0x%x %x %x %x", address, rxdata[0], rxdata[1], rxdata[2], rxdata[3]);
}

void GyroWrite(uint32_t address, uint32_t gain)
{
	unsigned char txdata[4];
	txdata[0] = gain & 0x00FF;
	txdata[1] = (gain & 0xFF00) >> 8;
	txdata[2] = (gain & 0xFF0000) >> 16;
	txdata[3] = (gain & 0xFF000000) >> 24;
	I2C_OIS_8bit_write(address, txdata[0]); /* write REG_GGX Little endian*/
	I2C_OIS_8bit_write(address+1, txdata[1]);
	I2C_OIS_8bit_write(address+2, txdata[2]);
	I2C_OIS_8bit_write(address+3, txdata[3]);
	CAM_ERR(CAM_OIS, "[GyroRead] gain = %u, address= 0x%x, write = 0x%x %x %x %x", gain, address, txdata[0], txdata[1], txdata[2], txdata[3]);
}

void SEM1217S_Gyro_gain_set(uint32_t X_gain, uint32_t Y_gain)
{
	uint8_t rxdata[SEM1217S_RX_BUFFER_SIZE];
	rxdata[0] = I2C_OIS_8bit__read(SEM1217S_REG_OIS_STS);
	CAM_ERR(CAM_OIS, "SEM1217S_Gyro_gain_set SEM1217S_REG_OIS_STS: %u", rxdata[0]);

        if (rxdata[0] == SEM1217S_STATE_READY)
        {
		/* Set Target Mode to Still Mode */
		I2C_OIS_8bit_write(SEM1217S_REG_OIS_MODE, SEM1217S_STILL_MODE); /* Write 1 Byte to SEM1217S_REG_OIS_MODE */
		/* Start Lens Control */
		I2C_OIS_8bit_write(SEM1217S_REG_OIS_CTRL, SEM1217S_OIS_ON); /* Write 1 Byte to REG_OIS_CTRL */
        }

	GyroRead(SEM1217S_REG_GX_GAIN); /* Read gyro gain x */
	GyroRead(SEM1217S_REG_GY_GAIN); /* Read gyro gain y */

	if (X_gain == 0x3F800000 && Y_gain == 0x3F800000)
	{
		CAM_ERR(CAM_OIS, "[SEM1217S_Gyro_gain_set] use default gyro gain, when X_gain= %u  Y_gain= %u", X_gain, Y_gain);
	}
	else
	{
		CAM_ERR(CAM_OIS, "[SEM1217S_Gyro_gain_set] newGyorGain  X_gain= %u  Y_gain= %u",X_gain, Y_gain);
		/* Set GYRO_GAIN_X */
		GyroWrite(SEM1217S_REG_GX_GAIN, X_gain); /* Write 4 Bytes to GYRO_GAIN_X */
		/* Set GYRO_GAIN_X_EN */
		I2C_OIS_8bit_write(SEM1217S_REG_GCAL_CTRL, SEM1217S_GX_GAIN_EN); /* Write 1 Byte to SEM1217S_REG_GCAL_CTRL */

		/* Set GYRO_GAIN_Y */
		GyroWrite(SEM1217S_REG_GY_GAIN, Y_gain); /* Write 4 Bytes to GYRO_GAIN_Y */
		/* Set GYRO_GAIN_Y_EN */
		I2C_OIS_8bit_write(SEM1217S_REG_GCAL_CTRL, SEM1217S_GY_GAIN_EN); /* Write 1 Byte to SEM1217S_REG_GCAL_CTRL */
	}

	GyroRead(SEM1217S_REG_GX_GAIN); /* Read gyro gain x */
	GyroRead(SEM1217S_REG_GY_GAIN); /* Read gyro gain y */
}

void SEM1217S_Hall_set(uint32_t X_Hall, uint32_t Y_Hall)
{
	unsigned char txdata[6];

	txdata[0] = (SEM1217S_REG_HALL_X >> 8);
	txdata[1] = (SEM1217S_REG_HALL_X & 0xFF);
	txdata[2] = X_Hall & 0x00FF;
	txdata[3] = (X_Hall & 0xFF00) >> 8;
	txdata[4] = Y_Hall & 0x00FF;
	txdata[5] = (Y_Hall & 0xFF00) >> 8;
	I2C_OIS_block_write(txdata, 4 + 2);
}

void SEM1217S_WriteGyroGainToFlash(void)
{
	int rc = 0;
	CAM_ERR(CAM_OIS, "[SEM1217S_WriteGyroGainToFlash]");
	rc = SEM1217S_Store_OIS_Cal_Data();
	GyroRead(SEM1217S_REG_GX_GAIN); /* Read gyro gain x */
	GyroRead(SEM1217S_REG_GY_GAIN); /* Read gyro gain y */
}

int sem1217s_fw_download(void)
{
	uint8_t txdata[SEM1217S_TX_BUFFER_SIZE + 2];
	uint8_t rxdata[SEM1217S_RX_BUFFER_SIZE];
	uint8_t* chkBuff = NULL;
	uint16_t txBuffSize;
	uint32_t i, chkIdx;
	uint16_t subaddr_FLASH_DATA_BIN_1;

	uint16_t idx = 0;
	uint16_t check_sum;
	uint32_t updated_ver;
	uint32_t new_fw_ver;
	uint32_t current_fw_ver;
	int rc = 0;
	chkBuff = (uint8_t*)kzalloc(SEM1217S_APP_FW_SIZE, GFP_KERNEL);

	rxdata[0] = I2C_OIS_8bit__read(SEM1217S_REG_APP_VER);
	rxdata[1] = I2C_OIS_8bit__read(SEM1217S_REG_APP_VER + 1);
	rxdata[2] = I2C_OIS_8bit__read(SEM1217S_REG_APP_VER + 2);
	rxdata[3] = I2C_OIS_8bit__read(SEM1217S_REG_APP_VER + 3);
	new_fw_ver = *(uint32_t *)&sem1217s_fw_1[SEM1217S_APP_FW_SIZE - 12];
	current_fw_ver = ((uint32_t *)rxdata)[0];

	CAM_ERR(CAM_OIS, "current_fw_ver: 0x%x, new_fw_ver: 0x%x", current_fw_ver, new_fw_ver);
	if (current_fw_ver == new_fw_ver)
	{
		CAM_ERR(CAM_OIS, "version is the same, no need to update");
		return 0;
	}

	if (current_fw_ver != 0)
	{
		rxdata[0] = I2C_OIS_8bit__read(SEM1217S_REG_OIS_STS);
		if (rxdata[0] != SEM1217S_STATE_READY)
		{
			txdata[0] = SEM1217S_OIS_OFF;
			rc = I2C_OIS_8bit_write(SEM1217S_REG_OIS_CTRL, txdata[0]);
			if (rc < 0)
			{
				CAM_ERR(CAM_OIS, "error 1");
				goto error_hand;
			}
		}
		rxdata[0] = I2C_OIS_8bit__read(SEM1217S_REG_AF_STS);
		if (rxdata[0] != SEM1217S_STATE_READY)
		{
			txdata[0] = SEM1217S_AF_OFF;
			rc = I2C_OIS_8bit_write(SEM1217S_REG_AF_CTRL, txdata[0]);
			if (rc < 0)
			{
				CAM_ERR(CAM_OIS, "error 2");
				goto error_hand;
			}
		}
	}

	txBuffSize = SEM1217S_TX_SIZE_256_BYTE;
	switch (txBuffSize)
	{
		case SEM1217S_TX_SIZE_32_BYTE:
			txdata[0] =  SEM1217S_FWUP_CTRL_32_SET;
            break;
		case SEM1217S_TX_SIZE_64_BYTE:
			txdata[0] = SEM1217S_FWUP_CTRL_64_SET;
			break;
		case SEM1217S_TX_SIZE_128_BYTE:
			txdata[0] = SEM1217S_FWUP_CTRL_128_SET;
			break;
		case SEM1217S_TX_SIZE_256_BYTE:
			txdata[0] = SEM1217S_FWUP_CTRL_256_SET;
			break;
		default:
			break;
	}
	rc = I2C_OIS_8bit_write(SEM1217S_REG_FWUP_CTRL, txdata[0]);
	if (rc < 0)
	{
		CAM_ERR(CAM_OIS, "error 3");
		goto error_hand;
	}
	Wait(60);
	check_sum = 0;

	subaddr_FLASH_DATA_BIN_1 = SEM1217S_REG_DATA_BUF;
	for (i = 0; i < (SEM1217S_APP_FW_SIZE / txBuffSize); i++)
	{
		memcpy(&chkBuff[txBuffSize * i], &sem1217s_fw_1[idx], txBuffSize);
		for (chkIdx = 0; chkIdx < txBuffSize; chkIdx += 2)
		{
			check_sum += ((chkBuff[chkIdx + 1 + (txBuffSize * i)] << 8) |  chkBuff[chkIdx + (txBuffSize * i)]);
		}
		memcpy(txdata + 2, &sem1217s_fw_1[idx], txBuffSize);
		txdata[0] = (subaddr_FLASH_DATA_BIN_1 >> 8);
		txdata[1] = (subaddr_FLASH_DATA_BIN_1 & 0xFF);
		rc = I2C_OIS_block_write(txdata, SEM1217S_TX_BUFFER_SIZE + 2);
		if (rc < 0)
		{
			CAM_ERR(CAM_OIS, "error 4");
			goto error_hand;
		}
		CAM_ERR(CAM_OIS, "update ois fw blk_num: %d 0x%x%x rc %d", i + 1, txdata[0], txdata[1], rc);
		idx += txBuffSize;
		Wait(20);
	}

	((uint16_t*)txdata)[1] = check_sum;
	CAM_ERR(CAM_OIS, "test %d",((uint16_t*)txdata)[1]);
	txdata[0] = (SEM1217S_REG_FWUP_CHKSUM >> 8);
	txdata[1] = (SEM1217S_REG_FWUP_CHKSUM & 0xFF);
	rc = I2C_OIS_block_write(txdata, 4);
	if (rc < 0)
	{
		CAM_ERR(CAM_OIS, "error 5");
		goto error_hand;
	}
	Wait(200);

	rxdata[0] = I2C_OIS_8bit__read(SEM1217S_REG_FWUP_ERR);
	if (rxdata[0] != SEM1217S_NO_ERROR)
	{
		CAM_ERR(CAM_OIS, "update fw error 0x%x", rxdata[0]);
		return -1;
	}

	txdata[0] = SEM1217S_RESET_REQ;
	rc = I2C_OIS_8bit_write(SEM1217S_REG_FWUP_CTRL,txdata[0]);
	if (rc < 0)
	{
		goto error_hand;
	}
	Wait(200);

	rxdata[0] = I2C_OIS_8bit__read(SEM1217S_REG_APP_VER);
	rxdata[1] = I2C_OIS_8bit__read(SEM1217S_REG_APP_VER + 1);
	rxdata[2] = I2C_OIS_8bit__read(SEM1217S_REG_APP_VER + 2);
	rxdata[3] = I2C_OIS_8bit__read(SEM1217S_REG_APP_VER + 3);

	updated_ver = *(uint32_t *)rxdata;

	CAM_ERR(CAM_OIS, "updated_ver: 0x%x, new_fw_ver: 0x%x", updated_ver, new_fw_ver);
	if (updated_ver != new_fw_ver)
	{
		CAM_ERR(CAM_OIS, "update fw failed, update version is not equal with read");
		return -1;
	}
error_hand:

	CAM_ERR(CAM_OIS, "update fw end, rc: %d", rc);

	return rc;
}