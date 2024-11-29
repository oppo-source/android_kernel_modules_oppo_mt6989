// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "cam_cal_config.h"
#include "oplus_kd_imgsensor.h"
#define DEBUG_CALIBRATION_LOAD
static unsigned int do_single_lsc_dunhuangmain(struct EEPROM_DRV_FD_DATA *pdata,
	unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_2a_gain_dunhuangmain(struct EEPROM_DRV_FD_DATA *pdata,
	unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_lens_id_dunhuangmain(struct EEPROM_DRV_FD_DATA *pdata,
	unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);

static struct STRUCT_CALIBRATION_LAYOUT_STRUCT cal_layout_table = {
	0x00000006, 0x00A2002A, CAM_CAL_SINGLE_EEPROM_DATA,  //addr: 0x06 SENSORID+LENS ID
	{
		{0x00000001, 0x00000001, 0x00000001, do_module_version},
		{0x00000001, 0x00000001, 0x00000001, do_part_number},
		{0x00000001, 0x0000004E, 0x0000074C, do_single_lsc_dunhuangmain},
		{0x00000001, 0x00000012, 0x0000000E, do_2a_gain_dunhuangmain},  //Start address, block size is useless
		{0x00000001, 0x00000000, 0x0000079B, do_dump_all},
		{0x00000001, 0x00000003, 0x00000001, do_lens_id_dunhuangmain}
	}
};

struct STRUCT_CAM_CAL_CONFIG_STRUCT dunhuangmain_op_eeprom = {
	.name = "dunhuangmain_op_eeprom",
	.check_layout_function = layout_check,
	.read_function = Common_read_region,
	.layout = &cal_layout_table,
	.sensor_id = DUNHUANGMAIN_SENSOR_ID,
	.i2c_write_id = 0xA0,
	.max_size = 0x2000,
	.enable_preload = 1,
	.preload_size = 0x2000,
};

static unsigned int do_single_lsc_dunhuangmain(struct EEPROM_DRV_FD_DATA *pdata,
	unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
		(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;

	int read_data_size;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];
	unsigned short table_size;

	error_log("do_single_lsc_dunhuangmain E\n");

	if (pCamCalData->DataVer >= CAM_CAL_TYPE_NUM) {
		err = CAM_CAL_ERR_NO_DEVICE;
		error_log("Read Failed\n");
		show_cmd_error_log(pCamCalData->Command);
		return err;
	}
	if (block_size != CAM_CAL_SINGLE_LSC_SIZE)
		error_log("block_size(%d) is not match (%d)\n",
			block_size, CAM_CAL_SINGLE_LSC_SIZE);

	pCamCalData->SingleLsc.LscTable.MtkLcsData.MtkLscType = 2;//mtk type
	pCamCalData->SingleLsc.LscTable.MtkLcsData.PixId = 8;

	table_size = 1868;

	error_log("lsc table_size %d\n", table_size);
	pCamCalData->SingleLsc.LscTable.MtkLcsData.TableSize = table_size;
	if (table_size > 0) {
		pCamCalData->SingleLsc.TableRotation = 0;
		error_log("u4Offset=%d u4Length=%d", start_addr, table_size);
		read_data_size = read_data(pdata,
			pCamCalData->sensorID, pCamCalData->deviceID,
			start_addr, table_size, (unsigned char *)
			&pCamCalData->SingleLsc.LscTable.MtkLcsData.SlimLscType);
		if (table_size == read_data_size)
			err = CAM_CAL_ERR_NO_ERR;
		else {
			error_log("Read Failed\n");
			err = CamCalReturnErr[pCamCalData->Command];
			show_cmd_error_log(pCamCalData->Command);
		}
	}
	#ifdef DEBUG_CALIBRATION_LOAD
	error_log("======================SingleLsc Data==================\n");
	error_log("[1st] = %x, %x, %x, %x\n",
		pCamCalData->SingleLsc.LscTable.Data[0],
		pCamCalData->SingleLsc.LscTable.Data[1],
		pCamCalData->SingleLsc.LscTable.Data[2],
		pCamCalData->SingleLsc.LscTable.Data[3]);
	error_log("[1st] = SensorLSC(1)?MTKLSC(2)?  %x\n",
		pCamCalData->SingleLsc.LscTable.MtkLcsData.MtkLscType);
	error_log("CapIspReg =0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[0],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[1],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[2],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[3],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[4]);
	error_log("RETURN = 0x%x\n", err);
	error_log("======================SingleLsc Data==================\n");
	#endif

	return err;
}

static unsigned int do_2a_gain_dunhuangmain(struct EEPROM_DRV_FD_DATA *pdata,
	unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
		(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
	int read_data_size;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];

	long long CalGain, FacGain, CalValue;
	unsigned char AWBAFConfig = 0xf;

	unsigned short AFInf, AFMacro;
	int tempMax = 0;
	int CalR = 1, CalGr = 1, CalGb = 1, CalG = 1, CalB = 1;
	int FacR = 1, FacGr = 1, FacGb = 1, FacG = 1, FacB = 1;
	int rgCalValue = 1, bgCalValue = 1;
	unsigned int awb_offset;

	(void) start_addr;
	(void) block_size;

	error_log("block_size=%d sensor_id=%x\n", block_size, pCamCalData->sensorID);
	memset((void *)&pCamCalData->Single2A, 0, sizeof(struct STRUCT_CAM_CAL_SINGLE_2A_STRUCT));
	/* Check rule */
	if (pCamCalData->DataVer >= CAM_CAL_TYPE_NUM) {
		err = CAM_CAL_ERR_NO_DEVICE;
		error_log("Read Failed\n");
		show_cmd_error_log(pCamCalData->Command);
		return err;
	}
	/* Check AWB & AF enable bit */
	pCamCalData->Single2A.S2aVer = 0x01;
	pCamCalData->Single2A.S2aBitEn = (0x03 & AWBAFConfig);
	pCamCalData->Single2A.S2aAfBitflagEn = (0x0C & AWBAFConfig);
	error_log("S2aBitEn=0x%02x", pCamCalData->Single2A.S2aBitEn);
	/* AWB Calibration Data*/
	if (0x1 & AWBAFConfig) {
		pCamCalData->Single2A.S2aAwb.rGainSetNum = 0x02;
		awb_offset = 0x24;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&CalValue);
		if (read_data_size > 0)	{
			error_log( "Read CalValue OK\n");
			rgCalValue  = CalValue & 0xFFFF;
			bgCalValue = (CalValue >> 16) & 0xFFFF;
			error_log("Light source calibration 5000K value R/G:%d, B/G:%d",rgCalValue, bgCalValue);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read CalGain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		/* AWB Unit Gain (5000K) */
		error_log("5000K AWB\n");
		awb_offset = 0x12;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&CalGain);
		if (read_data_size > 0)	{
			error_log("Read CalGain OK %x\n", read_data_size);
			CalR  = CalGain & 0xFFFF;
			CalGr = (CalGain >> 16) & 0xFFFF;
			CalGb = (CalGain >> 32) & 0xFFFF;
			CalG  = ((CalGr + CalGb) + 1) >> 1;
			CalB  = (CalGain >> 48) & 0xFFFF;
			CalR  = CalR * rgCalValue / 1000;
			CalB  = CalB * bgCalValue / 1000;
			if (CalR > CalG)
				/* R > G */
				if (CalR > CalB)
					tempMax = CalR;
				else
					tempMax = CalB;
			else
				/* G > R */
				if (CalG > CalB)
					tempMax = CalG;
				else
					tempMax = CalB;
			error_log("UnitR:%d, UnitG:%d, UnitB:%d, New Unit Max=%d",
					CalR, CalG, CalB, tempMax);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read CalGain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (CalGain != 0x0000000000000000 &&
			CalGain != 0xFFFFFFFFFFFFFFFF &&
			CalR    != 0x00000000 &&
			CalG    != 0x00000000 &&
			CalB    != 0x00000000) {
			pCamCalData->Single2A.S2aAwb.rGainSetNum = 1;
			pCamCalData->Single2A.S2aAwb.rUnitGainu4R =
					(unsigned int)((tempMax * 512 + (CalR >> 1)) / CalR);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4G =
					(unsigned int)((tempMax * 512 + (CalG >> 1)) / CalG);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4B =
					(unsigned int)((tempMax * 512 + (CalB >> 1)) / CalB);
		} else {
			error_log("There are something wrong on EEPROM, plz contact module vendor!!\n");
			error_log("Unit R=%d G=%d B=%d!!\n", CalR, CalG, CalB);
		}
		/* AWB Golden Gain (5000K) */
		awb_offset = 0x1A;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&FacGain);
		if (read_data_size > 0)	{
			error_log("Read FacGain OK\n");
			FacR  = FacGain & 0xFFFF;
			FacGr = (FacGain >> 16) & 0xFFFF;
			FacGb = (FacGain >> 32) & 0xFFFF;
			FacG  = ((FacGr + FacGb) + 1) >> 1;
			FacB  = (FacGain >> 48) & 0xFFFF;
			if (FacR > FacG)
				if (FacR > FacB)
					tempMax = FacR;
				else
					tempMax = FacB;
			else
				if (FacG > FacB)
					tempMax = FacG;
				else
					tempMax = FacB;
			error_log("GoldenR:%d, GoldenG:%d, GoldenB:%d, New Golden Max=%d",
					FacR, FacG, FacB, tempMax);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read FacGain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (FacGain != 0x0000000000000000 &&
			FacGain != 0xFFFFFFFFFFFFFFFF &&
			FacR    != 0x00000000 &&
			FacG    != 0x00000000 &&
			FacB    != 0x00000000)	{
			pCamCalData->Single2A.S2aAwb.rGoldGainu4R =
					(unsigned int)((tempMax * 512 + (FacR >> 1)) / FacR);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4G =
					(unsigned int)((tempMax * 512 + (FacG >> 1)) / FacG);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4B =
					(unsigned int)((tempMax * 512 + (FacB >> 1)) / FacB);
		} else {
			error_log("There are something wrong on EEPROM, plz contact module vendor!!");
			error_log("Golden R=%d G=%d B=%d\n", FacR, FacG, FacB);
		}
		/* Set AWB to 3A Layer */
		pCamCalData->Single2A.S2aAwb.rValueR   = CalR;
		pCamCalData->Single2A.S2aAwb.rValueGr  = CalGr;
		pCamCalData->Single2A.S2aAwb.rValueGb  = CalGb;
		pCamCalData->Single2A.S2aAwb.rValueB   = CalB;
		pCamCalData->Single2A.S2aAwb.rGoldenR  = FacR;
		pCamCalData->Single2A.S2aAwb.rGoldenGr = FacGr;
		pCamCalData->Single2A.S2aAwb.rGoldenGb = FacGb;
		pCamCalData->Single2A.S2aAwb.rGoldenB  = FacB;
		#ifdef DEBUG_CALIBRATION_LOAD
		error_log("======================AWB CAM_CAL==================\n");
		error_log("AWB Calibration @5000K\n");
		error_log("[rCalGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4R);
		error_log("[rCalGain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4G);
		error_log("[rCalGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4B);
		error_log("[rFacGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4R);
		error_log("[rFacGain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4G);
		error_log("[rFacGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4B);
		#endif
	}
	/* AF Calibration Data*/
	if (0x2 & AWBAFConfig) {
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				0x2E, 2, (unsigned char *)&AFInf);
		if (read_data_size > 0)
			err = CAM_CAL_ERR_NO_ERR;
		else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}

		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				0x2C, 2, (unsigned char *)&AFMacro);
		if (read_data_size > 0)
			err = CAM_CAL_ERR_NO_ERR;
		else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}

		pCamCalData->Single2A.S2aAf[0] = AFInf;
		pCamCalData->Single2A.S2aAf[1] = AFMacro;

		////Only AF Gathering <////
		#ifdef DEBUG_CALIBRATION_LOAD
		error_log("======================AF CAM_CAL==================\n");
		error_log("[AFInf] = %d\n", AFInf);
		error_log("[AFMacro] = %d\n", AFMacro);
		error_log("======================AF CAM_CAL==================\n");
		#endif
	}
	return err;
}

static unsigned int do_lens_id_dunhuangmain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	error_log("do_lens_id_dunhuangmain E\n");
	return do_lens_id_base(pdata, start_addr, block_size, pGetSensorCalData);
}
