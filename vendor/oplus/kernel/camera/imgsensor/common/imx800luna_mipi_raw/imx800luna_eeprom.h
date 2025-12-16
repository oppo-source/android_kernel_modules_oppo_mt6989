/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *     imx800luna_eeprom.h
 *
 * Project:
 * --------
 * Description:
 * ------------
 *     Add APIs to read from EEPROM
 *
 ****************************************************************************/
#ifndef __IMX800LUNA_EEPROM_H__
#define __IMX800LUNA_EEPROM_H__

#include "kd_camera_typedef.h"
#include "adaptor-subdrv.h"
#include "oplus_kd_eeprom.h"

/*
 * LRC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_imx800luna_LRC(struct subdrv_ctx *ctx, BYTE *data);

/*
 * DCC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_imx800luna_DCC(struct subdrv_ctx *ctx, BYTE *data);
unsigned int read_imx800luna_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size);
#endif
