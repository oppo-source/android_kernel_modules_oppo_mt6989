/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023 OPLUS Inc.
 */

#ifndef __OPLUS_CAM_CAL_LIST_H
#define __OPLUS_CAM_CAL_LIST_H

#include "oplus_kd_imgsensor.h"

#define MAX_EEPROM_SIZE_32K 0x8000
#define MAX_EEPROM_SIZE_16K 0x4000

struct stCAM_CAL_LIST_STRUCT g_oplusCamCalList[] = {
    {IMX709LUNA_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
    {IMX766LUNA_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
    {IMX800LUNA_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
    {S5KJN1LUNA_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_32K},
    /*  ADD before this line */
    {0, 0, 0}       /*end of list */
};

#endif        /* __OPLUS_CAM_CAL_LIST_H */