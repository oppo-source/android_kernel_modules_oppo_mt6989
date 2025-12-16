/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _PW_IRIS_I2C_H_
#define _PW_IRIS_I2C_H_

#include "pw_iris_def.h"

int iris_pure_i2c_burst_write(uint32_t addr, uint32_t *val, uint16_t reg_num);
int iris_pure_i2c_single_write(uint32_t addr, uint32_t val);
int iris_pure_i2c_single_read(uint32_t addr, uint32_t *val);
int iris_pure_i2c_multi_write(struct iris_i2c_msg *dsi_msg, uint32_t msg_num);
#endif
