/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _PW_IRIS_IOCTL_H_
#define _PW_IRIS_IOCTL_H_

#include "pw_iris_uapi.h"

void iris_set_brightness_v2(int level, int delay, int wait_vsync_count);

int iris_configure(u32 display, u32 type, u32 value);

int iris_configure_ex(u32 display, u32 type, u32 count, u32 *values);

int iris_configure_get(u32 display, u32 type, u32 count, u32 *values);

int iris_dbgfs_adb_type_init(void *display);

void iris_ioctl_lock(void);

void iris_ioctl_unlock(void);
bool iris_is_valid_type(u32 display, u32 type);

/* Internal API in kernel for I5 */

int iris_configure_i5(u32 display, u32 type, u32 value);

int iris_configure_ex_i5(u32 display, u32 type, u32 count, u32 *values);

int iris_configure_get_i5(u32 display, u32 type, u32 count, u32 *values);
int iris_dbgfs_adb_type_init_i5(void *display);
bool iris_is_valid_type(u32 display, u32 type);

/* Internal API in kernel for I7p */

int iris_configure_i7p(u32 display, u32 type, u32 value);

int iris_configure_ex_i7p(u32 display, u32 type, u32 count, u32 *values);

int iris_configure_get_i7p(u32 display, u32 type, u32 count, u32 *values);

int iris_dbgfs_adb_type_init_i7p(void *display);


/* Internal API in kernel for I7 */

int iris_configure_i7(u32 display, u32 type, u32 value);

int iris_configure_ex_i7(u32 display, u32 type, u32 count, u32 *values);

int iris_configure_get_i7(u32 display, u32 type, u32 count, u32 *values);

int iris_dbgfs_adb_type_init_i7(void *display);

/* Internal API in kernel for I8 */

int iris_configure_i8(u32 display, u32 type, u32 value);

int iris_configure_ex_i8(u32 display, u32 type, u32 count, u32 *values);

int iris_configure_get_i8(u32 display, u32 type, u32 count, u32 *values);

int iris_dbgfs_adb_type_init_i8(void *display);

int iris_operate_conf(struct msm_iris_operate_value *argp);
int iris_operate_tool(struct msm_iris_operate_value *argp);


#endif // _DSI_IRIS_IOCTL_H_
