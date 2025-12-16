/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_DUAL_
#define _DSI_IRIS_DUAL_

//#include "pw_iris_dual_i7.h"

#define IRIS_DSC_PPS_SIZE 128

void iris_register_osd_irq_ext_i7(void *disp);
void iris_register_osd_irq_ext_i8(void *disp);
int iris_create_pps_buf_cmd(char *buf, int pps_id, u32 len, bool is_secondary);
bool iris_aux_panel_initialized(void);
bool iris_aux_panel_existed(void);
//void iris_set_idlemgr(unsigned int crtc_id, unsigned int enable, bool need_lock);
#endif
