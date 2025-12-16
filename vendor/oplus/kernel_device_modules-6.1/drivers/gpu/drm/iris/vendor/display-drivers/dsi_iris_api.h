/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_API_H_
#define _DSI_IRIS_API_H_

#include <drm/drm_panel.h>
#include "pw_iris_api.h"

void iris_power_on(void);
void iris_reset_off(void *dev);
void iris_power_off(void);
int iris_abyp_send_panel_cmd(struct iris_cmd_set *cmdset);
int iris_enable( struct iris_cmd_set *on_cmds);
int iris_disable( bool dead, struct iris_cmd_set *off_cmds);
int iris_enable_secondary(struct drm_panel *panel);
int iris_disable_secondary(struct drm_panel *panel);
void iris_update_2nd_active_timing_mtk(int hdisplay, int vdisplay, int vrefresh, bool dsc);
void iris_set_panel_timing(uint32_t index,
		const struct iris_mode_info *timing);
void iris_pre_switch(
               struct iris_mode_info *new_timing);
void iris_set_valid(int step);
int iris_conver_one_panel_cmd(u8 *dest, u8 *src, int max);

int iris_find_secondary_name(char *name);
int iris_status_get(void);
void iris_send_cont_splash(void);
void iris_dump_ap_kickoff_fps(void);
//int iris_in_self_recovery(void);
//void iris_prepare(void);

//bool iris_is_display1_autorefresh_enabled(void *phys_enc);
//bool iris_is_virtual_encoder_phys(void *phys_enc);
void iris_register_osd_irq(void);

//void iris_init_tm_points_lut(void);
//bool iris_check_reg_read(void);
//bool iris_qsync_update_need(void);
//void iris_set_two_wire0_enable(void);
//void iris_sysfs_status_deinit(void);

//void iris_ddp_mutex_lock(void);
//void iris_ddp_mutex_unlock(void);
void iris_vdo_mode_send_cmd_with_handle(void *handle,
		void *data, int len, u32 flag, int type);
int iris_wait_for_bypass_cmdq_done(void);

#endif // _DSI_IRIS_API_H_
