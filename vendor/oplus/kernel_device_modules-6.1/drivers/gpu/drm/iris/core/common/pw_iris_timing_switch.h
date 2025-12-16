/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef __PW_IRIS_TIMING_SWITCH__
#define __PW_IRIS_TIMING_SWITCH__

#include "pw_iris_timing_switch_def.h"

void iris_init_timing_switch(void);
void iris_deinit_timing_switch(void);
int32_t iris_parse_timing_switch_info(struct device_node *np);
uint32_t iris_get_cont_type_with_timing_switch(struct iris_mode_info *timing);
uint32_t iris_get_cmd_list_cnt(void);
bool iris_is_abyp_timing(const struct iris_mode_info *new_timing);
void iris_update_last_pt_timing(void);
void iris_update_panel_timing(const struct iris_mode_info *panel_timing);
uint32_t iris_get_cmd_list_index(void);
bool iris_is_master_timing_supported(void);
uint8_t iris_get_master_timing_type(void);
void iris_sync_bitmask(struct iris_update_regval *pregval);
void iris_sync_payload(uint8_t ip, uint8_t opt_id, int32_t pos, uint32_t value);
void iris_sync_current_ipopt(uint8_t ip, uint8_t opt_id);
void iris_force_sync_payload(uint8_t ip, uint8_t opt_id, int32_t pos, uint32_t value);
void iris_force_sync_bitmask(uint8_t ip, uint8_t opt_id, int32_t reg_pos,
		int32_t bits, int32_t offset, uint32_t value);
void iris_switch_from_abyp_to_pt(void);
void iris_restore_capen(void);
void iris_get_timing_info(uint32_t count, uint32_t *values);

void iris_set_tm_sw_loglevel(uint32_t level);
void iris_set_tm_sw_dbg_param(uint32_t count, uint32_t *values);
uint32_t iris_get_tm_sw_loglevel(void);
void iris_dump_cmdlist(uint32_t val);
void iris_adjust_panel_te_scanline(bool enable);

bool iris_is_clk_switched_from_last_pt(void);
bool iris_is_same_timing_from_last_pt(void);
bool iris_is_res_switched_from_last_pt(void);
bool iris_is_freq_switched_from_last_pt(void);
void iris_send_timing_switch_pkt(void);
void iris_timing_switch_setup(void);
void iris_timing_switch_setup_i7(struct iris_timing_switch_ops *timing_switch_ops);
void iris_timing_switch_setup_i7p(struct iris_timing_switch_ops *timing_switch_ops);
void iris_timing_switch_setup_i8(struct iris_timing_switch_ops *timing_switch_ops);
void iris_set_special_mode(uint32_t count, uint32_t *values);

#endif //__PW_IRIS_TIMING_SWITCH__
