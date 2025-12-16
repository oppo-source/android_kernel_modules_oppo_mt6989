/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _PW_IRIS_API_H_
#define _PW_IRIS_API_H_

#include "pw_iris_def.h"

void iris_reset(void *dev);
int iris_set_pinctrl_state(bool enable);
int iris_pt_send_panel_cmd(struct iris_cmd_set *cmdset);
int iris_conver_one_panel_cmd(u8 *dest, u8 *src, int max);
int iris_vdo_panel_cmd(u8 *dest, u8 *src, int len);
int iris_update_backlight(u32 bl_lvl);
int iris_update_backlight_value(u32 bl_lvl);
void iris_control_pwr_regulator(bool on);
void iris_inc_osd_irq_cnt(void);
void iris_query_capability(void);
void iris_memc_func_init(void);
bool iris_is_chip_supported(void);
bool iris_is_softiris_supported(void);
bool iris_is_dual_supported(void);
bool iris_is_graphic_memc_supported(void);
void iris_update_2nd_active_timing(struct iris_mode_info *timing);
struct iris_chip_caps iris_get_chip_caps(void);
void iris_dsi_panel_dump_pps(struct iris_cmd_set *set);
void iris_dsi_ctrl_dump_desc_cmd(const struct mipi_dsi_msg *msg);
int iris_platform_get(void);
void iris_qsync_set(bool enable);
void iris_qsync_mode_update(u32 qsync_mode);
int iris_esd_ctrl_get(void);
void iris_dsi_rx_mode_switch(uint8_t rx_mode);
void iris_sw_te_enable(void);
uint32_t iris_schedule_line_no_get(void);
void iris_ioctl_lock(void);
void iris_ioctl_unlock(void);
int iris_pure_i2c_bus_init(void);
void iris_pure_i2c_bus_exit(void);
int iris_i2c_bus_init(void);
void iris_i2c_bus_exit(void);
bool iris_need_short_read_workaround(void);
void iris_fpga_adjust_read_cnt(u32 read_offset,
							   u32 rx_byte,
							   u32 read_cnt,
							   int *pcnt);
void iris_fpga_adjust_read_buf(u32 repeated_bytes,
							   u32 read_offset,
							   u32 rx_byte,
							   u32 read_cnt,
							   u8 *rd_buf);
void iris_update_panel_ap_te(void *handle, u32 new_te);
void iris_exit_abyp_update_panel_ap_te(void *handle, u32 new_te);
int iris_EDR_backlight_ctrl(u32 hdr_nit, u32 ratio_panel);
int iris_need_update_pps_one_time(void);
enum iris_chip_type iris_get_chip_type(void);
int iris_driver_register(void);
void iris_driver_unregister(void);
struct iris_cfg *iris_get_cfg(void);
void iris_set_dsi_cmd_log(uint32_t);
uint32_t iris_get_dsi_cmd_log(void);
void iris_panel_cmd_passthrough(unsigned int cmd, unsigned char count, unsigned char *para_list,
	uint8_t *buffer, uint8_t buffer_size, unsigned char rd_cmd);
void iris_set_valid(int step);
//u8 iris_get_cmd_type(u8 cmd, u32 count);
bool iris_is_pt_mode(bool is_secondary);
int iris_get_valid(void);
u32 iris_get_pq_disable_val(void);
int iris_blending_enable(bool enable);
bool iris_not_allow_off_primary(void);
bool iris_not_allow_off_secondary(void);
int iris_revise_return_value(int rc);
int iris_switch(struct iris_cmd_set *switch_cmds, struct iris_mode_info *new_timing);
int iris_switch_opt(void *handle, struct iris_cmd_set *switch_cmds,
		struct iris_mode_info *new_timing);
int iris_driver_ops_init_i8(void);
int iris_driver_ops_init_i7p(void);
int iris_driver_ops_init_i7(void);
int iris_driver_ops_init_i5(void);
void iris_abyp_power_down(void);
void iris_reset_mipi(void);
#endif // _PW_IRIS_API_H_
