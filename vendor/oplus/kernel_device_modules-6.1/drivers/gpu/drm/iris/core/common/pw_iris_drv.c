// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2024.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "pw_iris_api.h"
#include "pw_iris_dts_fw.h"
#include "pw_iris_dual.h"
#include "pw_iris_gpio.h"
#include "pw_iris_i2c.h"
#include "pw_iris_i3c.h"
#include "pw_iris_ioctl.h"
#include "pw_iris_lightup.h"
#include "pw_iris_lightup_ocp.h"
#include "pw_iris_log.h"
#include "pw_iris_loop_back.h"
#include "pw_iris_lp.h"
#include "pw_iris_lut.h"
#include "pw_iris_lut.h"
#include "pw_iris_memc.h"
#include "pw_iris_memc_helper.h"
#include "pw_iris_memc_helper_def.h"
#include "pw_iris_pq.h"
#include "pw_iris_timing_switch.h"

static int __init pw_iris_register(void)
{
	IRIS_LOGI("%s(), PW Iris driver module enter!", __func__);
	return 0;
}

static void __exit pw_iris_unregister(void)
{
	IRIS_LOGI("%s(), PW Iris driver module exit!", __func__);
}

/* Export symbols */
extern bool iris_debug_cap;
extern bool iris_osd_drm_autorefresh_enabled(bool is_secondary);
extern bool shadow_iris_HDR10_YCoCg;
extern int iris_get_drm_property(int id);
extern int iris_sde_connector_set_metadata(uint32_t val);
extern int iris_status_get(void);
extern struct i2c_client *iris_i2c_handle;
extern struct i2c_client *iris_pure_i2c_handle;
extern struct iris_cmd_desc *dynamic_lut_send_cmd;
extern struct iris_cmd_desc *dynamic_lutuvy_send_cmd;
extern struct lut_node iris_lut_param;
extern u8 *iris_ambient_lut_buf;
extern u8 *iris_maxcll_lut_buf;
extern u8 iris_sdr2hdr_mode;
extern void iris_pre_switch(struct iris_mode_info *new_timing);
extern void iris_register_osd_irq(void *disp);
extern void iris_set_panel_timing(uint32_t index, const struct iris_mode_info *timing);

EXPORT_SYMBOL(__traceiter_iris_tracing_mark_write);
EXPORT_SYMBOL(__tracepoint_iris_tracing_mark_write);
EXPORT_SYMBOL(_iris_dump_packet);
EXPORT_SYMBOL(_iris_fw_parse_dts);
EXPORT_SYMBOL(_iris_get_ctrl_seq_cs);
EXPORT_SYMBOL(_iris_is_valid_ip);
EXPORT_SYMBOL(_iris_load_mcu);
EXPORT_SYMBOL(_iris_pre_lightup);
EXPORT_SYMBOL(_iris_pre_lightup_i5); //I5
EXPORT_SYMBOL(_iris_read_chip_id);
EXPORT_SYMBOL(_iris_select_cont_splash_ipopt);
EXPORT_SYMBOL(_iris_send_cmds);
EXPORT_SYMBOL(_iris_send_lightup_pkt);
EXPORT_SYMBOL(_iris_set_cont_splash_type);
EXPORT_SYMBOL(dwCSC2CoffBuffer);
EXPORT_SYMBOL(dynamic_lut_send_cmd);
EXPORT_SYMBOL(dynamic_lutuvy_send_cmd);
EXPORT_SYMBOL(fw_calibrated_status);
EXPORT_SYMBOL(fw_loaded_status);
EXPORT_SYMBOL(iris_abyp_mode_get);
EXPORT_SYMBOL(iris_abyp_mode_set);
EXPORT_SYMBOL(iris_abyp_send_panel_cmd);
EXPORT_SYMBOL(iris_abyp_switch_proc);
EXPORT_SYMBOL(iris_al_enable);
EXPORT_SYMBOL(iris_alloc_seq_space);
EXPORT_SYMBOL(iris_alloc_update_ipopt_space);
EXPORT_SYMBOL(iris_ambient_lut_buf);
EXPORT_SYMBOL(iris_attach_cmd_to_ipidx);
EXPORT_SYMBOL(iris_brightness_level_set);
EXPORT_SYMBOL(iris_brightness_para_reset);
EXPORT_SYMBOL(iris_calc_inc);
EXPORT_SYMBOL(iris_calc_init_phase);
EXPORT_SYMBOL(iris_calc_left_top_offset);
EXPORT_SYMBOL(iris_calc_right_bot_offset);
EXPORT_SYMBOL(iris_change_lut_type_addr);
EXPORT_SYMBOL(iris_change_type_addr);
EXPORT_SYMBOL(iris_check_abyp_ready);
EXPORT_SYMBOL(iris_check_kickoff_fps_cadence);
EXPORT_SYMBOL(iris_check_kickoff_fps_cadence_2nd);
EXPORT_SYMBOL(iris_clear_aod_state);
EXPORT_SYMBOL(iris_cm_color_gamut_set);
EXPORT_SYMBOL(iris_cm_color_temp_set);
EXPORT_SYMBOL(iris_cm_colortemp_mode_set);
EXPORT_SYMBOL(iris_cm_ratio_set_for_iic);
EXPORT_SYMBOL(iris_color_temp_x_get);
EXPORT_SYMBOL(iris_configure_ex_memc);
EXPORT_SYMBOL(iris_configure_get_memc);
EXPORT_SYMBOL(iris_configure_memc);
EXPORT_SYMBOL(iris_crst_coef_check);
EXPORT_SYMBOL(iris_crstk_coef_buf);
EXPORT_SYMBOL(iris_csc_para_reset);
EXPORT_SYMBOL(iris_csc_para_set);
EXPORT_SYMBOL(iris_csc2_para_reset);
EXPORT_SYMBOL(iris_csc2_para_set);
EXPORT_SYMBOL(iris_dbg_gpio_init);
EXPORT_SYMBOL(iris_dbgfs_adb_type_init);
EXPORT_SYMBOL(iris_dbgfs_fw_calibrate_status_init);
EXPORT_SYMBOL(iris_dbgfs_loop_back_init);
EXPORT_SYMBOL(iris_dbgfs_memc_init);
EXPORT_SYMBOL(iris_dbgfs_pq_init);
EXPORT_SYMBOL(iris_dbgfs_scl_init); //I8 I7P
EXPORT_SYMBOL(iris_dbp_switch);
EXPORT_SYMBOL(iris_debug_cap);
EXPORT_SYMBOL(iris_debug_display_mode_get);
EXPORT_SYMBOL(iris_debug_pq_info_get);
EXPORT_SYMBOL(iris_deinit_timing_switch);
EXPORT_SYMBOL(iris_demo_wnd_conf);
EXPORT_SYMBOL(iris_display_mode_name_update);
EXPORT_SYMBOL(iris_dma_gen_ctrl);
EXPORT_SYMBOL(iris_dma_trig);
EXPORT_SYMBOL(iris_dom_set);
EXPORT_SYMBOL(iris_dpp_3dlut_gain);
EXPORT_SYMBOL(iris_dpp_apl_enable);
EXPORT_SYMBOL(iris_dpp_gammamode_set);
EXPORT_SYMBOL(iris_dpp_precsc_enable);
EXPORT_SYMBOL(iris_dpp_precsc_set);
EXPORT_SYMBOL(iris_driver_register);
EXPORT_SYMBOL(iris_driver_unregister);
EXPORT_SYMBOL(iris_dsi_ctrl_dump_desc_cmd);
EXPORT_SYMBOL(iris_dsi_panel_dump_pps);
EXPORT_SYMBOL(iris_dsi_rx_mode_switch);
EXPORT_SYMBOL(iris_dsi_write_mult_vals);
EXPORT_SYMBOL(iris_dtg_frame_rate_set);
EXPORT_SYMBOL(iris_dtg_update_reset);
EXPORT_SYMBOL(iris_dump_status);
EXPORT_SYMBOL(iris_dynamic_power_get);
EXPORT_SYMBOL(iris_dynamic_power_set);
EXPORT_SYMBOL(iris_enable_memc);
EXPORT_SYMBOL(iris_end_last_opt);
EXPORT_SYMBOL(iris_esd_ctrl_get);
EXPORT_SYMBOL(iris_exit_abyp);
EXPORT_SYMBOL(iris_find_ip_opt);
EXPORT_SYMBOL(iris_fomat_lut_cmds);
EXPORT_SYMBOL(iris_force_sync_payload);
EXPORT_SYMBOL(iris_fpga_adjust_read_buf);
EXPORT_SYMBOL(iris_fpga_adjust_read_cnt);
EXPORT_SYMBOL(iris_fpga_type_get);
EXPORT_SYMBOL(iris_frc_lp_switch);
EXPORT_SYMBOL(iris_frc_setting_init);
EXPORT_SYMBOL(iris_free_ipopt_buf);
EXPORT_SYMBOL(iris_free_seq_space);
EXPORT_SYMBOL(iris_get_abyp_mode);
EXPORT_SYMBOL(iris_get_ambient_lut);
EXPORT_SYMBOL(iris_get_cfg);
EXPORT_SYMBOL(iris_get_chip_caps);
EXPORT_SYMBOL(iris_get_chip_type);
EXPORT_SYMBOL(iris_get_cmd_list_cnt);
EXPORT_SYMBOL(iris_get_cmd_list_index);
EXPORT_SYMBOL(iris_get_cont_splash_type);
EXPORT_SYMBOL(iris_get_cont_type_with_timing_switch);
EXPORT_SYMBOL(iris_get_debug_cap);
EXPORT_SYMBOL(iris_get_drm_property);
EXPORT_SYMBOL(iris_get_dts_ops);
EXPORT_SYMBOL(iris_get_firmware_aplstatus_value);
EXPORT_SYMBOL(iris_get_fw_load_status);
EXPORT_SYMBOL(iris_get_hdr_enable);
EXPORT_SYMBOL(iris_get_ip_idx);
EXPORT_SYMBOL(iris_get_ipopt_payload_data);
EXPORT_SYMBOL(iris_get_ipopt_payload_len);
EXPORT_SYMBOL(iris_get_loglevel);
EXPORT_SYMBOL(iris_get_maxcll_info);
EXPORT_SYMBOL(iris_get_pq_disable_val);
EXPORT_SYMBOL(iris_get_setting);
EXPORT_SYMBOL(iris_get_sr_info);
EXPORT_SYMBOL(iris_get_timing_info);
EXPORT_SYMBOL(iris_get_tm_sw_loglevel);
EXPORT_SYMBOL(iris_get_trace_en);
EXPORT_SYMBOL(iris_global_var_init);
EXPORT_SYMBOL(iris_i2c_bit_en_op);
EXPORT_SYMBOL(iris_i2c_bus_exit);
EXPORT_SYMBOL(iris_i2c_bus_init);
EXPORT_SYMBOL(iris_i2c_handle);
EXPORT_SYMBOL(iris_i2c_write);
EXPORT_SYMBOL(iris_inc_osd_irq_cnt);
EXPORT_SYMBOL(iris_inc_osd_irq_cnt_impl);
EXPORT_SYMBOL(iris_init_lut_buf);
EXPORT_SYMBOL(iris_init_memc);
EXPORT_SYMBOL(iris_init_timing_switch);
EXPORT_SYMBOL(iris_init_update_ipopt_t);
EXPORT_SYMBOL(iris_ioctl_i2c_burst_write); //I7 common
EXPORT_SYMBOL(iris_ioctl_i2c_read); //I7 common
EXPORT_SYMBOL(iris_ioctl_i2c_write); //I7 common
EXPORT_SYMBOL(iris_ioctl_lock);
EXPORT_SYMBOL(iris_ioctl_unlock);
EXPORT_SYMBOL(iris_ioinc_filter_ratio_send);
EXPORT_SYMBOL(iris_is_abyp_timing);
EXPORT_SYMBOL(iris_is_chip_supported);
EXPORT_SYMBOL(iris_is_clk_switched_from_last_pt);
EXPORT_SYMBOL(iris_is_display1_autorefresh_enabled_impl);
EXPORT_SYMBOL(iris_is_dual_supported);
EXPORT_SYMBOL(iris_is_graphic_memc_supported);
EXPORT_SYMBOL(iris_is_freq_switched_from_last_pt);
EXPORT_SYMBOL(iris_is_pmu_dscu_on);
EXPORT_SYMBOL(iris_is_res_switched_from_last_pt);
EXPORT_SYMBOL(iris_is_same_timing_from_last_pt);
EXPORT_SYMBOL(iris_is_sleep_abyp_mode);
EXPORT_SYMBOL(iris_is_softiris_supported);
EXPORT_SYMBOL(iris_is_valid_type);
EXPORT_SYMBOL(iris_kickoff);
EXPORT_SYMBOL(iris_lightoff_memc);
EXPORT_SYMBOL(iris_lightup_opt_get);
EXPORT_SYMBOL(iris_linelock_set);
EXPORT_SYMBOL(iris_loop_back_reset);
EXPORT_SYMBOL(iris_low_latency_mode_get);
EXPORT_SYMBOL(iris_lp_enable_post);
EXPORT_SYMBOL(iris_lp_enable_pre);
EXPORT_SYMBOL(iris_lp_init);
EXPORT_SYMBOL(iris_lp_setting_off);
EXPORT_SYMBOL(iris_lut_param);
EXPORT_SYMBOL(iris_lux_set);
EXPORT_SYMBOL(iris_max_color_temp);
EXPORT_SYMBOL(iris_max_x_value);
EXPORT_SYMBOL(iris_maxcll_lut_buf);
EXPORT_SYMBOL(iris_mcu_ctrl_set);
EXPORT_SYMBOL(iris_mcu_state_set);
EXPORT_SYMBOL(iris_memc_chain_prepare);
EXPORT_SYMBOL(iris_memc_chain_process);
EXPORT_SYMBOL(iris_memc_dsc_config);
EXPORT_SYMBOL(iris_memc_dsc_info);
EXPORT_SYMBOL(iris_memc_func_init);
EXPORT_SYMBOL(iris_memc_get_pq_level);
EXPORT_SYMBOL(iris_memc_helper_change);
EXPORT_SYMBOL(iris_memc_helper_post);
EXPORT_SYMBOL(iris_memc_helper_setup);
EXPORT_SYMBOL(iris_memc_parse_info);
EXPORT_SYMBOL(iris_memc_set_pq_level);
EXPORT_SYMBOL(iris_memc_setting_off); //I8 I7P
EXPORT_SYMBOL(iris_min_color_temp);
EXPORT_SYMBOL(iris_min_x_value);
EXPORT_SYMBOL(iris_need_update_pps_one_time);
EXPORT_SYMBOL(iris_not_allow_off_primary);
EXPORT_SYMBOL(iris_not_allow_off_secondary);
EXPORT_SYMBOL(iris_ocp_read);
EXPORT_SYMBOL(iris_ocp_write_mult_vals);
EXPORT_SYMBOL(iris_ocp_write_val);
EXPORT_SYMBOL(iris_ocp_write_vals);
EXPORT_SYMBOL(iris_operate_conf);
EXPORT_SYMBOL(iris_operate_tool);
EXPORT_SYMBOL(iris_osd_auto_refresh_enable);
EXPORT_SYMBOL(iris_osd_drm_autorefresh_enabled);
EXPORT_SYMBOL(iris_osd_overflow_status_get);
EXPORT_SYMBOL(iris_ovs_dly_change);
EXPORT_SYMBOL(iris_parse_iris_golden_fw);
EXPORT_SYMBOL(iris_parse_lut_cmds);
EXPORT_SYMBOL(iris_parse_memc_param);
EXPORT_SYMBOL(iris_parse_misc_info);
EXPORT_SYMBOL(iris_platform_get);
EXPORT_SYMBOL(iris_pmu_bsram_set);
EXPORT_SYMBOL(iris_pmu_dscu_set);
EXPORT_SYMBOL(iris_pmu_frc_set);
EXPORT_SYMBOL(iris_pmu_mipi2_set);
EXPORT_SYMBOL(iris_pmu_power_set);
EXPORT_SYMBOL(iris_pq_update_path);
EXPORT_SYMBOL(iris_pre_switch);
EXPORT_SYMBOL(iris_pt_send_panel_cmd);
EXPORT_SYMBOL(iris_pt_sr_reset);
EXPORT_SYMBOL(iris_pt_sr_set);
EXPORT_SYMBOL(iris_pure_i2c_burst_write);
EXPORT_SYMBOL(iris_pure_i2c_bus_exit);
EXPORT_SYMBOL(iris_pure_i2c_bus_init);
EXPORT_SYMBOL(iris_pure_i2c_handle);
EXPORT_SYMBOL(iris_pure_i2c_single_read);
EXPORT_SYMBOL(iris_pure_i2c_single_write);
EXPORT_SYMBOL(iris_pwil_dport_disable);
EXPORT_SYMBOL(iris_qsync_mode_update);
EXPORT_SYMBOL(iris_qsync_set);
EXPORT_SYMBOL(iris_quality_setting_off);
EXPORT_SYMBOL(iris_query_capability);
EXPORT_SYMBOL(iris_reading_mode_set);
EXPORT_SYMBOL(iris_register_osd_irq);
EXPORT_SYMBOL(iris_register_osd_irq_impl);
EXPORT_SYMBOL(iris_release_firmware);
EXPORT_SYMBOL(iris_request_firmware);
EXPORT_SYMBOL(iris_reset_chip);
EXPORT_SYMBOL(iris_restore_capen);
EXPORT_SYMBOL(iris_revise_return_value);
EXPORT_SYMBOL(iris_rfb_helper_change);
EXPORT_SYMBOL(iris_rx_meta_dma_list_send);
EXPORT_SYMBOL(iris_scl_change_model);
EXPORT_SYMBOL(iris_scl_config);
EXPORT_SYMBOL(iris_scl_ioinc_filter);
EXPORT_SYMBOL(iris_scl_ioinc_pp_filter);
EXPORT_SYMBOL(iris_scl_ptsr_1to1);
EXPORT_SYMBOL(iris_scl_ptsr_config);
EXPORT_SYMBOL(iris_scl_ptsr_get);
EXPORT_SYMBOL(iris_scl_sr1d_filter);
EXPORT_SYMBOL(iris_scl_update_model);
//EXPORT_SYMBOL(iris_sde_connector_set_metadata);
EXPORT_SYMBOL(iris_sdr2hdr_mode);
EXPORT_SYMBOL(iris_send_assembled_pkt);
EXPORT_SYMBOL(iris_send_ipopt_cmds);
EXPORT_SYMBOL(iris_send_lut_for_dma);
EXPORT_SYMBOL(iris_send_one_wired_cmd);
EXPORT_SYMBOL(iris_send_timing_switch_pkt);
EXPORT_SYMBOL(iris_set_debug_cap);
EXPORT_SYMBOL(iris_set_dts_ops);
EXPORT_SYMBOL(iris_set_fps_trace_opt);
EXPORT_SYMBOL(iris_set_HDR10_YCoCg);
EXPORT_SYMBOL(iris_set_ipopt_payload_data);
EXPORT_SYMBOL(iris_set_loglevel);
EXPORT_SYMBOL(iris_set_metadata);
EXPORT_SYMBOL(iris_set_ocp_base_addr);
EXPORT_SYMBOL(iris_set_ocp_first_val);
EXPORT_SYMBOL(iris_set_ocp_type);
EXPORT_SYMBOL(iris_set_panel_timing);
EXPORT_SYMBOL(iris_set_pending_panel_brightness);
EXPORT_SYMBOL(iris_set_pinctrl_state);
EXPORT_SYMBOL(iris_set_pp_scl_aux);
EXPORT_SYMBOL(iris_set_sdr2hdr_mode);
EXPORT_SYMBOL(iris_set_skip_dma);
EXPORT_SYMBOL(iris_set_tm_sw_dbg_param);
EXPORT_SYMBOL(iris_set_trace_en);
EXPORT_SYMBOL(iris_set_two_wire0_enable);
EXPORT_SYMBOL(iris_setting);
EXPORT_SYMBOL(iris_skip_dma);
EXPORT_SYMBOL(iris_sleep_abyp_power_down);
EXPORT_SYMBOL(iris_sr_update);
EXPORT_SYMBOL(iris_status_get);
EXPORT_SYMBOL(iris_sw_te_enable);
EXPORT_SYMBOL(iris_switch);
EXPORT_SYMBOL(iris_sync_current_ipopt);
EXPORT_SYMBOL(iris_sys_pll_tx_phy_wa);
EXPORT_SYMBOL(iris_sysfs_status_deinit);
EXPORT_SYMBOL(iris_timing_switch_setup);
EXPORT_SYMBOL(iris_tx_buf_to_vc_set);
EXPORT_SYMBOL(iris_tx_pb_req_set);
EXPORT_SYMBOL(iris_ulps_enable_get);
EXPORT_SYMBOL(iris_update_2nd_active_timing);
EXPORT_SYMBOL(iris_update_ambient_lut);
EXPORT_SYMBOL(iris_update_backlight);
EXPORT_SYMBOL(iris_update_backlight_value);
EXPORT_SYMBOL(iris_update_bitmask_regval_nonread);
EXPORT_SYMBOL(iris_update_fw_load_status);
EXPORT_SYMBOL(iris_update_gamma);
EXPORT_SYMBOL(iris_update_ip_opt);
EXPORT_SYMBOL(iris_update_last_pt_timing);
EXPORT_SYMBOL(iris_update_maxcll_lut);
EXPORT_SYMBOL(iris_update_panel_ap_te);
EXPORT_SYMBOL(iris_update_panel_timing);
EXPORT_SYMBOL(iris_update_pq_opt);
EXPORT_SYMBOL(m_dpp_precsc_enable);
EXPORT_SYMBOL(pw_dbgfs_cont_splash_init);
EXPORT_SYMBOL(pw_dbgfs_status_init);
EXPORT_SYMBOL(pw_iris_dbgfs_lp_init);
EXPORT_SYMBOL(pw_iris_parse_param);
EXPORT_SYMBOL(shadow_iris_HDR10_YCoCg);
EXPORT_SYMBOL(iris_set_special_mode);

module_init(pw_iris_register);
module_exit(pw_iris_unregister);

MODULE_AUTHOR("SW support <support@pixelworks.com>");
MODULE_DESCRIPTION("Pixelworks IRIS Driver");
MODULE_LICENSE("GPL v2");
