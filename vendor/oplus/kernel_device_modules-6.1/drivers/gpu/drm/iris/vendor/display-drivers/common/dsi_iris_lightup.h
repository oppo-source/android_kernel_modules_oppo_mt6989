/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_LIGHTUP_H_
#define _DSI_IRIS_LIGHTUP_H_

#include <linux/completion.h>
#include <linux/err.h>
#include <linux/clk.h>
//#include "dsi_pwr.h"
#include "pw_iris_lightup.h"

struct mtk_panel_params;
#if 0
#define IRIS_CHIP_CNT   2
#define IRIS_SYSFS_TOP_DIR   "iris"
#define CHIP_VERSION_IS_I7   1
#define CHIP_VERSION_IS_I7P   2

//#define IRIS_EXT_CLK // use for external gpio clk

/* iris ip option, it will create according to opt_id.
 *  link_state will be create according to the last cmds
 */
struct iris_ip_opt {
	uint8_t opt_id; /*option identifier*/
	uint32_t cmd_cnt; /*option length*/
	uint8_t link_state; /*high speed or low power*/
	struct iris_cmd_desc *cmd; /*the first cmd of desc*/
};

/*ip search index*/
struct iris_ip_index {
	int32_t opt_cnt; /*ip option number*/
	struct iris_ip_opt *opt; /*option array*/
};

struct iris_pq_ipopt_val {
	int32_t opt_cnt;
	uint8_t ip;
	uint8_t *popt;
};

struct iris_pq_init_val {
	int32_t ip_cnt;
	struct iris_pq_ipopt_val *val;
};

/*used to control iris_ctrl opt sequence*/
struct iris_ctrl_opt {
	uint8_t ip;
	uint8_t opt_id;
	uint8_t chain;
};

struct iris_ctrl_seq {
	int32_t cnt;
	struct iris_ctrl_opt *ctrl_opt;
};

//will pack all the commands here
struct iris_out_cmds {
	/* will be used before cmds sent out */
	struct iris_cmd_desc *iris_cmds_buf;
	u32 cmds_index;
};

struct iris_pq_update_cmd {
	struct iris_update_ipopt *update_ipopt_array;
	u32 array_index;
};

struct iris_i2c_cfg {
	uint8_t *buf;
	uint32_t buf_index;
};

typedef int (*iris_i2c_read_cb)(u32 reg_addr, u32 *reg_val);
typedef int (*iris_i2c_write_cb)(u32 reg_addr, u32 reg_val);
typedef int (*iris_i2c_burst_write_cb)(u32 start_addr, u32 *lut_buffer, u16 reg_num);

enum IRIS_PARAM_VALID {
	PARAM_NONE = 0,
	PARAM_EMPTY,
	PARAM_PARSED,
	PARAM_PREPARED,
	PARAM_LIGHTUP,
};

typedef void (*iris_acquire_panel_lock_cb)(void);
typedef void (*iris_release_panel_lock_cb)(void);
typedef int (*iris_dsi_send_cmds_cb)(struct iris_cmd_desc *cmds, u32 count,
	enum iris_cmd_set_state state, u8 vc_id);
typedef int (*iris_obtain_cur_timing_info_cb)(struct iris_mode_info *);
typedef void (*iris_set_esd_status_cb)(bool enable);
typedef int (*iris_debug_display_info_get_cb)(char *kbuf, int size);
typedef int (*iris_wait_vsync_cb)(void);
struct iris_lightup_ops {
	iris_acquire_panel_lock_cb acquire_panel_lock;
	iris_release_panel_lock_cb release_panel_lock;
	iris_dsi_send_cmds_cb  transfer;
	iris_obtain_cur_timing_info_cb  obtain_cur_timing_info;
	iris_debug_display_info_get_cb get_display_info;
	iris_wait_vsync_cb wait_vsync;
};

typedef void (*iris_cmd_desc_para_fill_cb)(struct iris_cmd_desc *dsi_cmd);
struct iris_platform_ops {
	iris_cmd_desc_para_fill_cb fill_desc_para;
};

/* iris lightup configure commands */
struct iris_cfg {
	struct dsi_display *display;
	struct dsi_panel *panel;

	struct platform_device *pdev;
	struct {
		struct pinctrl *pinctrl;
		struct pinctrl_state *active;
		struct pinctrl_state *suspend;
	} pinctrl;
	int iris_reset_gpio;
	int iris_wakeup_gpio;
	int iris_abyp_ready_gpio;
	int iris_osd_gpio;
	int iris_vdd_gpio;

	/* hardware version and initialization status */
	uint8_t chip_id;
	uint32_t chip_ver;
	uint32_t chip_value[2];
	uint8_t valid; /* 0: none, 1: empty, 2: parse ok, 3: minimum light up, 4. full light up */
	bool iris_initialized;
	uint32_t platform_type; /* 0: FPGA, 1~: ASIC */
	uint32_t cmd_param_from_fw;
	bool mcu_code_downloaded;
	bool switch_bl_endian;

	/* static configuration */
	uint8_t panel_type;
	uint8_t lut_mode;
	uint32_t split_pkt_size;
	uint32_t min_color_temp;
	uint32_t max_color_temp;
	uint8_t rx_mode; /* 0: DSI_VIDEO_MODE, 1: DSI_CMD_MODE */
	uint8_t tx_mode;
	uint8_t read_path; /* 0: DSI, 1: I2C */

	/* current state */
	struct iris_lp_ctrl lp_ctrl;
	struct iris_abyp_ctrl abyp_ctrl;
	uint16_t panel_nits;
	uint32_t panel_dimming_brightness;
	uint8_t panel_hbm[2];
	struct iris_frc_setting frc_setting;
	int pwil_mode;
	struct iris_vc_ctrl vc_ctrl;

	uint32_t panel_te;
	uint32_t ap_te;
	uint8_t power_mode;
	bool n2m_enable;
	u8 n2m_ratio;
	u32 dtg_ctrl_pt;

	bool iris_osd_autorefresh_enabled;
	atomic_t osd_irq_cnt;
	atomic_t video_update_wo_osd;

	char display_mode_name[16];
	uint32_t app_version;
	uint32_t app_version1;
	uint8_t app_date[4];
	uint8_t abyp_prev_mode;
	struct clk *ext_clk;

	int32_t panel_pending;
	int32_t panel_delay;
	int32_t panel_level;

	bool aod;
	bool fod;
	bool fod_pending;
	atomic_t fod_cnt;

	//TODO: struct dsi_regulator_info iris_power_info; // iris pmic power
	//uint8_t *panel_name;
	/* configuration commands, parsed from dt, dynamically modified
	 * panel->panel_lock must be locked before access and for DSI command send
	 */
	uint32_t lut_cmds_cnt;
	uint32_t dtsi_cmds_cnt;
	uint32_t ip_opt_cnt;
	struct iris_ip_index ip_index_arr[IRIS_PIP_IDX_CNT][IRIS_IP_CNT];
	struct iris_ctrl_seq ctrl_seq[IRIS_CHIP_CNT];
	struct iris_ctrl_seq ctrl_seq_cs[IRIS_CHIP_CNT];
	struct iris_pq_init_val pq_init_val;
	struct iris_out_cmds iris_cmds;
	struct iris_pq_update_cmd pq_update_cmd;

	/* one wire gpio lock */
	spinlock_t iris_1w_lock;
	struct dentry *dbg_root;
	struct kobject *iris_kobj;
	struct work_struct lut_update_work;
	struct work_struct vfr_update_work;
	struct completion frame_ready_completion;

	/* hook for i2c extension */
	struct mutex gs_mutex;
	struct mutex ioctl_mutex;
	struct mutex i2c_read_mutex;
	iris_i2c_read_cb iris_i2c_read;
	iris_i2c_write_cb iris_i2c_write;
	iris_i2c_burst_write_cb iris_i2c_burst_write;
	struct iris_i2c_cfg iris_i2c_cfg;

	uint32_t metadata;

	/* iris status */
	bool iris_mipi1_power_st;
	bool ap_mipi1_power_st;
	bool iris_pwil_blend_st;
	bool iris_mipi1_power_on_pending;
	bool iris_osd_overflow_st;
	bool iris_frc_vfr_st;
	u32 iris_pwil_mode_state;
	bool dual_enabled;
	bool frc_enabled;
	bool proFPGA_detected;

	struct iris_switch_dump switch_dump;
	struct iris_mspwil_setting chip_mspwil_setting;

	/* memc info */
	struct iris_memc_info memc_info;
	int osd_label;
	int frc_label;
	int frc_demo_window;

	/* emv info */
	struct extmv_frc_meta emv_info;

	/* emv perf info */
	struct extmv_clockinout emv_perf;

	/* pt_sr info*/
	bool pt_sr_enable;
	bool pt_sr_enable_restore;
	int pt_sr_hsize;
	int pt_sr_vsize;
	int pt_sr_guided_level;
	int pt_sr_dejaggy_level;
	int pt_sr_peaking_level;
	int pt_sr_DLTI_level;
	/* frc_pq info*/
	int frc_pq_guided_level;
	int frc_pq_dejaggy_level;
	int frc_pq_peaking_level;
	int frc_pq_DLTI_level;
	/* frcgame_pq info*/
	int frcgame_pq_guided_level;
	int frcgame_pq_dejaggy_level;
	int frcgame_pq_peaking_level;
	int frcgame_pq_DLTI_level;
	/*dsi send mode select*/
	uint8_t dsi_trans_mode[2];
	uint8_t *dsi_trans_buf;
	/*lightup pqupdate*/
	uint32_t dsi_trans_len[3][2];
	uint32_t ovs_delay;
	uint32_t ovs_delay_frc;
	uint32_t vsw_vbp_delay;
	bool dtg_eco_enabled;
	uint32_t ocp_read_by_i2c;
	int aux_width_in_using;
	int aux_height_in_using;
	/* calibration golden fw name */
	const char *ccf1_name;
	const char *ccf2_name;
	const char *ccf3_name;
#ifdef IRIS_EXT_CLK
	bool clk_enable_flag;
#endif
	struct iris_mode_info timing;
	atomic_t iris_esd_flag;
	uint32_t status_reg_addr;
	uint32_t id_sys_enter_abyp;
	uint32_t id_sys_exit_abyp;
	uint32_t ulps_mask_value;
	uint32_t id_piad_blend_info;
	uint32_t te_swap_mask_value;
	uint32_t id_tx_te_flow_ctrl;
	uint32_t id_tx_bypass_ctrl;
	uint32_t id_sys_mpg;
	uint32_t id_sys_dpg;
	uint32_t id_sys_ulps;
	uint32_t id_sys_abyp_ctrl;
	uint32_t id_sys_dma_ctrl;
	uint32_t id_sys_dma_gen_ctrl;
	uint32_t id_sys_te_swap;
	uint32_t id_sys_te_bypass;
	uint32_t id_sys_pmu_ctrl;
	uint32_t pq_pwr;
	uint32_t frc_pwr;
	uint32_t bsram_pwr;
	uint32_t id_rx_dphy;
	uint32_t iris_rd_packet_data;
	uint32_t iris_tx_intstat_raw;
	uint32_t iris_tx_intclr;
	uint32_t iris_mipi_tx_header_addr;
	uint32_t iris_mipi_tx_payload_addr;
	uint32_t iris_mipi_tx_header_addr_i3;
	uint32_t iris_mipi_tx_payload_addr_i3;
	uint32_t iris_dtg_addr;
	uint32_t dtg_ctrl;
	uint32_t dtg_update;
	uint32_t ovs_dly;
	uint32_t iris_chip_type;
	u32 qsync_mode;

	struct device *dev;
	unsigned long long crtc0_old_interval;
	bool memc_chain;
	struct iris_lightup_ops lightup_ops;
	struct iris_platform_ops platform_ops;
};
#endif

struct iris_vendor_cfg {
	struct drm_connector *conn;
	struct drm_device *drm;

	struct mtk_ddp_comp *mtk_comp;
	struct drm_display_mode *dsp_mode;
	struct mtk_panel_ext *panel_ext;
	//struct mipi_dsi_device *dsi_dev;
	struct drm_crtc *crtc;

	//struct drm_panel *drm_panel_2nd;
	struct mtk_panel_ext *mtk_panel_ext_2nd;
	//struct drm_connector *drm_connector_2nd;
	struct mtk_dsi *mtk_dsi_2nd;
	//unsigned long long crtc0_old_interval;
	//bool memc_chain;

	//atomic_t iris_esd_flag;
	//enum iris_chip_type iris_chip_type;
	u32 esd_chk_val[ESD_CHK_NUM + 1];
	bool is_esd_check_ongoing;
	bool esd_read_flag;
	int esd_check_num;
	int esd_read_index;
};
#if 0
struct iris_data {
	const uint8_t *buf;
	uint32_t size;
};
#endif

//struct iris_cfg *iris_get_cfg(void);
struct iris_vendor_cfg *iris_get_vendor_cfg(void);

#ifdef IRIS_EXT_CLK
void iris_clk_enable(void);
void iris_clk_disable(void);
#endif

int iris_lightup(void);
int iris_lightoff(bool dead, struct iris_cmd_set *off_cmds);
#if 0
int32_t iris_send_ipopt_cmds(int32_t ip, int32_t opt_id);
void iris_update_pq_opt(uint8_t path, bool bcommit);
void iris_update_bitmask_regval_nonread(
		struct iris_update_regval *pregval, bool is_commit);
uint32_t iris_get_regval_bitmask(int32_t ip, int32_t opt_id);
void iris_alloc_seq_space(void);

void iris_init_update_ipopt(struct iris_update_ipopt *popt,
		uint8_t ip, uint8_t opt_old, uint8_t opt_new, uint8_t chain);
struct iris_pq_ipopt_val  *iris_get_cur_ipopt_val(uint8_t ip);

int iris_init_update_ipopt_t(uint8_t ip, uint8_t opt_old, uint8_t opt_new, uint8_t chain);

/*
 * @description  get assigned position data of ip opt
 * @param ip       ip sign
 * @param opt_id   option id of ip
 * @param pos      the position of option payload
 * @return   fail NULL/success payload data of position
 */
uint32_t  *iris_get_ipopt_payload_data(uint8_t ip, uint8_t opt_id, int32_t pos);
uint32_t iris_get_ipopt_payload_len(uint8_t ip, uint8_t opt_id, int32_t pos);
void iris_set_ipopt_payload_data(uint8_t ip, uint8_t opt_id, int32_t pos, uint32_t value);
/*
 *@Description: get current continue splash stage
 first light up panel only
 second pq effect
 */
uint8_t iris_get_cont_splash_type(void);

/*
 *@Description: print continuous splash commands for bootloader
 *@param: pcmd: cmds array  cnt: cmds could
 */
void iris_print_desc_cmds(struct iris_cmd_desc *pcmd, int cmd_cnt, int state);

int iris_init_cmds(void);
void iris_get_cmds(struct iris_cmd_set *cmds, char **ls_arr);
void iris_get_lightoff_cmds(struct iris_cmd_set *cmds, char **ls_arr);


int32_t iris_attach_cmd_to_ipidx(const struct iris_data *data,
		int32_t data_cnt, struct iris_ip_index *pip_index);

struct iris_ip_index *iris_get_ip_idx(int32_t type);

void iris_change_type_addr(struct iris_ip_opt *dest, struct iris_ip_opt *src);

struct iris_ip_opt *iris_find_specific_ip_opt(uint8_t ip, uint8_t opt_id, int32_t type);
struct iris_ip_opt *iris_find_ip_opt(uint8_t ip, uint8_t opt_id);

struct iris_cmd_desc *iris_get_specific_desc_from_ipopt(uint8_t ip,
		uint8_t opt_id, int32_t pos, uint32_t type);
#endif
int iris_wait_vsync(void);
//int iris_set_pending_panel_brightness(int32_t pending, int32_t delay, int32_t level);

//bool iris_virtual_display(void);
#if 0
void iris_free_ipopt_buf(uint32_t ip_type);
void iris_free_seq_space(void);

void iris_send_assembled_pkt(struct iris_ctrl_opt *arr, int seq_cnt);
int32_t iris_parse_dtsi_cmd(const struct device_node *lightup_node,
		uint32_t cmd_index);
int32_t iris_parse_optional_seq(struct device_node *np, const uint8_t *key,
		struct iris_ctrl_seq *pseq);
#endif
void iris_insert_delay_us(uint32_t payload_size, uint32_t cmd_num);
//int iris_driver_register(void);
//void iris_driver_unregister(void);
//int iris_parse_cmd_param(struct device_node *lightup_node);

struct mtk_panel_params *iris_get_ext_panel_params(void);
int iris_get_vtotal(void);
int iris_get_htotal(void);
bool iris_check_dsc_enable(void);
bool iris_check_2nd_dsc_enable(void);

void iris_get_panel_params(struct drm_device *drm, struct drm_display_mode **dsp_mode);
void iris_sync_timing(struct iris_mode_info *ptiming, struct drm_display_mode *dsp_mode);
void iris_sync_aux_timing(struct iris_mode_info *ptiming, struct drm_display_mode *dsp_mode);
void iris_sync_cur_timing(void);
//void iris_memc_chain_prepare(void);
//void iris_memc_chain_process(void);
void iris_ddp_mutex_lock(void);
void iris_ddp_mutex_unlock(void);
void iris_change_header(void *pcmd_comp);
void iris_send_pwil_cmd(struct iris_cmd_set *pcmdset, u32 addr, u32 meta);
int dsi_iris_get_panel_mode(void);
int dsi_iris_obtain_cur_timing_info(struct iris_mode_info *timing_info);

int iris_lightoff_i7p(bool dead, struct iris_cmd_set *off_cmds);
int iris_lightoff_i7(bool dead, struct iris_cmd_set *off_cmds);
int iris_lightoff_i8(bool dead, struct iris_cmd_set *off_cmds);
void iris_init_i7p(struct drm_panel *panel, struct mtk_panel_ext *panel_ext, struct drm_connector *conn);
void iris_init_i7(struct drm_panel *panel, struct mtk_panel_ext *panel_ext, struct drm_connector *conn);
void iris_init_i8(struct drm_panel *panel, struct mtk_panel_ext *panel_ext, struct drm_connector *conn);
//void iris_deinit(struct drm_connector *conn);

int iris_sync_panel_brightness(int32_t step, void *phys_enc);
int iris_dbgfs_status_init(void *display);
int iris_configure_get_i7_selected(u32 display, u32 type, u32 count, u32 *values);
void iris_send_pwil_cmd(struct iris_cmd_set *pcmdset, u32 addr, u32 meta);

int iris_dbgfs_cont_splash_init(void *display);
int iris_debug_display_info_get(char *kbuf, int size);
void iris_core_lightup(void);
void iris_core_clk_set(bool enable, bool is_secondary);
bool iris_virtual_connector(struct drm_connector *c);
void iris_init_panel_timing(void *dev);
#endif // #endif // _DSI_IRIS_LIGHTUP_H_
