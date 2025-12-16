/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _PW_IRIS_LIGHTUP_H_
#define _PW_IRIS_LIGHTUP_H_
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/clk.h>

#include "pw_iris_def.h"


#define calc_space_left(x, y) (x - y%x)
#define NON_EMBEDDED_BUF_SIZE (512*1024)  //512k
#define IRIS_CHIP_CNT   2
#define IRIS_SYSFS_TOP_DIR   "iris"
#define CHIP_VERSION_IS_I7   1
#define CHIP_VERSION_IS_I7P   2
#define IRIS_OCP_HEADER_ADDR_LEN  8

//#define IRIS_EXT_CLK // use for external gpio clk
enum {
	DSI_CMD_ONE_LAST_FOR_MULT_IPOPT = 0,
};

enum iris_op_type {
	IRIS_LIGHTUP_OP = 0,
	IRIS_PQUPDATE_OP,
};

enum iris_send_mode {
	DSI_NON_EMBEDDED_MODE = 0,
	DSI_EMBEDDED_NO_MA_MODE,
	DSI_EMBEDDED_MA_MODE,
};

struct iris_cmd_comp {
	int32_t link_state;
	int32_t cnt;
	struct iris_cmd_desc *cmd;
	enum iris_op_type op_type;
	enum iris_send_mode send_mode;
};

/*use to parse dtsi cmd list*/
struct iris_cmd_header {
	uint32_t dsi_type;  /* dsi command type 0x23 0x29*/
	uint32_t last_pkt; /*last in chain*/
	uint32_t wait_us; /*wait time*/
	uint32_t ip_type; /*ip type*/
	uint32_t opt_and_link; /*ip option and lp or hs*/
	uint32_t payload_len; /*payload len*/
};

struct iris_data {
	const uint8_t *buf;
	uint32_t size;
};

struct iris_cfg *iris_get_cfg(void);

int32_t iris_send_ipopt_cmds(int32_t ip, int32_t opt_id);
void iris_update_pq_opt(uint8_t path, bool bcommit);
void iris_end_last_opt(void);
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
void iris_set_ipopt_payload_data(uint8_t ip, uint8_t opt_id, int32_t pos, uint32_t value);
int32_t _iris_send_cmds(struct iris_cmd_comp *pcmd_comp, uint8_t path);
uint32_t iris_get_ipopt_payload_len(uint8_t ip, uint8_t opt_id);

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


int32_t iris_attach_cmd_to_ipidx(const struct iris_data *data,
		int32_t data_cnt, struct iris_ip_index *pip_index);

struct iris_ip_index *iris_get_ip_idx(int32_t type);

void iris_change_type_addr(struct iris_ip_opt *dest, struct iris_ip_opt *src);
struct iris_ip_opt *iris_find_specific_ip_opt(uint8_t ip, uint8_t opt_id, int32_t type);
struct iris_ip_opt *iris_find_ip_opt(uint8_t ip, uint8_t opt_id);

struct iris_cmd_desc *iris_get_specific_desc_from_ipopt(uint8_t ip,
		uint8_t opt_id, int32_t pos, uint32_t type);
int iris_set_pending_panel_brightness(int32_t pending, int32_t delay, int32_t level);
void iris_free_ipopt_buf(uint32_t ip_type);
void iris_free_seq_space(void);

void iris_send_assembled_pkt(struct iris_ctrl_opt *arr, int seq_cnt);
int32_t iris_parse_dtsi_cmd(const struct device_node *lightup_node,
		uint32_t cmd_index);
int32_t iris_parse_optional_seq(struct device_node *np, const uint8_t *key,
		struct iris_ctrl_seq *pseq);
int iris_parse_cmd_param(struct device_node *lightup_node);

void iris_mult_addr_pad_i7p(uint8_t **p, uint32_t *poff, uint32_t left_len);
void iris_mult_addr_pad_i7(uint8_t **p, uint32_t *poff, uint32_t left_len);
void iris_mult_addr_pad_i8(uint8_t **p, uint32_t *poff, uint32_t left_len);
void iris_set_ocp_type(const uint8_t *payload, uint32_t val);
void iris_set_ocp_base_addr(const uint8_t *payload, uint32_t val);
void iris_set_ocp_first_val(const uint8_t *payload, uint32_t val);
void iris_display_mode_name_update(void);
uint32_t iris_get_cnn_model_count(void);
struct iris_ctrl_seq *iris_get_current_seq(void);
int iris_debug_display_mode_get_i5(char *kbuf, int size, bool debug);
int iris_debug_display_mode_get_i7(char *kbuf, int size, bool debug);
int iris_debug_display_mode_get_i7p(char *kbuf, int size, bool debug);
int iris_debug_display_mode_get_i8(char *kbuf, int size, bool debug);
int iris_debug_display_mode_get(char *kbuf, int size, bool debug);
int iris_debug_pq_info_get_i7(char *kbuf, int size, bool debug);
int iris_debug_pq_info_get(char *kbuf, int size, bool debug);

bool _iris_is_valid_cmd_type(int32_t type);
int32_t _iris_get_ip_idx_type(const struct iris_ip_index *pip_index);
uint32_t _iris_get_ocp_type(const uint8_t *payload);
uint32_t _iris_get_ocp_base_addr(const uint8_t *payload);
const char *_iris_dsi_trans_mode_name(uint8_t mode);
void _iris_set_cont_splash_type(uint8_t type);
bool _iris_is_valid_ip(uint32_t ip);
int iris_post_fod(bool is_secondary);
int _iris_select_cont_splash_ipopt(
		int type, struct iris_ctrl_opt *opt_arr);
void _iris_load_mcu(void);
void _iris_pre_lightup(void);
void _iris_pre_lightup_i5(void);
void iris_alloc_update_ipopt_space(void);
struct iris_ctrl_seq *_iris_get_ctrl_seq(struct iris_cfg *pcfg);
struct iris_ctrl_seq *_iris_get_ctrl_seq_cs(struct iris_cfg *pcfg);
int _iris_read_chip_id(void);
void _iris_send_lightup_pkt(void);
void iris_fpga_type_get(void);
int _iris_fw_parse_dts(const char *display_name);
int pw_iris_parse_param(struct device_node *lightup_node);
int pw_dbgfs_status_init(void *display);
int iris_parse_iris_golden_fw(struct device_node *np, const char *chip_name);
int pw_dbgfs_cont_splash_init(void *display);
void iris_memc_chain_prepare(void);
void iris_memc_chain_process(void);
int32_t iris_send_ipopt_cmds_i2c(int32_t ip, int32_t opt_id);
int iris_convert_dsi_to_i2c_i7p(uint8_t *payload);
#endif // _DSI_IRIS_LIGHTUP_H_
