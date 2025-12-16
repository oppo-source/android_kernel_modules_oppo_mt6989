// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/delay.h>
#include "pw_iris_api.h"
#include "pw_iris_lightup.h"
#include "pw_iris_lightup_ocp.h"
#include "pw_iris_lp.h"
#include "pw_iris_dts_fw.h"
#include "pw_iris_memc.h"
#include "pw_iris_timing_switch.h"

static struct iris_timing_switch_ops timing_switch_ops;

enum {
	SWITCH_ABYP_TO_ABYP = 0,
	SWITCH_ABYP_TO_PT,
	SWITCH_PT_TO_ABYP,
	SWITCH_PT_TO_PT,
	SWITCH_NONE,
};

#define SWITCH_CASE(case)[SWITCH_##case] = #case
static const char * const switch_case_name[] = {
	SWITCH_CASE(ABYP_TO_ABYP),
	SWITCH_CASE(ABYP_TO_PT),
	SWITCH_CASE(PT_TO_ABYP),
	SWITCH_CASE(PT_TO_PT),
	SWITCH_CASE(NONE),
};
#undef SWITCH_CASE

enum {
	TIMING_SWITCH_RES_SEQ,
	TIMING_SWITCH_FPS_SEQ,
	TIMING_SWITCH_FPS_CLK_SEQ,
	TIMING_SWITCH_NORMAL_SEQ,
	TIMING_SWITCH_SPECIAL_SEQ,
	TIMING_SWITCH_SEQ_CNT,
};

struct IRIS_RFB_TIMING {
	uint8_t in_fps;
	uint8_t out_fps;
	uint8_t cmd_list_idx;
};

static struct iris_cfg_ts {
	struct iris_ctrl_seq tm_switch_seq[TIMING_SWITCH_SEQ_CNT];
	uint32_t switch_case;
	uint32_t cmd_list_index;
	uint32_t panel_tm_num;
	uint32_t iris_cmd_list_cnt;
	uint8_t *tm_cmd_map_arry;
	uint8_t *master_tm_cmd_map_arry;
	struct iris_mode_info *panel_tm_arry;
	uint32_t last_pt_tm_index;
	uint32_t new_tm_index;
	uint32_t cur_tm_index;
	enum iris_chip_type chip_type;
	bool clock_changed;
	uint32_t dbg_log_level;
	bool dbg_dtg_only;
	uint32_t rfb_tm_num;
	uint32_t boot_ap_te;
	struct IRIS_RFB_TIMING *rfb_timing_arry;
} gcfg_ts = {
	.switch_case = SWITCH_ABYP_TO_ABYP,
	.cmd_list_index = IRIS_DTSI_PIP_IDX_START,
	.panel_tm_num = 0,
	.iris_cmd_list_cnt = 0,
	.tm_cmd_map_arry = NULL,
	.master_tm_cmd_map_arry = NULL,
	.panel_tm_arry = NULL,
	.new_tm_index = 0,
	.last_pt_tm_index = 0,
	.cur_tm_index = 0,
	.chip_type = CHIP_UNKNOWN,
	.clock_changed = false,
	.dbg_log_level = 0,
	.dbg_dtg_only = false,
	.rfb_tm_num = 0,
	.boot_ap_te = 60,
	.rfb_timing_arry = NULL,
};


static inline struct iris_cfg_ts *_iris_get_ts_cfg(void)
{
	return &gcfg_ts;
};

void iris_init_timing_switch(void)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	pcfg_ts->chip_type = iris_get_chip_type();
	IRIS_LOGI("%s()", __func__);

	switch (pcfg_ts->chip_type) {
	case CHIP_IRIS7P:
	case CHIP_IRIS7:
	case CHIP_IRIS8:
		if (timing_switch_ops.iris_init_timing_switch_cb)
			timing_switch_ops.iris_init_timing_switch_cb();
		break;
	default:
		IRIS_LOGE("%s(), doesn't support for chip type: %#x",
				__func__, pcfg_ts->chip_type);
	}
}

void iris_deinit_timing_switch(void)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if (pcfg_ts->panel_tm_arry) {
		kvfree(pcfg_ts->panel_tm_arry);
		pcfg_ts->panel_tm_arry = NULL;
	}

	if (pcfg_ts->tm_cmd_map_arry) {
		kvfree(pcfg_ts->tm_cmd_map_arry);
		pcfg_ts->tm_cmd_map_arry = NULL;
	}

	if (pcfg_ts->master_tm_cmd_map_arry) {
		kvfree(pcfg_ts->master_tm_cmd_map_arry);
		pcfg_ts->master_tm_cmd_map_arry = NULL;
	}

	if (pcfg_ts->rfb_timing_arry) {
		kvfree(pcfg_ts->rfb_timing_arry);
		pcfg_ts->rfb_timing_arry = NULL;
	}
}

static bool _iris_support_timing_switch(void)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if (pcfg_ts == NULL || pcfg_ts->panel_tm_arry == NULL ||
		pcfg_ts->panel_tm_num == 0)
		return false;

	return true;
}

static bool _iris_check_sequence_status(struct device_node *np, const uint8_t *key)
{
	const uint8_t *pdata = NULL;
	int32_t item_cnt = 0;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return false;

	pdata = p_dts_ops->get_property(np, key, &item_cnt);
	if (!pdata) {
		IRIS_LOGI("%s(), dts ops not  %s", __func__, key);
		return false;
	}
	return true;
}

void iris_set_panel_timing(uint32_t index,
		const struct iris_mode_info *timing)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	//if (!iris_is_chip_supported())
	//	return;

	if (index >= pcfg_ts->panel_tm_num)
		return;

	// for primary display only, skip secondary
	//if (strcmp(display->display_type, "primary"))
	//	return;

	if (!_iris_support_timing_switch())
		return;

	IRIS_LOGI("%s(), timing@%u: %ux%u@%uHz, clk: %llu Hz, mdp transfer time: %u us",
		__func__, index,
		timing->h_active, timing->v_active, timing->refresh_rate,
		timing->clk_rate_hz, timing->mdp_transfer_time_us);
	memcpy(&pcfg_ts->panel_tm_arry[index], timing,
			sizeof(struct iris_mode_info));
}

static void _iris_init_param(struct device_node *np)
{
	int32_t pnl_tm_num = 0;
	int32_t rfb_timing_num = 0;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if (!p_dts_ops)
		return;

	pnl_tm_num = p_dts_ops->count_u8_elems(np, "pxlw,timing-cmd-map");
	if (pnl_tm_num < 1)
		pnl_tm_num = 0;

	pcfg_ts->panel_tm_num = pnl_tm_num;
	pcfg_ts->master_tm_cmd_map_arry = NULL;


	IRIS_LOGI("%s(), panel timing num: %d", __func__, pnl_tm_num);
	if (pcfg_ts->panel_tm_num > 1) {
		int32_t master_tm_num = 0;
		u32 buf_size = pcfg_ts->panel_tm_num * sizeof(struct iris_mode_info);

		pcfg_ts->panel_tm_arry = kvzalloc(buf_size, GFP_KERNEL);
		pcfg_ts->tm_cmd_map_arry = kvzalloc(pcfg_ts->panel_tm_num, GFP_KERNEL);

		master_tm_num = p_dts_ops->count_u8_elems(np, "pxlw,master-timing-cmd-map");
		if (master_tm_num > 0) {
			IRIS_LOGI("%s(), master timing map number: %d",
					__func__, master_tm_num);
			if (master_tm_num == pcfg_ts->panel_tm_num) {
				pcfg_ts->master_tm_cmd_map_arry = kvzalloc(pcfg_ts->panel_tm_num, GFP_KERNEL);
				IRIS_LOGI("%s(), support master panel timing", __func__);
			}
		} else {
			IRIS_LOGI("%s(), don't support master panel timing", __func__);
		}
	}

	p_dts_ops->read_u32(np, "pxlw,panel-te", &pcfg_ts->boot_ap_te);
	rfb_timing_num = p_dts_ops->count_u8_elems(np, "pxlw,rfb-timing-cmd-map");
	if (rfb_timing_num > 0) {
		int32_t param_cnt = sizeof(struct IRIS_RFB_TIMING) / sizeof(uint8_t);

		if (rfb_timing_num % param_cnt != 0)
			rfb_timing_num = 0;
		pcfg_ts->rfb_tm_num = rfb_timing_num / param_cnt;
		if (pcfg_ts->rfb_tm_num > 0)
			pcfg_ts->rfb_timing_arry =
				kvzalloc(sizeof(struct IRIS_RFB_TIMING) * pcfg_ts->rfb_tm_num, GFP_KERNEL);
	}

	IRIS_LOGI("%s(), rfb timing parameter count(timing num): %u(%u), boot ap te: %u",
		__func__, rfb_timing_num, pcfg_ts->rfb_tm_num, pcfg_ts->boot_ap_te);
}

static int32_t _iris_parse_timing_cmd_map(struct device_node *np)
{
	int32_t rc = 0;
	uint32_t i = 0;
	uint32_t j = 0;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();
	uint8_t *tm_cmd_map_arry = pcfg_ts->tm_cmd_map_arry;

	if (!p_dts_ops)
		return -EINVAL;

	rc = p_dts_ops->read_u8_array(np, "pxlw,timing-cmd-map",
			tm_cmd_map_arry, pcfg_ts->panel_tm_num);

	if (rc != 0) {
		IRIS_LOGE("%s(), failed to parse timing cmd map", __func__);
		return rc;
	}

	for (i = 0; i < pcfg_ts->panel_tm_num; i++) {
		IRIS_LOGI("%s(), cmd list %u for timing@%u",
				__func__, tm_cmd_map_arry[i], i);

		if (tm_cmd_map_arry[i] != IRIS_DTSI_NONE) {
			bool redup = false;

			for (j = 0; j < i; j++) {
				if (tm_cmd_map_arry[j] == tm_cmd_map_arry[i]) {
					redup = true;
					break;
				}
			}

			if (!redup)
				pcfg_ts->iris_cmd_list_cnt++;
		}
	}

	IRIS_LOGI("%s(), valid cmd list count is %u", __func__,
		pcfg_ts->iris_cmd_list_cnt);

	return rc;
}

static int32_t _iris_parse_master_timing_cmd_map(struct device_node *np)
{
	int32_t rc = 0;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if (!p_dts_ops)
		return -EINVAL;

	if (pcfg_ts->master_tm_cmd_map_arry == NULL)
		return rc;

	rc = p_dts_ops->read_u8_array(np, "pxlw,master-timing-cmd-map",
			pcfg_ts->master_tm_cmd_map_arry, pcfg_ts->panel_tm_num);
	if (rc != 0) {
		IRIS_LOGE("%s(), failed to parse master timing cmd map", __func__);
		return rc;
	}

	if (LOG_NORMAL_INFO) {
		uint32_t i = 0;

		for (i = 0; i < pcfg_ts->panel_tm_num; i++) {
			IRIS_LOGI("%s(), master timing map[%u] = %u", __func__,
					i, pcfg_ts->master_tm_cmd_map_arry[i]);
		}
	}

	return rc;
}

static int32_t _iris_parse_res_switch_seq(struct device_node *np)
{
	int32_t rc = 0;
	const uint8_t *key = "pxlw,iris-res-switch-sequence";
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if(!_iris_check_sequence_status(np, key))
		return -1;

	rc = iris_parse_optional_seq(np, key, &pcfg_ts->tm_switch_seq[TIMING_SWITCH_RES_SEQ]);
	IRIS_LOGI_IF(rc != 0, "%s(), failed to parse %s seq", __func__, key);

	return rc;
}

static int32_t _iris_parse_fps_switch_seq(struct device_node *np)
{
	int32_t rc = 0;
	const uint8_t *key = "pxlw,iris-fps-switch-sequence";
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if(!_iris_check_sequence_status(np, key))
		return -1;

	rc = iris_parse_optional_seq(np, key, &pcfg_ts->tm_switch_seq[TIMING_SWITCH_FPS_SEQ]);
	IRIS_LOGI_IF(rc != 0, "%s(), failed to parse %s seq", __func__, key);

	return rc;
}

static int32_t _iris_parse_fps_clk_switch_seq(struct device_node *np)
{
	int32_t rc = 0;
	const uint8_t *key = "pxlw,iris-fps-clk-switch-sequence";
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if(!_iris_check_sequence_status(np, key))
		return -1;

	rc = iris_parse_optional_seq(np, key, &pcfg_ts->tm_switch_seq[TIMING_SWITCH_FPS_CLK_SEQ]);
	IRIS_LOGI_IF(rc != 0, "%s(), failed to parse %s seq", __func__, key);

	return rc;
}

static int32_t _iris_parse_special_switch_seq(struct device_node *np)
{
	int32_t rc = 0;
	const uint8_t *key = "pxlw,iris-special-switch-sequence";
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if(!_iris_check_sequence_status(np, key))
		return -1;

	/* Both 'pxlw,iris-special-switch-sequence' and 'pxlw,iris-timing-switch-sequence-1' */
	/* are used for RFB switch. */
	rc = iris_parse_optional_seq(np, key, &pcfg_ts->tm_switch_seq[TIMING_SWITCH_SPECIAL_SEQ]);
	if (rc != 0) {
		key = "pxlw,iris-timing-switch-sequence-1";
		rc = iris_parse_optional_seq(np, key, &pcfg_ts->tm_switch_seq[TIMING_SWITCH_SPECIAL_SEQ]);
	}
	IRIS_LOGE_IF(rc != 0, "%s(), failed to parse %s seq", __func__, key);

	return rc;
}

static void _iris_check_rfb_map_count(struct device_node *np)
{
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t rfb_cmd_list_cnt = 1;
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();
	struct IRIS_RFB_TIMING *rfb_tm_arry = pcfg_ts->rfb_timing_arry;

	if (rfb_tm_arry == NULL)
		return;

	for (i = 0; i < pcfg_ts->rfb_tm_num; i++) {
		bool redup = false;

		for (j = 0; j < i; j++) {
			if (rfb_tm_arry[j].cmd_list_idx == rfb_tm_arry[i].cmd_list_idx) {
				redup = true;
				break;
			}
		}

		if (!redup)
			rfb_cmd_list_cnt++;
	}

	if (rfb_cmd_list_cnt > pcfg_ts->iris_cmd_list_cnt)
		pcfg_ts->iris_cmd_list_cnt = rfb_cmd_list_cnt;

	IRIS_LOGI("%s(), recheck valid cmd list count %u, rfb cmd list count %u.",
		__func__, pcfg_ts->iris_cmd_list_cnt, rfb_cmd_list_cnt);
}

static int32_t _iris_parse_rfb_timing(struct device_node *np)
{
	int32_t rc = 0;
	uint8_t i = 0;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if (!p_dts_ops)
		return -EINVAL;

	rc = p_dts_ops->read_u8_array(np, "pxlw,rfb-timing-cmd-map",
		(uint8_t *)pcfg_ts->rfb_timing_arry,
		pcfg_ts->rfb_tm_num * sizeof(struct IRIS_RFB_TIMING) / sizeof(uint8_t));
	if (rc != 0) {
		IRIS_LOGE("%s(), failed to parse rfb timing", __func__);
		return rc;
	}

	for (i = 0; i < pcfg_ts->rfb_tm_num; i++) {
		IRIS_LOGI("%s(), rfb timing[%u %u %u]", __func__,
			pcfg_ts->rfb_timing_arry[i].in_fps, pcfg_ts->rfb_timing_arry[i].out_fps,
			pcfg_ts->rfb_timing_arry[i].cmd_list_idx);
	}

	_iris_check_rfb_map_count(np);

	return rc;
}

uint32_t iris_get_cmd_list_cnt(void)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if (pcfg_ts->iris_cmd_list_cnt == 0)
		return IRIS_DTSI_PIP_IDX_CNT;

	return pcfg_ts->iris_cmd_list_cnt;
}

int32_t iris_parse_timing_switch_info(struct device_node *np)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();
	int32_t rc = 0;

	_iris_init_param(np);

	if (pcfg_ts->panel_tm_num > 1) {
		rc = _iris_parse_timing_cmd_map(np);
		IRIS_LOGI_IF(rc != 0,
				"%s(), [optional] have not timing cmd map", __func__);

		rc = _iris_parse_master_timing_cmd_map(np);
		IRIS_LOGI_IF(rc != 0,
				"%s(), [optional] have not master timing cmd map", __func__);

		rc = _iris_parse_res_switch_seq(np);
		IRIS_LOGI_IF(rc != 0,
				"%s(), [optional] have not resolution switch sequence", __func__);

		rc = _iris_parse_fps_switch_seq(np);
		IRIS_LOGI_IF(rc != 0,
				"%s(), [optional] have not fps switch sequence", __func__);

		rc = _iris_parse_fps_clk_switch_seq(np);
		IRIS_LOGI_IF(rc != 0,
				"%s(), [optional] have not fps clk switch sequence", __func__);
	}

	if (pcfg_ts->rfb_tm_num > 0) {
		rc = _iris_parse_special_switch_seq(np);
		IRIS_LOGI_IF(rc != 0,
				"%s(), [optional] have not special switch sequence", __func__);

		rc = _iris_parse_rfb_timing(np);
		IRIS_LOGI_IF(rc != 0,
				"%s(), [optional] have not rfb timing list", __func__);
	}

	return 0;
}

static void _iris_send_res_switch_pkt(void)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();
	struct iris_ctrl_seq *pseq = &pcfg_ts->tm_switch_seq[TIMING_SWITCH_RES_SEQ];
	struct iris_ctrl_opt *arr = NULL;
	ktime_t ktime = 0;

	IRIS_ATRACE_BEGIN(__func__);
	IRIS_LOGI_IF(LOG_DEBUG_INFO,
			"%s(), cmd list index: %02x", __func__, pcfg_ts->cmd_list_index);

	if (pseq == NULL) {
		IRIS_LOGE("%s(), seq is NULL", __func__);
		IRIS_ATRACE_END(__func__);
		return;
	}

	arr = pseq->ctrl_opt;

	if (LOG_VERBOSE_INFO) {
		int32_t i = 0;

		for (i = 0; i < pseq->cnt; i++) {
			IRIS_LOGI("%s(), i_p: %02x, opt: %02x, chain: %02x", __func__,
					arr[i].ip, arr[i].opt_id, arr[i].chain);
		}
	}

	if (LOG_DEBUG_INFO)
		ktime = ktime_get();
	iris_send_assembled_pkt(arr, pseq->cnt);
	IRIS_LOGI_IF(LOG_DEBUG_INFO,
			"%s(), send sequence cost '%d us'", __func__,
			(u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));

	usleep_range(100, 101);
	IRIS_ATRACE_END(__func__);
}

static bool _iris_need_fps_clk_seq(void)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();
	uint32_t new_tm_index = pcfg_ts->new_tm_index;
	uint32_t last_pt_tm_index = pcfg_ts->last_pt_tm_index;
	struct iris_mode_info *panel_tm_arry = pcfg_ts->panel_tm_arry;

	if (pcfg_ts->tm_switch_seq[TIMING_SWITCH_FPS_CLK_SEQ].ctrl_opt == NULL)
		return false;

	if (panel_tm_arry[new_tm_index].clk_rate_hz
			!= panel_tm_arry[last_pt_tm_index].clk_rate_hz) {
		IRIS_LOGI_IF(LOG_DEBUG_INFO,
				"%s(), switch with different clk, from %llu to %llu",
				__func__,
				panel_tm_arry[last_pt_tm_index].clk_rate_hz,
				panel_tm_arry[new_tm_index].clk_rate_hz);
		return true;
	}

	return false;
}

static void _iris_send_fps_switch_pkt(void)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();
	struct iris_ctrl_seq *pseq = &pcfg_ts->tm_switch_seq[TIMING_SWITCH_FPS_SEQ];
	struct iris_ctrl_opt *arr = NULL;
	ktime_t ktime = 0;

	IRIS_LOGI_IF(LOG_DEBUG_INFO,
			"%s(), cmd list index: %02x", __func__, pcfg_ts->cmd_list_index);

	if (pseq == NULL) {
		IRIS_LOGE("%s(), seq is NULL", __func__);
		return;
	}

	if (pcfg_ts->dbg_dtg_only)
		return;

	if (pcfg_ts->clock_changed) {
		pseq = &pcfg_ts->tm_switch_seq[TIMING_SWITCH_FPS_CLK_SEQ];
		IRIS_LOGI("with different clk");
	}

	arr = pseq->ctrl_opt;

	if (LOG_VERBOSE_INFO) {
		int32_t i = 0;

		for (i = 0; i < pseq->cnt; i++) {
			IRIS_LOGI("%s(), i_p: %02x, opt: %02x, chain: %02x", __func__,
					arr[i].ip, arr[i].opt_id, arr[i].chain);
		}
	}

	IRIS_ATRACE_BEGIN(__func__);
	if (LOG_DEBUG_INFO)
		ktime = ktime_get();

	iris_send_assembled_pkt(arr, pseq->cnt);
	IRIS_LOGI_IF(LOG_DEBUG_INFO,
			"%s(), send dtsi seq cost '%d us'", __func__,
			(u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));

	IRIS_ATRACE_END(__func__);
}

static uint32_t _iris_get_timing_index(const struct iris_mode_info *timing)
{
	uint32_t i = 0;
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if (!_iris_support_timing_switch())
		return 0;

	for (i = 0; i < pcfg_ts->panel_tm_num; i++) {
		struct iris_mode_info *t = &pcfg_ts->panel_tm_arry[i];

		if (timing->v_active == t->v_active &&
				timing->h_active == t->h_active &&
				timing->refresh_rate == t->refresh_rate)
			return i;
	}

	return 0;
}

static bool _iris_is_same_res(const struct iris_mode_info *new_timing,
		const struct iris_mode_info *old_timing)
{
	IRIS_LOGI_IF(LOG_VERBOSE_INFO,
			"%s(), switch from %ux%u to %ux%u",
			__func__,
			old_timing->h_active, old_timing->v_active,
			new_timing->h_active, new_timing->v_active);

	if (old_timing->h_active == new_timing->h_active
			&& old_timing->v_active == new_timing->v_active)
		return true;

	return false;
}

static uint32_t _iris_generate_switch_case(const struct iris_mode_info *new_timing)
{
	bool cur_pt_mode = false;
	u32 new_cmd_list_idx = 0;
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();
	struct iris_mode_info *last_pt_timing = &pcfg_ts->panel_tm_arry[pcfg_ts->last_pt_tm_index];

	if (!_iris_support_timing_switch())
		return SWITCH_ABYP_TO_ABYP;

	if (iris_get_abyp_mode() == PASS_THROUGH_MODE)
		cur_pt_mode = true;
	pcfg_ts->new_tm_index = _iris_get_timing_index(new_timing);
	new_cmd_list_idx = pcfg_ts->tm_cmd_map_arry[pcfg_ts->new_tm_index];

	if (new_cmd_list_idx != IRIS_DTSI_NONE)
		pcfg_ts->cmd_list_index = new_cmd_list_idx;

	if (cur_pt_mode) {
		if (new_cmd_list_idx == IRIS_DTSI_NONE)
			return SWITCH_PT_TO_ABYP;
		if (!_iris_is_same_res(new_timing, last_pt_timing)) {
			IRIS_LOGI("%s(), RES switch in PT mode", __func__);
			return SWITCH_PT_TO_ABYP;
		}

		return SWITCH_PT_TO_PT;
	}

	return SWITCH_ABYP_TO_ABYP;
}

static bool _iris_is_same_fps(const struct iris_mode_info *new_timing,
		const struct iris_mode_info *old_timing)
{
	IRIS_LOGI_IF(LOG_VERBOSE_INFO,
			"%s(), switch from %u to %u",
			__func__,
			old_timing->refresh_rate, new_timing->refresh_rate);

	if (new_timing->refresh_rate == old_timing->refresh_rate)
		return true;

	return false;
}

void iris_update_last_pt_timing(void)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if (!_iris_support_timing_switch())
		return;

	pcfg_ts->last_pt_tm_index = pcfg_ts->new_tm_index;
}

void iris_update_panel_timing(const struct iris_mode_info *panel_timing)
{
	u32 new_cmd_list_idx = 0;
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if (!_iris_support_timing_switch())
		return;

	pcfg_ts->new_tm_index = _iris_get_timing_index(panel_timing);
	new_cmd_list_idx = pcfg_ts->tm_cmd_map_arry[pcfg_ts->new_tm_index];

	if (new_cmd_list_idx != IRIS_DTSI_NONE)
		pcfg_ts->cmd_list_index = new_cmd_list_idx;

	pcfg_ts->cur_tm_index = pcfg_ts->new_tm_index;
}

bool iris_is_abyp_timing(const struct iris_mode_info *new_timing)
{
	uint32_t tm_index = 0;
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if (!new_timing || !pcfg_ts)
		return false;

	if (pcfg_ts->tm_cmd_map_arry == NULL)
		return false;

	tm_index = _iris_get_timing_index(new_timing);
	if (pcfg_ts->tm_cmd_map_arry[tm_index] == IRIS_DTSI_NONE)
		return true;

	return false;
}

static void _iris_pre_fps_switch(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t *payload = NULL;
	uint8_t val = 0;

	/* Follow _iris_abyp_ctrl_init() */
	payload = iris_get_ipopt_payload_data(IRIS_IP_SYS,
			pcfg->id_sys_abyp_ctrl, 2);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find: %02x, %02x",
				__func__, IRIS_IP_SYS, pcfg->id_sys_abyp_ctrl);
		return;
	}

	if (pcfg->lp_ctrl.abyp_lp == 1)
		val = 2;
	else if (pcfg->lp_ctrl.abyp_lp == 2)
		val = 1;
	else
		val = 0;

	payload[0] = BITS_SET(payload[0], 2, 22, val);
}

static void _iris_pre_switch_proc(void)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if (pcfg_ts->dbg_dtg_only)
		return;

	_iris_pre_fps_switch();

	if (timing_switch_ops.iris_pre_switch_proc)
		timing_switch_ops.iris_pre_switch_proc();
}

static void _iris_post_switch_proc(void)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if (pcfg_ts->dbg_dtg_only)
		return;

	if (timing_switch_ops.iris_post_switch_proc)
		timing_switch_ops.iris_post_switch_proc();
}
static void _iris_switch_fps_impl(void)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	_iris_pre_switch_proc();
	_iris_send_fps_switch_pkt();

	switch (pcfg_ts->chip_type) {
	case CHIP_IRIS7P:
	case CHIP_IRIS7:
	case CHIP_IRIS8:
		if (timing_switch_ops.iris_send_dynamic_seq)
			timing_switch_ops.iris_send_dynamic_seq();
		break;
	default:
		IRIS_LOGE("%s(), doesn't support for chip type: %#x",
				__func__, pcfg_ts->chip_type);
	}
	_iris_post_switch_proc();
}

uint32_t iris_get_cmd_list_index(void)
{
	return _iris_get_ts_cfg()->cmd_list_index;
}

bool iris_is_master_timing_supported(void)
{
	if (_iris_get_ts_cfg()->master_tm_cmd_map_arry != NULL)
		return true;

	return false;
}

uint8_t iris_get_master_timing_type(void)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if (iris_is_master_timing_supported())
		return pcfg_ts->master_tm_cmd_map_arry[pcfg_ts->cur_tm_index];

	return IRIS_DTSI_NONE;
}

static uint32_t _iris_cmd_to_timing(uint32_t cmd_index)
{
	uint32_t i = 0;
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();


	for (i = 0; i < pcfg_ts->panel_tm_num; i++) {
		if (pcfg_ts->tm_cmd_map_arry[i] == cmd_index)
			return i;
	}

	return pcfg_ts->cur_tm_index;
}

static bool _iris_skip_sync(uint32_t cmd_index)
{
	uint32_t cur_cmd_index = iris_get_cmd_list_index();
	struct iris_mode_info *cur_tm = NULL;
	struct iris_mode_info *tm = NULL;
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	IRIS_LOGI_IF(LOG_VERY_VERBOSE_INFO,
		"%s(), cmd index: %u, current cmd index: %u", __func__,
		cmd_index, cur_cmd_index);

	if (cmd_index == cur_cmd_index)
		return true;

	tm = &pcfg_ts->panel_tm_arry[_iris_cmd_to_timing(cmd_index)];
	cur_tm = &pcfg_ts->panel_tm_arry[_iris_cmd_to_timing(cur_cmd_index)];

	IRIS_LOGI_IF(LOG_VERY_VERBOSE_INFO,
		"%s(), timing: %ux%u@%u, %llu, current timing: %ux%u@%u, %llu",
		__func__,
		tm->h_active, tm->v_active, tm->refresh_rate, tm->clk_rate_hz,
		cur_tm->h_active, cur_tm->v_active, cur_tm->refresh_rate,
		cur_tm->clk_rate_hz);

	if (tm->h_active != cur_tm->h_active || tm->v_active != cur_tm->v_active ||
		tm->clk_rate_hz != cur_tm->clk_rate_hz)
		return true;

	if (tm->refresh_rate != cur_tm->refresh_rate)
		return false;

	return true;
}

void iris_sync_payload(uint8_t ip, uint8_t opt_id, int32_t pos, uint32_t value)
{
	struct iris_cmd_desc *pdesc = NULL;
	struct iris_cfg *pcfg = NULL;
	uint32_t *pvalue = NULL;
	uint32_t type = 0;
	uint32_t payload_offset = 0;

	if (!_iris_support_timing_switch())
		return;

	if (iris_is_master_timing_supported())
		return;

	if (ip >= LUT_IP_START && ip < LUT_IP_END)
		return;

	for (type = IRIS_DTSI_PIP_IDX_START; type < iris_get_cmd_list_cnt(); type++) {
		if (_iris_skip_sync(type))
			continue;

		pdesc = iris_get_specific_desc_from_ipopt(ip, opt_id, pos, type);
		if (!pdesc) {
			IRIS_LOGD("%s(), can't find it for type: %u, skip.", __func__, type);
			continue;
		}

		pcfg = iris_get_cfg();
		if (pos < 2)
			payload_offset = pos * 4;
		else
			payload_offset = (pos * 4 - IRIS_OCP_HEADER_ADDR_LEN) % pcfg->split_pkt_size
				+ IRIS_OCP_HEADER_ADDR_LEN;
		pvalue = (uint32_t *)((uint8_t *)pdesc->msg.tx_buf + payload_offset);
		pvalue[0] = value;
	}
}

void iris_sync_bitmask(struct iris_update_regval *pregval)
{
	int32_t ip = 0;
	int32_t opt_id = 0;
	uint32_t orig_val = 0;
	uint32_t *data = NULL;
	uint32_t val = 0;
	struct iris_ip_opt *popt = NULL;
	uint32_t type = 0;

	if (!_iris_support_timing_switch())
		return;

	if (iris_is_master_timing_supported())
		return;

	if (!pregval) {
		IRIS_LOGE("%s(), invalid input", __func__);
		return;
	}

	ip = pregval->ip;
	opt_id = pregval->opt_id;
	if (ip >= LUT_IP_START && ip < LUT_IP_END)
		return;

	for (type = IRIS_DTSI_PIP_IDX_START; type < iris_get_cmd_list_cnt(); type++) {
		if (_iris_skip_sync(type))
			continue;

		popt = iris_find_specific_ip_opt(ip, opt_id, type);
		if (popt == NULL) {
			IRIS_LOGW("%s(), can't find i_p: 0x%02x opt: 0x%02x, from type: %u",
					__func__, ip, opt_id, type);
			continue;
		} else if (popt->cmd_cnt != 1) {
			IRIS_LOGW("%s(), invalid bitmask for i_p: 0x%02x, opt: 0x%02x, type: %u, popt len: %d",
					__func__, ip, opt_id, type, popt->cmd_cnt);
			continue;
		}

		data = (uint32_t *)popt->cmd[0].msg.tx_buf;
		orig_val = cpu_to_le32(data[2]);
		val = orig_val & (~pregval->mask);
		val |= (pregval->value & pregval->mask);
		data[2] = val;
	}
}

void iris_sync_current_ipopt(uint8_t ip, uint8_t opt_id)
{
	struct iris_ip_opt *popt = NULL;
	struct iris_ip_opt *spec_popt = NULL;
	uint32_t type = 0;
	int i = 0;
	ktime_t ktime = 0;

	if (LOG_DEBUG_INFO)
		ktime = ktime_get();

	if (!_iris_support_timing_switch())
		return;

	if (iris_is_master_timing_supported())
		return;

	if (ip >= LUT_IP_START && ip < LUT_IP_END)
		return;

	popt = iris_find_ip_opt(ip, opt_id);
	if (popt == NULL)
		return;

	for (type = IRIS_DTSI_PIP_IDX_START; type < iris_get_cmd_list_cnt(); type++) {
		if (_iris_skip_sync(type))
			continue;

		spec_popt = iris_find_specific_ip_opt(ip, opt_id, type);
		if (spec_popt == NULL)
			continue;

		for (i = 0; i < popt->cmd_cnt; i++) {
			memcpy((void *)spec_popt->cmd[i].msg.tx_buf,
					popt->cmd[i].msg.tx_buf,
					popt->cmd[i].msg.tx_len);
			if (LOG_VERBOSE_INFO)
				print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 32, 4,
						popt->cmd[i].msg.tx_buf, popt->cmd[i].msg.tx_len, false);
		}
	}

	IRIS_LOGI_IF(LOG_DEBUG_INFO,
			"%s(), for i_p: %02x opt: 0x%02x, cost '%d us'",
			__func__, ip, opt_id,
			(u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));
}

void iris_force_sync_payload(uint8_t ip, uint8_t opt_id, int32_t pos, uint32_t value)
{
	struct iris_cmd_desc *pdesc = NULL;
	struct iris_cfg *pcfg = NULL;
	uint32_t *pvalue = NULL;
	uint32_t type = 0;
	uint32_t payload_offset = 0;

	if (!_iris_support_timing_switch())
		return;

	if (ip >= LUT_IP_START && ip < LUT_IP_END)
		return;

	for (type = IRIS_DTSI_PIP_IDX_START; type < iris_get_cmd_list_cnt(); type++) {
		if (_iris_skip_sync(type))
			continue;

		pdesc = iris_get_specific_desc_from_ipopt(ip, opt_id, pos, type);
		if (!pdesc) {
			IRIS_LOGD("%s(), can't find it for type: %u, skip.", __func__, type);
			continue;
		}

		pcfg = iris_get_cfg();
		if (pos < 2)
			payload_offset = pos * 4;
		else
			payload_offset = (pos * 4 - IRIS_OCP_HEADER_ADDR_LEN) % pcfg->split_pkt_size
				+ IRIS_OCP_HEADER_ADDR_LEN;
		pvalue = (uint32_t *)((uint8_t *)pdesc->msg.tx_buf + payload_offset);
		pvalue[0] = value;

	}
}

void iris_force_sync_bitmask(uint8_t ip, uint8_t opt_id, int32_t reg_pos,
		int32_t bits, int32_t offset, uint32_t value)
{
	struct iris_cmd_desc *pdesc = NULL;
	struct iris_cfg *pcfg = NULL;
	uint32_t *pvalue = NULL;
	uint32_t type = 0;
	uint32_t payload_offset = 0;

	if (!_iris_support_timing_switch())
		return;

	if (ip >= LUT_IP_START && ip < LUT_IP_END)
		return;

	for (type = IRIS_DTSI_PIP_IDX_START; type < iris_get_cmd_list_cnt(); type++) {
		if (_iris_skip_sync(type))
			continue;

		pdesc = iris_get_specific_desc_from_ipopt(ip, opt_id, reg_pos, type);
		if (!pdesc) {
			IRIS_LOGE("%s(), failed to find right desc.", __func__);
			return;
		}

		pcfg = iris_get_cfg();
		if (reg_pos < 2)
			payload_offset = reg_pos * 4;
		else
			payload_offset = (reg_pos * 4 - IRIS_OCP_HEADER_ADDR_LEN) % pcfg->split_pkt_size
				+ IRIS_OCP_HEADER_ADDR_LEN;
		pvalue = (uint32_t *)((uint8_t *)pdesc->msg.tx_buf + payload_offset);
		pvalue[0] = BITS_SET(pvalue[0], bits, offset, value);
	}
}

void iris_restore_capen(void)
{
	if (timing_switch_ops.iris_restore_capen)
		timing_switch_ops.iris_restore_capen();
}

void iris_pre_switch(struct iris_mode_info *new_timing)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();
	bool cur_pt_mode = false;

	if (new_timing == NULL || pcfg_ts == NULL)
		return;

	if (iris_get_abyp_mode() == PASS_THROUGH_MODE)
		cur_pt_mode = true;

	IRIS_ATRACE_BEGIN(__func__);
	pcfg_ts->switch_case = _iris_generate_switch_case(new_timing);
	if (cur_pt_mode && _iris_support_timing_switch()) {
		pcfg_ts->clock_changed = _iris_need_fps_clk_seq();
		if (timing_switch_ops.iris_pre_switch)
			timing_switch_ops.iris_pre_switch(new_timing->refresh_rate, pcfg_ts->clock_changed);
	}

	IRIS_LOGI("%s(), timing@%u, %ux%u@%uHz, cmd list: %u, case: %s",
			__func__,
			pcfg_ts->new_tm_index,
			new_timing->h_active,
			new_timing->v_active,
			new_timing->refresh_rate,
			pcfg_ts->cmd_list_index,
			switch_case_name[pcfg_ts->switch_case]);
	IRIS_LOGI_IF(cur_pt_mode,

			"%s(), FRC: %s, DUAL: %s",
			__func__,
			iris_get_cfg()->frc_enabled ? "true" : "false",
			iris_get_cfg()->dual_enabled ? "true" : "false");

	pcfg_ts->cur_tm_index = pcfg_ts->new_tm_index;
	IRIS_ATRACE_END(__func__);

	IRIS_LOGI_IF(LOG_VERBOSE_INFO,
			"%s(), exit.", __func__);
}

static void _iris_switch_proc(struct iris_mode_info *new_timing)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();
	struct iris_mode_info *last_pt_timing = &pcfg_ts->panel_tm_arry[pcfg_ts->last_pt_tm_index];

	if (new_timing == NULL || last_pt_timing == NULL)
		return;

	if (!_iris_is_same_res(new_timing, last_pt_timing)) {
		IRIS_LOGI("%s(), RES switch.", __func__);
		_iris_send_res_switch_pkt();
		return;
	}

	if (!_iris_is_same_fps(new_timing, last_pt_timing)) {
		IRIS_LOGI("%s(), FPS switch.", __func__);
		_iris_switch_fps_impl();
	}
}

int iris_switch_opt(void *handle, struct iris_cmd_set *switch_cmds,
		struct iris_mode_info *new_timing)
{
	int rc = 0;
	int lightup_opt = iris_lightup_opt_get();
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();
	u32 refresh_rate = new_timing->refresh_rate;
	ktime_t ktime = 0;

	if (pcfg_ts == NULL)
		return rc;

	IRIS_LOGI_IF(LOG_DEBUG_INFO,
			"%s(), new timing index: timing@%u", __func__, pcfg_ts->new_tm_index);

	if (LOG_NORMAL_INFO)
		ktime = ktime_get();

	IRIS_ATRACE_BEGIN(__func__);
	iris_update_panel_ap_te(handle, refresh_rate);

	if (lightup_opt & 0x8) {
		if (switch_cmds)
			rc = iris_abyp_send_panel_cmd(switch_cmds);
		IRIS_LOGI("%s(), force switch from ABYP to ABYP, total cost '%d us'",
				__func__,
				(u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));
		IRIS_ATRACE_END(__func__);
		return rc;
	}

	switch (pcfg_ts->switch_case) {
	case SWITCH_ABYP_TO_ABYP:
		IRIS_ATRACE_BEGIN("iris_abyp_send_panel_cmd");
		if (switch_cmds)
			rc = iris_abyp_send_panel_cmd(switch_cmds);
		IRIS_ATRACE_END("iris_abyp_send_panel_cmd");
		break;
	case SWITCH_PT_TO_PT:
		{
		ktime_t ktime_0 = ktime_get();

		IRIS_ATRACE_BEGIN("iris_pt_send_panel_cmd");
		if(switch_cmds)
			rc = iris_pt_send_panel_cmd(switch_cmds);
		IRIS_ATRACE_END("iris_pt_send_panel_cmd");
		IRIS_LOGI("%s(), send panel cmd cost '%d us'", __func__,
				(u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime_0)));

		_iris_switch_proc(new_timing);
		if (pcfg->qsync_mode > 0)
			iris_qsync_set(true);
		pcfg_ts->last_pt_tm_index = pcfg_ts->new_tm_index;
		iris_health_care();
		}
		break;
	case SWITCH_PT_TO_ABYP:
		iris_abyp_switch_proc(ANALOG_BYPASS_MODE);
		if(switch_cmds)
			rc = iris_abyp_send_panel_cmd(switch_cmds);
		break;
	default:
		IRIS_LOGE("%s(), invalid case: %u", __func__, pcfg_ts->switch_case);
		break;
	}

	IRIS_ATRACE_END(__func__);
	IRIS_LOGI("%s(), return %d, total cost '%d us'",
			__func__,
			rc, (u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));

	return rc;
}

int iris_switch(struct iris_cmd_set *switch_cmds,
		struct iris_mode_info *new_timing)
{
	return iris_switch_opt(NULL, switch_cmds, new_timing);
}

uint32_t iris_get_cont_type_with_timing_switch(struct iris_mode_info *timing)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t type = IRIS_CONT_SPLASH_NONE;
	uint32_t sw_case = SWITCH_NONE;

	if (pcfg->valid >= PARAM_PARSED)
		sw_case = _iris_generate_switch_case(timing);

	IRIS_LOGI("%s(), switch case: %s, %ux%u@%u",
			__func__,
			switch_case_name[sw_case],
			timing->h_active,
			timing->v_active,
			timing->refresh_rate);

	switch (sw_case) {
	case SWITCH_PT_TO_PT:
		type = IRIS_CONT_SPLASH_LK;
		break;
	case SWITCH_ABYP_TO_ABYP:
	case SWITCH_ABYP_TO_PT:
		type = IRIS_CONT_SPLASH_BYPASS_PRELOAD;
		break;
	case SWITCH_PT_TO_ABYP:
		// This case does not happen
	default:
		type = IRIS_CONT_SPLASH_NONE;
		break;
	}

	return type;
}

void iris_switch_from_abyp_to_pt(void)
{
	ktime_t ktime = 0;
	struct iris_mode_info *cur_timing = NULL;
	uint32_t cur_index = 0;
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if (!_iris_support_timing_switch())
		return;

	cur_index = pcfg_ts->cur_tm_index;
	IRIS_LOGI("%s(), current timing@%u, last pt timing@%u",
			__func__, cur_index, pcfg_ts->last_pt_tm_index);

	if (cur_index == pcfg_ts->last_pt_tm_index)
		return;

	if (pcfg_ts->tm_cmd_map_arry[cur_index] == IRIS_DTSI_NONE)
		return;

	if (LOG_NORMAL_INFO)
		ktime = ktime_get();

	cur_timing = &pcfg_ts->panel_tm_arry[cur_index];

	IRIS_ATRACE_BEGIN(__func__);
	iris_update_panel_ap_te(NULL, cur_timing->refresh_rate);
	_iris_switch_proc(cur_timing);
	pcfg_ts->last_pt_tm_index = cur_index;
	IRIS_ATRACE_END(__func__);

	IRIS_LOGI("%s(), total cost '%d us'",
			__func__,
			(u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));
}

void iris_get_timing_info(uint32_t count, uint32_t *values)
{
	int32_t i = 0;
	uint32_t *pval = NULL;
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	IRIS_LOGI("%s(), count: %u, timing num: %u",
		__func__, count, pcfg_ts->panel_tm_num);

	if (pcfg_ts->panel_tm_num <= 1 || pcfg_ts->panel_tm_arry == NULL)
		return;

	if (count < 7 || values == NULL)
		return;

	values[0] = pcfg_ts->panel_tm_num;
	pval = values + 1;

	for (i = 0; i < pcfg_ts->panel_tm_num; i++) {
		pval[i * 6 + 0] = pcfg_ts->panel_tm_arry[i].h_active;
		pval[i * 6 + 1] = pcfg_ts->panel_tm_arry[i].v_active;
		pval[i * 6 + 2] = pcfg_ts->panel_tm_arry[i].refresh_rate;
		pval[i * 6 + 3] = pcfg_ts->panel_tm_arry[i].clk_rate_hz & 0xFFFFFFFF;
		pval[i * 6 + 4] = (pcfg_ts->panel_tm_arry[i].clk_rate_hz >> 32) & 0xFFFFFFFF;
		pval[i * 6 + 5] = pcfg_ts->panel_tm_arry[i].mdp_transfer_time_us;

		if (i * 6 + 5 >= count) {
			IRIS_LOGE("%s(), out of range, index: %d.", __func__, i);
			return;
		}
	}
}

void iris_dump_cmdlist(uint32_t val)
{
	int ip_type = 0;
	int ip_index = 0;
	int opt_index = 0;
	int desc_index = 0;
	struct iris_ip_index *pip_index = NULL;
	uint32_t cmd_list_index = _iris_get_ts_cfg()->cmd_list_index;

	if (val == 0)
		return;

	IRIS_LOGW("%s() enter.", __func__);

	for (ip_type = 0; ip_type < iris_get_cmd_list_cnt(); ip_type++) {
		pip_index = iris_get_ip_idx(ip_type);
		if (pip_index == NULL)
			continue;
		pr_err("\n");
		if (ip_type == cmd_list_index)
			pr_err("*iris-cmd-list-%d*\n", ip_type);
		else
			pr_err("iris-cmd-list-%d\n", ip_type);

		for (ip_index = 0; ip_index < IRIS_IP_CNT; ip_index++) {
			if (pip_index[ip_index].opt_cnt == 0 || pip_index[ip_index].opt == NULL)
				continue;

			for (opt_index = 0; opt_index < pip_index[ip_index].opt_cnt; opt_index++) {
				if (pip_index[ip_index].opt[opt_index].cmd_cnt == 0 ||
						pip_index[ip_index].opt[opt_index].cmd == NULL)
					continue;

				pr_err("\n");
				pr_err("%02x %02x\n", ip_index, pip_index[ip_index].opt[opt_index].opt_id);
				for (desc_index = 0; desc_index < pip_index[ip_index].opt[opt_index].cmd_cnt;
						desc_index++) {
					if (pip_index[ip_index].opt[opt_index].cmd[desc_index].msg.tx_buf == NULL ||
						pip_index[ip_index].opt[opt_index].cmd[desc_index].msg.tx_len == 0)
						continue;

					print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 32, 4,
							pip_index[ip_index].opt[opt_index].cmd[desc_index].msg.tx_buf,
							pip_index[ip_index].opt[opt_index].cmd[desc_index].msg.tx_len,
							false);
				}
			}
		}
	}

	IRIS_LOGW("%s() exit.", __func__);
}

/* irisConfig 110 1 0~3 */
/* LOG_NORMAL_LEVEL       = 0 */
/* LOG_DEBUG_LEVEL        = 1 */
/* LOG_VERBOSE_LEVEL      = 2 */
/* LOG_VERY_VERBOSE_LEVEL = 3 */
void iris_set_tm_sw_loglevel(uint32_t level)
{
	if (level >= LOG_LEVEL_COUNT) {
		IRIS_LOGE("%s(), invalid level: %u", __func__, level);

		return;
	}

	_iris_get_ts_cfg()->dbg_log_level = level;
}

uint32_t iris_get_tm_sw_loglevel(void)
{
	return _iris_get_ts_cfg()->dbg_log_level;
}

/* irisConfig 110 2 type n */
void iris_set_tm_sw_dbg_param(uint32_t count, uint32_t *values)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if (count == 1) {
		iris_set_tm_sw_loglevel(values[0]);
		return;
	}

	switch (values[0]) {
	case DBG_DUMP_CMDLIST:  /* irisConfig 110 2 0 0 */
		iris_dump_cmdlist(1);
		break;
	case DBG_SEND_DTG_ONLY:  /* irisConfig 110 2 1 1/0 */
		if (pcfg_ts->chip_type == CHIP_IRIS8) {
			pcfg_ts->dbg_dtg_only = (values[1] > 0);
			if (timing_switch_ops.iris_set_tm_sw_dbg_param)
				timing_switch_ops.iris_set_tm_sw_dbg_param(values[0], values[1]);
		}
		break;
	case DBG_DPORT_SKIP_FRAME:     /* irisConfig 110 2 2 n */
	case DBG_PRE_CAP_SKIP_FRAME:   /* irisConfig 110 2 3 n */
	case DBG_POST_CAP_SKIP_FRAME:  /* irisConfig 110 2 4 n */
	case DBG_PRE_DELAY_MS:         /* irisConfig 110 2 5 n */
	case DBG_POST_DELAY_MS:        /* irisConfig 110 2 6 n */
	case DBG_DISABLE_ALL:          /* irisConfig 110 2 15 0 */
		if (pcfg_ts->chip_type == CHIP_IRIS8) {
			if (timing_switch_ops.iris_set_tm_sw_dbg_param)
				timing_switch_ops.iris_set_tm_sw_dbg_param(values[0], values[1]);
		}
		break;
	default:
		IRIS_LOGE("%s(), invalid type %u.", __func__, values[0]);
	}
}
/* TODO */
/* cmd data is adjust panel te scanline cmds,need modify it according panel */
/* If occur timing switch in PT mode and timing switch cmd codes will reset scanine register value */
/* It will happened below seqenuce: */
/* 1. adjust scanline-->enter PT -->timing switch -->reset scanline */
/* 2. timing switch -->reset scanline-->exit PT mode */
/* It happened abvoe seqcuene, it also introduced scanline noise issue*/
#define ADJUST_PANEL_TE_SCANLINE_CMD_COUNT 9
#define ADJUST_PANEL_TE_SCANLINE_CMD_DATA_COUNT 12
void iris_adjust_panel_te_scanline(bool enable)
{
#if 0
	int rc = 0;
	int i = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_cmd_desc desc[ADJUST_PANEL_TE_SCANLINE_CMD_COUNT] = {{{0}, 0, 0, 0, 0}};
	struct iris_cmd_set cmdset = {
		.state = IRIS_CMD_SET_STATE_HS,
		.count = ADJUST_PANEL_TE_SCANLINE_CMD_COUNT,
		.cmds = &desc[0],
	};


	u8 cmd_data[ADJUST_PANEL_TE_SCANLINE_CMD_COUNT][ADJUST_PANEL_TE_SCANLINE_CMD_DATA_COUNT] = {
		0x39, 0x00, 0x00, 0x40, 0x00, 0x00, 0x03, 0xF0, 0x5A, 0x5A},
		0x39, 0x00, 0x00, 0x40, 0x00, 0x00, 0x04, 0xB0, 0x00, 0x24, 0xB9},
		0x15, 0x00, 0x00, 0x40, 0x00, 0x00, 0x02, 0xB9, 0x21},
		0x39, 0x00, 0x00, 0x40, 0x00, 0x00, 0x04, 0xB0, 0x00, 0x38, 0xB9},
		0x15, 0x00, 0x00, 0x40, 0x00, 0x00, 0x02, 0xB9, 0x02},  // TSP_VSYNC Fixed TE 02:120 03:90 05:60
		0x39, 0x00, 0x00, 0x40, 0x00, 0x00, 0x04, 0xB0, 0x00, 0x2A, 0xB9},
		0x39, 0x00, 0x00, 0x40, 0x00, 0x00, 0x03, 0xB9, 0x03, 0xE8},  //TSP_VSYNC Fixed TE + 1000 H
		0x15, 0x00, 0x00, 0x40, 0x00, 0x00, 0x02, 0xF7, 0x0F},
		0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xF0, 0xA5, 0xA5}
	};

	switch (pcfg->panel_te) {
	case 60:
		cmd_data[4][8] = 0x05;
		break;
	case 90:
		cmd_data[4][8] = 0x03;
		break;
	default:
		break;
	}

	if (!enable) {
		cmd_data[6][8] = 0x00;
		cmd_data[6][9] = 0x00;
	}

	for (i = 0; i < ADJUST_PANEL_TE_SCANLINE_CMD_COUNT; i++) {
		desc[i].msg.type = cmd_data[i][0];
		desc[i].last_command = (cmd_data[i][3] ? false:true);
		desc[i].msg.channel = cmd_data[i][2];
		desc[i].post_wait_ms = cmd_data[i][4];
		desc[i].msg.tx_buf = &cmd_data[i][7];
		desc[i].msg.tx_len = cmd_data[i][6];
	}

	if (iris_get_abyp_mode() == PASS_THROUGH_MODE)
		rc = iris_pt_send_panel_cmd(&cmdset);
	else
		rc = iris_abyp_send_panel_cmd(&cmdset);

	if (rc)
		IRIS_LOGE("[%s] enable: %d failed\n", __func__, enable);
	else
		IRIS_LOGI("[%s] enable: %d successfully\n", __func__, enable);
#endif
}

bool iris_is_same_timing_from_last_pt(void)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if (!_iris_support_timing_switch())
		return true;

	return pcfg_ts->new_tm_index == pcfg_ts->last_pt_tm_index;
}


bool iris_is_res_switched_from_last_pt(void)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();
	uint32_t new_tm_index = pcfg_ts->new_tm_index;
	uint32_t last_pt_tm_index = pcfg_ts->last_pt_tm_index;

	if (!_iris_support_timing_switch())
		return false;

	if (new_tm_index == last_pt_tm_index)
		return false;

	if ((pcfg_ts->panel_tm_arry[new_tm_index].h_active
				!= pcfg_ts->panel_tm_arry[last_pt_tm_index].h_active)
			|| (pcfg_ts->panel_tm_arry[new_tm_index].v_active
				!= pcfg_ts->panel_tm_arry[last_pt_tm_index].v_active))
		return true;

	return false;
}

bool iris_is_freq_switched_from_last_pt(void)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();
	uint32_t new_tm_index = pcfg_ts->new_tm_index;
	uint32_t last_pt_tm_index = pcfg_ts->last_pt_tm_index;

	if (!_iris_support_timing_switch())
		return false;

	if (new_tm_index == last_pt_tm_index)
		return false;

	if (pcfg_ts->panel_tm_arry[new_tm_index].refresh_rate
			!= pcfg_ts->panel_tm_arry[last_pt_tm_index].refresh_rate)
		return true;

	return false;
}

bool iris_is_clk_switched_from_last_pt(void)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();
	uint32_t new_tm_index = pcfg_ts->new_tm_index;
	uint32_t last_pt_tm_index = pcfg_ts->last_pt_tm_index;

	if (!_iris_support_timing_switch())
		return false;

	if (new_tm_index == last_pt_tm_index)
		return false;

	if (pcfg_ts->panel_tm_arry[new_tm_index].clk_rate_hz
			!= pcfg_ts->panel_tm_arry[last_pt_tm_index].clk_rate_hz)
		return true;

	return false;
}

void iris_send_timing_switch_pkt(void)
{

	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();
	struct iris_ctrl_seq *pseq = &pcfg_ts->tm_switch_seq[TIMING_SWITCH_NORMAL_SEQ];
	struct iris_ctrl_opt *arr = NULL;

	IRIS_ATRACE_BEGIN("iris_send_timing_switch_pkt");
	IRIS_LOGI("%s(), cmd list index: %02x", __func__, pcfg_ts->cmd_list_index);

	if (pseq == NULL) {
		IRIS_LOGE("%s(), seq is NULL", __func__);
		IRIS_ATRACE_END("iris_send_timing_switch_pkt");
		return;
	}
	arr = pseq->ctrl_opt;

	iris_send_assembled_pkt(arr, pseq->cnt);
	udelay(100);
	IRIS_ATRACE_END("iris_send_timing_switch_pkt");
}

void iris_timing_switch_setup(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->pw_chip_func_ops.iris_timing_switch_setup_)
		pcfg->pw_chip_func_ops.iris_timing_switch_setup_(&timing_switch_ops);
}

static bool _iris_gen_rfb_cmd_list(uint32_t fps)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();

	if (pcfg_ts->rfb_tm_num == 0 || pcfg_ts->rfb_timing_arry == NULL) {
		IRIS_LOGI("%s(), doesn't support RFB switch.", __func__);

		return false;
	}

	{
		struct iris_cfg *pcfg = iris_get_cfg();
		int32_t i = 0;

		for (i = 0; i < pcfg_ts->rfb_tm_num; i++) {
			if (pcfg_ts->rfb_timing_arry[i].in_fps == pcfg->ap_te &&
				pcfg_ts->rfb_timing_arry[i].out_fps == fps) {
				pcfg_ts->cmd_list_index = pcfg_ts->rfb_timing_arry[i].cmd_list_idx;
				break;
			}
		}

		IRIS_LOGI("%s(), cmd list index: %u", __func__, pcfg_ts->cmd_list_index);
	}

	return true;
}

static void _iris_send_special_switch_pkt(uint32_t fps)
{
	struct iris_cfg_ts *pcfg_ts = _iris_get_ts_cfg();
	struct iris_ctrl_seq *pseq = &pcfg_ts->tm_switch_seq[TIMING_SWITCH_SPECIAL_SEQ];
	struct iris_ctrl_opt *arr = NULL;

	IRIS_LOGI("%s(), cmd list index: %u, for %u Hz", __func__, pcfg_ts->cmd_list_index, fps);

	if (pseq == NULL) {
		IRIS_LOGE("%s(), invalid configuration, seq is NULL", __func__);
		return;
	}
	arr = pseq->ctrl_opt;

	iris_send_assembled_pkt(arr, pseq->cnt);
	udelay(100);
}

static void _iris_send_panel_switch_cmd(uint32_t fps)
{
	IRIS_LOGD("%s(), switch panel to %u Hz.", __func__, fps);
}

void iris_set_special_mode(uint32_t count, uint32_t *values)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (count < 2 || values == NULL) {
		IRIS_LOGE("%s(), invalid input parameter.", __func__);
		return;
	}

	switch (values[0]) {
	case RFB_MODE: /* irisConfig 205 2 1 fps */
		if (pcfg->panel_te == values[1]) {
			IRIS_LOGI("%s(), same output fps: %u, do nothing.", __func__, values[1]);
			return;
		}
		if (_iris_gen_rfb_cmd_list(values[1])) {
			_iris_send_special_switch_pkt(values[1]);
			_iris_send_panel_switch_cmd(values[1]);
			pcfg->panel_te = values[1];
		}
		break;
	default:
		IRIS_LOGE("%s(), invalid parameter.", __func__);
		break;
	}
}
