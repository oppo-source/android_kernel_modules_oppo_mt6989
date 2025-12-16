// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2023.
 */
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <drm/drm_mipi_dsi.h>
#include <linux/vmalloc.h>
#include "pw_iris_lightup.h"
#include "pw_iris_lut.h"
#include "pw_iris_pq.h"
#include "pw_iris_log.h"
#include "pw_iris_api.h"

u8 fw_loaded_status = FIRMWARE_LOAD_FAIL;
u16 fw_calibrated_status;
u16 gamma_apl_status;
static u8 payload_size;
static uint32_t lut_lut2[LUT_LEN] = {};
static uint32_t LUT2_fw[LUT_LEN+LUT_LEN+LUT_LEN] = {};

static struct msmfb_iris_ambient_info iris_ambient_lut;
static struct msmfb_iris_maxcll_info iris_maxcll_lut;
static uint32_t lut_luty[LUT_LEN] = {};
static uint32_t lut_lutuv[LUT_LEN] = {};
static uint32_t LUTUVY_fw[LUT_LEN+LUT_LEN+LUT_LEN+LUT_LEN+LUT_LEN+LUT_LEN] = {};
struct lut_node iris_lut_param;
/* SDR2HDR_UVYGAIN_BLOCK_CNT > SDR2HDR_LUT2_BLOCK_CNT */
struct iris_cmd_desc *dynamic_lut_send_cmd;

struct iris_cmd_desc *dynamic_lutuvy_send_cmd;

u8 *iris_ambient_lut_buf;
u8 *iris_maxcll_lut_buf;
static DEFINE_MUTEX(fw_status_lock);

u8 iris_get_fw_load_status(void)
{
	u8 status = 0;

	mutex_lock(&fw_status_lock);
	status = fw_loaded_status;
	mutex_unlock(&fw_status_lock);

	return status;
}

void iris_update_fw_load_status(u8 value)
{
	mutex_lock(&fw_status_lock);
	fw_loaded_status = value;
	mutex_unlock(&fw_status_lock);
}

struct msmfb_iris_ambient_info *iris_get_ambient_lut(void)
{
	return &iris_ambient_lut;
}

struct msmfb_iris_maxcll_info *iris_get_maxcll_info(void)
{
	return &iris_maxcll_lut;
}

static void _iris_init_ambient_lut(void)
{
	iris_ambient_lut.ambient_lux = 0;
	iris_ambient_lut.ambient_bl_ratio = 0;
	iris_ambient_lut.lut_lut2_payload = &lut_lut2;

	if (iris_ambient_lut_buf != NULL) {
		vfree(iris_ambient_lut_buf);
		iris_ambient_lut_buf = NULL;
	}
}

static void _iris_init_maxcll_lut(void)
{
	iris_maxcll_lut.mMAXCLL = 2200;
	iris_maxcll_lut.lut_luty_payload = &lut_luty;
	iris_maxcll_lut.lut_lutuv_payload = &lut_lutuv;

	if (iris_maxcll_lut_buf != NULL) {
		vfree(iris_maxcll_lut_buf);
		iris_maxcll_lut_buf = NULL;
	}
}

void iris_init_lut_buf(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	payload_size = pcfg->split_pkt_size;
	memset(&iris_lut_param, 0x00, sizeof(iris_lut_param));

	/* for HDR ambient light */
	_iris_init_ambient_lut();

	/* for HDR maxcll */
	_iris_init_maxcll_lut();

	if (pcfg->pw_chip_func_ops.iris_init_tm_points_lut_)
		pcfg->pw_chip_func_ops.iris_init_tm_points_lut_();
}

u16 iris_get_firmware_aplstatus_value(void)
{
	return gamma_apl_status;
}

int32_t iris_request_firmware(const struct firmware **fw,
		const uint8_t *name)
{
	int32_t rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct device *dev = pcfg->dev;

	if (name == NULL) {
		IRIS_LOGE("%s(), firmware is null", __func__);
		return -EINVAL;
	}

	if (dev == NULL) {
		IRIS_LOGE("%s(), dev is null", __func__);
		return -EINVAL;
	}

	rc = request_firmware(fw, name, dev);
	if (rc) {
		IRIS_LOGE("%s(), failed to load firmware: %s, return: %d",
				__func__, name, rc);
		return rc;
	}

	IRIS_LOGI("%s(), load firmware [Success], name: %s, size: %zu bytes",
			__func__, name, (*fw)->size);

	return rc;
}

void iris_release_firmware(const struct firmware **fw)
{
	if (*fw) {
		release_firmware(*fw);
		*fw = NULL;
	}
}

int iris_change_lut_type_addr(
		struct iris_ip_opt *dest, struct iris_ip_opt *src)
{
	int rc = -EINVAL;
	struct iris_cmd_desc *desc = NULL;

	if (!src || !dest) {
		IRIS_LOGE("%s(), src or dest is null", __func__);
		return rc;
	}

	desc = src->cmd;
	if (!desc) {
		IRIS_LOGE("%s(), invalid desc.", __func__);
		return rc;
	}

	IRIS_LOGD("%s(), desc len: %zu", __func__, desc->msg.tx_len);
	iris_change_type_addr(dest, src);

	return 0;
}

static int _iris_change_gamma_type_addr(void)
{
	int i = 0;
	int rc = -EINVAL;
	u8 ip = IRIS_IP_DPP;
	u8 opt_id = 0xFE;
	struct iris_ip_opt *lut_popt = NULL;
	struct iris_ip_opt *popt = NULL;
	uint8_t get_optid = 0;
	uint8_t lutip = 0;
	int32_t type = 0;
	struct iris_ip_index *pip_index = NULL;
	bool bApl = 0;
	uint8_t level = 0x0;
	IRIS_LOGD("%s(%d)", __func__, __LINE__);
	popt = iris_find_ip_opt(ip, opt_id);
	if (!popt) {
		IRIS_LOGE("%s(%d), can't find valid option for i_p: %#x, opt: %#x",
			__func__, __LINE__, ip, opt_id);
		return rc;
	}

	gamma_apl_status = 0;
	type = IRIS_LUT_PIP_IDX;
	lutip = GAMMA_LUT - LUT_IP_START;

	pip_index = iris_get_ip_idx(type) + lutip;

	for (i = 0; i < pip_index->opt_cnt; i++) {
		lut_popt = pip_index->opt + i;
		rc = iris_change_lut_type_addr(lut_popt, popt);
		get_optid = lut_popt->opt_id;
		level = get_optid & 0xf;
		bApl = (get_optid & 0x80) ? 1 : 0;
		if (bApl)
			gamma_apl_status |= (0x1 << level);

	}

	return rc;
}

int iris_send_lut_for_dma(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int rc = 0;

	/*register level*/
	if (rc)
		IRIS_LOGE("%s(%d), failed to change dbc type", __func__, __LINE__);
	rc = _iris_change_gamma_type_addr();
	if (rc)
		IRIS_LOGE("%s(%d), failed to change gamma type", __func__, __LINE__);

	if (pcfg->iris_chip_type == CHIP_IRIS7) {
		if (pcfg->pw_chip_func_ops.iris_change_dpp_lutrgb_type_addr_i7_)
			rc = pcfg->pw_chip_func_ops.iris_change_dpp_lutrgb_type_addr_i7_();
		if (rc)
			IRIS_LOGE("%s(%d), failed to change dpp pre_lutRGB type", __func__, __LINE__);
	}

	return rc;
}

void iris_parse_misc_info(void)
{
	uint8_t ip, opt;
	uint8_t i, j, v;
	char str[41];
	uint32_t *p = NULL;
	uint8_t *pc = NULL;
	uint32_t len = 0;
	uint8_t Change_Id[21];
	uint32_t pcs_ver;
	uint32_t date;
	uint8_t calibration_status;
	uint16_t panel_nits;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGI("%s(),%d", __func__, __LINE__);
	/* iris7.fw: ip = MISC_INFO_LUT, opt = 0
	 * iris7_ccf1.fw: ip = MISC_INFO_LUT, opt = 1
	 * iris7_ccf2.fw: ip = MISC_INFO_LUT, opt = 2
	 */
	ip = MISC_INFO_LUT;
	opt = 0x1;

	if (iris_find_ip_opt(ip, opt) == NULL) {
		IRIS_LOGE("%s(%d), can not find misc info i_p-opt", __func__, __LINE__);
		return;
	}

	p = iris_get_ipopt_payload_data(ip, opt, 2);
	if (!p) {
		IRIS_LOGE("%s(%d), can not get misc info payload", __func__, __LINE__);
		return;
	}
	pc = (uint8_t *)p;

	len = iris_get_ipopt_payload_len(ip, opt);
	if (len != 40) {
		IRIS_LOGE("%s(%d), invalid payload len %d", __func__, __LINE__, len);
		return;
	}

	for (i = 0; i < 8; i++)
		IRIS_LOGD("p[%d] = 0x%08x", i, p[i]);

	memcpy(Change_Id, pc, 21);
	pcs_ver = pc[21]<<24 | pc[22]<<16 | pc[23]<<8 | pc[24];
	date = pc[25]<<24 | pc[26]<<16 | pc[27]<<8 | pc[28];
	calibration_status = pc[29];
	panel_nits = pc[30]<<8 | pc[31];

	pcfg->panel_nits = panel_nits;

	str[0] = (char)Change_Id[0];
	for (i = 1; i < 21; i++) {
		for (j = 0; j < 2; j++) {
			if (j == 0)
				v = Change_Id[i]/16;
			else
				v = Change_Id[i]%16;
			if (v <= 9)
				str[(i-1)*2+j+1] = v + 48;
			else
				str[(i-1)*2+j+1] = v + 87;
		}
	}

	IRIS_LOGI("Change_Id: %s", str);
	IRIS_LOGI("pcs_ver = %08x", pcs_ver);
	IRIS_LOGI("date = %08x", date);
	IRIS_LOGI("calibration_status = %d", calibration_status);
	IRIS_LOGI("panel_nits = %04x", panel_nits);
}

int iris_parse_lut_cmds(uint32_t flag)
{
	int ret = -1;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->pw_chip_func_ops.iris_parse_lut_cmds_)
		pcfg->pw_chip_func_ops.iris_parse_lut_cmds_(flag);

	return ret;
}

/*add lut cmds to bufs for sending*/
static void _iris_prepare_lut_cmds(struct iris_ip_opt *popt)
{
	int pos = 0;
	struct iris_cfg *pcfg = NULL;
	struct iris_cmd_desc *pdesc = NULL;

	pcfg = iris_get_cfg();

	pdesc = pcfg->iris_cmds.iris_cmds_buf;
	pos = pcfg->iris_cmds.cmds_index;

	IRIS_LOGD("%s(), %p %p len: %d",
			__func__, &pdesc[pos], popt, popt->cmd_cnt);
	memcpy(&pdesc[pos], popt->cmd, sizeof(*pdesc) * popt->cmd_cnt);
	pos += popt->cmd_cnt;
	pcfg->iris_cmds.cmds_index = pos;
}

void iris_fomat_lut_cmds(u8 lut_type, u8 opt_id)
{
	struct iris_ip_opt *popt = NULL;

	popt = iris_find_ip_opt(lut_type, opt_id);
	if (!popt) {
		IRIS_LOGW("%s(%d), invalid opt id: %#x.",
				__func__, __LINE__, opt_id);
		return;
	}
	_iris_prepare_lut_cmds(popt);
}

int iris_send_lut(u8 lut_type, u8 lut_table_index)
{
	int ret = -1;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->pw_chip_func_ops.iris_send_lut_)
		pcfg->pw_chip_func_ops.iris_send_lut_(lut_type, lut_table_index);

	return ret;
}

void iris_update_gamma(void)
{
	if (iris_get_fw_load_status() == FIRMWARE_LOAD_FAIL)
		iris_scaler_gamma_enable(0);
	else {
		iris_scaler_gamma_enable(1);
		iris_update_fw_load_status(FIRMWARE_IN_USING);
	}
}

int iris_dbgfs_fw_calibrate_status_init(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("debugfs_create_dir for iris_debug failed, error %ld",
					PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	debugfs_create_u16("fw_calibrated_status", 0644, pcfg->dbg_root,
			&fw_calibrated_status);

	return 0;
}


void iris_update_ambient_lut(enum LUT_TYPE lutType, u32 lutpos)
{
	u32 len = 0;
	u32 hdr_payload_size = payload_size;
	u32 hdr_pkt_size = hdr_payload_size + DIRECT_BUS_HEADER_SIZE;
	u32 hdr_block_pkt_cnt =
		(SDR2HDR_LUT_BLOCK_SIZE/2 + hdr_payload_size - 1)
		/ hdr_payload_size;
	u32 iris_lut_buf_index, lut_block_index, lut_block_cnt;
	u32 lut_pkt_cmd_index;
	u32 temp_index, index_i;
	u32 dbus_addr_start;
	u32 lut_fw_index;
	u32 cmd_payload_len;
	struct ocp_header ocp_dbus_header;

	memset(&ocp_dbus_header, 0, sizeof(ocp_dbus_header));
	ocp_dbus_header.header = 0x0004000C;
	ocp_dbus_header.address = SDR2HDR_LUT2_ADDRESS;

	if (lutpos == 0xFFE00000)
		hdr_block_pkt_cnt =
			(SDR2HDR_LUT_BLOCK_SIZE + hdr_payload_size - 1)
			/ hdr_payload_size;

	if (lutType != AMBINET_SDR2HDR_LUT) {
		IRIS_LOGE("%s input lutType error %d", __func__, lutType);
		return;
	}

	if (lutpos == 0xFFE00000)
		dbus_addr_start = SDR2HDR_LUT2_ADDRESS;
	else
		dbus_addr_start = SDR2HDR_LUT2_ADDRESS + lutpos * SDR2HDR_LUT_BLOCK_SIZE / 2;
	lut_block_cnt = SDR2HDR_LUT2_BLOCK_CNT;

	// copy lut2 to the firmware format.
	//  lut2 is EVEN+ODD,
	//  LUT2_fw is  EVEN ODD EVEN ODD EVEN ODD
	for (index_i = 0; index_i < LUT_LEN; index_i++) {
		if (lutpos == 0xFFE00000) {
			lut_fw_index = index_i / 2;
			if (index_i % 2 != 0)  // ODD
				lut_fw_index += LUT_LEN / 2;
			LUT2_fw[lut_fw_index] = lut_lut2[index_i];
			LUT2_fw[lut_fw_index + LUT_LEN] = lut_lut2[index_i];
			LUT2_fw[lut_fw_index + LUT_LEN + LUT_LEN] = lut_lut2[index_i];
		} else {
			if (index_i % 2 == 0) {
				lut_fw_index = index_i / 4;
				if (index_i % 4 != 0) /* ODD */
					lut_fw_index += LUT_LEN / 4;
				LUT2_fw[lut_fw_index] = lut_lut2[index_i];
				LUT2_fw[lut_fw_index + LUT_LEN / 2] = lut_lut2[index_i];
				LUT2_fw[lut_fw_index + LUT_LEN / 2 + LUT_LEN / 2] =
					lut_lut2[index_i];
			}
		}
	}

	if (dynamic_lut_send_cmd == NULL) {
		len = sizeof(struct iris_cmd_desc)
			* hdr_pkt_size * hdr_block_pkt_cnt
			* SDR2HDR_LUT2_BLOCK_NUMBER;
		dynamic_lut_send_cmd = vzalloc(len);
		if (dynamic_lut_send_cmd == NULL) {
			IRIS_LOGE("%s(), failed to alloc mem", __func__);
			return;
		}
		iris_lut_param.lut_cmd_cnts_max +=
			hdr_block_pkt_cnt * SDR2HDR_LUT2_BLOCK_NUMBER;
		iris_lut_param.hdr_lut2_pkt_cnt =
			hdr_block_pkt_cnt * SDR2HDR_LUT2_BLOCK_NUMBER;
	}

	if (iris_ambient_lut_buf)
		memset(iris_ambient_lut_buf, 0,
				hdr_pkt_size * iris_lut_param.hdr_lut2_pkt_cnt);

	if (!iris_ambient_lut_buf) {
		len = hdr_pkt_size * iris_lut_param.hdr_lut2_pkt_cnt;
		iris_ambient_lut_buf = vzalloc(len);
	}
	if (!iris_ambient_lut_buf) {
		vfree(dynamic_lut_send_cmd);
		dynamic_lut_send_cmd = NULL;
		return;
	}

	lut_fw_index = 0;
	/*parse LUT2*/
	for (lut_block_index = 0;
			lut_block_index < lut_block_cnt;
			lut_block_index++){

		ocp_dbus_header.address = dbus_addr_start
			+ lut_block_index
			* SDR2HDR_LUT_BLOCK_ADDRESS_INC;

		for (lut_pkt_cmd_index = 0;
				lut_pkt_cmd_index < hdr_block_pkt_cnt;
				lut_pkt_cmd_index++) {

			iris_lut_buf_index =
				lut_block_index * hdr_pkt_size
				* hdr_block_pkt_cnt
				+ lut_pkt_cmd_index * hdr_pkt_size;

			if (lut_pkt_cmd_index == hdr_block_pkt_cnt-1) {
				if (lutpos == 0xFFE00000)
					cmd_payload_len = SDR2HDR_LUT_BLOCK_SIZE
						- (hdr_block_pkt_cnt-1) * hdr_payload_size;
				else
					cmd_payload_len = SDR2HDR_LUT_BLOCK_SIZE/2
						- (hdr_block_pkt_cnt-1) * hdr_payload_size;
			} else
				cmd_payload_len = hdr_payload_size;

			temp_index = lut_pkt_cmd_index
				+ hdr_block_pkt_cnt * lut_block_index;
			dynamic_lut_send_cmd[temp_index].msg.type = 0x29;
			dynamic_lut_send_cmd[temp_index].msg.tx_len =
				cmd_payload_len + DIRECT_BUS_HEADER_SIZE;
			dynamic_lut_send_cmd[temp_index].post_wait_ms = 0;
			dynamic_lut_send_cmd[temp_index].msg.tx_buf =
				iris_ambient_lut_buf + iris_lut_buf_index;

			memcpy(&iris_ambient_lut_buf[iris_lut_buf_index],
					&ocp_dbus_header, DIRECT_BUS_HEADER_SIZE);
			iris_lut_buf_index += DIRECT_BUS_HEADER_SIZE;

			memcpy(&iris_ambient_lut_buf[iris_lut_buf_index],
					&LUT2_fw[lut_fw_index], cmd_payload_len);

			lut_fw_index += cmd_payload_len / 4;
			ocp_dbus_header.address += cmd_payload_len;
		}
	}
}

void iris_update_maxcll_lut(enum LUT_TYPE lutType, u32 lutpos)
{
	u32 hdr_payload_size = payload_size;
	u32 hdr_pkt_size = hdr_payload_size + DIRECT_BUS_HEADER_SIZE;
	u32 hdr_block_pkt_cnt = (SDR2HDR_LUT_BLOCK_SIZE / 2 + hdr_payload_size - 1) / hdr_payload_size;
	u32 iris_lut_buf_index, lut_block_index, lut_block_cnt, lut_pkt_cmd_index;
	u32 temp_index, index_i;
	u32 dbus_addr_start;
	u32 lut_fw_index;
	u32 cmd_payload_len;
	struct ocp_header ocp_dbus_header;

	memset(&ocp_dbus_header, 0, sizeof(ocp_dbus_header));
	ocp_dbus_header.header = 0x0004000C;
	ocp_dbus_header.address = SDR2HDR_LUTUVY_ADDRESS;

	if (lutpos == 0xFFFF0000)
		hdr_block_pkt_cnt = (SDR2HDR_LUT_BLOCK_SIZE + hdr_payload_size - 1) / hdr_payload_size;

	if (lutType != AMBINET_HDR_GAIN) {
		IRIS_LOGE("%s input lutType error %d", __func__, lutType);
		return;
	}

	dbus_addr_start = SDR2HDR_LUTUVY_ADDRESS;
	lut_block_cnt = SDR2HDR_LUTUVY_BLOCK_CNT;

	// copy lutuvy to the firmware format.
	// lutuvy is EVEN+ODD, LUT2_fw is  EVEN ODD EVEN ODD EVEN ODD
	for (index_i = 0; index_i < LUT_LEN; index_i++) {
		if (lutpos == 0xFFFF0000) {
			lut_fw_index = index_i / 2;
			if (index_i % 2 == 0) // ODD
				lut_fw_index += LUT_LEN / 2;
			LUTUVY_fw[lut_fw_index] = lut_lutuv[index_i];
			LUTUVY_fw[lut_fw_index + LUT_LEN] = lut_lutuv[index_i];
			LUTUVY_fw[lut_fw_index + 2 * LUT_LEN] = lut_lutuv[index_i];
			LUTUVY_fw[lut_fw_index + 3 * LUT_LEN] = lut_lutuv[index_i];
			LUTUVY_fw[lut_fw_index + 4 * LUT_LEN] = lut_luty[index_i];
			LUTUVY_fw[lut_fw_index + 5 * LUT_LEN] = lut_luty[index_i];
		} else {
			if (index_i % 2 == 0) {
				lut_fw_index = index_i / 4;
				if (index_i % 4 != 0) // ODD
					lut_fw_index += LUT_LEN / 4;
				LUTUVY_fw[lut_fw_index] = lut_lutuv[index_i];
				LUTUVY_fw[lut_fw_index + LUT_LEN / 2] = lut_lutuv[index_i];
				LUTUVY_fw[lut_fw_index + LUT_LEN] = lut_lutuv[index_i];
				LUTUVY_fw[lut_fw_index + 3 * LUT_LEN / 2] = lut_lutuv[index_i];
				LUTUVY_fw[lut_fw_index + 2 * LUT_LEN] = lut_luty[index_i];
				LUTUVY_fw[lut_fw_index + 5 * LUT_LEN / 2] = lut_luty[index_i];
			}
		}
	}

	if (dynamic_lutuvy_send_cmd == NULL) {
		dynamic_lutuvy_send_cmd = vzalloc(sizeof(struct iris_cmd_desc)
					* hdr_pkt_size * hdr_block_pkt_cnt * SDR2HDR_LUTUVY_BLOCK_NUMBER);
		if (dynamic_lutuvy_send_cmd == NULL) {
			IRIS_LOGE("%s: failed to alloc mem", __func__);
			return;
		}
		iris_lut_param.lut_cmd_cnts_max += hdr_block_pkt_cnt * SDR2HDR_LUTUVY_BLOCK_NUMBER;
		iris_lut_param.hdr_lutuvy_pkt_cnt = hdr_block_pkt_cnt * SDR2HDR_LUTUVY_BLOCK_NUMBER;
	}

	if (iris_maxcll_lut_buf)
		memset(iris_maxcll_lut_buf, 0, hdr_pkt_size * iris_lut_param.hdr_lutuvy_pkt_cnt);

	if (!iris_maxcll_lut_buf)
		iris_maxcll_lut_buf = vzalloc(hdr_pkt_size * iris_lut_param.hdr_lutuvy_pkt_cnt);

	if (!iris_maxcll_lut_buf) {
		vfree(dynamic_lutuvy_send_cmd);
		dynamic_lutuvy_send_cmd = NULL;
		IRIS_LOGE("%s: failed to alloc mem", __func__);
		return;
	}

	lut_fw_index = 0;
	//parse LUTUVY
	for (lut_block_index = 0; lut_block_index < lut_block_cnt; lut_block_index++) {
		ocp_dbus_header.address = dbus_addr_start + lut_block_index*SDR2HDR_LUT_BLOCK_ADDRESS_INC;
		if (lutpos != 0xFFFF0000)
			ocp_dbus_header.address += lutpos * SDR2HDR_LUT_BLOCK_SIZE/2;
		for (lut_pkt_cmd_index = 0; lut_pkt_cmd_index < hdr_block_pkt_cnt; lut_pkt_cmd_index++) {
			iris_lut_buf_index = lut_block_index*hdr_pkt_size*hdr_block_pkt_cnt
						+ lut_pkt_cmd_index*hdr_pkt_size;
			if (lut_pkt_cmd_index == hdr_block_pkt_cnt-1) {
				cmd_payload_len = SDR2HDR_LUT_BLOCK_SIZE/2 - (hdr_block_pkt_cnt-1) * hdr_payload_size;
				if (lutpos == 0xFFFF0000)
					cmd_payload_len += SDR2HDR_LUT_BLOCK_SIZE/2;
			} else
				cmd_payload_len = hdr_payload_size;

			temp_index = lut_pkt_cmd_index + hdr_block_pkt_cnt * lut_block_index;
			dynamic_lutuvy_send_cmd[temp_index].msg.type = 0x29;
			dynamic_lutuvy_send_cmd[temp_index].msg.tx_len = cmd_payload_len + DIRECT_BUS_HEADER_SIZE;
			dynamic_lutuvy_send_cmd[temp_index].post_wait_ms = 0;
			dynamic_lutuvy_send_cmd[temp_index].msg.tx_buf = iris_maxcll_lut_buf + iris_lut_buf_index;

			memcpy(&iris_maxcll_lut_buf[iris_lut_buf_index], &ocp_dbus_header, DIRECT_BUS_HEADER_SIZE);
			iris_lut_buf_index += DIRECT_BUS_HEADER_SIZE;

			memcpy(&iris_maxcll_lut_buf[iris_lut_buf_index], &LUTUVY_fw[lut_fw_index], cmd_payload_len);
			lut_fw_index += cmd_payload_len/4;
			ocp_dbus_header.address += cmd_payload_len;
		}
	}
}