// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#include <linux/vmalloc.h>
#include <linux/mm.h>
//#include <linux/slab.h>
//#include <linux/delay.h>
#include <linux/uaccess.h>
//#include <drm/drm_mipi_dsi.h>
//#include <video/mipi_display.h>

#include "pw_iris_api.h"
#include "pw_iris_lightup.h"
#include "pw_iris_lightup_ocp.h"
//#include "pw_iris_lp.h"
#include "pw_iris_lut.h"
//#include "pw_iris_pq.h"
#include "pw_iris_ioctl.h"
//#include "pw_iris_i3c.h"
//#include "dsi_iris_loop_back.h"
#include "pw_iris_log.h"
#include "pw_iris_api.h"
//#include "pw_iris_memc.h"
//#include "dsi_iris_memc_helper.h"
//#include "pw_iris_timing_switch.h"
#include "pw_iris_lp.h"

void iris_set_brightness_v2(int level, int delay, int wait_vsync_count)
{
	unsigned long flags;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGI("%s level %d delay %d wait_vsync_count %d\n", __func__, level, delay, wait_vsync_count);

	spin_lock_irqsave(&pcfg->backlight_v2.bl_spinlock, flags);
	pcfg->backlight_v2.level = level;
	pcfg->backlight_v2.delay = delay;
	pcfg->backlight_v2.wait_vsync_count = wait_vsync_count;
	spin_unlock_irqrestore(&pcfg->backlight_v2.bl_spinlock, flags);
}
EXPORT_SYMBOL(iris_set_brightness_v2);

static int mdss_mipi_dsi_command(void __user *values)
{
	struct msmfb_mipi_dsi_cmd cmd;
	struct iris_cmd_desc desc = { { 0 } };
	struct iris_cmd_desc *pdesc_multi = NULL;
	struct iris_cmd_desc *pdesc;
	struct iris_cmd_set cmdset = {
		.count = 1,
		.cmds = &desc
	};
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_ocp_dsi_tool_input iris_ocp_input = {0, 0, 0, 0, 0};
	char *pcmd_indx;
	int ret, indx, cmd_len, cmd_cnt;

	ret = copy_from_user(&cmd, values, sizeof(cmd));
	if (ret) {
		IRIS_LOGE("failed to copy from user");
		return -EPERM;
	}
	if (cmd.rx_length > 0 && cmd.rx_length < SZ_4K) {
		desc.msg.rx_buf = vzalloc(cmd.rx_length);
		if (!desc.msg.rx_buf)
			return -ENOMEM;
		desc.msg.rx_len = cmd.rx_length;
	}
	if (cmd.length > 0 && cmd.length < SZ_4K && cmd.payload) {
		desc.msg.tx_buf = vmalloc(cmd.length);
		if (!desc.msg.tx_buf) {
			ret = -ENOMEM;
			goto err;
		}
		desc.msg.tx_len = cmd.length;
		ret = copy_from_user((char *)desc.msg.tx_buf, cmd.payload, cmd.length);
		if (ret) {
			ret = -EFAULT;
			goto err;
		}
	}

	IRIS_LOGI("#### %s:%d vc=%u d=%02x f=%u l=%u", __func__, __LINE__,
			cmd.vc, cmd.dtype, cmd.flags, cmd.length);

	IRIS_LOGI("#### %s:%d %x, %x, %x", __func__, __LINE__,
			cmd.iris_ocp_type, cmd.iris_ocp_addr, cmd.iris_ocp_size);

	desc.msg.type = cmd.dtype;
	desc.msg.channel = cmd.vc;
	desc.last_command = (cmd.flags & MSMFB_MIPI_DSI_COMMAND_LAST) > 0;
	desc.post_wait_ms = 0;

	if (pcfg->iris_set_msg_ctrl)
		pcfg->iris_set_msg_ctrl(&desc);

	if (cmd.dtype == 0x0f) {
		if (!desc.msg.tx_buf) {
			ret = -ENOMEM;
			goto err;
		}

		cmd_cnt = *((u8 *)desc.msg.tx_buf);
		if (cmd_cnt > 256) {
			ret = -EINVAL;
			goto err;
		}
		pdesc_multi = vmalloc(sizeof(struct iris_cmd_desc) * cmd_cnt);
		pcmd_indx = (char *)desc.msg.tx_buf + cmd_cnt + 1;
		for (indx = 0; indx < cmd_cnt; indx++) {
			pdesc = pdesc_multi + indx;
			cmd_len = *((char *)desc.msg.tx_buf + 1 + indx);
			pdesc->msg.type = *pcmd_indx;
			pdesc->msg.channel = 0;
			pdesc->last_command = false;
			pdesc->msg.flags |= 0;
			pdesc->msg.tx_len = cmd_len - 1;
			pdesc->post_wait_ms = 0;
			pdesc->msg.tx_buf = pcmd_indx + 1;

			pcmd_indx += cmd_len;
			if (indx == (cmd_cnt - 1))
				pdesc->last_command = true;
			IRIS_LOGI("dtype:%x, dlen: %zu, last: %d",
					pdesc->msg.type,
					pdesc->msg.tx_len,
					pdesc->last_command);
		}
		cmdset.cmds = pdesc_multi;
		cmdset.count = cmd_cnt;
	}

	if ((cmd.flags & MSMFB_MIPI_DSI_COMMAND_ACK) && pcfg->iris_set_msg_flags)
		pcfg->iris_set_msg_flags(&desc, READ_FLAG);

	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_HS)
		cmdset.state = IRIS_CMD_SET_STATE_HS;
	if (pcfg->lightup_ops.acquire_panel_lock)
		pcfg->lightup_ops.acquire_panel_lock();

	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_TO_PANEL) {
		if (iris_get_abyp_mode() == PASS_THROUGH_MODE)
			iris_pt_send_panel_cmd(&cmdset);
		else
			iris_abyp_send_panel_cmd(&cmdset);
	} else if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_T) {
		u32 pktCnt = (cmd.iris_ocp_type >> 8) & 0xFF;

		//only test LUT send command
		if ((cmd.iris_ocp_type & 0xF) == PXLW_DIRECTBUS_WRITE) {
			u8 lut_type = (cmd.iris_ocp_type >> 8) & 0xFF;
			u8 lut_index = (cmd.iris_ocp_type >> 16) & 0xFF;
			u8 lut_parse = (cmd.iris_ocp_type >> 24) & 0xFF;

			if (lut_parse) // only parse firmware when value is not zero
				iris_parse_lut_cmds(LOAD_CALIBRATED_OR_GOLDEN);
			if (pcfg->iris_chip_type == CHIP_IRIS5) {
				if (pcfg->pw_chip_func_ops.iris_send_lut_i5_)
					pcfg->pw_chip_func_ops.iris_send_lut_i5_(lut_type,
							lut_index, cmd.iris_ocp_addr);
			}
			else
				iris_send_lut(lut_type, lut_index);
		} else { // test ocp write
			if (pktCnt > DSI_CMD_CNT)
				pktCnt = DSI_CMD_CNT;

			if (cmd.iris_ocp_size < OCP_MIN_LEN)
				cmd.iris_ocp_size = OCP_MIN_LEN;

			if (cmd.iris_ocp_size > SZ_4K)
				cmd.iris_ocp_size = SZ_4K;

			iris_ocp_input.iris_ocp_type = cmd.iris_ocp_type & 0xF;
			iris_ocp_input.iris_ocp_cnt = pktCnt;
			iris_ocp_input.iris_ocp_addr = cmd.iris_ocp_addr;
			iris_ocp_input.iris_ocp_value = cmd.iris_ocp_value;
			iris_ocp_input.iris_ocp_size = cmd.iris_ocp_size;

			if (pktCnt)
				iris_write_test_muti_pkt(&iris_ocp_input);
			else
				iris_write_test(cmd.iris_ocp_addr, cmd.iris_ocp_type & 0xF, cmd.iris_ocp_size);
			//iris_ocp_bitmask_write(ctrl,cmd.iris_ocp_addr,cmd.iris_ocp_size,cmd.iris_ocp_value);
		}
	} else
		iris_abyp_send_panel_cmd(&cmdset);
	if (pcfg->lightup_ops.release_panel_lock)
		pcfg->lightup_ops.release_panel_lock();

	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_ACK) {
		ret = copy_to_user(cmd.rx_buf, desc.msg.rx_buf, desc.msg.rx_len);
		if (ret) {
			ret = -EFAULT;
			goto err;
		}
	}
	ret = copy_to_user(values, &cmd, sizeof(cmd));
	if (ret)
		ret = -EFAULT;
err:
	IRIS_LOGI("#### %s:%d ret: %d", __func__, __LINE__, ret);
	if (desc.msg.tx_buf)
		vfree(desc.msg.tx_buf);
	if (desc.msg.rx_buf)
		vfree(desc.msg.rx_buf);
	if (pdesc_multi)
		vfree(pdesc_multi);
	return ret;
}

int iris_operate_tool(struct msm_iris_operate_value *argp)
{
	int ret = -1;
	uint32_t parent_type = 0;
	struct iris_cfg *pcfg = NULL;

	// FIXME: copy_from_user() is failed.
	// ret = copy_from_user(&configure, argp, sizeof(configure));
	// if (ret) {
	//	pr_err("1st %s type = %d, value = %d\n",
	//		__func__, configure.type, configure.count);
	//	return -EPERM;
	// }
	IRIS_LOGI("%s type = %d, value = %d", __func__, argp->type, argp->count);

	pcfg = iris_get_cfg();
	if (pcfg == NULL || pcfg->valid < PARAM_PARSED) {
		IRIS_LOGE("Target display does not exist!");
		return -EPERM;
	}

	parent_type = argp->type & 0xff;
	switch (parent_type) {
	case IRIS_OPRT_TOOL_DSI:
		ret = mdss_mipi_dsi_command(argp->values);
		break;
	default:
		IRIS_LOGE("could not find right opertat type = %d", argp->type);
		ret = -EINVAL;
		break;
	}
	return ret;
}

void iris_ioctl_lock(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGD("%s()", __func__);
	mutex_lock(&pcfg->ioctl_mutex);
}

void iris_ioctl_unlock(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	mutex_unlock(&pcfg->ioctl_mutex);
	IRIS_LOGD("%s()", __func__);
}

int iris_configure_t(uint32_t display, u32 type, void __user *argp)
{
	int ret = -1;
	uint32_t value = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	ret = copy_from_user(&value, argp, sizeof(uint32_t));
	if (ret) {
		IRIS_LOGE("can not copy from user");
		return -EPERM;
	}

	if (pcfg->pw_chip_func_ops.iris_configure_)
		ret = pcfg->pw_chip_func_ops.iris_configure_(display, type, value);

	return ret;
}

static int iris_configure_ex_t(uint32_t display, uint32_t type,
		uint32_t count, void __user *values)
{
	int ret = -1;
	uint32_t *val = NULL;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	val = vmalloc(sizeof(uint32_t) * count);
	if (!val) {
		IRIS_LOGE("can not vmalloc space");
		return -ENOSPC;
	}
	ret = copy_from_user(val, values, sizeof(uint32_t) * count);
	if (ret) {
		IRIS_LOGE("can not copy from user");
		vfree(val);
		return -EPERM;
	}
	if (pcfg->pw_chip_func_ops.iris_configure_ex_)
		ret = pcfg->pw_chip_func_ops.iris_configure_ex_(display, type, count, val);

	vfree(val);
	return ret;
}

int iris_configure_get_t(uint32_t display, uint32_t type,
		uint32_t count, void __user *values)
{
	int ret = -1;
	uint32_t *val = NULL;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	val = vmalloc(count * sizeof(uint32_t));
	if (val == NULL) {
		IRIS_LOGE("could not vmalloc space for func = %s", __func__);
		return -ENOSPC;
	}
	ret = copy_from_user(val, values, sizeof(uint32_t) * count);
	if (ret) {
		IRIS_LOGE("can not copy from user");
		vfree(val);
		return -EPERM;
	}
	if (pcfg->pw_chip_func_ops.iris_configure_get_)
		ret = pcfg->pw_chip_func_ops.iris_configure_get_(display, type, count, val);
	if (ret) {
		IRIS_LOGE("get error");
		vfree(val);
		return ret;
	}
	ret = copy_to_user(values, val, sizeof(uint32_t) * count);
	if (ret) {
		IRIS_LOGE("copy to user error");
		vfree(val);
		return -EPERM;
	}
	vfree(val);
	return ret;
}

int iris_operate_conf(struct msm_iris_operate_value *argp)
{
	int ret = -1;
	uint32_t parent_type = 0;
	uint32_t child_type = 0;
	uint32_t display_type = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGD("%s type=0x%04x abyp:%d", __func__, argp->type,
			pcfg->abyp_ctrl.abypass_mode);

	parent_type = argp->type & 0xff;
	child_type = (argp->type >> 8) & 0xff;
	// always set to 0 when only one iris_cfg
	display_type = 0; //(argp->type >> 16) & 0xff;
	if (pcfg->valid < PARAM_PARSED) {
		if (child_type == IRIS_WAIT_VSYNC ||
				child_type == IRIS_CHIP_VERSION || child_type == IRIS_WORK_MODE) {
			IRIS_LOGI("%s allow type: 0x%04x(%u) for Soft Iris", __func__, child_type, child_type);
		} else {
			IRIS_LOGE("Target display does not exist!");
			return -EPERM;
		}
	}

	switch (parent_type) {
	case IRIS_OPRT_CONFIGURE:
		ret = iris_configure_t(display_type, child_type, argp->values);
		break;
	case IRIS_OPRT_CONFIGURE_NEW:
		ret = iris_configure_ex_t(display_type, child_type, argp->count, argp->values);
		break;
	case IRIS_OPRT_CONFIGURE_NEW_GET:
		ret = iris_configure_get_t(display_type, child_type, argp->count, argp->values);
		break;
	default:
		IRIS_LOGE("could not find right operate type = %d", argp->type);
		break;
	}

	return ret;
}

int iris_dbgfs_adb_type_init(void *display)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->pw_chip_func_ops.iris_dbgfs_adb_type_init_)
		pcfg->pw_chip_func_ops.iris_dbgfs_adb_type_init_(display);

	return 0;
}

/* Iris log level definition, for 'dsi_iris7_log.h' */
static int iris_log_level = 2;

void iris_set_loglevel(int level)
{
	iris_log_level = level;
}

inline int iris_get_loglevel(void)
{
	return iris_log_level;
}

static int iris_trace_en;
void iris_set_trace_en(int trace_en)
{
	iris_trace_en = trace_en;
}

inline int iris_get_trace_en(void)
{
	return iris_trace_en;
}

static bool iris_special_config(u32 type)
{
	switch (type) {
	case IRIS_OSD_ENABLE:
	case IRIS_OSD_AUTOREFRESH:
	case IRIS_OSD_OVERFLOW_ST:
	case IRIS_DBG_KERNEL_LOG_LEVEL:
	case IRIS_DBG_TIMING_SWITCH_LEVEL:
	case USER_DEMO_WND:
	case IRIS_MEMC_LEVEL:
	case IRIS_WAIT_VSYNC:
	case IRIS_BRIGHTNESS_V2_INFO:
	case IRIS_CHIP_VERSION:
	case IRIS_FW_UPDATE:
	case IRIS_DEBUG_SET:
	case IRIS_LOOP_BACK_MODE:
	case IRIS_FRC_PQ_LEVEL:
	case IRIS_SET_DPP_APL_ABS:
	case IRIS_SET_DPP_APL_RES:
	case IRIS_ENABLE_DPP_APL:
	case IRIS_CLEAR_FRC_MIF_INT:
	case IRIS_PT_SR_LUT_SET:
		return true;
	default:
		break;
	}

	return false;
}

bool iris_is_valid_type(u32 display, u32 type)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (type >= IRIS_CONFIG_TYPE_MAX)
		return false;

	if (!iris_special_config(type)
			&& type != IRIS_ANALOG_BYPASS_MODE
			&& pcfg->abyp_ctrl.abypass_mode == ANALOG_BYPASS_MODE)
		return false;

	if (type != IRIS_DBG_KERNEL_LOG_LEVEL
			&& pcfg->chip_ver == IRIS3_CHIP_VERSION)
		return false;

	return true;
}
