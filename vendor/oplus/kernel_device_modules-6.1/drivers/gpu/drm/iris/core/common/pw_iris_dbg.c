#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/slab.h>
//#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/kobject.h>
#include "pw_iris_def.h"
#include "pw_iris_log.h"
#include "pw_iris_lightup.h"
#include "pw_iris_timing_switch.h"
#include "pw_iris_pq.h"
#include "pw_iris_memc_helper.h"
#include "pw_iris_api.h"

#define IRIS_DBG_FUNCSTATUS_FILE "iris_func_status"
static uint32_t iris_enable_dsi_cmd_log;

void iris_set_dsi_cmd_log(uint32_t val)
{
	IRIS_LOGI("%s(), set value: %u", __func__, val);
	iris_enable_dsi_cmd_log = val;
}

uint32_t iris_get_dsi_cmd_log(void)
{
	IRIS_LOGI("%s(), get value: %u", __func__, iris_enable_dsi_cmd_log);
	return iris_enable_dsi_cmd_log;
}


int iris_dbg_fstatus_open(struct inode *inode, struct file *file)
{
	if (inode->i_private)
		file->private_data = inode->i_private;
	return 0;
}

/**
 * read module's status
 */
static ssize_t iris_dbg_fstatus_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	char *kbuf = NULL;
	int size = count < PAGE_SIZE ? PAGE_SIZE : (int)count;
	int len = 0;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (*ppos)
		return 0;

	kbuf = vzalloc(size);
	if (kbuf == NULL) {
		IRIS_LOGE("Fatal erorr: No mem!\n");
		return -ENOMEM;
	}

	len += snprintf(kbuf+len, size - len,
			"***system_brightness***\n"
			"%-20s:\t%d\n",
			"system_brightness",
			pqlt_cur_setting->system_brightness);

	len += snprintf(kbuf+len, size - len,
			"***dspp_dirty***\n"
			"%-20s:\t%d\n",
			"dspp_dirty", pqlt_cur_setting->dspp_dirty);


	len += snprintf(kbuf+len, size - len,
			"***cm_setting***\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n",
			"cmcolortempmode",
			pqlt_cur_setting->pq_setting.cmcolortempmode,
			"colortempvalue", pqlt_cur_setting->colortempvalue,
			"min_colortempvalue",
			pqlt_cur_setting->min_colortempvalue,
			"max_colortempvalue",
			pqlt_cur_setting->max_colortempvalue,
			"cmcolorgamut",
			pqlt_cur_setting->pq_setting.cmcolorgamut,
			"demomode", pqlt_cur_setting->pq_setting.demomode,
			"source_switch", pqlt_cur_setting->source_switch);

	len += snprintf(kbuf+len, size - len,
			"***lux_value***\n"
			"%-20s:\t%d\n",
			"luxvalue", pqlt_cur_setting->luxvalue);

	len += snprintf(kbuf+len, size - len,
			"***cct_value***\n"
			"%-20s:\t%d\n",
			"cctvalue", pqlt_cur_setting->cctvalue);

	len += snprintf(kbuf+len, size - len,
			"***reading_mode***\n"
			"%-20s:\t%d\n",
			"readingmode", pqlt_cur_setting->pq_setting.readingmode);

	len += snprintf(kbuf+len, size - len,
			"***ambient_lut***\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n",
			"al_en", pqlt_cur_setting->pq_setting.alenable,
			"al_luxvalue", pqlt_cur_setting->luxvalue,
			"al_bl_ratio", pqlt_cur_setting->al_bl_ratio);

	len += snprintf(kbuf+len, size - len,
			"***sdr2hdr***\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d\n",
			"sdr2hdr", pqlt_cur_setting->pq_setting.sdr2hdr,
			"maxcll", pqlt_cur_setting->maxcll);

	len += snprintf(kbuf+len, size - len,
			"***analog_abypass***\n"
			"%-20s:\t%d\n",
			"abyp_mode", pcfg->abyp_ctrl.abypass_mode);

	len += snprintf(kbuf+len, size - len,
			"***n2m***\n"
			"%-20s:\t%d\n",
			"n2m_en", pcfg->n2m_enable);

	len += snprintf(kbuf+len, size - len,
			"***osd protect window***\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n"
			"%-20s:\t0x%x\n",
			"osd0_tl", pcfg->frc_setting.iris_osd0_tl,
			"osd0_br", pcfg->frc_setting.iris_osd0_br,
			"osd1_tl", pcfg->frc_setting.iris_osd1_tl,
			"osd1_br", pcfg->frc_setting.iris_osd1_br,
			"osd2_tl", pcfg->frc_setting.iris_osd2_tl,
			"osd2_br", pcfg->frc_setting.iris_osd2_br,
			"osd3_tl", pcfg->frc_setting.iris_osd3_tl,
			"osd3_br", pcfg->frc_setting.iris_osd3_br,
			"osd4_tl", pcfg->frc_setting.iris_osd4_tl,
			"osd4_br", pcfg->frc_setting.iris_osd4_br,
			"osd_win_ctrl", pcfg->frc_setting.iris_osd_window_ctrl,
			"osd_win_ctrl", pcfg->frc_setting.iris_osd_win_dynCompensate);

	len += snprintf(kbuf+len, size - len,
			"***firmware_version***\n"
			"%-20s:\t%d\n"
			"%-20s:\t%d%d/%d/%d\n",
			"version", pcfg->app_version,
			"date", pcfg->app_date[3], pcfg->app_date[2], pcfg->app_date[1], pcfg->app_date[0]);

	size = len;
	if (len >= count)
		size = count - 1;

	if (copy_to_user(ubuf, kbuf, size)) {
		vfree(kbuf);
		return -EFAULT;
	}

	vfree(kbuf);

	*ppos += size;

	return size;
}


static const struct file_operations iris_dbg_fstatus_fops = {
	.open = iris_dbg_fstatus_open,
	.read = iris_dbg_fstatus_read,
};

void iris_display_mode_name_update(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg == NULL)
		return;

	strscpy(pcfg->display_mode_name, "Not Set", sizeof(pcfg->display_mode_name));

	if (pcfg->abyp_ctrl.abypass_mode == ANALOG_BYPASS_MODE) {
		if (pcfg->lp_ctrl.abyp_lp == 2)
			strscpy(pcfg->display_mode_name, "SLEEP-ABYPASS", sizeof(pcfg->display_mode_name));
		else
			strscpy(pcfg->display_mode_name, "STANDBY-ABYPASS", sizeof(pcfg->display_mode_name));
	} else {
		static const char * const iris_status[] = {
			"SINGLE-PT",
			"SINGLE-PT-SR",
			"SINGLE-MEMC",
			"SINGLE-MEMC-SR",
			"DUAL-PT",
			"DUAL-PT-SR",
			"DUAL-MEMC",
			"DUAL-MEMC-SR",
		};
		uint32_t status_index = 0;
		const char *status = NULL;

		if (pcfg->dual_enabled)
			status_index = 4;
		if (pcfg->pwil_mode == FRC_MODE)
			status_index += 2;

		switch (pcfg->iris_chip_type) {
		case CHIP_IRIS5:
		case CHIP_IRIS7:
			if (pcfg->pwil_mode == FRC_MODE) {
				if (pcfg->frcgame_pq_guided_level > 1)
					status_index += 1;
			} else {
				if (pcfg->pt_sr_enable)
					status_index += 1;
			}
			strscpy(pcfg->display_mode_name, iris_status[status_index],
					sizeof(pcfg->display_mode_name));
			break;
		case CHIP_IRIS7P:
		case CHIP_IRIS8:
			status = iris_ptsr_status();
			if (status == NULL)
				strscpy(pcfg->display_mode_name, iris_status[status_index],
						sizeof(pcfg->display_mode_name));
			else
				strscpy(pcfg->display_mode_name, status,
						sizeof(pcfg->display_mode_name));
			break;
		default:
			IRIS_LOGE("%s(), invalid chip type: %u", __func__,
					pcfg->iris_chip_type);
			break;
		}

		if (pcfg->pwil_mode == RFB_MODE) {
			status = "SINGLE-RFB";
			if (pcfg->dual_enabled)
				status = "DUAL-RFB";
			strscpy(pcfg->display_mode_name, status,
				sizeof(pcfg->display_mode_name));
		}
	}
}

int iris_debug_display_mode_get(char *kbuf, int size, bool debug)
{
	int rc = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->pw_chip_func_ops.iris_debug_display_mode_get_)
		rc = pcfg->pw_chip_func_ops.iris_debug_display_mode_get_(kbuf, size, debug);

	return rc;
}

int iris_debug_pq_info_get(char *kbuf, int size, bool debug)
{
	int rc = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->pw_chip_func_ops.iris_debug_pq_info_get_)
		pcfg->pw_chip_func_ops.iris_debug_pq_info_get_(kbuf, size, debug);

	return rc;
}

static ssize_t iris_dbg_display_mode_show(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	char *kbuf = NULL;
	int size = count < PAGE_SIZE ? PAGE_SIZE : count;

	if (*ppos)
		return 0;

	kbuf = vzalloc(size);
	if (kbuf == NULL) {
		IRIS_LOGE("Fatal erorr: No mem!\n");
		return -ENOMEM;
	}

	size = iris_debug_display_mode_get(kbuf, size, false);
	if (size >= count)
		size = count - 1;

	if (copy_to_user(ubuf, kbuf, size)) {
		vfree(kbuf);
		return -EFAULT;
	}

	vfree(kbuf);

	*ppos += size;

	return size;
}

static ssize_t display_mode_show(struct kobject *obj, struct kobj_attribute *attr, char *buf)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	iris_display_mode_name_update();
	return snprintf(buf, PAGE_SIZE, "%s\n", pcfg->display_mode_name);
}

#define IRIS_ATTR(_name, _mode, _show, _store)\
struct kobj_attribute iris_attr_##_name = __ATTR(_name, _mode, _show, _store)
static IRIS_ATTR(display_mode, 0444, display_mode_show, NULL);

static struct attribute *iris_dbg_dev_attrs[] = {
	&iris_attr_display_mode.attr,
	NULL,
};

static const struct attribute_group iris_dbg_attr_group = {
	.attrs = iris_dbg_dev_attrs,
};

static const struct file_operations iris_dbg_dislay_mode_fops = {
	.open = simple_open,
	.read = iris_dbg_display_mode_show,
};



static ssize_t _iris_dsi_cmd_log_write(
		struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	uint32_t val = 0;

	if (count > SZ_32)
		return -EFAULT;

	if (kstrtouint_from_user(buff, count, 0, &val))
		return -EFAULT;

	iris_set_dsi_cmd_log(val);

	return count;
}

static ssize_t _iris_dsi_cmd_log_read(
		struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	uint32_t val = 0;
	uint32_t len = 0;
	char bf[SZ_32];

	if (*ppos)
		return 0;

	val = iris_get_dsi_cmd_log();
	len += scnprintf(bf, SZ_32, "%u\n", val);

	len = min_t(size_t, count, len);
	if (copy_to_user(buff, bf, len))
		return -EFAULT;

	*ppos += len;

	return len;
}

static const struct file_operations iris_dsi_cmd_log_fops = {
	.open = simple_open,
	.write = _iris_dsi_cmd_log_write,
	.read = _iris_dsi_cmd_log_read,
};

static ssize_t _iris_tm_sw_log_write(
		struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	uint32_t val = 0;

	if (count > SZ_32)
		return -EFAULT;

	if (kstrtouint_from_user(buff, count, 0, &val))
		return -EFAULT;

	iris_set_tm_sw_loglevel(val);

	return count;
}

static ssize_t _iris_tm_sw_log_read(
		struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	uint32_t val = 0;
	uint32_t len = 0;
	char bf[SZ_32];

	if (*ppos)
		return 0;

	val = iris_get_tm_sw_loglevel();
	len += scnprintf(bf, SZ_32, "%u\n", val);

	len = min_t(size_t, count, len);
	if (copy_to_user(buff, bf, len))
		return -EFAULT;

	*ppos += len;

	return len;
}

static const struct file_operations iris_tm_sw_fops = {
	.open = simple_open,
	.write = _iris_tm_sw_log_write,
	.read = _iris_tm_sw_log_read,
};


static ssize_t _iris_cmd_list_write(
		struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	uint32_t val = 0;

	if (count > SZ_32)
		return -EFAULT;

	if (kstrtouint_from_user(buff, count, 0, &val))
		return -EFAULT;

	iris_dump_cmdlist(val);

	return count;
}

static const struct file_operations iris_cmd_list_fops = {
	.open = simple_open,
	.write = _iris_cmd_list_write,
};

int pw_dbgfs_status_init(void *display)
{
	struct iris_cfg *pcfg = NULL;
	int retval;

	pcfg = iris_get_cfg();
	if (pcfg->iris_kobj) {
		retval = sysfs_create_group(pcfg->iris_kobj, &iris_dbg_attr_group);
		if (retval) {
			kobject_put(pcfg->iris_kobj);
			IRIS_LOGE("sysfs create display_mode node fail");
		} else {
			IRIS_LOGI("sysfs create display_mode node successfully");
		}
	}

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir(IRIS_DBG_TOP_DIR, NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("create dir for iris failed, error %ld",
					PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	if (debugfs_create_file(IRIS_DBG_FUNCSTATUS_FILE, 0644,
				pcfg->dbg_root, display,
				&iris_dbg_fstatus_fops) == NULL)
		IRIS_LOGE("create file func_status failed");

	if (debugfs_create_file("display_mode", 0644,
				pcfg->dbg_root, display,
				&iris_dbg_dislay_mode_fops) == NULL)
		IRIS_LOGE("create file display_mode failed");

	if (debugfs_create_file("dsi_cmd_log", 0644,
				pcfg->dbg_root, display,
				&iris_dsi_cmd_log_fops) == NULL)
		IRIS_LOGE("create file dsi_cmd_log failed");

	if (debugfs_create_file("tm_sw_loglevel", 0644,
				pcfg->dbg_root, display,
				&iris_tm_sw_fops) == NULL)
		IRIS_LOGE("create file tm_sw_loglevel failed");

	if (debugfs_create_file("cmd_list_status", 0644,
				pcfg->dbg_root, display,
				&iris_cmd_list_fops) == NULL)
		IRIS_LOGE("create file cmd_list_status failed");


	return 0;
}

void iris_dsi_ctrl_dump_desc_cmd(const struct mipi_dsi_msg *msg)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (iris_enable_dsi_cmd_log == 0)
		return;

	WARN_ON(iris_enable_dsi_cmd_log > 1);

	if (!iris_is_chip_supported())
		return;

	if (msg == NULL || msg->tx_buf == NULL || msg->tx_len == 0)
		return;

	IRIS_LOGI("%s()", __func__);

	{
#define DUMP_BUF_SIZE 1024
		int len = 0;
		size_t i;
		size_t data_len;
		char *tx_buf = (char *)msg->tx_buf;
		char buf[DUMP_BUF_SIZE];

		memset(buf, 0, sizeof(buf));

		/* Packet Info */
		buf[len] = msg->type;
		len++;
		/* Last bit */
		if (pcfg->iris_is_last_cmd) {
			buf[len] = pcfg->iris_is_last_cmd(msg) ? 1 : 0;
			len++;
		}
		buf[len] = msg->channel;
		len++;
		buf[len] = (unsigned int)msg->flags;
		len++;
		buf[len] = (msg->tx_len >> 8) & 0xFF;
		len++;
		buf[len] = msg->tx_len & 0xFF;
		len++;

		/* Packet Payload */
		data_len = msg->tx_len < (DUMP_BUF_SIZE - len) ? msg->tx_len : (DUMP_BUF_SIZE - len);
		for (i = 0; i < data_len; i++)
			buf[len + i] = tx_buf[i];

		print_hex_dump(KERN_ERR, "IRIS_LOG ", DUMP_PREFIX_NONE, 16, 1, buf, (len + data_len), false);
	}
}

void iris_dsi_panel_dump_pps(struct iris_cmd_set *set)
{
	if (!iris_is_chip_supported())
		return;

	if (!set)
		return;

	IRIS_LOGI("%s(), qcom pps table", __func__);
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 16, 4,
			set->cmds->msg.tx_buf, set->cmds->msg.tx_len, false);
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 4, 10,
			set->cmds->msg.tx_buf, set->cmds->msg.tx_len, false);
}
