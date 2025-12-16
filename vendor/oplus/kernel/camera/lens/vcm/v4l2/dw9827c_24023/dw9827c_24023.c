// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define DRIVER_NAME "dw9827c_24023"

#define LOG_INF(format, args...)                                               \
	pr_info(DRIVER_NAME " [%s] " format, __func__, ##args)

#define DW9827C_24023_NAME				"dw9827c_24023"
#define DW9827C_24023_MAX_FOCUS_POS			1023
#define DW9827C_24023_ORIGIN_FOCUS_POS		0
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define DW9827C_24023_FOCUS_STEPS			1
#define DW9827C_24023_SET_POSITION_ADDR		0x00

#define DW9827C_24023_CMD_DELAY			0xff
#define DW9827C_24023_CTRL_DELAY_US			10000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define DW9827C_24023_MOVE_STEPS			100
#define DW9827C_24023_MOVE_DELAY_US			1000
#define DW9827C_24023_INIT_DELAY_US			3000

#define DW9827C_24023_MOVE_INIT_POS			230

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif /* OPLUS_FEATURE_CAMERA_COMMON */

/* dw9827c_24023 device structure */
struct dw9827c_24023_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *vin;
	struct regulator *vdd;
	struct pinctrl *vcamaf_pinctrl;
	struct pinctrl_state *vcamaf_on;
	struct pinctrl_state *vcamaf_off;
	/* active or standby mode */
	bool active;
};

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
	unsigned char delay;
};

static struct regval_list dw9827c_24023_init_regs[] = {
	{0x02, 0x40, 3},
	{0x04, 0x01, 3},
	{0x00, ((DW9827C_24023_MOVE_INIT_POS << 6) >> 8) & 0xff, 0},
	{0x01, (DW9827C_24023_MOVE_INIT_POS << 6) & 0xff       , 0},
	{0x02, 0x00, 3},
};

static struct regval_list dw9827c_24023_PID_V3[] = {
	{0x02, 0x40, 5},
	{0x34, 0x85, 0},
	{0x45, 0xAF, 0},
	{0x48, 0x10, 0},
	{0x49, 0x00, 0},
	{0x4A, 0x19, 0},
	{0x4B, 0x00, 0},
	{0x4C, 0x01, 0},
	{0x4D, 0x0A, 0},
	{0x4E, 0x20, 0},
	{0x4F, 0x00, 0},
	{0x50, 0x0B, 0},
	{0x51, 0xFC, 0},
	{0x52, 0x0A, 0},
	{0x53, 0xA6, 0},
	{0x54, 0x3A, 0},
	{0x74, 0x05, 0},
	{0x75, 0x58, 0},
	{0x76, 0x88, 0},
	{0x77, 0x20, 0},
	{0x78, 0x14, 0},
	{0x79, 0x14, 0},
	{0x7A, 0x30, 0},
	{0x7B, 0x00, 0},
	{0x7C, 0x00, 0},
	{0x7D, 0x10, 0},
	{0x7E, 0x03, 0},// pid version
	{0x80, 0x00, 0},
	{0x81, 0x00, 0},
	{0x82, 0x0D, 0},
	{0x83, 0xB4, 0},
	{0x86, 0x7A, 0},
	{0x87, 0x47, 0},
	{0x88, 0xC4, 0},
	{0x89, 0x20, 0},
	{0x8A, 0x3D, 0},
	{0x8B, 0xF1, 0},
	{0x8C, 0x85, 0},
	{0x8D, 0xB9, 0},
	{0x8E, 0x3D, 0},
	{0x8F, 0xF0, 0},
	{0x90, 0xCC, 0},
	{0x91, 0x7D, 0},
	{0x92, 0xA3, 0},
	{0x93, 0x55, 0},
	{0x94, 0x02, 0},
	{0x95, 0x00, 0},
	{0x96, 0x00, 0},
	{0x97, 0x0F, 0},
	{0x98, 0x80, 0},
	{0x99, 0x05, 0},
	{0x9A, 0x00, 0},
	{0x9B, 0x32, 0},
	{0x9C, 0xFF, 0},
	{0x9D, 0x00, 0},
	{0x9E, 0x10, 0},
	{0x9F, 0x10, 0},
	{0xA0, 0x00, 0},
	{0xA1, 0x00, 0},
	{0xA2, 0x80, 0},
	{0xA3, 0x80, 0},
	{0xA8, 0x10, 0},
	{0xA9, 0x00, 0},
	{0xAA, 0x00, 0},
	{0xAB, 0x00, 0},
	{0xAC, 0x10, 0},
	{0x02, 0x40, 0},
	{0x03, 0x01, 10},

};

#define VCM_IOC_POWER_ON         _IO('V', BASE_VIDIOC_PRIVATE + 3)
#define VCM_IOC_POWER_OFF        _IO('V', BASE_VIDIOC_PRIVATE + 4)

static inline struct dw9827c_24023_device *to_dw9827c_24023_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct dw9827c_24023_device, ctrls);
}

static inline struct dw9827c_24023_device *sd_to_dw9827c_24023_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct dw9827c_24023_device, sd);
}

static int dw9827c_24023_set_position(struct dw9827c_24023_device *dw9827c_24023, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c_24023->sd);
	int retry = 3;
	int ret;

	LOG_INF("dw9827c_24023 Set postition:%u.", val);
	while (--retry > 0) {
        ret = i2c_smbus_write_word_data(client, DW9827C_24023_SET_POSITION_ADDR,
                                        swab16(val << 6));
		if (ret < 0) {
			usleep_range(DW9827C_24023_MOVE_DELAY_US,
				     DW9827C_24023_MOVE_DELAY_US + 1000);
			#ifdef OPLUS_FEATURE_CAMERA_COMMON
			LOG_INF("dw9827c_24023 Set postition:%u fail.", val);
			#endif /*OPLUS_FEATURE_CAMERA_COMMON*/
		} else {
			break;
		}
	}
	return ret;
}

static int dw9827c_24023_release(struct dw9827c_24023_device *dw9827c_24023)
{
	int ret, val;
	int diff_dac = 0;
	int nStep_count = 0;
	int i = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c_24023->sd);

	if (!dw9827c_24023->active) {
		return 0;
	}
	pr_info("%s +\n", __func__);
	diff_dac = DW9827C_24023_ORIGIN_FOCUS_POS - dw9827c_24023->focus->val;

	nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
		DW9827C_24023_MOVE_STEPS;

	val = dw9827c_24023->focus->val;

	for (i = 0; i < nStep_count; ++i) {
		val += (diff_dac < 0 ? (DW9827C_24023_MOVE_STEPS*(-1)) : DW9827C_24023_MOVE_STEPS);

		ret = dw9827c_24023_set_position(dw9827c_24023, val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
		usleep_range(DW9827C_24023_MOVE_DELAY_US,
			     DW9827C_24023_MOVE_DELAY_US + 1000);
	}

	// last step to origin
	ret = dw9827c_24023_set_position(dw9827c_24023, DW9827C_24023_ORIGIN_FOCUS_POS);
	if (ret) {
		LOG_INF("%s I2C failure: %d",
			__func__, ret);
		return ret;
	}

	i2c_smbus_write_byte_data(client, 0x02, 0x20);
	dw9827c_24023->active = false;

	usleep_range(5000, 5000 + 1000);

	pr_info("%s -\n", __func__);

	return 0;
}


static int dw9827c_24023_write_array(struct dw9827c_24023_device *dw9827c_24023,
			      struct regval_list *vals, u32 len)
{
	unsigned int i;
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c_24023->sd);

	for (i = 0; i < len; i++) {
		LOG_INF("Init write [0x%x, 0x%x, %d]", vals[i].reg_num, vals[i].value, vals[i].delay);
		ret = i2c_smbus_write_byte_data(client, vals[i].reg_num, vals[i].value);
		if (ret < 0)
			return ret;
		if(vals[i].delay) {
			usleep_range(vals[i].delay * 1000, vals[i].delay * 1000 + 1000);
		}
	}

	return 0;
}


static int dw9827c_24023_init(struct dw9827c_24023_device *dw9827c_24023)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c_24023->sd);
	int ret = 0;
	u8 value = 0xFF;

	if (dw9827c_24023->active) {
		return 0;
	}
	LOG_INF("+\n");

	i2c_smbus_write_byte_data(client, 0x02, 0x40);//standby
	usleep_range(3 * 1000, 3 * 1000 + 1000);
	value = i2c_smbus_read_byte_data(client, 0x7D);
	LOG_INF("States standby(0x7D) = 0x%x\n", value);

   	if(value != 0x00) {
		i2c_smbus_write_byte_data(client, 0x02, 0x40);//standby
		usleep_range(5 * 1000, 5 * 1000 + 1000);
		i2c_smbus_write_byte_data(client, 0x34, 0x85);//pt off
		i2c_smbus_write_byte_data(client, 0x7D, 0x00);// States standby
		i2c_smbus_write_byte_data(client, 0x95, 0x00);// SAL OFF
		i2c_smbus_write_byte_data(client, 0x03, 0x01);// store
		usleep_range(10 * 1000, 10 * 1000 + 1000);
		i2c_smbus_write_byte_data(client, 0x34, 0x52);// pt on
		i2c_smbus_write_byte_data(client, 0x04, 0x01);// sw reset
		usleep_range(5 * 1000, 5 * 1000 + 1000);
	}

	value = i2c_smbus_read_byte_data(client, 0x03);
	LOG_INF("product_id: %x\n", value);

	LOG_INF("dw9827c_24023_init value: %d", value);

	ret = dw9827c_24023_write_array(dw9827c_24023, dw9827c_24023_init_regs,
                    ARRAY_SIZE(dw9827c_24023_init_regs));
	usleep_range(1000, 1000 + 1000);
	LOG_INF("dw9827c_24023_init position: %d", dw9827c_24023->focus->val);
	ret = dw9827c_24023_set_position(dw9827c_24023, dw9827c_24023->focus->val);

	dw9827c_24023->active = true;

	pr_info("%s -\n", __func__);

	return 0;
}

/* Power handling */
static int dw9827c_24023_power_off(struct dw9827c_24023_device *dw9827c_24023)
{
	int ret;

	LOG_INF("+\n");

	ret = dw9827c_24023_release(dw9827c_24023);
	if (ret)
		LOG_INF("dw9827c_24023 release failed!\n");

	ret = regulator_disable(dw9827c_24023->vin);
	if (ret)
		return ret;

	ret = regulator_disable(dw9827c_24023->vdd);
	if (ret)
		return ret;

	if (dw9827c_24023->vcamaf_pinctrl && dw9827c_24023->vcamaf_off)
		ret = pinctrl_select_state(dw9827c_24023->vcamaf_pinctrl,
					dw9827c_24023->vcamaf_off);

	LOG_INF("-\n");

	return ret;
}

static int dw9827c_24023_power_on(struct dw9827c_24023_device *dw9827c_24023)
{
	int ret;
	u8 value = 0xFF;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c_24023->sd);

	LOG_INF("+\n");

	ret = regulator_enable(dw9827c_24023->vin);
	if (ret < 0)
		return ret;

	ret = regulator_enable(dw9827c_24023->vdd);
	if (ret < 0)
		return ret;

	if (dw9827c_24023->vcamaf_pinctrl && dw9827c_24023->vcamaf_on)
		ret = pinctrl_select_state(dw9827c_24023->vcamaf_pinctrl,
					dw9827c_24023->vcamaf_on);

	if (ret < 0)
		return ret;

	/*
	 * TODO(b/139784289): Confirm hardware requirements and adjust/remove
	 * the delay.
	 */
	usleep_range(DW9827C_24023_CTRL_DELAY_US, DW9827C_24023_CTRL_DELAY_US + 100);

	ret = dw9827c_24023_init(dw9827c_24023);
	if (ret < 0)
		goto fail;
	// update PID_V3
	value = i2c_smbus_read_byte_data(client, 0x7E);
	LOG_INF("current PID version:%d \n", value);
	if (value < 0x03){
		LOG_INF("need update PID:\n");
		ret = dw9827c_24023_write_array(dw9827c_24023, dw9827c_24023_PID_V3,
                    ARRAY_SIZE(dw9827c_24023_PID_V3));
		if (ret < 0)
			goto fail;
	}

	LOG_INF("-\n");

	return 0;

fail:
	regulator_disable(dw9827c_24023->vin);
	regulator_disable(dw9827c_24023->vdd);
	if (dw9827c_24023->vcamaf_pinctrl && dw9827c_24023->vcamaf_off) {
		pinctrl_select_state(dw9827c_24023->vcamaf_pinctrl,
				dw9827c_24023->vcamaf_off);
	}

	return ret;
}

static int dw9827c_24023_set_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct dw9827c_24023_device *dw9827c_24023 = to_dw9827c_24023_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		LOG_INF("pos(%d)\n", ctrl->val);
		ret = dw9827c_24023_set_position(dw9827c_24023, ctrl->val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
	}
	return 0;
}

static const struct v4l2_ctrl_ops dw9827c_24023_vcm_ctrl_ops = {
	.s_ctrl = dw9827c_24023_set_ctrl,
};

static int dw9827c_24023_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	struct dw9827c_24023_device *dw9827c_24023 = sd_to_dw9827c_24023_vcm(sd);

	ret = dw9827c_24023_power_on(dw9827c_24023);
	if (ret < 0) {
		LOG_INF("power on fail, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int dw9827c_24023_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct dw9827c_24023_device *dw9827c_24023 = sd_to_dw9827c_24023_vcm(sd);

	dw9827c_24023_power_off(dw9827c_24023);

	return 0;
}

static int dw9827c_24023_vcm_resume(struct dw9827c_24023_device *dw9827c_24023)
{
    int ret = 0;
    ret = dw9827c_24023_init(dw9827c_24023);
    pr_info("%s exit stand by mode, active:%d\n", __func__, dw9827c_24023->active);
    return ret;
}

static int dw9827c_24023_vcm_suspend(struct dw9827c_24023_device *dw9827c_24023)
{
    int ret = 0;
    ret = dw9827c_24023_release(dw9827c_24023);
    pr_info("%s entry stand by mode, active:%d\n", __func__, dw9827c_24023->active);
    return ret;
}

static long dw9827c_24023_ops_core_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct dw9827c_24023_device *dw9827c_24023 = sd_to_dw9827c_24023_vcm(sd);
	pr_info("%s +\n", __func__);

	switch (cmd) {
	case VCM_IOC_POWER_ON:
        ret = dw9827c_24023_vcm_resume(dw9827c_24023);
        pr_info("%s VCM_IOC_POWER_ON, cmd:%d, ret: %d\n", __func__, cmd, ret);
        break;
    case VCM_IOC_POWER_OFF:
        ret = dw9827c_24023_vcm_suspend(dw9827c_24023);
        pr_info("%s VCM_IOC_POWER_OFF, cmd:%d, ret:%d\n", __func__, cmd, ret);
        break;
	break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	pr_info("%s -\n", __func__);
	return ret;
}


static const struct v4l2_subdev_internal_ops dw9827c_24023_int_ops = {
	.open = dw9827c_24023_open,
	.close = dw9827c_24023_close,
};

static struct v4l2_subdev_core_ops dw9827c_24023_ops_core = {
	.ioctl = dw9827c_24023_ops_core_ioctl,
};

static const struct v4l2_subdev_ops dw9827c_24023_ops = {
	.core = &dw9827c_24023_ops_core,
};

static void dw9827c_24023_subdev_cleanup(struct dw9827c_24023_device *dw9827c_24023)
{
	v4l2_async_unregister_subdev(&dw9827c_24023->sd);
	v4l2_ctrl_handler_free(&dw9827c_24023->ctrls);
#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&dw9827c_24023->sd.entity);
#endif
}

static int dw9827c_24023_init_controls(struct dw9827c_24023_device *dw9827c_24023)
{
	struct v4l2_ctrl_handler *hdl = &dw9827c_24023->ctrls;
	const struct v4l2_ctrl_ops *ops = &dw9827c_24023_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	dw9827c_24023->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, DW9827C_24023_MAX_FOCUS_POS, DW9827C_24023_FOCUS_STEPS, 0);

	if (hdl->error)
		return hdl->error;

	dw9827c_24023->sd.ctrl_handler = hdl;

	return 0;
}

static int dw9827c_24023_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dw9827c_24023_device *dw9827c_24023;
	int ret;

	LOG_INF("+\n");

	dw9827c_24023 = devm_kzalloc(dev, sizeof(*dw9827c_24023), GFP_KERNEL);
	if (!dw9827c_24023)
		return -ENOMEM;

	dw9827c_24023->vin = devm_regulator_get(dev, "vin");
	if (IS_ERR(dw9827c_24023->vin)) {
		ret = PTR_ERR(dw9827c_24023->vin);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vin regulator\n");
		return ret;
	}

	dw9827c_24023->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(dw9827c_24023->vdd)) {
		ret = PTR_ERR(dw9827c_24023->vdd);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vdd regulator\n");
		return ret;
	}

	dw9827c_24023->vcamaf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(dw9827c_24023->vcamaf_pinctrl)) {
		ret = PTR_ERR(dw9827c_24023->vcamaf_pinctrl);
		dw9827c_24023->vcamaf_pinctrl = NULL;
		LOG_INF("cannot get pinctrl\n");
	} else {
		dw9827c_24023->vcamaf_on = pinctrl_lookup_state(
			dw9827c_24023->vcamaf_pinctrl, "vcamaf_on");

		if (IS_ERR(dw9827c_24023->vcamaf_on)) {
			ret = PTR_ERR(dw9827c_24023->vcamaf_on);
			dw9827c_24023->vcamaf_on = NULL;
			LOG_INF("cannot get vcamaf_on pinctrl\n");
		}

		dw9827c_24023->vcamaf_off = pinctrl_lookup_state(
			dw9827c_24023->vcamaf_pinctrl, "vcamaf_off");

		if (IS_ERR(dw9827c_24023->vcamaf_off)) {
			ret = PTR_ERR(dw9827c_24023->vcamaf_off);
			dw9827c_24023->vcamaf_off = NULL;
			LOG_INF("cannot get vcamaf_off pinctrl\n");
		}
	}

	v4l2_i2c_subdev_init(&dw9827c_24023->sd, client, &dw9827c_24023_ops);
	dw9827c_24023->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dw9827c_24023->sd.internal_ops = &dw9827c_24023_int_ops;

	ret = dw9827c_24023_init_controls(dw9827c_24023);
	if (ret)
		goto err_cleanup;

#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&dw9827c_24023->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	dw9827c_24023->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

	ret = v4l2_async_register_subdev(&dw9827c_24023->sd);
	if (ret < 0)
		goto err_cleanup;

	LOG_INF("-\n");

	return 0;

err_cleanup:
	dw9827c_24023_subdev_cleanup(dw9827c_24023);
	return ret;
}

static void dw9827c_24023_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9827c_24023_device *dw9827c_24023 = sd_to_dw9827c_24023_vcm(sd);

	LOG_INF("+\n");

	dw9827c_24023_subdev_cleanup(dw9827c_24023);

	LOG_INF("-\n");
	//return 0;
}

static const struct i2c_device_id dw9827c_24023_id_table[] = {
	{ DW9827C_24023_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, dw9827c_24023_id_table);

static const struct of_device_id dw9827c_24023_of_table[] = {
	{ .compatible = "oplus,dw9827c_24023" },
	{ },
};
MODULE_DEVICE_TABLE(of, dw9827c_24023_of_table);

static struct i2c_driver dw9827c_24023_i2c_driver = {
	.driver = {
		.name = DW9827C_24023_NAME,
		.of_match_table = dw9827c_24023_of_table,
	},
	.probe_new  = dw9827c_24023_probe,
	.remove = dw9827c_24023_remove,
	.id_table = dw9827c_24023_id_table,
};

module_i2c_driver(dw9827c_24023_i2c_driver);

MODULE_AUTHOR("Po-Hao Huang <Po-Hao.Huang@mediatek.com>");
MODULE_DESCRIPTION("DW9827C_24023 VCM driver");
MODULE_LICENSE("GPL v2");
