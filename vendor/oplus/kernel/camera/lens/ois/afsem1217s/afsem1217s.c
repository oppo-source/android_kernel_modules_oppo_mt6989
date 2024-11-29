/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023 OPLUS Inc.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include "../adaptor-i2c.h"

#define DRIVER_NAME "afsem1217s"
#define AFSEM1217S_I2C_SLAVE_ADDR 0xC2

#define LOG_INF(format, args...)                                               \
	pr_info(DRIVER_NAME " [%s] " format, __func__, ##args)

#define AFSEM1217S_NAME			"afsem1217s"
#define AFSEM1217S_MAX_FOCUS_POS		1023
#define AFSEM1217S_ORIGIN_FOCUS_POS		0
#define AFSEM1217S_CTRL_STEPS1		128
#define AFSEM1217S_CTRL_STEPS2		32
#define AFSEM1217S_CTRL_POS		256
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define AFSEM1217S_FOCUS_STEPS			1
#define AFSEM1217S_SET_POSITION_ADDR		0x0204

#define AFSEM1217S_CMD_DELAY			0xff
#define AFSEM1217S_CTRL_DELAY_US			10000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define AFSEM1217S_MOVE_STEPS			100
#define AFSEM1217S_MOVE_DELAY_US		1000

static u16 g_last_pos = AFSEM1217S_MAX_FOCUS_POS / 2;
static u16 g_origin_pos = AFSEM1217S_MAX_FOCUS_POS / 2;

/* afsem1217s device structure */
struct afsem1217s_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *vin;
	struct regulator *vdd;
	struct regulator *avdd;
	struct pinctrl *vcamaf_pinctrl;
	struct pinctrl_state *vcamaf_on;
	struct pinctrl_state *vcamaf_off;
	/* active or standby mode */
	bool active;
};

#define VCM_IOC_POWER_ON         _IO('V', BASE_VIDIOC_PRIVATE + 3)
#define VCM_IOC_POWER_OFF        _IO('V', BASE_VIDIOC_PRIVATE + 4)

static inline struct afsem1217s_device *to_afsem1217s_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct afsem1217s_device, ctrls);
}

static inline struct afsem1217s_device *sd_to_afsem1217s_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct afsem1217s_device, sd);
}

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};


static int afsem1217s_set_position(struct afsem1217s_device *afsem1217s, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&afsem1217s->sd);

	return adaptor_i2c_wr_u16(client, client->addr, AFSEM1217S_SET_POSITION_ADDR,
					 		swab16(val << 4));
}

static int afsem1217s_shaking_lock_on(struct afsem1217s_device *afsem1217s)
{
	struct i2c_client *client = v4l2_get_subdevdata(&afsem1217s->sd);
	int ret = 0;
	LOG_INF("+\n");
	/* AF Lock*/
	ret = adaptor_i2c_wr_u8(client, client->addr, 0x0200, 0x01);
	if (ret < 0) {
		LOG_INF("%s I2C failure: %d", __func__, ret);
		return ret;
	}
	mdelay(5);
	/* OIS Lock*/
	ret = adaptor_i2c_wr_u8(client, client->addr, 0x0002, 0x03);
	if (ret < 0) {
		LOG_INF("%s I2C failure: %d", __func__, ret);
		return ret;
	}
	mdelay(5);
	ret = adaptor_i2c_wr_u8(client, client->addr, 0x0000, 0x01);
	if (ret < 0) {
		LOG_INF("%s I2C failure: %d", __func__, ret);
		return ret;
	}
	mdelay(5);
	LOG_INF("-\n");
	return 0;
}

static int afsem1217s_shaking_lock_off(struct afsem1217s_device *afsem1217s)
{
	LOG_INF("+\n");

	return 0;
}

static int afsem1217s_shaking_lock(struct afsem1217s_device *afsem1217s, u16 val)
{
	if (val) {
		return afsem1217s_shaking_lock_on(afsem1217s);
	} else {
		return afsem1217s_shaking_lock_off(afsem1217s);
	}
}

static int afsem1217s_goto_last_pos(struct afsem1217s_device *afsem1217s)
{
	int ret, i, nStep_count, val = g_origin_pos, diff_dac;
	int firstStep = AFSEM1217S_MOVE_STEPS / 4;
	diff_dac = g_last_pos - val;
	if (diff_dac == 0) {
		return 0;
	}
	nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
			AFSEM1217S_MOVE_STEPS;
	for (i = 0; i < nStep_count; ++i) {
		if (i == 0 && g_origin_pos == AFSEM1217S_MAX_FOCUS_POS) {
			val += (diff_dac < 0 ? (firstStep*(-1)) : firstStep);
		} else {
			val += (diff_dac < 0 ? (AFSEM1217S_MOVE_STEPS*(-1)) : AFSEM1217S_MOVE_STEPS);
		}
		ret = afsem1217s_set_position(afsem1217s, val);
		if (ret < 0) {
			LOG_INF("%s I2C failure: %d", __func__, ret);
		}
		usleep_range(AFSEM1217S_MOVE_DELAY_US, AFSEM1217S_MOVE_DELAY_US + 1000);
	}
	ret = afsem1217s_set_position(afsem1217s, g_last_pos);
	if (ret < 0) {
		LOG_INF("%s I2C failure: %d", __func__, ret);
	}
	return 0;
}

static int afsem1217s_release(struct afsem1217s_device *afsem1217s)
{
	int ret, val;
	struct i2c_client *client = v4l2_get_subdevdata(&afsem1217s->sd);

	for (val = afsem1217s->focus->val & ~(AFSEM1217S_CTRL_STEPS1 - 1);
		val > AFSEM1217S_CTRL_POS ; val -= AFSEM1217S_CTRL_STEPS1) {
		ret = afsem1217s_set_position(afsem1217s, val);
		if (ret < 0) {
			LOG_INF("%s I2C failure: %d", __func__, ret);
			return ret;
		}
		usleep_range(AFSEM1217S_MOVE_DELAY_US, AFSEM1217S_MOVE_DELAY_US + 10);
	}

	for (val = AFSEM1217S_CTRL_POS & ~(AFSEM1217S_CTRL_STEPS2 - 1);
		val >= 0; val -= AFSEM1217S_CTRL_STEPS2) {
		ret = afsem1217s_set_position(afsem1217s, val);
		if (ret < 0) {
			LOG_INF("%s I2C failure: %d", __func__, ret);
			return ret;
		}
		usleep_range(AFSEM1217S_MOVE_DELAY_US, AFSEM1217S_MOVE_DELAY_US + 10);
	}

	adaptor_i2c_wr_u8(client, client->addr, 0x0200, 0x00);
	afsem1217s->active = false;
	LOG_INF("-\n");
	return 0;
}

static int afsem1217s_init(struct afsem1217s_device *afsem1217s)
{
	struct i2c_client *client = v4l2_get_subdevdata(&afsem1217s->sd);
	int ret = 0;
	unsigned char checksum_byte[2];

	LOG_INF("+\n");
	client->addr = AFSEM1217S_I2C_SLAVE_ADDR >> 1;
	adaptor_i2c_rd_p8(client, client->addr, 0x1008, checksum_byte, 2);
	LOG_INF("Check HW version: 0x%x%x\n", checksum_byte[0], checksum_byte[1]);
	ret = afsem1217s_set_position(afsem1217s, g_origin_pos);
	if (ret < 0) {
		LOG_INF("%s set position I2C failure: %d",
			__func__, ret);
		return ret;
	}
	/* 00:active mode , 10:Standby mode , x1:Sleep mode */
	ret = adaptor_i2c_wr_u8(client, client->addr, 0x0200, 0x01);
	if (ret < 0) {
		LOG_INF("%s set mode I2C failure: %d",
			__func__, ret);
		return ret;
	}

	afsem1217s->active = true;
	afsem1217s_goto_last_pos(afsem1217s);

	LOG_INF("-\n");

	return 0;
}

/* Power handling */
static int afsem1217s_power_off(struct afsem1217s_device *afsem1217s)
{
	int ret;

	LOG_INF("+\n");

	ret = afsem1217s_release(afsem1217s);
	if (ret)
		LOG_INF("afsem1217s release failed!\n");

	ret = regulator_disable(afsem1217s->vin);
	if (ret)
		return ret;

	ret = regulator_disable(afsem1217s->vdd);
	if (ret)
		return ret;

	ret = regulator_disable(afsem1217s->avdd);
	if (ret)
		return ret;

	if (afsem1217s->vcamaf_pinctrl && afsem1217s->vcamaf_off)
		ret = pinctrl_select_state(afsem1217s->vcamaf_pinctrl,
					afsem1217s->vcamaf_off);

	LOG_INF("-\n");

	return ret;
}

static int afsem1217s_power_on(struct afsem1217s_device *afsem1217s)
{
	int ret;
	LOG_INF("+\n");

	ret = regulator_set_voltage(afsem1217s->vdd, 1780000, 1820000);
	if (ret < 0) {
		LOG_INF("%s afsem1217s->vdd set failed ret[%d] ", __func__,
			ret);
		return -1;
	}
	ret = regulator_enable(afsem1217s->vdd);
	if (ret < 0)
		return ret;

	ret = regulator_set_voltage(afsem1217s->avdd, 2780000, 2820000);
	if (ret < 0) {
		LOG_INF("%s afsem1217s->avdd set failed ret[%d]", __func__,
			ret);
		return -1;
	}
	ret = regulator_enable(afsem1217s->avdd);
	if (ret < 0)
		return ret;

	ret = regulator_set_voltage(afsem1217s->vin, 2780000, 2820000);
	if (ret < 0) {
		LOG_INF("%s afsem1217s->vin set failed ret[%d]", __func__,
			ret);
		return -1;
	}
	ret = regulator_enable(afsem1217s->vin);
	if (ret < 0)
		return ret;

	if (afsem1217s->vcamaf_pinctrl && afsem1217s->vcamaf_on)
		ret = pinctrl_select_state(afsem1217s->vcamaf_pinctrl,
					afsem1217s->vcamaf_on);

	if (ret < 0)
		return ret;

	/*
	 * TODO(b/139784289): Confirm hardware requirements and adjust/remove
	 * the delay.
	 */
	usleep_range(AFSEM1217S_CTRL_DELAY_US, AFSEM1217S_CTRL_DELAY_US + 100);

	ret = afsem1217s_init(afsem1217s);
	if (ret < 0)
		goto fail;

	LOG_INF("-\n");

	return 0;

fail:
	regulator_disable(afsem1217s->vin);
	regulator_disable(afsem1217s->vdd);
	regulator_disable(afsem1217s->avdd);
	if (afsem1217s->vcamaf_pinctrl && afsem1217s->vcamaf_off) {
		pinctrl_select_state(afsem1217s->vcamaf_pinctrl,
				afsem1217s->vcamaf_off);
	}

	return ret;
}

static int afsem1217s_set_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct afsem1217s_device *afsem1217s = to_afsem1217s_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		LOG_INF("pos(%d)\n", ctrl->val);
		ret = afsem1217s_set_position(afsem1217s, ctrl->val);
		if (ret < 0) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
		g_last_pos = ctrl->val;
	} else if (ctrl->id == V4L2_CID_FOCUS_AUTO) {
		LOG_INF("lock(%d)\n", ctrl->val);
		ret = afsem1217s_shaking_lock(afsem1217s, ctrl->val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
	}
	return 0;
}

static const struct v4l2_ctrl_ops afsem1217s_vcm_ctrl_ops = {
	.s_ctrl = afsem1217s_set_ctrl,
};

static int afsem1217s_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	struct afsem1217s_device *afsem1217s = sd_to_afsem1217s_vcm(sd);

	ret = afsem1217s_power_on(afsem1217s);
	if (ret < 0) {
		LOG_INF("power on fail, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int afsem1217s_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct afsem1217s_device *afsem1217s = sd_to_afsem1217s_vcm(sd);

	afsem1217s_power_off(afsem1217s);

	return 0;
}

static int afsem1217s_vcm_suspend(struct afsem1217s_device *afsem1217s)
{
	int ret, val;
	struct i2c_client *client = v4l2_get_subdevdata(&afsem1217s->sd);
	if (!afsem1217s->active)
		return 0;

	for (val = afsem1217s->focus->val & ~(AFSEM1217S_CTRL_STEPS1 - 1);
		val > AFSEM1217S_CTRL_POS; val -= AFSEM1217S_CTRL_STEPS1) {
		ret = afsem1217s_set_position(afsem1217s, val);
		if (ret < 0) {
			LOG_INF("%s I2C failure: %d", __func__, ret);
			return ret;
		}
		usleep_range(AFSEM1217S_MOVE_DELAY_US, AFSEM1217S_MOVE_DELAY_US + 10);
	}

	for (val = AFSEM1217S_CTRL_POS & ~(AFSEM1217S_CTRL_STEPS2 - 1);
		val >= 0; val -= AFSEM1217S_CTRL_STEPS2) {
		ret = afsem1217s_set_position(afsem1217s, val);
		if (ret < 0) {
			LOG_INF("%s I2C failure: %d", __func__, ret);
			return ret;
		}
		usleep_range(AFSEM1217S_MOVE_DELAY_US, AFSEM1217S_MOVE_DELAY_US + 10);
	}

	ret = adaptor_i2c_wr_u8(client, client->addr, 0x0200, 0x00);
	if (ret < 0) {
		LOG_INF("I2C failure!!!\n");
	} else {
		afsem1217s->active = false;
		LOG_INF("enter stand by mode\n");
	}
	return ret;
}

static int afsem1217s_vcm_resume(struct afsem1217s_device *afsem1217s)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(&afsem1217s->sd);
	if (afsem1217s->active)
		return 0;
	ret = adaptor_i2c_wr_u8(client, client->addr, 0x0200, 0x01);
	if (ret < 0) {
		LOG_INF("%s I2C failure: %d", __func__, ret);
		return ret;
	}
	ret = afsem1217s_set_position(afsem1217s, g_origin_pos);
	if (ret < 0) {
		LOG_INF("%s set position I2C failure: %d", __func__, ret);
	}
	afsem1217s_goto_last_pos(afsem1217s);
	afsem1217s->active = true;
	LOG_INF("exit stand by mode\n");
	return 0;
}

static long afsem1217s_ops_core_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct afsem1217s_device *afsem1217s = sd_to_afsem1217s_vcm(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&afsem1217s->sd);
	LOG_INF("+\n");
	client->addr = AFSEM1217S_I2C_SLAVE_ADDR >> 1;
	switch (cmd) {
	case VCM_IOC_POWER_ON:
		ret = afsem1217s_vcm_resume(afsem1217s);
		break;
	case VCM_IOC_POWER_OFF:
		ret = afsem1217s_vcm_suspend(afsem1217s);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	LOG_INF("-\n");
	return ret;
}

static const struct v4l2_subdev_internal_ops afsem1217s_int_ops = {
	.open = afsem1217s_open,
	.close = afsem1217s_close,
};

static struct v4l2_subdev_core_ops afsem1217s_ops_core = {
	.ioctl = afsem1217s_ops_core_ioctl,
};

static const struct v4l2_subdev_ops afsem1217s_ops = {
	.core = &afsem1217s_ops_core,
};

static void afsem1217s_subdev_cleanup(struct afsem1217s_device *afsem1217s)
{
	v4l2_async_unregister_subdev(&afsem1217s->sd);
	v4l2_ctrl_handler_free(&afsem1217s->ctrls);
#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&afsem1217s->sd.entity);
#endif
}

static int afsem1217s_init_controls(struct afsem1217s_device *afsem1217s)
{
	struct v4l2_ctrl_handler *hdl = &afsem1217s->ctrls;
	const struct v4l2_ctrl_ops *ops = &afsem1217s_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	afsem1217s->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, AFSEM1217S_MAX_FOCUS_POS, AFSEM1217S_FOCUS_STEPS, 0);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_AUTO, 0, 0, 0, 0);
	if (hdl->error)
		return hdl->error;

	afsem1217s->sd.ctrl_handler = hdl;

	return 0;
}

static int afsem1217s_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct afsem1217s_device *afsem1217s;
	int ret;

	LOG_INF("+\n");

	afsem1217s = devm_kzalloc(dev, sizeof(*afsem1217s), GFP_KERNEL);
	if (!afsem1217s)
		return -ENOMEM;

	afsem1217s->vin = devm_regulator_get(dev, "vin");
	if (IS_ERR(afsem1217s->vin)) {
		ret = PTR_ERR(afsem1217s->vin);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vin regulator\n");
		return ret;
	}

	afsem1217s->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(afsem1217s->vdd)) {
		ret = PTR_ERR(afsem1217s->vdd);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vdd regulator\n");
		return ret;
	}

	afsem1217s->avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(afsem1217s->avdd)) {
		ret = PTR_ERR(afsem1217s->avdd);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get avdd regulator\n");
		return ret;
	}

	afsem1217s->vcamaf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(afsem1217s->vcamaf_pinctrl)) {
		ret = PTR_ERR(afsem1217s->vcamaf_pinctrl);
		afsem1217s->vcamaf_pinctrl = NULL;
		LOG_INF("cannot get pinctrl\n");
	} else {
		afsem1217s->vcamaf_on = pinctrl_lookup_state(
			afsem1217s->vcamaf_pinctrl, "vcamaf_on");

		if (IS_ERR(afsem1217s->vcamaf_on)) {
			ret = PTR_ERR(afsem1217s->vcamaf_on);
			afsem1217s->vcamaf_on = NULL;
			LOG_INF("cannot get vcamaf_on pinctrl\n");
		}

		afsem1217s->vcamaf_off = pinctrl_lookup_state(
			afsem1217s->vcamaf_pinctrl, "vcamaf_off");

		if (IS_ERR(afsem1217s->vcamaf_off)) {
			ret = PTR_ERR(afsem1217s->vcamaf_off);
			afsem1217s->vcamaf_off = NULL;
			LOG_INF("cannot get vcamaf_off pinctrl\n");
		}
	}

	v4l2_i2c_subdev_init(&afsem1217s->sd, client, &afsem1217s_ops);
	afsem1217s->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	afsem1217s->sd.internal_ops = &afsem1217s_int_ops;

	ret = afsem1217s_init_controls(afsem1217s);
	if (ret)
		goto err_cleanup;

#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&afsem1217s->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	afsem1217s->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

	ret = v4l2_async_register_subdev(&afsem1217s->sd);
	if (ret < 0)
		goto err_cleanup;

	LOG_INF("-\n");

	return 0;

err_cleanup:
	afsem1217s_subdev_cleanup(afsem1217s);
	return ret;
}

static void afsem1217s_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct afsem1217s_device *afsem1217s = sd_to_afsem1217s_vcm(sd);

	LOG_INF("+\n");

	afsem1217s_subdev_cleanup(afsem1217s);

	LOG_INF("-\n");
}

static const struct i2c_device_id afsem1217s_id_table[] = {
	{ AFSEM1217S_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, afsem1217s_id_table);

static const struct of_device_id afsem1217s_of_table[] = {
	{ .compatible = "oplus,afsem1217s" },
	{ },
};
MODULE_DEVICE_TABLE(of, afsem1217s_of_table);

static struct i2c_driver afsem1217s_i2c_driver = {
	.driver = {
		.name = AFSEM1217S_NAME,
		.of_match_table = afsem1217s_of_table,
	},
	.probe_new  = afsem1217s_probe,
	.remove = afsem1217s_remove,
	.id_table = afsem1217s_id_table,
};

module_i2c_driver(afsem1217s_i2c_driver);

MODULE_AUTHOR("huji");
MODULE_DESCRIPTION("AFSEM1217S OIS driver");
MODULE_LICENSE("GPL v2");
