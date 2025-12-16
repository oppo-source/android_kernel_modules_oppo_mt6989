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

#define DRIVER_NAME                  "dw9800sw"
#define DW9800SW_I2C_SLAVE_ADDR        0x18

#define LOG_INF(format, args...)                                               \
	pr_info(DRIVER_NAME " [%s] " format, __func__, ##args)

#define DW9800SW_NAME				 "dw9800sw"
#define DW9800SW_MAX_FOCUS_POS		  1023
#define DW9800SW_ORIGIN_FOCUS_POS	  512
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define DW9800SW_FOCUS_STEPS			  1
#define DW9800SW_SET_POSITION_ADDR	  0x03
#define DW9800SW_STATUS_ADDR			  0x05

#define DW9800SW_CMD_DELAY			  0xff
#define DW9800SW_CTRL_DELAY_US		  1000
#define DW9800SW_POS_CTRL_DELAY_US    1000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define DW9800SW_MOVE_STEPS			  50
// base on 0x06 and 0x07 setting
// tVIB = (6.3 + (SACT[5:0]) *0.1)*DIV[2:0] ms
// 0x06 = 0x40 ==> SAC3
// 0x07 = 0x60 ==> tVIB = 9.4ms
// op_time = 9.4 * 0.72 = 6.77ms
// tolerance -+ 19%
#define DW9800SW_MOVE_DELAY_US		  8100

static int g_last_pos = DW9800SW_ORIGIN_FOCUS_POS;

/* dw9800sw device structure */
struct dw9800sw_device {
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

#define VCM_IOC_POWER_ON         _IO('V', BASE_VIDIOC_PRIVATE + 3)
#define VCM_IOC_POWER_OFF        _IO('V', BASE_VIDIOC_PRIVATE + 4)

static inline struct dw9800sw_device *to_dw9800sw_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct dw9800sw_device, ctrls);
}

static inline struct dw9800sw_device *sd_to_dw9800sw_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct dw9800sw_device, sd);
}

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};


static int dw9800sw_set_position(struct dw9800sw_device *dw9800sw, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dw9800sw->sd);

	return i2c_smbus_write_word_data(client, DW9800SW_SET_POSITION_ADDR,
					 swab16(val));
}

static int dw9800sw_release(struct dw9800sw_device *dw9800sw)
{
	int ret, val;
	int diff_dac = 0;
	int nStep_count = 0;
	int i = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9800sw->sd);

	diff_dac = DW9800SW_ORIGIN_FOCUS_POS - dw9800sw->focus->val;

	nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
		DW9800SW_MOVE_STEPS;

	val = dw9800sw->focus->val;

	for (i = 0; i < nStep_count; ++i) {
		val += (diff_dac < 0 ? (DW9800SW_MOVE_STEPS*(-1)) : DW9800SW_MOVE_STEPS);

		ret = dw9800sw_set_position(dw9800sw, val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
		usleep_range(DW9800SW_MOVE_DELAY_US,
			     DW9800SW_MOVE_DELAY_US + 1000);
	}

	// last step to origin
	ret = dw9800sw_set_position(dw9800sw, DW9800SW_ORIGIN_FOCUS_POS);
	if (ret) {
		LOG_INF("%s I2C failure: %d",
			__func__, ret);
		return ret;
	}

	i2c_smbus_write_byte_data(client, 0x02, 0x20);
	dw9800sw->active  = false;

	LOG_INF("-\n");

	return 0;
}

static int dw9800sw_init(struct dw9800sw_device *dw9800sw)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dw9800sw->sd);
	int ret = 0;
	char puSendCmdArray[7][2] = {
	{0x02, 0x01}, {0x02, 0x00}, {0xFE, 0xFE},
	{0x02, 0x02}, {0x06, 0x40}, {0x07, 0x60}, {0xFE, 0xFE},
	};
	unsigned char cmd_number;

	LOG_INF("+\n");

	client->addr = DW9800SW_I2C_SLAVE_ADDR >> 1;
	//ret = i2c_smbus_read_byte_data(client, 0x02);

	LOG_INF("Check HW version: %x\n", ret);

	for (cmd_number = 0; cmd_number < 7; cmd_number++) {
		if (puSendCmdArray[cmd_number][0] != 0xFE) {
			ret = i2c_smbus_write_byte_data(client,
					puSendCmdArray[cmd_number][0],
					puSendCmdArray[cmd_number][1]);

			if (ret < 0)
				return -1;
		} else {
			udelay(100);
		}
	}
	dw9800sw->active  = true;
	LOG_INF("-\n");

	return ret;
}

/* Power handling */
static int dw9800sw_power_off(struct dw9800sw_device *dw9800sw)
{
	int ret;

	LOG_INF("+\n");

	ret = dw9800sw_release(dw9800sw);
	if (ret)
		LOG_INF("dw9800sw release failed!\n");

	ret = regulator_disable(dw9800sw->vin);
	if (ret)
		return ret;

	ret = regulator_disable(dw9800sw->vdd);
	if (ret)
		return ret;

	if (dw9800sw->vcamaf_pinctrl && dw9800sw->vcamaf_off)
		ret = pinctrl_select_state(dw9800sw->vcamaf_pinctrl,
					dw9800sw->vcamaf_off);

	LOG_INF("-\n");

	return ret;
}

static int dw9800sw_power_on(struct dw9800sw_device *dw9800sw)
{
	int ret;

	LOG_INF("+\n");

	ret = regulator_enable(dw9800sw->vin);
	if (ret < 0)
		return ret;

	ret = regulator_enable(dw9800sw->vdd);
	if (ret < 0)
		return ret;

	if (dw9800sw->vcamaf_pinctrl && dw9800sw->vcamaf_on)
		ret = pinctrl_select_state(dw9800sw->vcamaf_pinctrl,
					dw9800sw->vcamaf_on);

	if (ret < 0)
		return ret;

	/*
	 * TODO(b/139784289): Confirm hardware requirements and adjust/remove
	 * the delay.
	 */
	usleep_range(DW9800SW_CTRL_DELAY_US, DW9800SW_CTRL_DELAY_US + 10);

	ret = dw9800sw_init(dw9800sw);
	if (ret < 0)
		goto fail;

	LOG_INF("-\n");

	return 0;

fail:
	regulator_disable(dw9800sw->vin);
	regulator_disable(dw9800sw->vdd);
	if (dw9800sw->vcamaf_pinctrl && dw9800sw->vcamaf_off) {
		pinctrl_select_state(dw9800sw->vcamaf_pinctrl,
				dw9800sw->vcamaf_off);
	}

	return ret;
}

static int dw9800sw_set_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	int loop_time = 0, status = 0;
	struct dw9800sw_device *dw9800sw = to_dw9800sw_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		/*wait for I2C bus idle*/
		while (loop_time < 20)
		{
			status = i2c_smbus_read_byte_data(v4l2_get_subdevdata(&dw9800sw->sd), DW9800SW_STATUS_ADDR);
			status = status & 0x01;//get reg 05 status
			LOG_INF("dw9800sw 0x05 status:%x", status);
			if(status == 0){
				break;
			}
			loop_time++;
			usleep_range(DW9800SW_POS_CTRL_DELAY_US, DW9800SW_POS_CTRL_DELAY_US + 100);
		}
		LOG_INF("pos(%d)\n", ctrl->val);
		ret = dw9800sw_set_position(dw9800sw, ctrl->val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
		g_last_pos = ctrl->val;
	}
	return 0;
}

static const struct v4l2_ctrl_ops dw9800sw_vcm_ctrl_ops = {
	.s_ctrl = dw9800sw_set_ctrl,
};

static int dw9800sw_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	struct dw9800sw_device *dw9800sw = sd_to_dw9800sw_vcm(sd);

	ret = dw9800sw_power_on(dw9800sw);
	if (ret < 0) {
		LOG_INF("power on fail, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int dw9800sw_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct dw9800sw_device *dw9800sw = sd_to_dw9800sw_vcm(sd);

	dw9800sw_power_off(dw9800sw);

	return 0;
}

static int dw9800sw_vcm_suspend(struct dw9800sw_device *dw9800sw)
{
	int ret, val;
	int diff_dac = 0;
	int nStep_count = 0;
	int i = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9800sw->sd);

	if (!dw9800sw->active)
		return 0;

	diff_dac = DW9800SW_ORIGIN_FOCUS_POS - dw9800sw->focus->val;
	nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
		DW9800SW_MOVE_STEPS;
	val = dw9800sw->focus->val;
	for (i = 0; i < nStep_count; ++i) {
		val += (diff_dac < 0 ? (DW9800SW_MOVE_STEPS*(-1)) : DW9800SW_MOVE_STEPS);

		ret = dw9800sw_set_position(dw9800sw, val);
		if (ret) {
			LOG_INF("%s I2C failure: %d", __func__, ret);
			return ret;
		}
		usleep_range(DW9800SW_CTRL_DELAY_US, DW9800SW_CTRL_DELAY_US + 10);
	}

	// last step to origin
	ret = dw9800sw_set_position(dw9800sw, DW9800SW_ORIGIN_FOCUS_POS);
	if (ret) {
		LOG_INF("%s I2C failure: %d", __func__, ret);
		return ret;
	}
	ret = i2c_smbus_write_byte_data(client, 0x02, 0x01);
	if (ret) {
		LOG_INF("I2C failure!!!\n");
	} else {
		dw9800sw->active = false;
		LOG_INF("enter stand by mode\n");
	}
	return ret;
}

static int dw9800sw_vcm_resume(struct dw9800sw_device *dw9800sw)
{
	int ret, diff_dac, nStep_count, i;
	int val = DW9800SW_ORIGIN_FOCUS_POS;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9800sw->sd);
	if (dw9800sw->active)
		return 0;
	ret = i2c_smbus_write_byte_data(client, 0x02, 0x00);
	if (ret) {
		LOG_INF("I2C failure!!!\n");
		return ret;
	}
	usleep_range(DW9800SW_CTRL_DELAY_US, DW9800SW_CTRL_DELAY_US + 10);
	diff_dac = dw9800sw->focus->val - DW9800SW_ORIGIN_FOCUS_POS;
	nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
			DW9800SW_MOVE_STEPS;
	for (i = 0; i < nStep_count; ++i) {
		val += (diff_dac < 0 ? (DW9800SW_MOVE_STEPS*(-1)) : DW9800SW_MOVE_STEPS);
		ret = dw9800sw_set_position(dw9800sw, val);
		if (ret) {
			LOG_INF("%s I2C failure: %d", __func__, ret);
			return ret;
		}
		usleep_range(DW9800SW_CTRL_DELAY_US, DW9800SW_CTRL_DELAY_US + 10);
	}
	ret = dw9800sw_set_position(dw9800sw, dw9800sw->focus->val);
	if (ret) {
		LOG_INF("%s I2C failure: %d", __func__, ret);
		return ret;
	}
	dw9800sw->active = true;
	LOG_INF("exit stand by mode\n");
	return ret;
}

static long dw9800sw_ops_core_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct dw9800sw_device *dw9800sw = sd_to_dw9800sw_vcm(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&dw9800sw->sd);
	LOG_INF("+\n");
	client->addr = DW9800SW_I2C_SLAVE_ADDR >> 1;

	switch (cmd) {
	case VCM_IOC_POWER_ON:
		ret = dw9800sw_vcm_resume(dw9800sw);
		break;
	case VCM_IOC_POWER_OFF:
		ret = dw9800sw_vcm_suspend(dw9800sw);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

static const struct v4l2_subdev_internal_ops dw9800sw_int_ops = {
	.open = dw9800sw_open,
	.close = dw9800sw_close,
};

static struct v4l2_subdev_core_ops dw9800sw_ops_core = {
	.ioctl = dw9800sw_ops_core_ioctl,
};

static const struct v4l2_subdev_ops dw9800sw_ops = {
	.core = &dw9800sw_ops_core,
};

static void dw9800sw_subdev_cleanup(struct dw9800sw_device *dw9800sw)
{
	v4l2_async_unregister_subdev(&dw9800sw->sd);
	v4l2_ctrl_handler_free(&dw9800sw->ctrls);
#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&dw9800sw->sd.entity);
#endif
}

static int dw9800sw_init_controls(struct dw9800sw_device *dw9800sw)
{
	struct v4l2_ctrl_handler *hdl = &dw9800sw->ctrls;
	const struct v4l2_ctrl_ops *ops = &dw9800sw_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	dw9800sw->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, DW9800SW_MAX_FOCUS_POS, DW9800SW_FOCUS_STEPS, 0);

	if (hdl->error)
		return hdl->error;

	dw9800sw->sd.ctrl_handler = hdl;

	return 0;
}

static int dw9800sw_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dw9800sw_device *dw9800sw;
	int ret;

	LOG_INF("+\n");

	dw9800sw = devm_kzalloc(dev, sizeof(*dw9800sw), GFP_KERNEL);
	if (!dw9800sw)
		return -ENOMEM;

	dw9800sw->vin = devm_regulator_get(dev, "vin");
	if (IS_ERR(dw9800sw->vin)) {
		ret = PTR_ERR(dw9800sw->vin);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vin regulator\n");
		return ret;
	}

	dw9800sw->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(dw9800sw->vdd)) {
		ret = PTR_ERR(dw9800sw->vdd);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vdd regulator\n");
		return ret;
	}

	dw9800sw->vcamaf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(dw9800sw->vcamaf_pinctrl)) {
		ret = PTR_ERR(dw9800sw->vcamaf_pinctrl);
		dw9800sw->vcamaf_pinctrl = NULL;
		LOG_INF("cannot get pinctrl\n");
	} else {
		dw9800sw->vcamaf_on = pinctrl_lookup_state(
			dw9800sw->vcamaf_pinctrl, "vcamaf_on");

		if (IS_ERR(dw9800sw->vcamaf_on)) {
			ret = PTR_ERR(dw9800sw->vcamaf_on);
			dw9800sw->vcamaf_on = NULL;
			LOG_INF("cannot get vcamaf_on pinctrl\n");
		}

		dw9800sw->vcamaf_off = pinctrl_lookup_state(
			dw9800sw->vcamaf_pinctrl, "vcamaf_off");

		if (IS_ERR(dw9800sw->vcamaf_off)) {
			ret = PTR_ERR(dw9800sw->vcamaf_off);
			dw9800sw->vcamaf_off = NULL;
			LOG_INF("cannot get vcamaf_off pinctrl\n");
		}
	}

	v4l2_i2c_subdev_init(&dw9800sw->sd, client, &dw9800sw_ops);
	dw9800sw->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dw9800sw->sd.internal_ops = &dw9800sw_int_ops;

	ret = dw9800sw_init_controls(dw9800sw);
	if (ret)
		goto err_cleanup;

#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&dw9800sw->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	dw9800sw->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

	ret = v4l2_async_register_subdev(&dw9800sw->sd);
	if (ret < 0)
		goto err_cleanup;

	LOG_INF("-\n");

	return 0;

err_cleanup:
	dw9800sw_subdev_cleanup(dw9800sw);
	return ret;
}

static void dw9800sw_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9800sw_device *dw9800sw = sd_to_dw9800sw_vcm(sd);

	LOG_INF("+\n");

	dw9800sw_subdev_cleanup(dw9800sw);

	LOG_INF("-\n");
}

static const struct i2c_device_id dw9800sw_id_table[] = {
	{ DW9800SW_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, dw9800sw_id_table);

static const struct of_device_id dw9800sw_of_table[] = {
	{ .compatible = "oplus,dw9800sw" },
	{ },
};
MODULE_DEVICE_TABLE(of, dw9800sw_of_table);

static struct i2c_driver dw9800sw_i2c_driver = {
	.driver = {
		.name = DW9800SW_NAME,
		.of_match_table = dw9800sw_of_table,
	},
	.probe_new  = dw9800sw_probe,
	.remove = dw9800sw_remove,
	.id_table = dw9800sw_id_table,
};

module_i2c_driver(dw9800sw_i2c_driver);

MODULE_AUTHOR("XXX");
MODULE_DESCRIPTION("DW9800SW VCM driver");
MODULE_LICENSE("GPL v2");
