// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2021 Oplus

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/videodev2.h>
#include <linux/pinctrl/consumer.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/pm_runtime.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>

#include <mt-plat/mtk_pwm.h>
#include <mt-plat/mtk_pwm_hal_pub.h>
#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT)
#include "flashlight-core.h"
#include <linux/power_supply.h>
#endif

#define DUNHUANG_NAME	"flashlights_dunhuang"

/* define device tree */
#ifndef DUNHUANG_DTNAME
#define DUNHUANG_DTNAME "mediatek,flashlights_dunhuang"
#endif

#define FAULT_TIMEOUT	(1<<0)
#define FAULT_THERMAL_SHUTDOWN	(1<<2)
#define FAULT_LED0_SHORT_CIRCUIT	(1<<5)
#define FAULT_LED1_SHORT_CIRCUIT	(1<<4)

/*  FLASH TIMEOUT DURATION
 *	min 32ms, step 32ms, max 1024ms
 */
#define DUNHUANG_FLASH_TOUT_MIN 40
#define DUNHUANG_FLASH_TOUT_STEP 100
#define DUNHUANG_FLASH_TOUT_MAX 700

/* fault mask */
#define FAULT_TIMEOUT	(1<<0)
#define FAULT_THERMAL_SHUTDOWN	(1<<2)
#define FAULT_LED0_SHORT_CIRCUIT	(1<<5)
#define FAULT_LED1_SHORT_CIRCUIT	(1<<4)
#define DUNHUANG_PINCTRL_PIN_FLASH_EN 0
#define DUNHUANG_PINCTRL_PIN_PWM_EN 1
#define DUNHUANG_PINCTRL_PIN_PWM_GPIO 2
#define DUNHUANG_PINCTRL_PIN_STATE_LOW 0
#define DUNHUANG_PINCTRL_PIN_STATE_HIGH 1

#define DUNHUANG_PINCTRL_STATE_FLASH_EN_HIGH "flash_light_en_pin_1"  //POIO 16
#define DUNHUANG_PINCTRL_STATE_FLASH_EN_LOW  "flash_light_en_pin_0"
#define DUNHUANG_PINCTRL_STATE_PWM_GPIO_HIGH "flash_light_flash_pin_1"//POIO 120
#define DUNHUANG_PINCTRL_STATE_PWM_GPIO_LOW  "flash_light_flash_pin_0"
#define DUNHUANG_PINCTRL_STATE_PWM "flash_light_pwm_pin"	//POIO 120

#define DUNHUANG_LEVEL_NUM 30
#define DUNHUANG_LEVEL_TORCH 3
#define DUNHUANG_LED_BRT_STEP 1000
#define DUNHUANG_TORCH_MIN_CURRENT 24000
#define DUNHUANG_TORCH_MAX_CURRENT 120000
#define DUNHUANG_TORCH_STEP 1000
#define DUNHUANG_FLASH_MIN_CURRENT 28000
#define DUNHUANG_FLASH_MAX_CURRENT 1180000
#define DUNHUANG_FLASH_STEP 12000

static const int dunhuang_current[DUNHUANG_LEVEL_NUM] = {40, 76, 100, 124, 148, 184, 220, 256, 292, 328, 364, 400, 436, 472, 508, 544, 580, 616, 652, 700, 748, 796, 844, 892, 940, 988, 1036, 1084, 1132, 1180};

static struct hrtimer dunhuang_timer;
static unsigned int dunhuang_timeout_ms;
static ktime_t ktime;
static int current_mode = V4L2_FLASH_LED_MODE_NONE;

static struct work_struct dunhuang_work;

enum dunhuang_led_id {
	DUNHUANG_LED0 = 0,
	DUNHUANG_LED1,
	DUNHUANG_LED_MAX
};

enum v4l2_flash_led_nums {
	DUNHUANG_CONTROL_LED0 = 2,
	DUNHUANG_CONTROL_LED1,
};
/* struct dunhuang_platform_data
 *
 * @max_flash_timeout: flash timeout
 * @max_flash_brt: flash mode led brightness
 * @max_torch_brt: torch mode led brightness
 */
struct dunhuang_platform_data {
	u32 max_flash_timeout;
	u32 max_flash_brt[DUNHUANG_LED_MAX];
	u32 max_torch_brt[DUNHUANG_LED_MAX];
};

enum led_enable {
	MODE_SHDN = 0x0,
	MODE_TORCH = 0x08,
	MODE_FLASH = 0x0C,
};

/**
 * struct dunhuang_flash
 *
 * @dev: pointer to &struct device
 * @pdata: platform data
 * @regmap: reg. map for i2c
 * @lock: muxtex for serial access.
 * @led_mode: V4L2 LED mode
 * @ctrls_led: V4L2 controls
 * @subdev_led: V4L2 subdev
 */
struct dunhuang_flash {
	struct device *dev;
	struct dunhuang_platform_data *pdata;
	struct regmap *regmap;
	struct mutex lock;

	enum v4l2_flash_led_mode led_mode;
	struct v4l2_ctrl_handler ctrls_led[DUNHUANG_LED_MAX];
	struct v4l2_subdev subdev_led[DUNHUANG_LED_MAX];
	struct device_node *dnode[DUNHUANG_LED_MAX];
	struct pinctrl *dunhuang_pinctrl;
	struct pinctrl_state *dunhuang_flash_en_high;
	struct pinctrl_state *dunhuang_flash_en_low;
	struct pinctrl_state *dunhuang_pwm_gpio_high;
	struct pinctrl_state *dunhuang_pwm_gpio_low;
	struct pinctrl_state *dunhuang_flash_pwm;
#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT)
	struct flashlight_device_id flash_dev_id[DUNHUANG_LED_MAX];
#endif
	unsigned int ori_current[DUNHUANG_LED_MAX];
};

/* define usage count */
static int use_count;

static struct dunhuang_flash *dunhuang_flash_data;

#define to_dunhuang_flash(_ctrl, _no)	\
	container_of(_ctrl->handler, struct dunhuang_flash, ctrls_led[_no])


/******************************************************************************
 * Pinctrl configuration 
 *****************************************************************************/
static int dunhuang_pinctrl_init(struct dunhuang_flash *flash)
{
	int ret = 0;
	pr_err("flashlights:dunhuang_pinctrl_init");
	/* get pinctrl */
	flash->dunhuang_pinctrl = devm_pinctrl_get(flash->dev);
	if (IS_ERR(flash->dunhuang_pinctrl)) {
		pr_info("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(flash->dunhuang_pinctrl);
		return ret;
	}

	/* Flashlight HWEN pin initialization */
	flash->dunhuang_flash_en_high = pinctrl_lookup_state(
			flash->dunhuang_pinctrl,
			DUNHUANG_PINCTRL_STATE_FLASH_EN_HIGH);
	if (IS_ERR(flash->dunhuang_flash_en_high)) {
		pr_info("Failed to init (%s)\n",
			DUNHUANG_PINCTRL_STATE_FLASH_EN_HIGH);
		ret = PTR_ERR(flash->dunhuang_flash_en_high);
	}
	flash->dunhuang_flash_en_low = pinctrl_lookup_state(
			flash->dunhuang_pinctrl,
			DUNHUANG_PINCTRL_STATE_FLASH_EN_LOW);
	if (IS_ERR(flash->dunhuang_flash_en_low)) {
		pr_info("Failed to init (%s)\n", DUNHUANG_PINCTRL_STATE_FLASH_EN_LOW);
		ret = PTR_ERR(flash->dunhuang_flash_en_low);
	}

	flash->dunhuang_pwm_gpio_high = pinctrl_lookup_state(
			flash->dunhuang_pinctrl,
			DUNHUANG_PINCTRL_STATE_PWM_GPIO_HIGH);
	if (IS_ERR(flash->dunhuang_pwm_gpio_high)) {
		pr_info("Failed to init (%s)\n",
			DUNHUANG_PINCTRL_STATE_PWM_GPIO_HIGH);
		ret = PTR_ERR(flash->dunhuang_pwm_gpio_high);
	}
	flash->dunhuang_pwm_gpio_low = pinctrl_lookup_state(
			flash->dunhuang_pinctrl,
			DUNHUANG_PINCTRL_STATE_PWM_GPIO_LOW);
	if (IS_ERR(flash->dunhuang_pwm_gpio_low)) {
		pr_info("Failed to init (%s)\n", DUNHUANG_PINCTRL_STATE_PWM_GPIO_LOW);
		ret = PTR_ERR(flash->dunhuang_pwm_gpio_low);
	}

    flash->dunhuang_flash_pwm = pinctrl_lookup_state(
			flash->dunhuang_pinctrl, DUNHUANG_PINCTRL_STATE_PWM);
	if (IS_ERR(flash->dunhuang_flash_pwm)) {
		pr_info("Failed to init (%s)\n", DUNHUANG_PINCTRL_STATE_PWM);
		ret = PTR_ERR(flash->dunhuang_flash_pwm);
	}

	return ret;
}

int mt_flashlight_led_set_pwm(int pwm_num,u32 level )
{
	struct pwm_spec_config pwm_setting;
	memset(&pwm_setting, 0, sizeof(struct pwm_spec_config));
	pwm_setting.pwm_no = pwm_num; /* PWM0 set 0,PWM1 set 1,PWM2 set 2,PWM3 set 3 */
	pwm_setting.mode = PWM_MODE_OLD;
	pwm_setting.pmic_pad = 0;
	pwm_setting.clk_div = CLK_DIV1;
	pwm_setting.clk_src = PWM_CLK_OLD_MODE_BLOCK;
	pwm_setting.PWM_MODE_OLD_REGS.IDLE_VALUE = 0;
	pwm_setting.PWM_MODE_OLD_REGS.GUARD_VALUE = 0;
	pwm_setting.PWM_MODE_OLD_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_OLD_REGS.WAVE_NUM = 0;
	pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 1000;
	pwm_setting.PWM_MODE_OLD_REGS.THRESH = level *10;
	mt_pwm_clk_sel_hal(pwm_num,CLK_26M);
	pwm_set_spec_config(&pwm_setting);

	return 0;
}

static int dunhuang_pinctrl_set(struct dunhuang_flash *flash, int pin, int state)
{
	int ret = 0;
	pr_err("flashlights:dunhuang_pinctrl_set");
	if (IS_ERR(flash->dunhuang_pinctrl)) {
		pr_info("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
		case DUNHUANG_PINCTRL_PIN_FLASH_EN:
			if (state == DUNHUANG_PINCTRL_PIN_STATE_LOW &&!IS_ERR(flash->dunhuang_flash_en_low))
				ret = pinctrl_select_state(flash->dunhuang_pinctrl, flash->dunhuang_flash_en_low);
			else if (state == DUNHUANG_PINCTRL_PIN_STATE_HIGH &&!IS_ERR(flash->dunhuang_flash_en_high))
				ret = pinctrl_select_state(flash->dunhuang_pinctrl, flash->dunhuang_flash_en_high);
			else
				pr_info("set err, pin(%d) state(%d)\n", pin, state);
			break;
		case DUNHUANG_PINCTRL_PIN_PWM_EN:
			ret =pinctrl_select_state(flash->dunhuang_pinctrl, flash->dunhuang_flash_pwm);
			break;
		case DUNHUANG_PINCTRL_PIN_PWM_GPIO:
			if (state == DUNHUANG_PINCTRL_PIN_STATE_LOW &&!IS_ERR(flash->dunhuang_pwm_gpio_low))
				ret = pinctrl_select_state(flash->dunhuang_pinctrl, flash->dunhuang_pwm_gpio_low);
			else if (state == DUNHUANG_PINCTRL_PIN_STATE_HIGH &&!IS_ERR(flash->dunhuang_pwm_gpio_high))
				ret = pinctrl_select_state(flash->dunhuang_pinctrl, flash->dunhuang_pwm_gpio_high);
			else
				pr_info("set err, pin(%d) state(%d)\n", pin, state);
			break;

	default:
		pr_info("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_info("pin(%d) state(%d)\n", pin, state);

	return ret;
}

/* led1/2 enable/disable */
static int dunhuang_enable_ctrl(struct dunhuang_flash *flash,
								enum dunhuang_led_id led_no, bool on)
{
	int rval = 0;
	int tempPWM = -1;
	pr_info("%s: enable:%d led_mode:0x%x, brt = %u", __func__, on, flash->led_mode, flash->ori_current[led_no]);
	if (on) {
		if (flash->led_mode == V4L2_FLASH_LED_MODE_TORCH) {
			tempPWM = (flash->ori_current[led_no] - DUNHUANG_TORCH_MIN_CURRENT) / DUNHUANG_TORCH_STEP;
			if (tempPWM > 96)
				tempPWM = 96;
			dunhuang_pinctrl_set(flash, DUNHUANG_PINCTRL_PIN_FLASH_EN,0);	//pull down ENF
			dunhuang_pinctrl_set(flash, DUNHUANG_PINCTRL_PIN_PWM_GPIO,1);	//pull up ENM
			mdelay(5);														//delay more than 5ms
			dunhuang_pinctrl_set(flash, DUNHUANG_PINCTRL_PIN_PWM_EN,1);	//set pwm mode
			mt_flashlight_led_set_pwm(1, tempPWM);
			pr_info("DUNHUANG_TORCH:current pwm:%d", tempPWM);
		} else if (flash->led_mode == V4L2_FLASH_LED_MODE_FLASH) {
			tempPWM = (flash->ori_current[led_no] - DUNHUANG_FLASH_MIN_CURRENT) / DUNHUANG_FLASH_STEP;
			if (tempPWM > 96)
				tempPWM = 96;
			dunhuang_pinctrl_set(flash, DUNHUANG_PINCTRL_PIN_FLASH_EN,1);	//pull down ENF
			dunhuang_pinctrl_set(flash, DUNHUANG_PINCTRL_PIN_PWM_EN,1);	//set pwm mode

			mt_flashlight_led_set_pwm(1, tempPWM);				//flash pwm tempPWM%
			pr_info("DUNHUANG_FLASH:current pwm:%d",tempPWM);
		}
	} else {
		int state = DUNHUANG_PINCTRL_PIN_STATE_LOW;
		dunhuang_pinctrl_set(flash, DUNHUANG_PINCTRL_PIN_FLASH_EN, state);
		dunhuang_pinctrl_set(flash, DUNHUANG_PINCTRL_PIN_PWM_GPIO, state);
		mt_pwm_disable(1,1);
	}
	pr_debug("%s: return val:%d", __func__,  rval);
	return rval;
}

static void dunhuang_work_disable(struct work_struct *data)
{
	pr_info("work queue callback\n");
	dunhuang_enable_ctrl(dunhuang_flash_data, 0, false);
}

static enum hrtimer_restart dunhuang_timer_func(struct hrtimer *timer)
{
	schedule_work(&dunhuang_work);
	return HRTIMER_NORESTART;
}

/* torch1/2 brightness control */
static int dunhuang_torch_brt_ctrl(struct dunhuang_flash *flash,
				 enum dunhuang_led_id led_no, unsigned int brt)
{
    pr_info("%s %d brt:%u", __func__, led_no, brt);
	if (brt < DUNHUANG_TORCH_MIN_CURRENT) {
		pr_info("current set error, close flash");
		return dunhuang_enable_ctrl(flash, led_no, false);
	}
	if (brt > DUNHUANG_TORCH_MAX_CURRENT) {
		brt = DUNHUANG_TORCH_MAX_CURRENT;
	}
	flash->ori_current[led_no] = brt;
	if (current_mode == V4L2_FLASH_LED_MODE_TORCH) {
		pr_info("dunhuang_torch:current mode is %d,turn on torch", current_mode);
		dunhuang_enable_ctrl(flash, led_no, true);
	}
    pr_info("dunhuang_flashlights current brt:%u", brt);

	return 0;
}

/* flash1/2 brightness control */
static int dunhuang_flash_brt_ctrl(struct dunhuang_flash *flash,
				 enum dunhuang_led_id led_no, unsigned int brt)
{
	int rval = 1;
	pr_info("%s %d brt:%u", __func__, led_no, brt);
	if (brt < DUNHUANG_FLASH_MIN_CURRENT + DUNHUANG_FLASH_STEP) {
		pr_info("current set error, close flash");
		return dunhuang_enable_ctrl(flash, led_no, false);
	}
	if (brt > DUNHUANG_FLASH_MAX_CURRENT) {
		brt = DUNHUANG_FLASH_MAX_CURRENT;
	}
	flash->ori_current[led_no] = brt;
    pr_info("dunhuang_flashlights current brt:%u", brt);

	return rval;
}

/* flash1/2 timeout control */
static int dunhuang_flash_tout_ctrl(struct dunhuang_flash *flash,
				unsigned int tout)
{
	int rval = 0;
	unsigned int s, ns;

	dunhuang_timeout_ms = tout;
	if (dunhuang_timeout_ms) {
		s = dunhuang_timeout_ms / 1000;
		ns = dunhuang_timeout_ms % 1000 * 1000000;
		ktime = ktime_set(s, ns);
	}

	return rval;
}

static int dunhuang_flash_set_tout(bool enable)
{
	if (enable)
		hrtimer_start(&dunhuang_timer, ktime, HRTIMER_MODE_REL);
	else
		hrtimer_cancel(&dunhuang_timer);
	return 0;
}

/* v4l2 controls  */
static int dunhuang_get_ctrl(struct v4l2_ctrl *ctrl, enum dunhuang_led_id led_no)
{
	struct dunhuang_flash *flash = to_dunhuang_flash(ctrl, led_no);
	int rval = 0;

	mutex_lock(&flash->lock);

	pr_err("%s: ", __func__);
	if (ctrl->id == V4L2_CID_FLASH_FAULT) {
		ctrl->cur.val = FAULT_LED0_SHORT_CIRCUIT;
	}

//out:
	mutex_unlock(&flash->lock);
	return rval;
}

static int dunhuang_set_ctrl(struct v4l2_ctrl *ctrl, enum dunhuang_led_id led_no)
{
	struct dunhuang_flash *flash = to_dunhuang_flash(ctrl, led_no);
	int rval = -EINVAL;

	pr_err("%s: HYL led_no:%d ctrID:0x%x, ctrlVal:0x%x", __func__, led_no, ctrl->id, ctrl->val);
	mutex_lock(&flash->lock);

	switch (ctrl->id) {
	case V4L2_CID_FLASH_LED_MODE:
		pr_err("%s: 1", __func__);
		flash->led_mode = ctrl->val;
		rval = 0;
		current_mode = ctrl->val;
		if (flash->led_mode == V4L2_FLASH_LED_MODE_NONE)
			rval = dunhuang_enable_ctrl(flash, led_no, false);
		else if (flash->led_mode == V4L2_FLASH_LED_MODE_TORCH)
			rval = dunhuang_enable_ctrl(flash, led_no, true);
		break;
	case V4L2_CID_FLASH_STROBE_SOURCE:
		pr_err("%s: 2", __func__);
		rval = 0;
		if (rval < 0)
			goto err_out;
		break;

	case V4L2_CID_FLASH_STROBE:
		pr_err("%s: 3", __func__);
		rval = 0;
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH) {
			rval = -EBUSY;
			goto err_out;
		}
		flash->led_mode = V4L2_FLASH_LED_MODE_FLASH;
		dunhuang_flash_set_tout(true);
		rval = dunhuang_enable_ctrl(flash, led_no, true);
		break;

	case V4L2_CID_FLASH_STROBE_STOP:
		pr_err("%s: 4", __func__);
		rval = 0;
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH) {
			rval = -EBUSY;
			goto err_out;
		}
		dunhuang_flash_set_tout(false);
		rval = dunhuang_enable_ctrl(flash, led_no, false);
		flash->led_mode = V4L2_FLASH_LED_MODE_NONE;
		break;

	case V4L2_CID_FLASH_TIMEOUT:
		pr_err("%s: 5", __func__);
		rval = dunhuang_flash_tout_ctrl(flash, ctrl->val);
		break;

	case V4L2_CID_FLASH_INTENSITY:
		pr_err("%s: 6", __func__);
		rval = dunhuang_flash_brt_ctrl(flash, led_no, ctrl->val);
		break;

	case V4L2_CID_FLASH_TORCH_INTENSITY:
		pr_err("%s: 7", __func__);
		rval = dunhuang_torch_brt_ctrl(flash, led_no, ctrl->val);
		break;
	}

err_out:
	mutex_unlock(&flash->lock);
	return rval;
}

static int dunhuang_led0_get_ctrl(struct v4l2_ctrl *ctrl)
{
	return dunhuang_get_ctrl(ctrl, DUNHUANG_LED0);
}

static int dunhuang_led0_set_ctrl(struct v4l2_ctrl *ctrl)
{
	return dunhuang_set_ctrl(ctrl, DUNHUANG_LED0);
}

static const struct v4l2_ctrl_ops dunhuang_led_ctrl_ops[DUNHUANG_LED_MAX] = {
	[DUNHUANG_LED0] = {
			.g_volatile_ctrl = dunhuang_led0_get_ctrl,
			.s_ctrl = dunhuang_led0_set_ctrl,
			}
};

static int dunhuang_init_controls(struct dunhuang_flash *flash,
				enum dunhuang_led_id led_no)
{
	struct v4l2_ctrl *fault;
	u32 max_flash_brt = flash->pdata->max_flash_brt[led_no];
	u32 max_torch_brt = flash->pdata->max_torch_brt[led_no];
	struct v4l2_ctrl_handler *hdl = &flash->ctrls_led[led_no];
	const struct v4l2_ctrl_ops *ops = &dunhuang_led_ctrl_ops[led_no];

	v4l2_ctrl_handler_init(hdl, 8);

	/* flash mode */
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_FLASH_LED_MODE,
			       V4L2_FLASH_LED_MODE_TORCH, ~0x7,
			       V4L2_FLASH_LED_MODE_NONE);
	flash->led_mode = V4L2_FLASH_LED_MODE_NONE;

	/* flash source */
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_FLASH_STROBE_SOURCE,
			       0x1, ~0x3, V4L2_FLASH_STROBE_SOURCE_SOFTWARE);

	/* flash strobe */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_STROBE, 0, 0, 0, 0);

	/* flash strobe stop */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_STROBE_STOP, 0, 0, 0, 0);

	/* flash strobe timeout */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_TIMEOUT,
			  DUNHUANG_FLASH_TOUT_MIN,
			  flash->pdata->max_flash_timeout,
			  DUNHUANG_FLASH_TOUT_STEP,
			  flash->pdata->max_flash_timeout);

	/* flash brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_INTENSITY,
			  DUNHUANG_FLASH_MIN_CURRENT + DUNHUANG_FLASH_STEP, max_flash_brt,
			  DUNHUANG_LED_BRT_STEP, max_flash_brt);

	/* torch brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_TORCH_INTENSITY,
			  DUNHUANG_TORCH_MIN_CURRENT + DUNHUANG_TORCH_STEP, max_torch_brt,
			  DUNHUANG_LED_BRT_STEP, max_torch_brt);

	/* fault */
	fault = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_FAULT, 0,
				  V4L2_FLASH_FAULT_OVER_VOLTAGE
				  | V4L2_FLASH_FAULT_OVER_TEMPERATURE
				  | V4L2_FLASH_FAULT_SHORT_CIRCUIT
				  | V4L2_FLASH_FAULT_TIMEOUT, 0, 0);
	if (fault != NULL)
		fault->flags |= V4L2_CTRL_FLAG_VOLATILE;

	if (hdl->error)
		return hdl->error;

	flash->subdev_led[led_no].ctrl_handler = hdl;
	return 0;
}

/* initialize device */
static const struct v4l2_subdev_ops dunhuang_ops = {
	.core = NULL,
};

static void dunhuang_v4l2_pdev_subdev_init(struct v4l2_subdev *sd,
		struct device *client,
		const struct v4l2_subdev_ops *ops)
{
	pr_err("flashlights:dunhuang_v4l2_i2c_subdev_init");
	v4l2_subdev_init(sd, ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	/* the owner is the same as the i2c_client's driver owner */
	sd->owner = client->driver->owner;
	sd->dev = client;
	/* i2c_client and v4l2_subdev point to one another */
	v4l2_set_subdevdata(sd, client);
	/* initialize name */
	/*snprintf(sd->name, sizeof(sd->name), "%s-%04x",
		client->dev.driver->name,
		client->addr);*/
}

static int dunhuang_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;

	pr_info("%s\n", __func__);

	ret = pm_runtime_get_sync(sd->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(sd->dev);
		return ret;
	}

	return 0;
}

static int dunhuang_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pr_info("%s\n", __func__);

	pm_runtime_put(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops dunhuang_int_ops = {
	.open = dunhuang_open,
	.close = dunhuang_close,
};

static int dunhuang_subdev_init(struct dunhuang_flash *flash,
			      enum dunhuang_led_id led_no, char *led_name)
{
	struct device_node *np = flash->dev->of_node, *child;
	const char *fled_name = "flash";
	int rval;

	// pr_info("%s %d", __func__, led_no);

	dunhuang_v4l2_pdev_subdev_init(&flash->subdev_led[led_no], flash->dev, &dunhuang_ops);
	flash->subdev_led[led_no].flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	flash->subdev_led[led_no].internal_ops = &dunhuang_int_ops;
	strscpy(flash->subdev_led[led_no].name, led_name,
		sizeof(flash->subdev_led[led_no].name));

	for (child = of_get_child_by_name(np, fled_name); child;
			child = of_find_node_by_name(child, fled_name)) {
		int rv;
		u32 reg = 0;

		rv = of_property_read_u32(child, "reg", &reg);
		if (rv)
			continue;

		if (reg == led_no) {
			flash->dnode[led_no] = child;
			flash->subdev_led[led_no].fwnode =
				of_fwnode_handle(flash->dnode[led_no]);
		}
	}

	rval = dunhuang_init_controls(flash, led_no);
	if (rval)
		goto err_out;
	rval = media_entity_pads_init(&flash->subdev_led[led_no].entity, 0, NULL);
	if (rval < 0)
		goto err_out;
	flash->subdev_led[led_no].entity.function = MEDIA_ENT_F_FLASH;

	rval = v4l2_async_register_subdev(&flash->subdev_led[led_no]);
	if (rval < 0)
		goto err_out;

	return rval;

err_out:
	v4l2_ctrl_handler_free(&flash->ctrls_led[led_no]);
	return rval;
}

/* flashlight init */
static int dunhuang_init(struct dunhuang_flash *flash)
{
	int rval = 0;
	//unsigned int reg_val = 0;

	int state = DUNHUANG_PINCTRL_PIN_STATE_LOW;

	dunhuang_pinctrl_set(flash, DUNHUANG_PINCTRL_PIN_FLASH_EN, state);
	dunhuang_pinctrl_set(flash, DUNHUANG_PINCTRL_PIN_PWM_GPIO, state);

	/* set timeout */
	rval = dunhuang_flash_tout_ctrl(flash, 400);
	if (rval < 0)
		return rval;
	/* output disable */
	flash->led_mode = V4L2_FLASH_LED_MODE_NONE;
	//rval = dunhuang_mode_ctrl(flash);

	return rval;
}

/* flashlight uninit */
static int dunhuang_uninit(struct dunhuang_flash *flash)
{
	int state = DUNHUANG_PINCTRL_PIN_STATE_LOW;

	dunhuang_pinctrl_set(flash, DUNHUANG_PINCTRL_PIN_FLASH_EN, state);
	dunhuang_pinctrl_set(flash, DUNHUANG_PINCTRL_PIN_PWM_GPIO, state);
	return 0;
}

static int dunhuang_flash_open(void)
{

	pr_info("%s: dunhuang_flash_open", __func__);

	return 0;
}

static int dunhuang_flash_release(void)
{
	return 0;
}

static int dunhuang_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_ONOFF:
		pr_err("dunhuang_ioctl FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if ((int)fl_arg->arg) {
			dunhuang_torch_brt_ctrl(dunhuang_flash_data, channel, 25000);
			dunhuang_flash_data->led_mode = V4L2_FLASH_LED_MODE_TORCH;
			dunhuang_enable_ctrl(dunhuang_flash_data, channel, true);
		} else {
			dunhuang_flash_data->led_mode = V4L2_FLASH_LED_MODE_NONE;
			dunhuang_enable_ctrl(dunhuang_flash_data, channel, false);
		}
		break;

	case OPLUS_FLASH_IOC_SELECT_LED_NUM:
		pr_err("dunhuang_ioctl OPLUS_FLASH_IOC_SELECT_LED_NUM(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == DUNHUANG_CONTROL_LED0 || fl_arg->arg == DUNHUANG_CONTROL_LED1) {
			dunhuang_flash_data->led_mode = V4L2_FLASH_LED_MODE_FLASH;
		} else {
			dunhuang_flash_data->led_mode = V4L2_FLASH_LED_MODE_NONE;
		//	dunhuang_mode_ctrl(dunhuang_flash_data);
			dunhuang_enable_ctrl(dunhuang_flash_data, channel, false);
		}
		break;

	default:
		pr_err("dunhuang_ioctl No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int dunhuang_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	//mutex_lock(&dunhuang_mutex);
	if (set) {
		if (!use_count)
			ret = dunhuang_init(dunhuang_flash_data);
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = dunhuang_uninit(dunhuang_flash_data);
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	//mutex_unlock(&dunhuang_mutex);

	return 0;
}

static ssize_t dunhuang_strobe_store(struct flashlight_arg arg)
{
	dunhuang_set_driver(1);
	dunhuang_flash_brt_ctrl(dunhuang_flash_data, arg.channel,
				dunhuang_current[arg.level] * 1000);

	dunhuang_flash_data->led_mode = V4L2_FLASH_LED_MODE_FLASH;
	dunhuang_enable_ctrl(dunhuang_flash_data, arg.channel, true);
	msleep(arg.dur);
	dunhuang_flash_data->led_mode = V4L2_FLASH_LED_MODE_NONE;
	dunhuang_enable_ctrl(dunhuang_flash_data, arg.channel, false);
	dunhuang_set_driver(0);
	return 0;
}

static struct flashlight_operations dunhuang_flash_ops = {
	dunhuang_flash_open,
	dunhuang_flash_release,
	dunhuang_ioctl,
	dunhuang_strobe_store,
	dunhuang_set_driver
};

static int dunhuang_parse_dt(struct dunhuang_flash *flash)
{
	struct device_node *np, *cnp;
	struct device *dev = flash->dev;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node)
		return -ENODEV;

	np = dev->of_node;
	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type",
					&flash->flash_dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp,
					"ct", &flash->flash_dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp,
					"part", &flash->flash_dev_id[i].part))
			goto err_node_put;
		snprintf(flash->flash_dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				flash->subdev_led[i].name);
		flash->flash_dev_id[i].channel = i;
		flash->flash_dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				flash->flash_dev_id[i].type,
				flash->flash_dev_id[i].ct,
				flash->flash_dev_id[i].part,
				flash->flash_dev_id[i].name,
				flash->flash_dev_id[i].channel,
				flash->flash_dev_id[i].decouple);
		if (flashlight_dev_register_by_device_id(&flash->flash_dev_id[i],
			&dunhuang_flash_ops))
			return -EFAULT;
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int dunhuang_probe(struct platform_device *pdev)
{
	struct dunhuang_flash *flash;
	struct dunhuang_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int rval;

	pr_info("%s:%d", __func__, __LINE__);

	flash = devm_kzalloc(&pdev->dev, sizeof(*flash), GFP_KERNEL);
	if (flash == NULL)
		return -ENOMEM;

	/* if there is no platform data, use chip default value */
	if (pdata == NULL) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL)
			return -ENODEV;
		pdata->max_flash_timeout = 700;
		/* led 1 */
		pdata->max_flash_brt[DUNHUANG_LED0] = DUNHUANG_FLASH_MAX_CURRENT + 1;
		pdata->max_torch_brt[DUNHUANG_LED0] = DUNHUANG_TORCH_MAX_CURRENT + 1;
	}
	flash->pdata = pdata;
	flash->dev = &pdev->dev;
	mutex_init(&flash->lock);
	dunhuang_flash_data = flash;

	INIT_WORK(&dunhuang_work, dunhuang_work_disable);
	hrtimer_init(&dunhuang_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dunhuang_timer.function = dunhuang_timer_func;
	dunhuang_timeout_ms = 700;
    dunhuang_flash_tout_ctrl(flash, dunhuang_timeout_ms);
	rval = dunhuang_pinctrl_init(flash);
	if (rval < 0)
		return rval;

	rval = dunhuang_subdev_init(flash, DUNHUANG_LED0, DUNHUANG_NAME);
	if (rval < 0)
		return rval;

	pm_runtime_enable(flash->dev);

	rval = dunhuang_parse_dt(flash);

	pr_info("%s:%d", __func__, __LINE__);
	return 0;
}

static int dunhuang_remove(struct platform_device *pdev)
{
	struct dunhuang_flash *flash = dunhuang_flash_data;
	unsigned int i;

	for (i = DUNHUANG_LED0; i < DUNHUANG_LED_MAX; i++) {
		v4l2_device_unregister_subdev(&flash->subdev_led[i]);
		v4l2_ctrl_handler_free(&flash->ctrls_led[i]);
		media_entity_cleanup(&flash->subdev_led[i].entity);
	}

	pm_runtime_disable(&pdev->dev);
	flush_work(&dunhuang_work);
	pm_runtime_set_suspended(&pdev->dev);
	return 0;
}

static int __maybe_unused dunhuang_suspend(struct device *dev)
{
	pr_info("%s %d", __func__, __LINE__);

	return 0;//dunhuang_uninit(flash);
}

static int __maybe_unused dunhuang_resume(struct device *dev)
{

	pr_info("%s %d", __func__, __LINE__);

	return 0;//dunhuang_init(flash);
}

#ifdef CONFIG_OF
static const struct of_device_id dunhuang_of_table[] = {
	{ .compatible = DUNHUANG_DTNAME },
	{ },
};
MODULE_DEVICE_TABLE(of, dunhuang_of_table);
#else
static struct platform_device dunhuang_pwm_platform_device[] = {
	{
		.name = DUNHUANG_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, dunhuang_pwm_platform_device);
#endif

static const struct dev_pm_ops dunhuang_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
		pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(dunhuang_suspend, dunhuang_resume, NULL)
};

static struct platform_driver dunhuang_platform_driver = {
	.driver = {
		   .name = DUNHUANG_NAME,
		   .pm = &dunhuang_pm_ops,
#ifdef CONFIG_OF
		   .of_match_table = dunhuang_of_table,
#endif
		   },
	.probe = dunhuang_probe,
	.remove = dunhuang_remove,
};

static int __init flashlight_dunhuang_init(void)
{
	int ret;

	pr_info("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&dunhuang_pwm_platform_device);
	if (ret) {
		pr_info("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&dunhuang_platform_driver);
	if (ret) {
		pr_info("Failed to register platform driver\n");
		return ret;
	}

	pr_info("Init done.\n");

	return 0;
}

static void __exit flashlight_dunhuang_exit(void)
{
	pr_info("Exit start.\n");

	platform_driver_unregister(&dunhuang_platform_driver);

	pr_info("Exit done.\n");
}

module_init(flashlight_dunhuang_init);
module_exit(flashlight_dunhuang_exit);

MODULE_AUTHOR("XXX");
MODULE_DESCRIPTION("dunhuang LED flash v4l2 driver");
MODULE_LICENSE("GPL");
