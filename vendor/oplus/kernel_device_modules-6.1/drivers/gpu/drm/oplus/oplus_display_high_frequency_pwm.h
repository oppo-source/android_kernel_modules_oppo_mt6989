/***************************************************************
** Copyright (C),  2023,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_high_frequency_pwm.h
** Description : oplus high frequency PWM
** Version : 1.0
** Date : 2023/05/23
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Ping      2023/05/23        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_DISPLAY_HIGH_FREQUENCY_PWM_H_
#define _OPLUS_DISPLAY_HIGH_FREQUENCY_PWM_H_

/* please just only include linux common head file  */
#include <linux/kobject.h>
#include <linux/iio/consumer.h>

enum oplus_pwm_trubo_type {
	OPLUS_PWM_TRUBO_CLOSE = 0,
	OPLUS_PWM_TRUBO_SWITCH = 1,
	OPLUS_PWM_TRUBO_GLOBAL_OPEN_NO_SWITCH = 2,
};

enum PWM_SWITCH_STATE{
	PWM_SWITCH_DC_STATE = 0,
	PWM_SWITCH_HPWM_STATE,
	PWM_SWITCH_ONEPULSE_STATE,
};

enum PWM_SWITCH_BL_THRESHOLD{
	PWM_NORMAL_BL = 0,
	PWM_LOW_THRESHOLD = 1,
	PWM_HIGH_THRESHOLD = 2,
	PWM_MAX_THRESHOLD
};

enum oplus_pwm_turbo_log_level {
	OPLUS_PWM_TURBO_LOG_LEVEL_ERR = 0,
	OPLUS_PWM_TURBO_LOG_LEVEL_WARN = 1,
	OPLUS_PWM_TURBO_LOG_LEVEL_INFO = 2,
	OPLUS_PWM_TURBO_LOG_LEVEL_DEBUG = 3,
};

enum oplus_demura_mode {
	OPLUS_DEMURA_DBV_MODE0 = 0,
	OPLUS_DEMURA_DBV_MODE1 = 1,
	OPLUS_DEMURA_DBV_MODE2 = 2,
	OPLUS_DEMURA_DBV_MODE3 = 3,
	OPLUS_DEMURA_DBV_MODE4 = 4,
	OPLUS_DEMURA_DBV_MODE_MAX,
};

struct oplus_demura_setting_table {
	bool oplus_bl_demura_dbv_support;
	int bl_demura_mode;
	int demura_switch_dvb1;
	int demura_switch_dvb2;
	int demura_switch_dvb3;
	int demura_switch_dvb4;
};

enum pwm_switch_cmd_id {
	PWM_SWITCH_3TO18_RESTORE = 0,
	PWM_SWITCH_18TO3_RESTORE = 1,
	PWM_SWITCH_18TO3 = 2,
	PWM_SWITCH_3TO18 = 3,
	PWM_SWITCH_1TO3 = 4,
	PWM_SWITCH_3TO1 = 5,
	PWM_SWITCH_1TO18_RESTORE = 6,
	PWM_SWITCH_18TO1_RESTORE = 7,
	PWM_SWITCH_18TO1 = 8,
	PWM_SWITCH_1TO18 = 9,
};

/***  pwm turbo initialize params dtsi config   *****************
oplus,pwm-turbo-support= <1>;
oplus,pwm-turbo-plus-dbv=<0x643>;
oplus,pwm-turbo-wait-te=<1>;
********************************************************/
/* oplus pwm turbo initialize params ***************/
struct oplus_pwm_turbo_params {
	unsigned int   config;									/* int oplus,pwm-turbo-support */
	unsigned int   pwm_wait_te;
	unsigned int   lhbm_off_wait;
	unsigned int   pwm_fps_mode;
	bool pwm_turbo_support;									/* bool oplus,pwm-turbo-support */
	bool pwm_turbo_enabled;
	bool pwm_switch_support;
	bool pwm_power_on;
	u32  pwm_bl_threshold;									/* switch bl plus oplus,pwm-switch-backlight-threshold */
	u32  oplus_pwm_switch_state;
	u32  oplus_pwm_threshold;
	u32  pwm_pul_cmd_id;
	bool pwm_onepulse_support;
	bool pwm_onepulse_enabled;
	bool oplus_pwm_switch_state_changed;
	u32  pwm_onepulse_switch_mode;
};

/* -------------------- extern ---------------------------------- */
extern unsigned int pwm_turbo_log;

/* -------------------- pwm turbo debug log-------------------------------------------  */
#define PWM_TURBO_ERR(fmt, arg...)	\
	do {	\
		if (pwm_turbo_log >= OPLUS_PWM_TURBO_LOG_LEVEL_ERR)	\
			pr_err("[PWM_TURBO][ERR][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define PWM_TURBO_WARN(fmt, arg...)	\
	do {	\
		if (pwm_turbo_log >= OPLUS_PWM_TURBO_LOG_LEVEL_WARN)	\
			pr_warn("[PWM_TURBO][WARN][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define PWM_TURBO_INFO(fmt, arg...)	\
	do {	\
		if (pwm_turbo_log >= OPLUS_PWM_TURBO_LOG_LEVEL_INFO)	\
			pr_info("[PWM_TURBO][INFO][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

#define PWM_TURBO_DEBUG(fmt, arg...)	\
	do {	\
		if (pwm_turbo_log >= OPLUS_PWM_TURBO_LOG_LEVEL_DEBUG)	\
			pr_info("[PWM_TURBO][DEBUG][%s:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg);	\
	} while (0)

/* -------------------- function implementation ---------------------------------------- */
int oplus_pwm_turbo_probe(struct device *dev);
void oplus_display_panel_wait_te(int cnt);
bool pwm_turbo_support(void);
int get_pwm_turbo_plus_bl(void);
int get_pwm_turbo_fps_mode(void);
inline bool get_pwm_turbo_states(void);
inline bool oplus_panel_pwm_turbo_is_enabled(void);
inline bool oplus_panel_pwm_turbo_switch_state(void);
int oplus_display_panel_set_pwm_status(void *data);
int oplus_display_panel_get_pwm_status(void *buf);
void set_pwm_turbo_switch_state(enum PWM_SWITCH_STATE state);
void set_pwm_turbo_power_on(bool en);
/* -------------------- oplus api nodes ----------------------------------------------- */
ssize_t  oplus_display_get_high_pwm(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf);
ssize_t  oplus_display_set_high_pwm(struct kobject *kobj,
                struct kobj_attribute *attr, const char *buf, size_t count);
inline bool oplus_panel_pwm_onepulse_is_enabled(void);
inline bool oplus_panel_pwm_onepulse_switch_state(void);
int oplus_panel_update_pwm_pulse_lock(bool enabled);
int oplus_display_panel_set_pwm_pulse(void *data);
int oplus_display_panel_get_pwm_pulse(void *data);
ssize_t oplus_get_pwm_pulse_debug(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
ssize_t oplus_set_pwm_pulse_debug(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count);

#endif /* _OPLUS_DISPLAY_HIGH_FREQUENCY_PWM_H_ */
