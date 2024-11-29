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
#ifndef _OPLUS_DISPLAY_APOLLO_BRIGHTNESS_H_
#define _OPLUS_DISPLAY_APOLLO_BRIGHTNESS_H_


/* -------------------- extern        -------------------------------------------------- */
extern int mutex_sof_ns;


/* -------------------- debug log     -------------------------------------------------- */


/*  --------------------  config    ---------------------------------------------------- */
struct oplus_apollo_brightness {
	bool oplus_backlight_updated;
	int oplus_pending_backlight;
	bool oplus_backlight_need_sync;
	bool oplus_power_on;
	bool oplus_refresh_rate_switching;
	int oplus_te_tag_ns;
	int oplus_te_diff_ns;
	int cur_vsync;
	int limit_superior_ns;
	int limit_inferior_ns;
	int transfer_time_us;
	int pending_vsync;
	int pending_limit_superior_ns;
	int pending_limit_inferior_ns;
	int pending_transfer_time_us;
};
#define BACKLIGHT_CACHE_MAX 50
#define APOLLO_CUR_VSYNC_60 16666666
enum oplus_display_id {
	DISPLAY_PRIMARY = 0,
	DISPLAY_SECONDARY = 1,
	DISPLAY_MAX,
};

static struct backlight_log {
	u32 bl_count;
	unsigned int backlight[BACKLIGHT_CACHE_MAX];
	struct timespec64 past_times[BACKLIGHT_CACHE_MAX];
}oplus_bl_log[DISPLAY_MAX];


/* -------------------- function implementation ---------------------------------------- */



/* -------------------- oplus api nodes ------------------------------------------------ */


#endif /* _OPLUS_DISPLAY_APOLLO_BRIGHTNESS_H_ */
