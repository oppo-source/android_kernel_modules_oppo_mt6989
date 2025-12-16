#ifndef __OPLUS25680_TD4376B_FHDP_HX_DSI_VDO__
#define __OPLUS25680_TD4376B_FHDP_HX_DSI_VDO__

#define REGFLAG_CMD				0xFFFA
#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD

#define BRIGHTNESS_MAX          3663

enum MODE_ID {
	FHD_SDC60 = 0,
	FHD_SDC90 = 1,
	FHD_SDC120 = 2,
};

struct ba {
	u32 brightness;
	u32 alpha;
};

struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[256];
};

/* -------------------------doze mode setting start------------------------- */
static struct LCM_setting_table AOD_off_setting[] = {
	{REGFLAG_CMD, 2, {0x53, 0x2c}},
	{REGFLAG_CMD, 2, {0x55, 0x00}},
	{REGFLAG_CMD, 3, {0x51, 0x00, 0x00}},
};


static struct LCM_setting_table lcm_setbrightness_normal[] = {
	{REGFLAG_CMD,3, {0x51, 0x00, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table AOD_on_setting[] = {
	{REGFLAG_CMD, 2, {0x53, 0x2c}},
	{REGFLAG_CMD, 2, {0x55, 0x00}},
	{REGFLAG_CMD, 3, {0x51, 0x01, 0x40}},
};

static struct LCM_setting_table aod_high_bl_level[] = {
	{REGFLAG_CMD, 2, {0x53, 0x2c}},
	{REGFLAG_CMD, 2, {0x55, 0x00}},
	{REGFLAG_CMD, 3, {0x51, 0x01, 0x40}},
};

static struct LCM_setting_table aod_low_bl_level[] = {
	{REGFLAG_CMD, 2, {0x53, 0x2c}},
	{REGFLAG_CMD, 2, {0x55, 0x00}},
	{REGFLAG_CMD, 3, {0x51, 0x00, 0x40}},
};
/* -------------------------doze mode setting end------------------------- */

#endif
