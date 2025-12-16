/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef __LEDS_AW21024_H__
#define __LEDS_AW21024_H__
#include "aw_lamp_interface.h"
#include "leds_aw21024_reg.h"
/******************************************************
 *
 * Register List
 *
 *****************************************************/
#define AW21024_REG_GCR               0x00
#define AW21024_REG_BR0               0x01
#define AW21024_REG_BR1               0x02
#define AW21024_REG_BR2               0x03
#define AW21024_REG_BR3               0x04
#define AW21024_REG_BR4               0x05
#define AW21024_REG_BR5               0x06
#define AW21024_REG_BR6               0x07
#define AW21024_REG_BR7               0x08
#define AW21024_REG_BR8               0x09
#define AW21024_REG_BR9               0x0A
#define AW21024_REG_BR10              0x0B
#define AW21024_REG_BR11              0x0C
#define AW21024_REG_BR12              0x0D
#define AW21024_REG_BR13              0x0E
#define AW21024_REG_BR14              0x0F
#define AW21024_REG_BR15              0x10
#define AW21024_REG_BR16              0x11
#define AW21024_REG_BR17              0x12
#define AW21024_REG_BR18              0x13
#define AW21024_REG_BR19              0x14
#define AW21024_REG_BR20              0x15
#define AW21024_REG_BR21              0x16
#define AW21024_REG_BR22              0x17
#define AW21024_REG_BR23              0x18
#define AW21024_REG_UPDATE            0x49
#define AW21024_REG_COL0              0x4A
#define AW21024_REG_COL1              0x4B
#define AW21024_REG_COL2              0x4C
#define AW21024_REG_COL3              0x4D
#define AW21024_REG_COL4              0x4E
#define AW21024_REG_COL5              0x4F
#define AW21024_REG_COL6              0x50
#define AW21024_REG_COL7              0x51
#define AW21024_REG_COL8              0x52
#define AW21024_REG_COL9              0x53
#define AW21024_REG_COL10             0x54
#define AW21024_REG_COL11             0x55
#define AW21024_REG_COL12             0x56
#define AW21024_REG_COL13             0x57
#define AW21024_REG_COL14             0x58
#define AW21024_REG_COL15             0x59
#define AW21024_REG_COL16             0x5A
#define AW21024_REG_COL17             0x5B
#define AW21024_REG_COL18             0x5C
#define AW21024_REG_COL19             0x5D
#define AW21024_REG_COL20             0x5E
#define AW21024_REG_COL21             0x5F
#define AW21024_REG_COL22             0x60
#define AW21024_REG_COL23             0x61
#define AW21024_REG_GCCR              0x6E
#define AW21024_REG_PHCR              0x70
#define AW21024_REG_OSDCR             0x71
#define AW21024_REG_OSST0             0x72
#define AW21024_REG_OSST1             0x73
#define AW21024_REG_OSST2             0x74
#define AW21024_REG_OTCR              0x77
#define AW21024_REG_SSCR              0x78
#define AW21024_REG_UVCR              0x79
#define AW21024_REG_GCR2              0x7A
#define AW21024_REG_GCR4              0x7C
#define AW21024_REG_VER               0x7E
#define AW21024_REG_RESET             0x7F
#define AW21024_REG_WBR               0x90
#define AW21024_REG_WBG               0x91
#define AW21024_REG_WBB               0x92
#define AW21024_REG_PATCFG            0xA0
#define AW21024_REG_PATGO             0xA1
#define AW21024_REG_PATT0             0xA2
#define AW21024_REG_PATT1             0xA3
#define AW21024_REG_PATT2             0xA4
#define AW21024_REG_PATT3             0xA5
#define AW21024_REG_FADEH             0xA6
#define AW21024_REG_FADEL             0xA7
#define AW21024_REG_GCOLR             0xA8
#define AW21024_REG_GCOLG             0xA9
#define AW21024_REG_GCOLB             0xAA
#define AW21024_REG_GCFG0             0xAB
#define AW21024_REG_GCFG1             0xAC

#define AW21024_UPDATE_BR_SL			0x00

/******************************************************
 *
 * Register Write/Read Access
 *
 *****************************************************/
#define AW21024_I2C_NAME "aw21024_led"
#define REG_NONE_ACCESS            0
#define REG_RD_ACCESS              (1<<0)
#define REG_WR_ACCESS              (1<<1)
#define AW21024_REG_MAX            0xFF
#define LED_MAX_NUM							(8)
#define AW_DEBUG 							(1)
#define LEDMODE_MAX_NUM						(11)


#if AW_DEBUG
#define AW_LOG(fmt, args...)	pr_info("[%s] %s %d: " fmt, AW21024_I2C_NAME, \
  		__func__, __LINE__, ##args)
#else
#define AW_LOG(fmt, args...)
#endif
#define AW_ERR(fmt, args...)	pr_err("[%s] %s %d: " fmt, AW21024_I2C_NAME, \
  		__func__, __LINE__, ##args)
  
#define	AWLOGD(format, arg...)	printf(format, ##arg)

const unsigned char aw21024_reg_access[AW21024_REG_MAX] = {
	[AW21024_REG_GCR] = REG_RD_ACCESS,
	[AW21024_REG_BR0] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR4] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR5] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR6] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR7] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR8] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR9] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR10] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR11] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR12] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR13] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR14] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR15] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR16] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR17] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR18] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR19] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR20] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR21] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR22] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_BR23] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_UPDATE] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL0] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL4] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL5] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL6] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL7] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL8] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL9] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL10] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL11] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL12] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL13] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL14] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL15] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL16] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL17] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL18] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL19] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL20] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL21] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL22] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_COL23] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_GCCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_PHCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_OSDCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_OSST0] = REG_RD_ACCESS,
	[AW21024_REG_OSST1] = REG_RD_ACCESS,
	[AW21024_REG_OSST2] = REG_RD_ACCESS,
	[AW21024_REG_OTCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_SSCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_UVCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_GCR2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_GCR4] = REG_NONE_ACCESS,
	[AW21024_REG_VER] = REG_RD_ACCESS,
	[AW21024_REG_RESET] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_WBR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_WBG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_WBB] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_PATCFG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_PATGO] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_PATT0] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_PATT1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_PATT2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_PATT3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_FADEH] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_FADEL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_GCOLR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_GCOLG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_GCOLB] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_GCFG0] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21024_REG_GCFG1] = REG_RD_ACCESS | REG_WR_ACCESS,
};

/*********************************************************
 *
 * 0-7bite set
 *
 ********************************************************/

/* GCR  0x00 */
#define AW21024_BIT_CHIP_EN_CLOSE_MASK     (~(1<<0))
#define AW21024_BIT_CHIP_EN                (1<<0)
#define AW21024_BIT_CHIP_CLOSE             (0<<0)
#define AW21024_BIT_CLKPRQ_MASK            (~(7<<4))
#define AW21024_BIT_CLKPRQ_16MH            (0<<4)
#define AW21024_BIT_CLKPRQ_8MH             (1<<4)
#define AW21024_BIT_CLKPRQ_1MH             (2<<4)
#define AW21024_BIT_CLKPRQ_512KH           (3<<4)
#define AW21024_BIT_CLKPRQ_256KH           (4<<4)
#define AW21024_BIT_CLKPRQ_125KH           (5<<4)
#define AW21024_BIT_CLKPRQ_62KH            (6<<4)
#define AW21024_BIT_CLKPRQ_31KH            (7<<4)
#define AW21024_BIT_APSE_MASK              (~(1<<7))
#define AW21024_BIT_APSE_ENABLE            (1<<7)
#define AW21024_BIT_APSE_DISABLE           (0<<7)

/*********************************************************
 *
 * chip info
 *
 ********************************************************/

#define AW21024_CHIP_ID           0x18

/*********************************************************
 *
 * struct
 *
 ********************************************************/
enum aw21024_clk_pwm_mode {
	AW21024_CLKPRQ_16MH = 0,
	AW21024_CLKPRQ_8MH = 1,
	AW21024_CLKPRQ_1MH = 2,
	AW21024_CLKPRQ_512KH = 3,
	AW21024_CLKPRQ_256KH = 4,
	AW21024_CLKPRQ_125KH = 5,
	AW21024_CLKPRQ_62KH = 6,
	AW21024_CLKPRQ_31KH = 7,
};

enum AW2023_LED_MODE{
	AW21024_LED_CCMODE = 0,
	AW21024_LED_POWERON_MODE,
	AW21024_LED_NEW_ALWAYSON, /* used for preview_color */
	AW21024_LED_INCALL_MODE,
	AW21024_LED_MUSICMODE,
	AW21024_LED_GAMEMODE,
	AW21024_LED_NOTIFY_MODE,
	AW21024_LED_CHARGE_MODE,
	AW21024_LED_BLINKMODE,
	AW21024_LED_BREATHMODE,
	AW21024_LED_INDIVIDUAL_CTL_BREATH,
	AW21024_LED_NONE,
	AW21024_LED_MAXMODE,
};
	
enum AW21024_LED_TYPE {
	AW21024_LED_RED = 0,
	AW21024_LED_GREEN,
	AW21024_LED_BLUE,
	AW21024_LED_ON,
	AW21024_LED_OFF,
};
	
struct aw21024_platform_data {
	int imax;
	const char *led_default_trigger;
	int led_mode;
	int rise_time_ms;
	int hold_time_ms;
	int fall_time_ms;
	int off_time_ms;
	struct aw21024 *led;
};
struct aw21024 {
	struct i2c_client *i2c;
	struct device *dev;
	struct led_classdev cdev;
	struct aw21024_platform_data *pdata;
	struct work_struct brightness_work;
	struct mutex lock;
	int num_leds;
	int led_groups_num;
	int led_allocation_order[8];
	int id;
	u32 light_sensor_state;
	uint32_t glo_current_max;
	uint32_t glo_current_min;
	unsigned int clk_pwm;
	bool apse_mode;
	unsigned int dts_led_current;
	unsigned int conversion_led_current;
	unsigned int effect;
	unsigned int rgbcolor;
	int reset_gpio;
	int vbled_enable_gpio;
	struct regulator *vbled;
	uint32_t vbled_volt;
	int rgb_isnk_on;
	bool led_enable;
	struct delayed_work   breath_work;
};

typedef struct aw21024_cfg {
	AW_MULTI_BREATH_DATA_STRUCT *p;
	unsigned int count;
} AW21024_CFG;

/*********************************************************
 *
 * algorithm variable
 *
 ********************************************************/
typedef enum {
	INCALL_EFFECT,
	POWERON_EFFECT,
	CHARGE_EFFECT,
	GAME_ENTER_EFFECT,
	NOTIFY_EFFECT,
	CHARGE_STAGE2_EFFECT,
	NEWALWAYSON_EFFECT,
	MUSIC_EFFECT,
	MAX_EFFECT,
} effect_select_t;

static int blevel = -1;
static int last_run_effect = 0;
static unsigned char aw21024_br_data[MAX_EFFECT][LED_NUM] = {0};
static unsigned char aw21024_col_data[MAX_EFFECT][LED_NUM] = {0};
static unsigned char dim_data[MAX_EFFECT][LED_NUM];
static unsigned char fade_data[MAX_EFFECT][LED_NUM];
static unsigned char effect_stop_val[MAX_EFFECT];
static AW_COLORFUL_INTERFACE_STRUCT aw21024_interface[MAX_EFFECT];//defined in aw_lamp_interface.h

#define RGB_NUM		(LED_NUM/3)
static unsigned char loop_end[MAX_EFFECT][RGB_NUM] = {0};
static AW_MULTI_BREATH_DATA_STRUCT *effect_data[MAX_EFFECT] = {NULL};
static unsigned char num[MAX_EFFECT] = {0};
static AW_COLOR_STRUCT source_color[MAX_EFFECT][RGB_NUM];//defined in leds_aw21024_reg.h
static AW_COLOR_STRUCT destination_color[MAX_EFFECT][RGB_NUM];
static ALGO_DATA_STRUCT algo_data[MAX_EFFECT];//defined in aw_breath_algorithm.h
static ALGO_DATA_STRUCT aw21024_algo_data[MAX_EFFECT][RGB_NUM];
static ALGO_DATA_STRUCT aw21024_algo_color[MAX_EFFECT][RGB_NUM];
static unsigned char colorful_phase_nums[MAX_EFFECT][RGB_NUM] = {0};
static unsigned char colorful_cur_phase[MAX_EFFECT][RGB_NUM] = {0};
static unsigned char breath_cur_phase[MAX_EFFECT][RGB_NUM] = {0};
static unsigned char breath_cur_loop[MAX_EFFECT][RGB_NUM] = {0};
static unsigned char breath_phase_nums[MAX_EFFECT][RGB_NUM] = {0};
static unsigned char colorful_total_frames[MAX_EFFECT][RGB_NUM] = {0};
static unsigned char colorful_cur_frame[MAX_EFFECT][RGB_NUM] = {0};
static unsigned char colorful_cur_color_index[MAX_EFFECT][RGB_NUM] = {0};
static AW21024_CFG aw21024_cfg_array[] = {
    {incall_effect_data1,sizeof(incall_effect_data1)/sizeof(AW_MULTI_BREATH_DATA_STRUCT)},
	{poweron_effect_data1, sizeof(poweron_effect_data1)/sizeof(AW_MULTI_BREATH_DATA_STRUCT)},
	{charge_stage1_effect_data1, sizeof(charge_stage1_effect_data1)/sizeof(AW_MULTI_BREATH_DATA_STRUCT)},
	{gameenter_effect_data1, sizeof(gameenter_effect_data1)/sizeof(AW_MULTI_BREATH_DATA_STRUCT)},
	{notify_effect_data1, sizeof(notify_effect_data1)/sizeof(AW_MULTI_BREATH_DATA_STRUCT)},
	{charger_stage2_effect_data, sizeof(charger_stage2_effect_data)/sizeof(AW_MULTI_BREATH_DATA_STRUCT)},
};

typedef struct effect_control {
	int cur_effect;
	int prev_effect;
	int state[AW21024_LED_MAXMODE]; // 0 disable, 1 init, 2 running, 3 pause, 4 second stage running (for charger),5 second stage pause, 6 100% recover  stage (for charger)
	int data[AW21024_LED_MAXMODE];  // currently used for charge level
} effect_control_t;

static effect_control_t effect_state;
enum lights_types {
	L_BATTERY = 0,
	L_NOTIFY,
	L_KEYBOARD,
	L_GAMES,
	L_INCALL,
	L_WIFI,
	L_MAX,
};

/*
:: Avatar MG sequence
::     L0(1)   L7(4)
::
:: L1(5)           L6(0) 
:: 
:: L2(2)           L5(7)
:: 
::    L3(6)    L4(3)

::0 2 4 6 -- ic 0
::1 3 5 7 -- ic 1 

:: L0 IC 0 led_index= 1 color index = 1  effect_index = 2
:: L1 IC 1 led_index= 1 color index = 5  effect_index = 3
:: L2 IC 0 led_index= 2 color index = 2  effect_index = 4
:: L3 IC 1 led_index= 2 color index = 6  effect_index = 5
:: L4 IC 0 led_index= 3 color index = 3  effect_index = 6
:: L5 IC 1 led_index= 3 color index = 7  effect_index = 7
:: L6 IC 0 led_index= 0 color index = 0  effect_index = 0
:: L7 IC 1 led_index= 0 color index = 4  effect_index = 1
*/ 

#endif
