#ifndef AC222_P_7_A0014_H
#define AC222_P_7_A0014_H

#define REGFLAG_CMD				0xFFFA
#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD

enum MODE_ID {
	FHD_SDC60 = 0,
	FHD_SDC90 = 1,
	FHD_SDC120 = 2,
};

struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[256];
};

struct LCM_setting_table init_setting_60Hz[] = {
    //VESAOSC=121.9MHz
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
    {REGFLAG_CMD, 2, {0x6F,0x31}},
    {REGFLAG_CMD, 3, {0xF8,0x01,0xB9}},
    //DSPOSC=138.6MHz
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
    {REGFLAG_CMD, 2, {0x6F,0x15}},
    {REGFLAG_CMD, 3, {0xF8,0x01,0x8C}},
    //-----------------------CMD1-------------------------
    {REGFLAG_CMD, 2, {0x5A,0x01}},
    //CORNER OFF
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x07}},
    {REGFLAG_CMD, 2, {0xC0,0x06}},
    //POWERseq
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
    {REGFLAG_CMD, 2, {0x6F,0x0A}},
    {REGFLAG_CMD, 6, {0xF6,0x70,0x70,0x70,0x70,0x70}},
    //TE_opt
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x81}},
    {REGFLAG_CMD, 2, {0x6F,0x0D}},
    {REGFLAG_CMD, 2, {0xFB,0x80}},
    //forEM_DT=0condition
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x01}},
    {REGFLAG_CMD, 2, {0xE5,0x00}},
    ////////////////////////////////////////DDICinternalsetting//////////////////////////////////////////
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x81}},
    {REGFLAG_CMD, 2, {0x6F,0x0C}},
    {REGFLAG_CMD, 2, {0xFD,0x08}},
    {REGFLAG_CMD, 2, {0x6F,0x02}},
    {REGFLAG_CMD, 2, {0xF9,0x84}},
    {REGFLAG_CMD, 2, {0x6F,0x10}},
    {REGFLAG_CMD, 2, {0xFB,0x40}},
    //tuning aftercode OTP
    {REGFLAG_CMD, 2, {0x6F,0x0B}},
    {REGFLAG_CMD, 2, {0xFD,0x00}},
    //LVDETSkipframeoff
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
    {REGFLAG_CMD, 2, {0x6F,0x48}},
    {REGFLAG_CMD, 2, {0xF2,0x00}},
    //ASRon
    {REGFLAG_CMD, 2, {0x6F,0x16}},
    {REGFLAG_CMD, 3, {0xF4,0x02,0x74}},
    //internal
    {REGFLAG_CMD, 2, {0x6F,0x42}},
    {REGFLAG_CMD, 2, {0xF4,0x00}},
    //internal
    {REGFLAG_CMD, 2, {0x6F,0x49}},
    {REGFLAG_CMD, 2, {0xF2,0x10}},
    //internal
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x81}},
    {REGFLAG_CMD, 2, {0x6F,0x05}},
    {REGFLAG_CMD, 3, {0xFE,0x3C,0x3C}},
    //internal
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x82}},
    {REGFLAG_CMD, 2, {0x6F,0x09}},
    {REGFLAG_CMD, 2, {0xF2,0xFF}},
    //internal
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
    {REGFLAG_CMD, 2, {0x6F,0x04}},
    {REGFLAG_CMD, 2, {0xF2,0x3C}},
    ////////////////////////////////////////DDICinternalsetting//////////////////////////////////////////
    //RM=1,DM=0
    {REGFLAG_CMD, 2, {0x17,0x30}},
    //60Hz
    {REGFLAG_CMD, 2, {0x2F,0x03}},
    //GIRONLIRONAPLAutoSwitchOFF
    {REGFLAG_CMD, 3, {0x5F,0x00,0x00}},
    //GIRONLIRONAPLAutoSwitchON
    //REGW0x5F,0x00,0x01
    //TEOn
    {REGFLAG_CMD, 2, {0x35,0x00}},
    //BCTRL=1,DD=0
    {REGFLAG_CMD, 2, {0x53,0x20}},
    {REGFLAG_CMD, 5, {0x2A,0x00,0x00,0x04,0xE7}},
    {REGFLAG_CMD, 5, {0x2B,0x00,0x00,0x0A,0xC7}},
    {REGFLAG_CMD, 3, {0x90,0x03,0x43}},
    //1256x2760_1.2_RGB_10Bit_3.75
    {REGFLAG_CMD, 19, {0x91,0xAB,0xA8,0x00,0x0C,0xC2,0x00,0x02,0x3B,0x01,0x35,0x00,0x08,0x08,0xBB,0x07,0x44,0x10,0xF0}},
    //GC=1
    {REGFLAG_CMD, 2, {0x26,0x00}},
    //******************************I2C***************************
    //PVEE SLECT TA
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x01}},
    {REGFLAG_CMD, 3, {0x81,0x01,0x19}},
    //----------------------- Page 0 ----------------------
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
    //I2C_ELVDD_ON_EN=0, swire_1stline_pos=1
    {REGFLAG_CMD, 2, {0x6F,0x03}},
    {REGFLAG_CMD, 2, {0xB5,0x65}},
    //PWRCOMSEL(AVDD/ELVDD/ELVSS), 0: Swire, 1: I2C
    {REGFLAG_CMD, 2, {0x6F,0x08}},
    {REGFLAG_CMD, 2, {0xD5,0x04}},
    {REGFLAG_CMD, 2, {0x6F,0x4D}},
    //ENABLE I2C_MANU_WRITE
    {REGFLAG_CMD, 2, {0xD5,0x01}},
    {REGFLAG_CMD, 2, {0x6F,0x19}},
    //DVDD=1.125 0x11
    {REGFLAG_CMD, 2, {0xD5,0x11}},
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x01}},
    //Swire_i2c_swen_apr=0x55 --> independent Swire/I2C
    {REGFLAG_CMD, 2, {0xCE,0x00}},
    //*************************I2C***********************************
    {REGFLAG_CMD, 1, {0x11}},
    {REGFLAG_DELAY, 120, {} },
    {REGFLAG_CMD, 1, {0x29}},
};

struct LCM_setting_table init_setting_90Hz[] = {
    //VESAOSC=121.9MHz
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
    {REGFLAG_CMD, 2, {0x6F,0x31}},
    {REGFLAG_CMD, 3, {0xF8,0x01,0xB9}},
    //DSPOSC=138.6MHz
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
    {REGFLAG_CMD, 2, {0x6F,0x15}},
    {REGFLAG_CMD, 3, {0xF8,0x01,0x8C}},
    //-----------------------CMD1-------------------------
    {REGFLAG_CMD, 2, {0x5A,0x01}},
    //CORNER OFF
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x07}},
    {REGFLAG_CMD, 2, {0xC0,0x06}},
    //POWERseq
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
    {REGFLAG_CMD, 2, {0x6F,0x0A}},
    {REGFLAG_CMD, 6, {0xF6,0x70,0x70,0x70,0x70,0x70}},
    //TE_opt
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x81}},
    {REGFLAG_CMD, 2, {0x6F,0x0D}},
    {REGFLAG_CMD, 2, {0xFB,0x80}},
    //forEM_DT=0condition
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x01}},
    {REGFLAG_CMD, 2, {0xE5,0x00}},
    ////////////////////////////////////////DDICinternalsetting//////////////////////////////////////////
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x81}},
    {REGFLAG_CMD, 2, {0x6F,0x0C}},
    {REGFLAG_CMD, 2, {0xFD,0x08}},
    {REGFLAG_CMD, 2, {0x6F,0x02}},
    {REGFLAG_CMD, 2, {0xF9,0x84}},
    {REGFLAG_CMD, 2, {0x6F,0x10}},
    {REGFLAG_CMD, 2, {0xFB,0x40}},
    //tuning aftercode OTP
    {REGFLAG_CMD, 2, {0x6F,0x0B}},
    {REGFLAG_CMD, 2, {0xFD,0x00}},
    //LVDETSkipframeoff
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
    {REGFLAG_CMD, 2, {0x6F,0x48}},
    {REGFLAG_CMD, 2, {0xF2,0x00}},
    //ASRon
    {REGFLAG_CMD, 2, {0x6F,0x16}},
    {REGFLAG_CMD, 3, {0xF4,0x02,0x74}},
    //internal
    {REGFLAG_CMD, 2, {0x6F,0x42}},
    {REGFLAG_CMD, 2, {0xF4,0x00}},
    //internal
    {REGFLAG_CMD, 2, {0x6F,0x49}},
    {REGFLAG_CMD, 2, {0xF2,0x10}},
    //internal
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x81}},
    {REGFLAG_CMD, 2, {0x6F,0x05}},
    {REGFLAG_CMD, 3, {0xFE,0x3C,0x3C}},
    //internal
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x82}},
    {REGFLAG_CMD, 2, {0x6F,0x09}},
    {REGFLAG_CMD, 2, {0xF2,0xFF}},
    //internal
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
    {REGFLAG_CMD, 2, {0x6F,0x04}},
    {REGFLAG_CMD, 2, {0xF2,0x3C}},
    ////////////////////////////////////////DDICinternalsetting//////////////////////////////////////////
    //RM=1,DM=0
    {REGFLAG_CMD, 2, {0x17,0x30}},
    //90Hz
    {REGFLAG_CMD, 2, {0x2F,0x02}},
    //GIRONLIRONAPLAutoSwitchOFF
    {REGFLAG_CMD, 3, {0x5F,0x00,0x00}},
    //GIRONLIRONAPLAutoSwitchON
    //REGW0x5F,0x00,0x01
    //TEOn
    {REGFLAG_CMD, 2, {0x35,0x00}},
    //BCTRL=1,DD=0
    {REGFLAG_CMD, 2, {0x53,0x20}},
    {REGFLAG_CMD, 5, {0x2A,0x00,0x00,0x04,0xE7}},
    {REGFLAG_CMD, 5, {0x2B,0x00,0x00,0x0A,0xC7}},
    {REGFLAG_CMD, 3, {0x90,0x03,0x43}},
    //1256x2760_1.2_RGB_10Bit_3.75
    {REGFLAG_CMD, 19, {0x91,0xAB,0xA8,0x00,0x0C,0xC2,0x00,0x02,0x3B,0x01,0x35,0x00,0x08,0x08,0xBB,0x07,0x44,0x10,0xF0}},
    //GC=1
    {REGFLAG_CMD, 2, {0x26,0x00}},
    //******************************I2C***************************
    //PVEE SLECT TA
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x01}},
    {REGFLAG_CMD, 3, {0x81,0x01,0x19}},
    //----------------------- Page 0 ----------------------
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
    //I2C_ELVDD_ON_EN=0, swire_1stline_pos=1
    {REGFLAG_CMD, 2, {0x6F,0x03}},
    {REGFLAG_CMD, 2, {0xB5,0x65}},
    //PWRCOMSEL(AVDD/ELVDD/ELVSS), 0: Swire, 1: I2C
    {REGFLAG_CMD, 2, {0x6F,0x08}},
    {REGFLAG_CMD, 2, {0xD5,0x04}},
    {REGFLAG_CMD, 2, {0x6F,0x4D}},
    //ENABLE I2C_MANU_WRITE
    {REGFLAG_CMD, 2, {0xD5,0x01}},
    {REGFLAG_CMD, 2, {0x6F,0x19}},
    //DVDD=1.125 0x11
    {REGFLAG_CMD, 2, {0xD5,0x11}},
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x01}},
    //Swire_i2c_swen_apr=0x55 --> independent Swire/I2C
    {REGFLAG_CMD, 2, {0xCE,0x00}},
    //*************************I2C***********************************
    {REGFLAG_CMD, 1, {0x11}},
    {REGFLAG_DELAY, 120, {} },
    {REGFLAG_CMD, 1, {0x29}},
};

struct LCM_setting_table init_setting_120Hz[] = {
    //VESAOSC=121.9MHz
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
    {REGFLAG_CMD, 2, {0x6F,0x31}},
    {REGFLAG_CMD, 3, {0xF8,0x01,0xB9}},
    //DSPOSC=138.6MHz
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
    {REGFLAG_CMD, 2, {0x6F,0x15}},
    {REGFLAG_CMD, 3, {0xF8,0x01,0x8C}},
    //-----------------------CMD1-------------------------
    {REGFLAG_CMD, 2, {0x5A,0x01}},
    //CORNER OFF
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x07}},
    {REGFLAG_CMD, 2, {0xC0,0x06}},
    //POWERseq
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
    {REGFLAG_CMD, 2, {0x6F,0x0A}},
    {REGFLAG_CMD, 6, {0xF6,0x70,0x70,0x70,0x70,0x70}},
    //TE_opt
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x81}},
    {REGFLAG_CMD, 2, {0x6F,0x0D}},
    {REGFLAG_CMD, 2, {0xFB,0x80}},
    //forEM_DT=0condition
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x01}},
    {REGFLAG_CMD, 2, {0xE5,0x00}},
    ////////////////////////////////////////DDICinternalsetting//////////////////////////////////////////
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x81}},
    {REGFLAG_CMD, 2, {0x6F,0x0C}},
    {REGFLAG_CMD, 2, {0xFD,0x08}},
    {REGFLAG_CMD, 2, {0x6F,0x02}},
    {REGFLAG_CMD, 2, {0xF9,0x84}},
    {REGFLAG_CMD, 2, {0x6F,0x10}},
    {REGFLAG_CMD, 2, {0xFB,0x40}},
    //tuning aftercode OTP
    {REGFLAG_CMD, 2, {0x6F,0x0B}},
    {REGFLAG_CMD, 2, {0xFD,0x00}},
    //LVDETSkipframeoff
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
    {REGFLAG_CMD, 2, {0x6F,0x48}},
    {REGFLAG_CMD, 2, {0xF2,0x00}},
    //ASRon
    {REGFLAG_CMD, 2, {0x6F,0x16}},
    {REGFLAG_CMD, 3, {0xF4,0x02,0x74}},
    //internal
    {REGFLAG_CMD, 2, {0x6F,0x42}},
    {REGFLAG_CMD, 2, {0xF4,0x00}},
    //internal
    {REGFLAG_CMD, 2, {0x6F,0x49}},
    {REGFLAG_CMD, 2, {0xF2,0x10}},
    //internal
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x81}},
    {REGFLAG_CMD, 2, {0x6F,0x05}},
    {REGFLAG_CMD, 3, {0xFE,0x3C,0x3C}},
    //internal
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x82}},
    {REGFLAG_CMD, 2, {0x6F,0x09}},
    {REGFLAG_CMD, 2, {0xF2,0xFF}},
    //internal
    {REGFLAG_CMD, 5, {0xFF,0xAA,0x55,0xA5,0x80}},
    {REGFLAG_CMD, 2, {0x6F,0x04}},
    {REGFLAG_CMD, 2, {0xF2,0x3C}},
    ////////////////////////////////////////DDICinternalsetting//////////////////////////////////////////
    //RM=1,DM=0
    {REGFLAG_CMD, 2, {0x17,0x30}},
    //120Hz
    {REGFLAG_CMD, 2, {0x2F,0x01}},
    //GIRONLIRONAPLAutoSwitchOFF
    {REGFLAG_CMD, 3, {0x5F,0x00,0x00}},
    //GIRONLIRONAPLAutoSwitchON
    //REGW0x5F,0x00,0x01
    //TEOn
    {REGFLAG_CMD, 2, {0x35,0x00}},
    //BCTRL=1,DD=0
    {REGFLAG_CMD, 2, {0x53,0x20}},
    {REGFLAG_CMD, 5, {0x2A,0x00,0x00,0x04,0xE7}},
    {REGFLAG_CMD, 5, {0x2B,0x00,0x00,0x0A,0xC7}},
    {REGFLAG_CMD, 3, {0x90,0x03,0x43}},
    //1256x2760_1.2_RGB_10Bit_3.75
    {REGFLAG_CMD, 19, {0x91,0xAB,0xA8,0x00,0x0C,0xC2,0x00,0x02,0x3B,0x01,0x35,0x00,0x08,0x08,0xBB,0x07,0x44,0x10,0xF0}},
    //GC=1
    {REGFLAG_CMD, 2, {0x26,0x00}},
    //******************************I2C***************************
    //PVEE SLECT TA
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x01}},
    {REGFLAG_CMD, 3, {0x81,0x01,0x19}},
    //----------------------- Page 0 ----------------------
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x00}},
    //I2C_ELVDD_ON_EN=0, swire_1stline_pos=1
    {REGFLAG_CMD, 2, {0x6F,0x03}},
    {REGFLAG_CMD, 2, {0xB5,0x65}},
    //PWRCOMSEL(AVDD/ELVDD/ELVSS), 0: Swire, 1: I2C
    {REGFLAG_CMD, 2, {0x6F,0x08}},
    {REGFLAG_CMD, 2, {0xD5,0x04}},
    {REGFLAG_CMD, 2, {0x6F,0x4D}},
    //ENABLE I2C_MANU_WRITE
    {REGFLAG_CMD, 2, {0xD5,0x01}},
    {REGFLAG_CMD, 2, {0x6F,0x19}},
    //DVDD=1.125 0x11
    {REGFLAG_CMD, 2, {0xD5,0x11}},
    {REGFLAG_CMD, 6, {0xF0,0x55,0xAA,0x52,0x08,0x01}},
    //Swire_i2c_swen_apr=0x55 --> independent Swire/I2C
    {REGFLAG_CMD, 2, {0xCE,0x00}},
    //*************************I2C***********************************
    {REGFLAG_CMD, 1, {0x11}},
    {REGFLAG_DELAY, 120, {} },
    {REGFLAG_CMD, 1, {0x29}},
};

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
/* aod/fod command */
struct LCM_setting_table aod_on_cmd[] = {
	/* AOD Mode ON */
	{REGFLAG_CMD, 1, {0x39}},
	{REGFLAG_CMD, 2, {0x6F,0x04}},
	{REGFLAG_CMD, 3, {0x51,0x03,0xFF}},
};

struct LCM_setting_table aod_off_cmd_60hz[] = {
	{REGFLAG_CMD, 1, {0x38}},
};

struct LCM_setting_table aod_off_cmd_120hz[] = {
	{REGFLAG_CMD, 1, {0x38}},
};

struct LCM_setting_table aod_off_cmd_90hz[] = {
	{REGFLAG_CMD, 1, {0x38}},
};

struct LCM_setting_table aod_high_mode[] = {
	{REGFLAG_CMD, 2, {0x6F,0x04}},
	{REGFLAG_CMD, 3, {0x51,0x03,0xFF}},
};

struct LCM_setting_table aod_low_mode[] = {
	{REGFLAG_CMD, 2, {0x6F,0x04}},
	{REGFLAG_CMD, 3, {0x51,0x01,0xFF}},
};

struct LCM_setting_table ultra_low_power_aod_on_cmd_60hz[] = {
};

struct LCM_setting_table ultra_low_power_aod_on_cmd_90hz[] = {
};

struct LCM_setting_table ultra_low_power_aod_on_cmd_120hz[] = {
};

struct LCM_setting_table ultra_low_power_aod_off_cmd_60hz[] = {
};

struct LCM_setting_table ultra_low_power_aod_off_cmd_90hz[] = {
};

struct LCM_setting_table ultra_low_power_aod_off_cmd_120hz[] = {
};

struct LCM_setting_table hbm_on_cmd_60hz[] = {
	{REGFLAG_CMD, 3, {0x51,0x0F,0xFE}},
};

struct LCM_setting_table hbm_on_cmd_90hz[] = {
	{REGFLAG_CMD, 3, {0x51,0x0F,0xFE}},
};

struct LCM_setting_table hbm_on_cmd_120hz[] = {
	{REGFLAG_CMD, 3, {0x51,0x0F,0xFE}},
};

struct LCM_setting_table hbm_off_cmd_60hz[] = {
	{REGFLAG_CMD, 3, {0x51,0x0D,0xBB}},
};

struct LCM_setting_table hbm_off_cmd_90hz[] = {
	{REGFLAG_CMD, 3, {0x51,0x0D,0xBB}},
};

struct LCM_setting_table hbm_off_cmd_120hz[] = {
	{REGFLAG_CMD, 3, {0x51,0x0D,0xBB}},
};

struct LCM_setting_table lhbm_pressed_icon_gamma_cmd[] = {
};

struct LCM_setting_table lhbm_pressed_icon_grayscale_cmd[] = {
};

struct LCM_setting_table lhbm_pressed_icon_on_cmd[] = {
};

struct LCM_setting_table lhbm_pressed_icon_off_cmd[] = {
};
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

struct LCM_setting_table mode_switch_to_60[] = {
    {REGFLAG_CMD, 2, {0x2F,0x03}},
};

struct LCM_setting_table mode_switch_to_90[] = {
    {REGFLAG_CMD, 2, {0x2F,0x02}},
};

struct LCM_setting_table mode_switch_to_120[] = {
    {REGFLAG_CMD, 2, {0x2F,0x01}},
};

struct LCM_setting_table dsi_set_backlight[] = {
	{REGFLAG_CMD, 4, {0xFF,0x5A,0xA5,0x00}},
	{REGFLAG_CMD, 3, {0x51,0x00,0x00}},
	/*long write EOT to the end*/
	{REGFLAG_CMD, 3,  {0x00,0x00,0x00}},
};

struct LCM_setting_table dsi_switch_hbm_apl_on[] = {

};

struct LCM_setting_table dsi_switch_hbm_apl_off[] = {

};
#endif
