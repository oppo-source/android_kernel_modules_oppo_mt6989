#ifndef _OPLUS23661_AW37501_BIAS_H_
#define _OPLUS23661_AW37501_BIAS_H_

#define LCM_I2C_ID_NAME "I2C_LCD_BIAS_AW37501"

int lcm_i2c_write_bytes(u8 reg , u8 val);
int lcm_i2c_read_bytes(u8 reg , u8 *data);

#endif