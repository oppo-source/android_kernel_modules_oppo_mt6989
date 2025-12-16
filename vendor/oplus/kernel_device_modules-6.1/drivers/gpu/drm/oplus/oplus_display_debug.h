/***************************************************************
** Copyright (C),  2023,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_debug.c
** Description : oplus display debug
** Version : 1.0
** Date : 2023/07/19
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Ping      2023/07/19        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_DISPLAY_DEBUG_H_
#define _OPLUS_DISPLAY_DEBUG_H_

/* -------------------- please just only include linux common head file ---------------- */



/* -------------------- oplus initialize params ---------------------------------------- */


/* -------------------- extern params  ------------------------------------------------- */
extern bool g_mobile_log;
extern bool g_detail_log;
extern int trig_db_enable;
extern bool logger_enable;



/* -------------------- oplus debug log     -------------------------------------------  */


/* -------------------- function implementation ---------------------------------------- */
void oplus_display_set_logger_en(bool en);
void oplus_display_set_mobile_log(bool en);
void oplus_display_set_detail_log(bool en);
void oplus_display_kill_surfaceflinger(void);


/* -------------------- oplus api nodes ------------------------------------------------ */


/*  -------------------- oplus config   ------------------------------------------------ */



#endif /* _OPLUS_DISPLAY_DEBUG_H_ */
