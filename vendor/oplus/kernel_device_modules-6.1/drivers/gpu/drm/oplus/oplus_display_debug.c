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
#include <linux/kobject.h>
#include <linux/signal.h>
#include <linux/sched/signal.h>


#include "oplus_display_debug.h"
#include <soc/oplus/system/oplus_project.h>

/* -------------------- macro ----------------------------------------------- */


/* -------------------- parameters ------------------------------------------ */


/* -------------------- extern ---------------------------------------------- */


/* -------------------- oplus  functions  ----------------------------------  */

void oplus_display_set_logger_en(bool en) {
	if (get_eng_version() == AGING) {
		if (en)
			logger_enable = 1;
		else
			logger_enable = 0;
	}
}

void oplus_display_set_mobile_log(bool en) {
	if ((get_eng_version() == AGING) || trig_db_enable)
		g_mobile_log = en;
}

void oplus_display_set_detail_log(bool en) {
	if ((get_eng_version() == AGING) || trig_db_enable)
		g_detail_log = en;
}

void oplus_display_kill_surfaceflinger(void) {
	 struct task_struct *p;
	if ((get_eng_version() == AGING) || trig_db_enable) {
		read_lock(&tasklist_lock);
		for_each_process(p) {
			get_task_struct(p);
			if (strcmp(p->comm, "surfaceflinger") == 0) {
				send_sig_info(SIGABRT, SEND_SIG_PRIV, p);
			}
			put_task_struct(p);
		}
		read_unlock(&tasklist_lock);
	}
}
