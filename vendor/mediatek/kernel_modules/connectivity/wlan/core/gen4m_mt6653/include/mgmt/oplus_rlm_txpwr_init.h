/* SPDX-License-Identifier: BSD-2-Clause */
  /*
   * Copyright (c) 2021 MediaTek Inc.
   */
  
  /*! \file   "oplus_rlm_txpwr_init.h"
   *    \brief
   */
  
  
  /*******************************************************************************
   *                         C O M P I L E R   F L A G S
   *******************************************************************************
   */
  
  /*******************************************************************************
   *                    E X T E R N A L   R E F E R E N C E S
   *******************************************************************************
   */
  
  /*******************************************************************************
   *                              C O N S T A N T S
   *******************************************************************************
   */
#if CFG_SUPPORT_PWR_LIMIT_COUNTRY
#if (CFG_SUPPORT_WIFI_6G == 1)
/*Set to MAX_TX_PWR = 63dBm if larger than it*/
/* For 802.11ax 6G Low Power Indoor mode setting*/
struct COUNTRY_POWER_LIMIT_TABLE_DEFAULT
	g_rRlmPowerLimitDefault_24749[] = {
	{	{'G', '0'}
		, {29, 28, 20, 20, 12,12, 63, 63, 63}
		, 0
	}
	,
	{	{'G', '1'}
		, {36, 28, 28, 28, 36,16, 7, 10, 16}
		, 0
	}
	,
	{	{'G', '2'}
		, {36, 34, 34, 34, 34,63, 63, 63, 63}
		, 0
	}
	,
	{	{'G', '3'}
		, {36, 20, 28, 28, 36,16, 63, 63, 63}
		, 0
	}
	,
	{	{'G', '4'}
		, {36, 29, 29, 29, 36,16, 63, 63, 63}
		, 0
	}
	,
	{	{'G', '9'}
		, {36, 28, 28, 28, 36,16, 63, 63, 63}
		, 0
	}
	,
	{	{'G', 'a'}
		, {36, 28, 28, 28, 36,12, 63, 63, 63}
		, 0
	}
	,
	/*Default*/
	{	{0, 0}
		, {63, 63, 63, 63, 63, 63, 63, 63, 63}
		, 0
	}
};
#if (CFG_SUPPORT_WIFI_6G_PWR_MODE == 1)
/* For 802.11ax 6G Very Low Power mode setting */
struct COUNTRY_POWER_LIMIT_TABLE_DEFAULT
	g_rRlmPowerLimitDefault_VLP_24749[] = {
	{	{'G', '0'}
		, {29, 28, 20, 20, 12,12, 63, 63, 63}
		, 0
	}
	,
	{	{'G', '1'}
		, {36, 28, 28, 28, 36,16, 7, 10, 16}
		, 0
	}
	,
	{	{'G', '2'}
		, {36, 34, 34, 34, 34,63, 63, 63, 63}
		, 0
	}
	,
	{	{'G', '3'}
		, {36, 20, 28, 28, 36,16, 63, 63, 63}
		, 0
	}
	,
	{	{'G', '4'}
		, {36, 29, 29, 29, 36,16, 63, 63, 63}
		, 0
	}
	,
	{	{'G', '9'}
		, {36, 28, 28, 28, 36,16, 63, 63, 63}
		, 0
	}
	,
	{	{'G', 'a'}
		, {36, 28, 28, 28, 36,12, 63, 63, 63}
		, 0
	}
	,
	/*Default*/
	{	{0, 0}
		, {63, 63, 63, 63, 63, 63, 63, 63, 63}
		, 0
	}
};
/* For 802.11ax 6G Standard Power mode setting */
struct COUNTRY_POWER_LIMIT_TABLE_DEFAULT
	g_rRlmPowerLimitDefault_SP_24749[] = {
	{	{'G', '0'}
		, {29, 28, 20, 20, 12,12, 63, 63, 63}
		, 0
	}
	,
	{	{'G', '1'}
		, {36, 28, 28, 28, 36,16, 7, 10, 16}
		, 0
	}
	,
	{	{'G', '2'}
		, {36, 34, 34, 34, 34,63, 63, 63, 63}
		, 0
	}
	,
	{	{'G', '3'}
		, {36, 20, 28, 28, 36,16, 63, 63, 63}
		, 0
	}
	,
	{	{'G', '4'}
		, {36, 29, 29, 29, 36,16, 63, 63, 63}
		, 0
	}
	,
	{	{'G', '9'}
		, {36, 28, 28, 28, 36,16, 63, 63, 63}
		, 0
	}
	,
	{	{'G', 'a'}
		, {36, 28, 28, 28, 36,12, 63, 63, 63}
		, 0
	}
	,
	/*Default*/
	{	{0, 0}
		, {63, 63, 63, 63, 63, 63, 63, 63, 63}
		, 0
	}
};
#endif /* CFG_SUPPORT_WIFI_6G_PWR_MODE */
#else /* CFG_SUPPORT_WIFI_6G */
struct COUNTRY_POWER_LIMIT_TABLE_DEFAULT
	g_rRlmPowerLimitDefault_24749[] = {
	{	{'G', '0'}
		, {29, 28, 20, 20, 12,12, 63, 63, 63}
		, 0
	}
	,
	{	{'G', '1'}
		, {36, 28, 28, 28, 36,16, 7, 10, 16}
		, 0
	}
	,
	{	{'G', '2'}
		, {36, 34, 34, 34, 34,63, 63, 63, 63}
		, 0
	}
	,
	{	{'G', '3'}
		, {36, 20, 28, 28, 36,16, 63, 63, 63}
		, 0
	}
	,
	{	{'G', '4'}
		, {36, 29, 29, 29, 36,16, 63, 63, 63}
		, 0
	}
	,
	{	{'G', '9'}
		, {36, 28, 28, 28, 36,16, 63, 63, 63}
		, 0
	}
	,
	{	{'G', 'a'}
		, {36, 28, 28, 28, 36,12, 63, 63, 63}
		, 0
	}
	,
	/*Default*/
	{	{0, 0}
		, {63, 63, 63, 63, 63}
		, 0
	}
};
#endif
#endif

#if CFG_SUPPORT_PWR_LIMIT_COUNTRY
#if (CFG_SUPPORT_WIFI_6G == 1)
/*Set to MAX_TX_PWR = 63dBm if larger than it*/
/* For 802.11ax 6G Low Power Indoor mode setting*/
struct COUNTRY_POWER_LIMIT_TABLE_DEFAULT
	g_rRlmPowerLimitDefault_24622[] = {
	{	{'G', '6'}
		, {63, 63, 63, 63, 63, 63, 63, 63, 63}
		, 0
	}
	,
	/*Default*/
	{	{0, 0}
		, {63, 63, 63, 63, 63, 63, 63, 63, 63}
		, 0
	}
	
};
#if (CFG_SUPPORT_WIFI_6G_PWR_MODE == 1)
/* For 802.11ax 6G Very Low Power mode setting */
struct COUNTRY_POWER_LIMIT_TABLE_DEFAULT
	g_rRlmPowerLimitDefault_VLP_24622[] = {
	{	{'G', '6'}
		, {63, 63, 63, 63, 63, 63, 63, 63, 63}
		, 0
	}
	,
	/*Default*/
	{	{0, 0}
		, {63, 63, 63, 63, 63, 63, 63, 63, 63}
		, 0
	}

};
/* For 802.11ax 6G Standard Power mode setting */
struct COUNTRY_POWER_LIMIT_TABLE_DEFAULT
	g_rRlmPowerLimitDefault_SP_24622[] = {
	{	{'G', '6'}
		, {63, 63, 63, 63, 63, 63, 63, 63, 63}
		, 0
	}
	,
	/*Default*/
	{	{0, 0}
		, {63, 63, 63, 63, 63, 63, 63, 63, 63}
		, 0
	}

};
#endif /* CFG_SUPPORT_WIFI_6G_PWR_MODE */
#else /* CFG_SUPPORT_WIFI_6G */
struct COUNTRY_POWER_LIMIT_TABLE_DEFAULT
	g_rRlmPowerLimitDefault_24622[] = {
	{	{'G', '6'}
		, {63, 63, 63, 63, 63, 63, 63, 63, 63}
		, 0
	}
	,
	/*Default*/
	{	{0, 0}
		, {63, 63, 63, 63, 63}
		, 0
	}
};
#endif
#endif

 #if CFG_SUPPORT_PWR_LIMIT_COUNTRY
#if (CFG_SUPPORT_WIFI_6G == 1)
/*Set to MAX_TX_PWR = 63dBm if larger than it*/
/* For 802.11ax 6G Low Power Indoor mode setting*/
struct COUNTRY_POWER_LIMIT_TABLE_DEFAULT
	g_rRlmPowerLimitDefault_24825[] = {
	{	{'G', '6'}
		, {63, 63, 34, 63, 63, 63, 63, 63, 63}
		, 0
	}
	,
	/*Default*/
	{	{0, 0}
		, {63, 63, 63, 63, 63, 63, 63, 63, 63}
		, 0
	}
};
#if (CFG_SUPPORT_WIFI_6G_PWR_MODE == 1)
/* For 802.11ax 6G Very Low Power mode setting */
struct COUNTRY_POWER_LIMIT_TABLE_DEFAULT
	g_rRlmPowerLimitDefault_VLP_24825[] = {
	{	{'G', '6'}
		, {63, 63, 34, 63, 63, 63, 63, 63, 63}
		, 0
	}
	,
	/*Default*/
	{	{0, 0}
		, {63, 63, 63, 63, 63, 63, 63, 63, 63}
		, 0
	}
};
/* For 802.11ax 6G Standard Power mode setting */
struct COUNTRY_POWER_LIMIT_TABLE_DEFAULT
	g_rRlmPowerLimitDefault_SP_24825[] = {
	{	{'G', '6'}
		, {63, 63, 34, 63, 63, 63, 63, 63, 63}
		, 0
	}
	,
	/*Default*/
	{	{0, 0}
		, {63, 63, 63, 63, 63, 63, 63, 63, 63}
		, 0
	}
};
#endif /* CFG_SUPPORT_WIFI_6G_PWR_MODE */
#else /* CFG_SUPPORT_WIFI_6G */
struct COUNTRY_POWER_LIMIT_TABLE_DEFAULT
	g_rRlmPowerLimitDefault_24825[] = {
	{	{'G', '6'}
		, {63, 63, 34, 63, 63}
		, 0
	}
	,
	/*Default*/
	{	{0, 0}
		, {63, 63, 63, 63, 63}
		, 0
	}
};
#endif
#endif
