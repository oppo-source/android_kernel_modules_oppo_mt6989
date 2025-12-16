/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IDLE_BOOST_H_
#define _DSI_IDLE_BOOST_H_
#include <linux/types.h>

int iris_boost_init(void);
int iris_boost_deinit(void);
int iris_boost_enable(void);
int iris_boost_disable(void);

#endif
