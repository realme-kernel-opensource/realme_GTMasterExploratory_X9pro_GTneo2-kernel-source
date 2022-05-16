// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Copyright (c) 2015-2019, The Linux Foundataion. All rights reserved.
 *  Copyright (c) 2017-2020, Pixelworks, Inc.
 *
 *  These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#ifndef _DSI_IRIS5_BACK_H_
#define _DSI_IRIS5_BACK_H_

u32 iris5_loop_back_verify(void);

/* API in kernel for recovery mode */
int iris_loop_back_validate(void);

#endif // _DSI_IRIS5_BACK_H_
