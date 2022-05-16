// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Copyright (c) 2015-2019, The Linux Foundataion. All rights reserved.
 *  Copyright (c) 2017-2020, Pixelworks, Inc.
 *
 *  These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#ifndef _DSI_IRIS5_LUT_H_
#define _DSI_IRIS5_LUT_H_

int iris5_parse_lut_cmds(void);
int iris_lut_send(u8 lut_type, u8 lut_table_index, u32 lut_abtable_index);
void iris_ambient_lut_update(enum LUT_TYPE lutType, u32 lutPos);
void iris_maxcll_lut_update(enum LUT_TYPE lutType, u32 lutpos);
u8 iris_get_firmware_status(void);
void iris_set_firmware_status(u8 value);
#ifdef CONFIG_DEBUG_FS
int iris_fw_calibrate_status_debugfs_init(void);
#endif
#endif // _DSI_IRIS5_LUT_H_
