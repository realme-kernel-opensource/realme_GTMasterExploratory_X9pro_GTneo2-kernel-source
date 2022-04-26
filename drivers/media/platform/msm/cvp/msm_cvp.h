/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 */

#ifndef _MSM_CVP_H_
#define _MSM_CVP_H_

#include "msm_cvp_internal.h"
#include "msm_cvp_common.h"
#include "msm_cvp_clocks.h"
#include "msm_cvp_debug.h"
#include "msm_cvp_dsp.h"
int msm_cvp_handle_syscall(struct msm_cvp_inst *inst, struct cvp_kmd_arg *arg);
int msm_cvp_session_init(struct msm_cvp_inst *inst);
int msm_cvp_session_deinit(struct msm_cvp_inst *inst);
#ifdef OPLUS_FEATURE_CAMERA_COMMON
/*LiuBo@Camera 20201029, apply qcom patch for case: 04896694, cvpDme_Async cost many times*/
int msm_cvp_session_queue_stop(struct msm_cvp_inst *inst);
#endif
#endif
