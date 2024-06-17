// Copyright (c) 2011-2023 Columbia University, System Level Design Group
// SPDX-License-Identifier: Apache-2.0
#ifndef __ESP_CFG_000_H__
#define __ESP_CFG_000_H__

#include "libesp.h"
#include "espacc_rtl.h"

typedef int32_t token_t;

/* <<--params-def-->> */
#define REG0 8
#define REG1 8

/* <<--params-->> */
const int32_t reg0 = REG0;
const int32_t reg1 = REG1;

#define NACC 1

struct espacc_rtl_access espacc_cfg_000[] = {
	{
		/* <<--descriptor-->> */
		.reg0 = REG0,
                .reg1 = REG1,
		.src_offset = 0,
		.dst_offset = 0,
		.esp.coherence = ACC_COH_NONE,
		.esp.p2p_store = 0,
		.esp.p2p_nsrcs = 0,
		.esp.p2p_srcs = {"", "", "", ""},
	}
};

esp_thread_info_t cfg_000[] = {
	{
		.run = true,
		.devname = "espacc_rtl.0",
		.ioctl_req = ESPACC_RTL_IOC_ACCESS,
		.esp_desc = &(espacc_cfg_000[0].esp),
	}
};

#endif /* __ESP_CFG_000_H__ */
