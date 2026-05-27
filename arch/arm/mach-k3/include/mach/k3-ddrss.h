/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Texas Instruments' K3 DDRSS header
 *
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com/
 */

#ifndef _ASM_ARCH_K3_DDRSS_H_
#define _ASM_ARCH_K3_DDRSS_H_

#include <compiler.h>

struct k3_ddrss_regs {
	u32 ctl_141;
	u32 phy_1305;
	u32 ctl_88;
	u32 pi_134;
	u32 pi_7;
	u32 ctl_20;
	u32 wdqlvl_f1;
	u32 wdqlvl_f2;
};

/* LPDDR4 power management functions */
void k3_ddrss_lpddr4_exit_retention(struct udevice *dev, struct k3_ddrss_regs *regs);
void k3_ddrss_lpddr4_change_freq(struct udevice *dev);
void k3_ddrss_lpddr4_exit_low_power(struct udevice *dev, struct k3_ddrss_regs *regs);
#endif
