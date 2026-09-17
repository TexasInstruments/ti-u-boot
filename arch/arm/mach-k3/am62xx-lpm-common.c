// SPDX-License-Identifier: GPL-2.0+
/*
 * am62xx common LPM functions
 *
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com/
 * Copyright (C) 2025 BayLibre, SAS
 */

#include <config.h>
#include <asm/arch/hardware.h>
#include <asm/io.h>
#include <dm/of_access.h>
#include <dm/ofnode.h>
#include <linux/soc/ti/ti_sci_protocol.h>
#include <vsprintf.h>
#include <wait_bit.h>

#include <asm/arch/am62xx-j722s-lpm-hardware.h>
#include "am62xx-lpm-common.h"
#include "common.h"

static int wkup_ctrl_remove_can_io_isolation(void)
{
	const void *wait_reg = (const void *)(WKUP_CTRL_MMR0_BASE +
					      WKUP_CTRL_MMR_CANUART_WAKE_STAT1);
	int ret;
	u32 reg = 0;

	/* Program magic word */
	reg = readl(WKUP_CTRL_MMR0_BASE + WKUP_CTRL_MMR_CANUART_WAKE_CTRL);
	reg |= WKUP_CTRL_MMR_CANUART_WAKE_CTRL_MW << WKUP_CTRL_MMR_CANUART_WAKE_CTRL_MW_SHIFT;
	writel(reg, WKUP_CTRL_MMR0_BASE + WKUP_CTRL_MMR_CANUART_WAKE_CTRL);

	/* Set enable bit. */
	reg |= WKUP_CTRL_MMR_CANUART_WAKE_CTRL_MW_LOAD_EN;
	writel(reg, WKUP_CTRL_MMR0_BASE + WKUP_CTRL_MMR_CANUART_WAKE_CTRL);

	/* Clear enable bit. */
	reg &= ~WKUP_CTRL_MMR_CANUART_WAKE_CTRL_MW_LOAD_EN;
	writel(reg, WKUP_CTRL_MMR0_BASE + WKUP_CTRL_MMR_CANUART_WAKE_CTRL);

	/* wait for CAN_ONLY_IO signal to be 0 */
	ret = wait_for_bit_32(wait_reg,
			      WKUP_CTRL_MMR_CANUART_WAKE_STAT1_CANUART_IO_MODE,
			      false,
			      CLKSTOP_TRANSITION_TIMEOUT_MS,
			      false);
	if (ret < 0)
		return ret;

	/* Reset magic word */
	writel(0, WKUP_CTRL_MMR0_BASE + WKUP_CTRL_MMR_CANUART_WAKE_CTRL);

	/* Remove WKUP IO isolation */
	reg = readl(WKUP_CTRL_MMR0_BASE + WKUP_CTRL_MMR_PMCTRL_IO_0);
	reg = reg & WKUP_CTRL_MMR_PMCTRL_IO_0_WRITE_MASK & ~WKUP_CTRL_MMR_PMCTRL_IO_0_GLOBAL_WUEN_0;
	writel(reg, WKUP_CTRL_MMR0_BASE + WKUP_CTRL_MMR_PMCTRL_IO_0);

	/* clear global IO isolation */
	reg = readl(WKUP_CTRL_MMR0_BASE + WKUP_CTRL_MMR_PMCTRL_IO_0);
	reg = reg & WKUP_CTRL_MMR_PMCTRL_IO_0_WRITE_MASK & ~WKUP_CTRL_MMR_PMCTRL_IO_0_IO_ISO_CTRL_0;
	writel(reg, WKUP_CTRL_MMR0_BASE + WKUP_CTRL_MMR_PMCTRL_IO_0);

	/* Release all IOs from deepsleep mode and clear IO daisy chain control */
	writel(0, WKUP_CTRL_MMR0_BASE + WKUP_CTRL_MMR_DEEPSLEEP_CTRL);
	writel(0, WKUP_CTRL_MMR0_BASE + WKUP_CTRL_MMR_PMCTRL_IO_GLB);

	return 0;
}

static bool wkup_ctrl_canuart_wakeup_active(void)
{
	return !!(readl(WKUP_CTRL_MMR0_BASE + WKUP_CTRL_MMR_CANUART_WAKE_STAT1) &
		WKUP_CTRL_MMR_CANUART_WAKE_STAT1_CANUART_IO_MODE);
}

static bool wkup_ctrl_canuart_magic_word_set(void)
{
	return readl(WKUP_CTRL_MMR0_BASE + WKUP_CTRL_MMR_CANUART_WAKE_OFF_MODE_STAT) ==
		WKUP_CTRL_MMR_CANUART_WAKE_OFF_MODE_STAT_MW;
}

bool wkup_ctrl_is_lpm_exit(void)
{
	return IS_ENABLED(CONFIG_K3_IODDR) &&
		wkup_ctrl_canuart_wakeup_active() &&
		wkup_ctrl_canuart_magic_word_set();
}

int __maybe_unused wkup_ctrl_remove_can_io_isolation_if_set(void)
{
	if (wkup_ctrl_canuart_wakeup_active() && !wkup_ctrl_canuart_magic_word_set())
		return wkup_ctrl_remove_can_io_isolation();
	return 0;
}

#if IS_ENABLED(CONFIG_K3_IODDR)
int wkup_r5f_am62_lpm_meta_data_addr(u64 *meta_data_addr)
{
	struct ofnode_phandle_args memregion_phandle;
	ofnode memregion;
	ofnode wkup_bus;
	int ret;

	wkup_bus = ofnode_path("/bus@f0000/bus@b00000");
	if (!ofnode_valid(wkup_bus)) {
		printf("Failed to find wkup bus\n");
		return -EINVAL;
	}

	memregion = ofnode_by_compatible(wkup_bus, "ti,am62-r5f");
	if (!ofnode_valid(memregion)) {
		printf("Failed to find r5f devicetree node ti,am62-r5f\n");
		return -EINVAL;
	}

	ret = ofnode_parse_phandle_with_args(memregion, "memory-region", NULL,
					     0, 1, &memregion_phandle);
	if (ret) {
		printf("Failed to parse phandle for second memory region\n");
		return ret;
	}

	ret = ofnode_read_u64_index(memregion_phandle.node, "reg", 0, meta_data_addr);
	if (ret) {
		printf("Failed to read memory region offset\n");
		return ret;
	}

	*meta_data_addr += K3_R5_MEMREGION_LPM_METADATA_OFFSET;

	return 0;
}

/*
 * Restore TIFS context for LPM resume.
 * @ctx_addr: DDR address pointing to the location containing the actual
 *            TIFS context address. TIFS will read from this DDR location
 *            to get the context address, so DDR must be accessible.
 */
static int lpm_restore_context(u64 ctx_addr)
{
	struct ti_sci_handle *ti_sci = get_ti_sci_handle();
	int ret;

	ret = ti_sci->ops.lpm_ops.min_context_restore(ti_sci, ctx_addr);
	if (ret)
		printf("Failed to restore context from DDR %d\n", ret);

	return ret;
}

struct lpm_meta_data {
	u64 dm_jump_address;
	u64 tifs_context_save_address;
	u64 reserved[30];
} __packed;

void __noreturn lpm_resume_from_ddr(u64 meta_data_addr)
{
	struct lpm_meta_data lpm_data;
	typedef void __noreturn (*image_entry_noargs_t)(void);
	image_entry_noargs_t image_entry;
	int ret;

	/*
	 * DDR not accessible yet - cannot dereference meta_data_addr.
	 * Pass DDR pointer to TIFS for context restore using pointer arithmetic.
	 */
	u64 tifs_ctx_addr_ptr = meta_data_addr +
				offsetof(struct lpm_meta_data, tifs_context_save_address);

	ret = lpm_restore_context(tifs_ctx_addr_ptr);
	if (ret)
		panic("Failed to restore context via DDR pointer 0x%x%08x\n",
		      (u32)(tifs_ctx_addr_ptr >> 32),
		      (u32)tifs_ctx_addr_ptr);

	/* Now it's safe to read from DDR after context restore */
	lpm_data = *(struct lpm_meta_data *)(uintptr_t)meta_data_addr;

	image_entry = (image_entry_noargs_t)(uintptr_t)lpm_data.dm_jump_address;
	printf("Resuming from DDR, jumping to stored DM loadaddr 0x%x%08x, TIFS context restored from 0x%x%08x\n",
	       (u32)(lpm_data.dm_jump_address >> 32),
	       (u32)lpm_data.dm_jump_address,
	       (u32)(lpm_data.tifs_context_save_address >> 32),
	       (u32)lpm_data.tifs_context_save_address);

	image_entry();
}
#else

int wkup_r5f_am62_lpm_meta_data_addr(u64 *meta_data_addr)
{
	return -EINVAL;
}

void __noreturn lpm_resume_from_ddr(u64 meta_data_addr)
{
	panic("No IO+DDR support");
}
#endif
