// SPDX-License-Identifier: GPL-2.0+
/*
 * K3: R5 Common LPM Architecture initialization
 *
 * Copyright (C) 2023-2026 Texas Instruments Incorporated - https://www.ti.com/
 * Copyright (C) 2026 Bootlin
 */

#include <clk.h>
#include <dm/read.h>
#include <elf.h>
#include <linux/printk.h>
#include <linux/soc/ti/ti_sci_protocol.h>
#include <power-domain.h>
#include <remoteproc.h>
#include <mach/security.h>

#include "../common.h"
#include "../lpm-common.h"

#define FW_IMAGE_SIZE	0x80000
#define GTC_CNTCR_REG	0x0
#define GTC_CNTFID0_REG	0x20
#define GTC_CNTR_EN	0x3

struct lpm_addr_info {
	u32 *context_save_addr;
	u32 *atf_cert_addr;
	u32 *optee_cert_addr;
	u32 *dm_save_addr;
	u32 size;
};

struct lpm_addr_info mem_addr_lpm;

static int extract_lpm_region(void)
{
	ofnode node;
	fdt_addr_t lpm_reg_addr;
	fdt_size_t lpm_reg_size;

	node = ofnode_path("/reserved-memory/lpm-memory");
	if (!ofnode_valid(node)) {
		printf("lpm will not be functional\n");
		return -ENODEV;
	}

	lpm_reg_addr = ofnode_get_addr(node);
	if (lpm_reg_addr == FDT_ADDR_T_NONE) {
		printf("Can't find a valid reserved node!\n");
		return -ENODEV;
	}

	lpm_reg_size = ofnode_get_size(node);
	if (lpm_reg_size == FDT_ADDR_T_NONE) {
		printf("Can't find a valid reserved node!\n");
		return -ENODEV;
	}

	mem_addr_lpm.context_save_addr = (u32 *)lpm_reg_addr;
	mem_addr_lpm.atf_cert_addr =
		(u32 *)((u8 *)mem_addr_lpm.context_save_addr + FW_IMAGE_SIZE);
	mem_addr_lpm.optee_cert_addr =
		(u32 *)((u8 *)mem_addr_lpm.atf_cert_addr + FW_IMAGE_SIZE);
	mem_addr_lpm.dm_save_addr =
		(u32 *)((u8 *)mem_addr_lpm.optee_cert_addr + (2 * FW_IMAGE_SIZE));
	mem_addr_lpm.size = lpm_reg_size;

	return 0;
}

static int save_certificate(void)
{
	int ret;

	if (!fit_image_info[IMAGE_ID_ATF].image_start ||
	    !fit_image_info[IMAGE_ID_OPTEE].image_start ||
	    !fit_image_info[IMAGE_ID_DM_FW].image_start) {
		pr_err("Invalid images to save\n");
		return -EINVAL;
	}

	ret = extract_lpm_region();
	if (ret) {
		pr_err("Cannot find valid LPM address range..\n");
		return -ENOMEM;
	}

	memcpy(mem_addr_lpm.atf_cert_addr,
	       (void *)fit_image_info[IMAGE_ID_ATF].image_start,
	       fit_image_info[IMAGE_ID_ATF].image_len);

	memcpy(mem_addr_lpm.optee_cert_addr,
	       (void *)fit_image_info[IMAGE_ID_OPTEE].image_start,
	       fit_image_info[IMAGE_ID_OPTEE].image_len);

	memcpy(mem_addr_lpm.dm_save_addr,
	       (void *)fit_image_info[IMAGE_ID_DM_FW].image_start,
	       fit_image_info[IMAGE_ID_DM_FW].image_len);

	return 0;
}

__weak void k3_deassert_ddr_ret(void) {}

void k3_lpm_process(void)
{
	int ret = 0;
	unsigned long save_addr;
	struct ti_sci_handle *ti_sci = get_ti_sci_handle();

	ret = save_certificate();
	if (ret)
		return;
	save_addr = (unsigned long)mem_addr_lpm.context_save_addr;
	ret = ti_sci->ops.lpm_ops.lpm_save_addr(ti_sci, save_addr,
						mem_addr_lpm.size);
	if (ret)
		pr_err("TIFS lpm save addr fail\n");
}

static u32 resume_to_dm_f(void)
{
	struct ti_sci_handle *ti_sci = get_ti_sci_handle();
	u32 loadaddr = 0, save_addr = 0;
	int ret = 0;

	loadaddr = (u32)mem_addr_lpm.dm_save_addr;
	if (!valid_elf_image(loadaddr))
		panic("%s: DM-Firmware image is not valid, it cannot be loaded\n",
		      __func__);

	loadaddr = load_elf_image_phdr(loadaddr);
	save_addr = (uintptr_t)mem_addr_lpm.context_save_addr;
	ret = ti_sci->ops.lpm_ops.lpm_save_addr(ti_sci, save_addr, mem_addr_lpm.size);
	if (ret)
		panic("TIFS lpm save addr fail : %x\n", ret);

	/*
	 * TIFS minimal context restore
	 * This restores also the firewall
	 */
	ret = ti_sci->ops.lpm_ops.min_context_restore(ti_sci, 0);
	if (ret)
		panic("TIFS min_context_restore failed (%d)\n", ret);

	/*
	 * Restore TFA in msmc memory
	 */
	ret = ti_sci->ops.lpm_ops.decrypt_tfa(ti_sci,
					      CONFIG_K3_ATF_LOAD_ADDR);
	if (ret)
		panic("%s: TIFS failed to decrytp TFA : %x\n", __func__, ret);

	/* restore TFA resume vectore address in main core */
	ret = ti_sci->ops.lpm_ops.core_resume(ti_sci);
	if (ret)
		panic("ATF failed to resume (%d)\n", ret);

	return loadaddr;
}

static void resume_rproc_f(void)
{
	struct power_domain rproc_pwrdmn;
	struct udevice *dev;
	void *gtc_base;
	int ret;

	ret = uclass_get_device_by_seq(UCLASS_REMOTEPROC, 1, &dev);
	if (ret)
		panic("Unknown remote processor 1 (%d)\n", ret);

	ret = power_domain_get_by_index(dev, &rproc_pwrdmn, 1);
	if (ret)
		panic("power_domain_get_rproc() failed: %d\n", ret);

	gtc_base = dev_read_addr_ptr(dev);
	if (!gtc_base)
		panic("Get GTC address failed\n");

	/* GTC counter values have been restored by TIFS, so enable the counter */
	writel(GTC_CNTR_EN, gtc_base + GTC_CNTCR_REG);

	ret = power_domain_on(&rproc_pwrdmn);
	if (ret)
		panic("power_domain_on failed: %d\n", ret);
}

typedef void __noreturn (*image_entry_noargs_t)(void);

void __noreturn k3_do_resume(void)
{
	image_entry_noargs_t image_entry;
	u32 loadaddr, size_int;
	void *image_addr;
	int ret;

	ret = extract_lpm_region();
	if (ret)
		panic("Cannot find valid LPM address range..LPM resume failed\n");

	image_addr = (void *)mem_addr_lpm.atf_cert_addr;
	ret = rproc_load(1, (ulong)image_addr, 0x200);
	if (ret)
		panic("rproc failed to be initialized (%d)\n", ret);

	image_addr = mem_addr_lpm.atf_cert_addr;
	size_int = FW_IMAGE_SIZE;
	ti_secure_image_replay_cert(&image_addr, &size_int);

	image_addr = mem_addr_lpm.optee_cert_addr;
	ti_secure_image_replay_cert(&image_addr, &size_int);

	loadaddr = resume_to_dm_f();
	printf("Starting ATF on ARM64 core...\n\n");
	resume_rproc_f();

	image_entry = (image_entry_noargs_t)loadaddr;
	image_entry();
}
