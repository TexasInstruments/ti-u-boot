// SPDX-License-Identifier: GPL-2.0+
/*
 * K3: R5 Common Architecture initialization
 *
 * Copyright (C) 2023 Texas Instruments Incorporated - https://www.ti.com/
 */

#include <linux/printk.h>
#include <linux/types.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <image.h>
#include <fs_loader.h>
#include <linux/soc/ti/ti_sci_protocol.h>
#include <spl.h>
#include <remoteproc.h>
#include <elf.h>
#include <cpu_func.h>
#include <hang.h>
#include <power-domain.h>
#include <clk.h>
#include <dm/read.h>

#include "../common.h"
#include "../common_fdt.h"

#define PROC_BOOT_CTRL_RESET_FLAG_HSM_M4	0x00000001
#define HSM_SRAM0_0_ADDR			0x43C00000
#define PROC_ID_HSM_M4F				0x00000080

#if IS_ENABLED(CONFIG_SYS_K3_SPL_ATF)
enum {
	IMAGE_ID_HSM,
	IMAGE_ID_ATF,
	IMAGE_ID_OPTEE,
	IMAGE_ID_SPL,
	IMAGE_ID_DM_FW,
	IMAGE_ID_TIFSSTUB_HS,
	IMAGE_ID_TIFSSTUB_FS,
	IMAGE_ID_T,
	IMAGE_AMT,
};

#if CONFIG_IS_ENABLED(FIT_IMAGE_POST_PROCESS)
static const char *image_os_match[IMAGE_AMT] = {
	"hsm",
	"arm-trusted-firmware",
	"tee",
	"U-Boot",
	"DM",
	"tifsstub-hs",
	"tifsstub-fs",
	"tifsstub-gp",
};
#endif

static struct image_info fit_image_info[IMAGE_AMT];

struct lpm_addr_info mem_addr_lpm;

__weak void lpm_process(void)
{
	return;
}

__weak int extract_lpm_region(void)
{
	return -EINVAL;
}

void init_env(void)
{
#ifdef CONFIG_SPL_ENV_SUPPORT
	char *part;

	env_init();
	env_relocate();
	switch (spl_boot_device()) {
	case BOOT_DEVICE_MMC2:
		part = env_get("bootpart");
		env_set("storage_interface", "mmc");
		env_set("fw_dev_part", part);
		break;
	case BOOT_DEVICE_SPI:
		env_set("storage_interface", "ubi");
		env_set("fw_ubi_mtdpart", "UBI");
		env_set("fw_ubi_volume", "UBI0");
		break;
	default:
		printf("%s from device %u not supported!\n",
		       __func__, spl_boot_device());
		return;
	}
#endif
}

int load_firmware(char *name_fw, char *name_loadaddr, u32 *loadaddr)
{
	struct udevice *fsdev;
	char *name = NULL;
	int size = 0;

	if (!CONFIG_IS_ENABLED(FS_LOADER))
		return 0;

	*loadaddr = 0;
#ifdef CONFIG_SPL_ENV_SUPPORT
	switch (spl_boot_device()) {
	case BOOT_DEVICE_MMC2:
		name = env_get(name_fw);
		*loadaddr = env_get_hex(name_loadaddr, *loadaddr);
		break;
	default:
		printf("Loading rproc fw image from device %u not supported!\n",
		       spl_boot_device());
		return 0;
	}
#endif
	if (!*loadaddr)
		return 0;

	if (!get_fs_loader(&fsdev)) {
		size = request_firmware_into_buf(fsdev, name, (void *)*loadaddr,
						 0, 0);
	}

	return size;
}

void release_resources_for_core_shutdown(void)
{
	struct ti_sci_handle *ti_sci = get_ti_sci_handle();
	struct ti_sci_dev_ops *dev_ops = &ti_sci->ops.dev_ops;
	struct ti_sci_proc_ops *proc_ops = &ti_sci->ops.proc_ops;
	int ret;
	u32 i;

	/* Iterate through list of devices to put (shutdown) */
	for (i = 0; i < ARRAY_SIZE(put_device_ids); i++) {
		u32 id = put_device_ids[i];

		ret = dev_ops->put_device(ti_sci, id);
		if (ret)
			panic("Failed to put device %u (%d)\n", id, ret);
	}

	/* Iterate through list of cores to put (shutdown) */
	for (i = 0; i < ARRAY_SIZE(put_core_ids); i++) {
		u32 id = put_core_ids[i];

		/*
		 * Queue up the core shutdown request. Note that this call
		 * needs to be followed up by an actual invocation of an WFE
		 * or WFI CPU instruction.
		 */
		ret = proc_ops->proc_shutdown_no_wait(ti_sci, id);
		if (ret)
			panic("Failed sending core %u shutdown message (%d)\n",
			      id, ret);
	}
}

#if IS_ENABLED(CONFIG_K3_HSM_FW)
static int boot_hsm_core(void)
{
	struct ti_sci_handle *ti_sci = get_ti_sci_handle();
	struct ti_sci_proc_ops *proc_ops = &ti_sci->ops.proc_ops;
	u64 hsm_image_addr;
	u32 hsm_image_size;
	int device_type, ret;

	hsm_image_addr = (u64)fit_image_info[IMAGE_ID_HSM].image_start;
	hsm_image_size = (u32)fit_image_info[IMAGE_ID_HSM].image_len;

	/* Request Control of Remote Processor */
	ret = proc_ops->proc_request(ti_sci, PROC_ID_HSM_M4F);
	if (ret) {
		printf("Unable to request processor control for core %d\n",
		       PROC_ID_HSM_M4F);
		return ret;
	}

	/* Put the remote processor into reset */
	ret = proc_ops->set_proc_boot_ctrl(ti_sci, PROC_ID_HSM_M4F,
					   PROC_BOOT_CTRL_RESET_FLAG_HSM_M4, 0);
	if (ret) {
		printf("Unable to Halt core %d\n", PROC_ID_HSM_M4F);
		goto release_proc_ctrl;
	}

	/*
	 * Load the HSM firmware into core's internal memory.
	 *
	 * In case of HS device types, request secure entity to authenticate and
	 * load the HSM firmware into the core memory.
	 * In case of GP device types, copy the HSM firmware into the core
	 * memory manually.
	 */
	device_type = get_device_type();
	if (device_type == K3_DEVICE_TYPE_HS_SE ||
	    device_type == K3_DEVICE_TYPE_HS_FS) {
		ret = proc_ops->proc_auth_boot_image(ti_sci, &hsm_image_addr,
						     &hsm_image_size);
		if (ret) {
			printf("Unable to Authenticate and Boot HSM image; ret = %d\n",
			       ret);
			goto release_proc_ctrl;
		}
	} else if (device_type == K3_DEVICE_TYPE_GP) {
		debug("Loading HSM GP binary into SRAM0_0\n");
		memcpy((void *)HSM_SRAM0_0_ADDR, (void *)(u32)hsm_image_addr,
		       hsm_image_size);
		flush_dcache_range(HSM_SRAM0_0_ADDR,
				   HSM_SRAM0_0_ADDR + hsm_image_size);
	}

	/* Release the reset from the remote processor*/
	ret = proc_ops->set_proc_boot_ctrl(ti_sci, PROC_ID_HSM_M4F, 0,
					   PROC_BOOT_CTRL_RESET_FLAG_HSM_M4);
	if (ret)
		printf("Unable to Run core %d\n", PROC_ID_HSM_M4F);

release_proc_ctrl:
	/* Release Control of Remote Processor */
	proc_ops->proc_release(ti_sci, PROC_ID_HSM_M4F);
	return ret;
}
#endif

__weak int k3_mem_map_init(void)
{
	return 0;
}

void save_certificate(void) {

	if(!fit_image_info[IMAGE_ID_ATF].image_start || !fit_image_info[IMAGE_ID_OPTEE].image_start || !fit_image_info[IMAGE_ID_DM_FW].image_start) {
		pr_err("Invalid images to save\n");
		return;
	}

	int ret = extract_lpm_region();
	if(ret)
		pr_err("Cannot find valid LPM address range..\n");
	else {
		memcpy(mem_addr_lpm.atf_cert_addr , (void *)fit_image_info[IMAGE_ID_ATF].image_start, fit_image_info[IMAGE_ID_ATF].image_len);
		memcpy(mem_addr_lpm.optee_cert_addr, (void *)fit_image_info[IMAGE_ID_OPTEE].image_start, fit_image_info[IMAGE_ID_OPTEE].image_len);
		memcpy(mem_addr_lpm.dm_save_addr, (void *)fit_image_info[IMAGE_ID_DM_FW].image_start, fit_image_info[IMAGE_ID_DM_FW].image_len);
	}
}

void __noreturn jump_to_image_no_args(struct spl_image_info *spl_image)
{
	typedef void __noreturn (*image_entry_noargs_t)(void);
	struct ti_sci_handle *ti_sci = get_ti_sci_handle();
	u32 loadaddr = 0;
	int ret, size = 0, shut_cpu = 0;

	/* Release all the exclusive devices held by SPL before starting ATF */
	ti_sci->ops.dev_ops.release_exclusive_devices();

	ret = rproc_init();
	if (ret)
		panic("rproc failed to be initialized (%d)\n", ret);

	init_env();

	if (!fit_image_info[IMAGE_ID_DM_FW].image_start) {
		size = load_firmware("name_mcur5f0_0fw", "addr_mcur5f0_0load",
				     &loadaddr);
	}

#if IS_ENABLED(CONFIG_K3_HSM_FW)
	ret = boot_hsm_core();
	if (ret)
		printf("HSM core failed to boot, %d\n", ret);
	else
		printf("Successfully booted HSM core\n");
#endif
	/*
	 * It is assumed that remoteproc device 1 is the corresponding
	 * Cortex-A core which runs ATF. Make sure DT reflects the same.
	 */
	if (!fit_image_info[IMAGE_ID_ATF].image_start)
		fit_image_info[IMAGE_ID_ATF].image_start =
			spl_image->entry_point;

	ret = rproc_load(1, fit_image_info[IMAGE_ID_ATF].image_start, 0x200);
	if (ret)
		panic("%s: ATF failed to load on rproc (%d)\n", __func__, ret);

	lpm_process();
#if CONFIG_IS_ENABLED(FIT_IMAGE_POST_PROCESS)
	/* Authenticate ATF */
	void *image_addr = (void *)fit_image_info[IMAGE_ID_ATF].image_start;

	debug("%s: Authenticating image: addr=%lx, size=%ld, os=%s\n", __func__,
	      fit_image_info[IMAGE_ID_ATF].image_start,
	      fit_image_info[IMAGE_ID_ATF].image_len,
	      image_os_match[IMAGE_ID_ATF]);

	ti_secure_image_post_process(&image_addr,
				     (size_t *)&fit_image_info[IMAGE_ID_ATF].image_len);

	/* Authenticate OPTEE */
	image_addr = (void *)fit_image_info[IMAGE_ID_OPTEE].image_start;

	debug("%s: Authenticating image: addr=%lx, size=%ld, os=%s\n", __func__,
	      fit_image_info[IMAGE_ID_OPTEE].image_start,
	      fit_image_info[IMAGE_ID_OPTEE].image_len,
	      image_os_match[IMAGE_ID_OPTEE]);

	ti_secure_image_post_process(&image_addr,
				     (size_t *)&fit_image_info[IMAGE_ID_OPTEE].image_len);
#endif

	if (!fit_image_info[IMAGE_ID_DM_FW].image_len &&
	    !(size > 0 && valid_elf_image(loadaddr))) {
		shut_cpu = 1;
		goto start_arm64;
	}

	if (!fit_image_info[IMAGE_ID_DM_FW].image_start) {
		loadaddr = load_elf_image_phdr(loadaddr);
	} else {
		loadaddr = fit_image_info[IMAGE_ID_DM_FW].image_start;
		if (valid_elf_image(loadaddr)) {
#if IS_ENABLED(CONFIG_SOC_K3_J721E) || IS_ENABLED(CONFIG_SOC_K3_J784S4)
			ret = ti_sci->ops.lpm_ops.lpm_save_addr(ti_sci, (u32)mem_addr_lpm.context_save_addr, mem_addr_lpm.size);
			if (ret)
				pr_err("TIFS lpm save addr fail\n");
			loadaddr = fit_image_info[IMAGE_ID_DM_FW].image_start;
#endif
			loadaddr = load_elf_image_phdr(loadaddr);
		}
	}

	debug("%s: jumping to address %x\n", __func__, loadaddr);

start_arm64:
	/* Add an extra newline to differentiate the ATF logs from SPL */
	printf("Starting ATF on ARM64 core...\n\n");

	ret = rproc_start(1);
	if (ret)
		panic("%s: ATF failed to start on rproc (%d)\n",
		      __func__, ret);

	if (shut_cpu) {
		debug("Shutting down...\n");
		release_resources_for_core_shutdown();

		while (1)
			asm volatile("wfe");
	}
	image_entry_noargs_t image_entry = (image_entry_noargs_t)loadaddr;

	image_entry();
}
#endif

u32 resume_to_dm_f(void)
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
	ret = ti_sci->ops.lpm_ops.restore_context(ti_sci, 0);
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

void resume_rproc_f(void)
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

#define GTC_CNTCR_REG	0x0
#define GTC_CNTR_EN	0x3

	/* GTC counter values have been restored by TIFS, so enable the counter */
	writel(GTC_CNTR_EN, gtc_base + GTC_CNTCR_REG);

	ret = power_domain_on(&rproc_pwrdmn);
	if (ret)
		panic("power_domain_on failed: %d\n", ret);
}

void disable_linefill_optimization(void)
{
	u32 actlr;

	/*
	 * On K3 devices there are 2 conditions where R5F can deadlock:
	 * 1.When software is performing series of store operations to
	 *   cacheable write back/write allocate memory region and later
	 *   on software execute barrier operation (DSB or DMB). R5F may
	 *   hang at the barrier instruction.
	 * 2.When software is performing a mix of load and store operations
	 *   within a tight loop and store operations are all writing to
	 *   cacheable write back/write allocates memory regions, R5F may
	 *   hang at one of the load instruction.
	 *
	 * To avoid the above two conditions disable linefill optimization
	 * inside Cortex R5F.
	 */
	asm("mrc p15, 0, %0, c1, c0, 1" : "=r" (actlr));
	actlr |= (1 << 13); /* Set DLFO bit  */
	asm("mcr p15, 0, %0, c1, c0, 1" : : "r" (actlr));
}

int remove_fwl_region(struct fwl_data *fwl)
{
	struct ti_sci_handle *sci = get_ti_sci_handle();
	struct ti_sci_fwl_ops *ops = &sci->ops.fwl_ops;
	struct ti_sci_msg_fwl_region region;
	int ret;

	region.fwl_id = fwl->fwl_id;
	region.region = fwl->region;
	region.n_permission_regs = 3;

	ops->get_fwl_region(sci, &region);

	/* zero out the enable field of the firewall */
	region.control = region.control & ~0xF;

	pr_debug("Disabling firewall id: %d region: %d\n",
	       region.fwl_id, region.region);

	ret = ops->set_fwl_region(sci, &region);
	if (ret)
		pr_err("Could not disable firewall\n");
	return ret;
}

static void remove_fwl_regions(struct fwl_data fwl_data, size_t num_regions,
			       enum k3_firewall_region_type fwl_type)
{
	struct ti_sci_fwl_ops *fwl_ops;
	struct ti_sci_handle *ti_sci;
	struct ti_sci_msg_fwl_region region;
	size_t j;

	ti_sci = get_ti_sci_handle();
	fwl_ops = &ti_sci->ops.fwl_ops;

	for (j = 0; j < fwl_data.region; j++) {
		region.fwl_id = fwl_data.fwl_id;
		region.region = j;
		region.n_permission_regs = 3;

		fwl_ops->get_fwl_region(ti_sci, &region);

		/* Don't disable the background regions */
		if (region.control != 0 &&
		    ((region.control >> K3_FIREWALL_BACKGROUND_BIT) & 1) == fwl_type) {
			pr_debug("Attempting to disable firewall %5d (%25s)\n",
				 region.fwl_id, fwl_data.name);
			region.control = 0;

			if (fwl_ops->set_fwl_region(ti_sci, &region))
				pr_err("Could not disable firewall %5d (%25s)\n",
				       region.fwl_id, fwl_data.name);
		}
	}
}

void remove_fwl_configs(struct fwl_data *fwl_data, size_t fwl_data_size)
{
	size_t i;

	for (i = 0; i < fwl_data_size; i++) {
		remove_fwl_regions(fwl_data[i], fwl_data[i].region,
				   K3_FIREWALL_REGION_FOREGROUND);
		remove_fwl_regions(fwl_data[i], fwl_data[i].region,
				   K3_FIREWALL_REGION_BACKGROUND);
	}
}

#if CONFIG_IS_ENABLED(FIT_IMAGE_POST_PROCESS)
void board_fit_image_post_process(const void *fit, int node, void **p_image,
				  size_t *p_size)
{
	int len;
	int i;
	const char *os;
	u32 addr;

	os = fdt_getprop(fit, node, "os", &len);
	addr = fdt_getprop_u32_default_node(fit, node, 0, "entry", -1);

	debug("%s: processing image: addr=%x, size=%d, os=%s\n", __func__,
	      addr, *p_size, os);

	for (i = 0; i < IMAGE_AMT; i++) {
		if (!strcmp(os, image_os_match[i])) {
			fit_image_info[i].image_start = addr;
			fit_image_info[i].image_len = *p_size;
			debug("%s: matched image for ID %d\n", __func__, i);
			break;
		}
	}

	if (i < IMAGE_AMT && i > IMAGE_ID_DM_FW) {
		int device_type = get_device_type();

		if ((device_type == K3_DEVICE_TYPE_HS_SE &&
		     strcmp(os, "tifsstub-hs")) ||
		   (device_type == K3_DEVICE_TYPE_HS_FS &&
		     strcmp(os, "tifsstub-fs")) ||
		   (device_type == K3_DEVICE_TYPE_GP &&
		     strcmp(os, "tifsstub-gp"))) {
			*p_size = 0;
		} else {
			debug("tifsstub-type: %s\n", os);
		}

		return;
	}

	/*
	 * Only DM and the DTBs are being authenticated here,
	 * rest will be authenticated when A72 cluster is up
	 */
	if (i != IMAGE_ID_ATF && i != IMAGE_ID_OPTEE && i != IMAGE_ID_HSM) {
		ti_secure_image_check_binary(p_image, p_size);
		ti_secure_image_post_process(p_image, p_size);
	} else {
		ti_secure_image_check_binary(p_image, p_size);
	}
}
#endif

#ifdef CONFIG_SPL_OS_BOOT
int k3_r5_falcon_bootmode(void)
{
	char *mmcdev = env_get("mmcdev");

	if (!mmcdev)
		return BOOT_DEVICE_NOBOOT;

	if (strncmp(mmcdev, "0", sizeof("0")) == 0)
		return BOOT_DEVICE_MMC1;
	else if (strncmp(mmcdev, "1", sizeof("1")) == 0)
		return BOOT_DEVICE_MMC2;
	else
		return BOOT_DEVICE_NOBOOT;
}
#endif
