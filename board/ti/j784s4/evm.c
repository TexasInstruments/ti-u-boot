// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Board specific initialization for J784S4 EVM
 *
 * Copyright (C) 2023-2024 Texas Instruments Incorporated - https://www.ti.com/
 *	Hari Nagalla <hnagalla@ti.com>
 *
 */

#include <dm.h>
#include <efi_loader.h>
#include <env.h>
#include <fdt_support.h>
#include <image.h>
#include <init.h>
#include <log.h>
#include <asm/arch/hardware.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <spl.h>
#include <asm/arch/k3-ddr.h>
#include <power/pmic.h>
#include <wait_bit.h>
#include <mach/k3-ddrss.h>
#include "../common/fdt_ops.h"

DECLARE_GLOBAL_DATA_PTR;

struct efi_fw_image fw_images[] = {
	{
		.image_type_id = AM69_SK_TIBOOT3_IMAGE_GUID,
		.fw_name = u"AM69_SK_TIBOOT3",
		.image_index = 1,
	},
	{
		.image_type_id = AM69_SK_SPL_IMAGE_GUID,
		.fw_name = u"AM69_SK_SPL",
		.image_index = 2,
	},
	{
		.image_type_id = AM69_SK_UBOOT_IMAGE_GUID,
		.fw_name = u"AM69_SK_UBOOT",
		.image_index = 3,
	}
};

struct efi_capsule_update_info update_info = {
	.dfu_string = "sf 0:0=tiboot3.bin raw 0 80000;"
	"tispl.bin raw 80000 200000;u-boot.img raw 280000 400000",
	.num_images = ARRAY_SIZE(fw_images),
	.images = fw_images,
};

#if IS_ENABLED(CONFIG_SET_DFU_ALT_INFO)
void set_dfu_alt_info(char *interface, char *devstr)
{
	if (IS_ENABLED(CONFIG_EFI_HAVE_CAPSULE_SUPPORT))
		env_set("dfu_alt_info", update_info.dfu_string);
}
#endif

int board_init(void)
{
	return 0;
}

/* Enables the spi-nand dts node, if onboard mux is set to spinand */
static void __maybe_unused detect_enable_spinand(void *blob)
{
	if (IS_ENABLED(CONFIG_DM_GPIO) && IS_ENABLED(CONFIG_OF_LIBFDT)) {
		struct gpio_desc desc = {0};
		char *ospi_mux_sel_gpio = "6";
		int nand_offset, nor_offset;

		if (dm_gpio_lookup_name(ospi_mux_sel_gpio, &desc))
			return;

		if (dm_gpio_request(&desc, ospi_mux_sel_gpio))
			return;

		if (dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN))
			return;

		nand_offset = fdt_node_offset_by_compatible(blob, -1, "spi-nand");
		if (nand_offset < 0)
			return;

		nor_offset = fdt_node_offset_by_compatible(blob,
							   fdt_parent_offset(blob, nand_offset),
							   "jedec,spi-nor");

		if (dm_gpio_get_value(&desc)) {
			fdt_status_okay(blob, nand_offset);
			fdt_del_node(blob, nor_offset);
		} else {
			fdt_del_node(blob, nand_offset);
		}
	}
}

#if defined(CONFIG_XPL_BUILD)
void spl_perform_fixups(struct spl_image_info *spl_image)
{
#if (IS_ENABLED(CONFIG_TARGET_J784S4_R5_EVM) || IS_ENABLED(CONFIG_TARGET_J742S2_R5_EVM))
	if (board_is_resuming())
		return;
#endif
	if (IS_ENABLED(CONFIG_K3_DDRSS)) {
		if (IS_ENABLED(CONFIG_K3_INLINE_ECC))
			fixup_ddr_driver_for_ecc(spl_image);
	} else {
		fixup_memory_node(spl_image);
	}
	detect_enable_spinand(spl_image->fdt_addr);
}
#endif

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
	ti_set_fdt_env(NULL, NULL);
	return 0;
}
#endif

#if (IS_ENABLED(CONFIG_SPL_BUILD) && (IS_ENABLED(CONFIG_TARGET_J784S4_R5_EVM) || IS_ENABLED(CONFIG_TARGET_J742S2_R5_EVM)))

#define SCRATCH_PAD_REG_3 0xCB
#define MAGIC_SUSPEND 0xBA
#define LPM_WAKE_SOURCE_PMIC_GPIO 0x91
#define LPM_WAKE_SOURCE_MCU_IO    0x81

static void clear_isolation(void)
{
	int ret;
	const void *wait_reg = (const void *)(WKUP_CTRL_MMR0_BASE + MCU_GEN_WAKE_STAT1);

	/* un-set the magic word for MCU_GENERAL IOs */
	writel(IO_ISO_MAGIC_VAL, WKUP_CTRL_MMR0_BASE + MCU_GEN_WAKE_CTRL);
	writel((IO_ISO_MAGIC_VAL + 0x1), WKUP_CTRL_MMR0_BASE + MCU_GEN_WAKE_CTRL);
	writel(IO_ISO_MAGIC_VAL, WKUP_CTRL_MMR0_BASE + MCU_GEN_WAKE_CTRL);

	/* wait for MCU_GEN_IO_MODE bit to be cleared */
	ret = wait_for_bit_32(wait_reg,
			      MCU_GEN_WAKE_STAT1_MCU_GEN_IO_MODE,
			      false,
			      DEISOLATION_TIMEOUT_MS,
			      false);
	if (ret < 0)
		pr_err("Deisolation timeout");
}

/* in board_init_f(), there's no BSS, so we can't use global/static variables */
int board_is_resuming(void)
{
	struct udevice *pmic;
	u32 pmctrl_val = readl(WKUP_CTRL_MMR0_BASE + PMCTRL_IO_0);
	struct lpm_scratch_space *lpm_scratch = (struct lpm_scratch_space *)TI_SRAM_LPM_SCRATCH;
	int err;

	if (gd_k3_resuming() >= 0)
		goto end;

	if ((pmctrl_val & IO_ISO_STATUS) == IO_ISO_STATUS) {
		lpm_scratch->wake_src = LPM_WAKE_SOURCE_MCU_IO;
		clear_isolation();
		gd_set_k3_resuming(1);
		debug("board is resuming from IO_DDR mode\n");
		return gd_k3_resuming();
	}

	err = uclass_get_device_by_name(UCLASS_PMIC,
					"pmic@48", &pmic);
	if (err) {
		printf("Getting PMIC init failed: %d\n", err);
		goto end;
	}
	debug("%s: PMIC is detected (%s)\n", __func__, pmic->name);

	if (pmic_reg_read(pmic, SCRATCH_PAD_REG_3) == MAGIC_SUSPEND) {
		debug("%s: board is resuming\n", __func__);
		lpm_scratch->wake_src = LPM_WAKE_SOURCE_PMIC_GPIO;
		gd_set_k3_resuming(1);

		/* clean magic suspend */
		if (pmic_reg_write(pmic, SCRATCH_PAD_REG_3, 0))
			printf("Failed to clean magic value for suspend detection in PMIC\n");
	} else {
		debug("%s: board is booting (no resume detected)\n", __func__);
		lpm_scratch->wake_src = 0;
		lpm_scratch->reserved = 0;
		gd_set_k3_resuming(0);
	}
end:
	return gd_k3_resuming();
}

#endif /* CONFIG_SPL_BUILD && CONFIG_TARGET_J784s4_R5_EVM */

void spl_board_init(void)
{
	struct udevice *dev;
	int ret;

	if (IS_ENABLED(CONFIG_ESM_K3)) {
		const char * const esms[] = {"esm@700000", "esm@40800000", "esm@42080000"};

		for (int i = 0; i < ARRAY_SIZE(esms); ++i) {
			ret = uclass_get_device_by_name(UCLASS_MISC, esms[i],
							&dev);
			if (ret) {
				printf("MISC init for %s failed: %d\n", esms[i], ret);
				break;
			}
		}
	}

	if (IS_ENABLED(CONFIG_ESM_PMIC) && ret == 0) {
		ret = uclass_get_device_by_driver(UCLASS_MISC,
						  DM_DRIVER_GET(pmic_esm),
						  &dev);
		if (ret)
			printf("ESM PMIC init failed: %d\n", ret);
	}
}

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
int ft_board_setup(void *blob, struct bd_info *bd)
{
	detect_enable_spinand(blob);

	return 0;
}
#endif

ofnode cadence_qspi_get_subnode(struct udevice *dev)
{
	if (IS_ENABLED(CONFIG_SPL_BUILD) &&
	    IS_ENABLED(CONFIG_TARGET_J784S4_R5_EVM)) {
		if (spl_boot_device() == BOOT_DEVICE_SPINAND)
			return ofnode_by_compatible(dev_ofnode(dev), "spi-nand");
	}

	return dev_read_first_subnode(dev);
}
