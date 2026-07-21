// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * J784S4: SoC specific initialization
 *
 * Copyright (C) 2023-2024 Texas Instruments Incorporated - https://www.ti.com/
 *	Hari Nagalla <hnagalla@ti.com>
 */

#include <init.h>
#include <spl.h>
#include <asm/io.h>
#include <asm/armv7_mpu.h>
#include <asm/arch/hardware.h>
#include <linux/soc/ti/ti_sci_protocol.h>
#include <soc/ti/k3-security.h>
#include <dm.h>
#include <dm/uclass-internal.h>
#include <dm/pinctrl.h>
#include <mmc.h>
#include <remoteproc.h>
#include <k3_bist.h>
#include <power/pmic.h>
#include <mach/k3-ddrss.h>

#include "../sysfw-loader.h"
#include "../common.h"

#if IS_ENABLED(CONFIG_TARGET_J742S2_R5_EVM)
#define MAX_DDR_CONTROLLERS	2
#else
#define MAX_DDR_CONTROLLERS	4
#endif

#define CTRL_MMR_CFG0_AUDIO_REFCLK1_CTRL	0x001082e4
#define AUDIO_REFCLK1_DEFAULT			0x1c

/* NAVSS North Bridge (NB) */
#define NAVSS0_NBSS_NB0_CFG_MMRS		0x03702000
#define NAVSS0_NBSS_NB1_CFG_MMRS		0x03703000
#define NAVSS0_NBSS_NB0_CFG_NB_THREADMAP	(NAVSS0_NBSS_NB0_CFG_MMRS + 0x10)
#define NAVSS0_NBSS_NB1_CFG_NB_THREADMAP	(NAVSS0_NBSS_NB1_CFG_MMRS + 0x10)
/*
 * Thread Map for North Bridge Configuration
 * Each bit is for each VBUSM source.
 * Bit[0] maps orderID 0-3 to VBUSM.C thread number
 * Bit[1] maps orderID 4-9 to VBUSM.C thread number
 * Bit[2] maps orderID 10-15 to VBUSM.C thread number
 * When bit has value 0: VBUSM.C thread 0 (non-real time traffic)
 * When bit has value 1: VBUSM.C thread 2 (real time traffic)
 */
#define NB_THREADMAP_BIT0				BIT(0)
#define NB_THREADMAP_BIT1				BIT(1)
#define NB_THREADMAP_BIT2				BIT(2)

/*
 * RAT mapping for errata ID: i2437
 */
#define RAT_ERRATA2437_BASE_REGION0		0x40f90000
#define RAT_ERRATA2437_IN_ADDR			0xc0000000
#define RAT_ERRATA2437_OUT_ADDR_U		0x0000004d
#define RAT_ERRATA2437_OUT_ADDR_L		0x21000000
#define RAT_ERRATA2437_CTRL			0x80000010

struct fwl_data infra_cbass0_fwls[] = {
	{ "PSC0", 5, 1 },
	{ "PLL_CTRL0", 6, 1 },
	{ "PLL_MMR0", 8, 26 },
	{ "CTRL_MMR0", 9, 16 },
	{ "GPIO0", 16, 1 },
}, wkup_cbass0_fwls[] = {
	{ "WKUP_PSC0", 129, 1 },
	{ "WKUP_PLL_CTRL0", 130, 1 },
	{ "WKUP_CTRL_MMR0", 131, 16 },
	{ "WKUP_GPIO0", 132, 1 },
	{ "WKUP_I2C0", 144, 1 },
	{ "WKUP_USART0", 160, 1 },
}, mcu_cbass0_fwls[] = {
	{ "MCU_R5FSS0_CORE0", 1024, 4 },
	{ "MCU_R5FSS0_CORE0_CFG", 1025, 3 },
	{ "MCU_R5FSS0_CORE1", 1028, 4 },
	{ "MCU_R5FSS0_CORE1_CFG", 1029, 1 },
	{ "MCU_FSS0_CFG", 1032, 12 },
	{ "MCU_FSS0_S1", 1033, 8 },
	{ "MCU_FSS0_S0", 1036, 8 },
	{ "MCU_PSROM49152X32", 1048, 1 },
	{ "MCU_MSRAM128KX64", 1050, 8 },
	{ "MCU_MSRAM128KX64_CFG", 1051, 1 },
	{ "MCU_TIMER0", 1056, 1 },
	{ "MCU_TIMER9", 1065, 1 },
	{ "MCU_USART0", 1120, 1 },
	{ "MCU_I2C0", 1152, 1 },
	{ "MCU_CTRL_MMR0", 1200, 8 },
	{ "MCU_PLL_MMR0", 1201, 3 },
	{ "MCU_CPSW0", 1220, 2 },
}, cbass_rc_cfg0_fwls[] = {
	{ "EMMCSD4SS0_CFG", 2400, 4 },
}, cbass_hc2_fwls[] = {
	{ "PCIE0", 2547, 24 },
}, cbass_hc_cfg0_fwls[] = {
	{ "PCIE0_CFG", 2577, 7 },
	{ "EMMC8SS0_CFG", 2579, 4 },
	{ "USB3SS0_CORE", 2580, 4 },
	{ "USB3SS1_CORE", 2581, 1 },
}, navss_cbass0_fwls[] = {
	{ "NACSS_VIRT0", 6253, 1 },
};

static void ctrl_mmr_unlock(void)
{
	/* Unlock all WKUP_CTRL_MMR0 module registers */
	mmr_unlock(WKUP_CTRL_MMR0_BASE, 0);
	mmr_unlock(WKUP_CTRL_MMR0_BASE, 1);
	mmr_unlock(WKUP_CTRL_MMR0_BASE, 2);
	mmr_unlock(WKUP_CTRL_MMR0_BASE, 3);
	mmr_unlock(WKUP_CTRL_MMR0_BASE, 4);
	mmr_unlock(WKUP_CTRL_MMR0_BASE, 6);
	mmr_unlock(WKUP_CTRL_MMR0_BASE, 7);

	/* Unlock all MCU_CTRL_MMR0 module registers */
	mmr_unlock(MCU_CTRL_MMR0_BASE, 0);
	mmr_unlock(MCU_CTRL_MMR0_BASE, 1);
	mmr_unlock(MCU_CTRL_MMR0_BASE, 2);
	mmr_unlock(MCU_CTRL_MMR0_BASE, 3);
	mmr_unlock(MCU_CTRL_MMR0_BASE, 4);

	/* Unlock all CTRL_MMR0 module registers */
	mmr_unlock(CTRL_MMR0_BASE, 0);
	mmr_unlock(CTRL_MMR0_BASE, 1);
	mmr_unlock(CTRL_MMR0_BASE, 2);
	mmr_unlock(CTRL_MMR0_BASE, 3);
	mmr_unlock(CTRL_MMR0_BASE, 5);
	mmr_unlock(CTRL_MMR0_BASE, 7);
}

/* Setup North Bridge registers to map ORDERID 10-15 to RT traffic */
static void setup_navss_nb(void)
{
	writel(NB_THREADMAP_BIT1, (uintptr_t)NAVSS0_NBSS_NB0_CFG_NB_THREADMAP);
	writel(NB_THREADMAP_BIT2, (uintptr_t)NAVSS0_NBSS_NB1_CFG_NB_THREADMAP);
}

static void open_mpu_clec_firewalls(void)
{
	int ret = 0;
	/*
	 * The firewall regions are being configured for backward compatibility,
	 * Future firmware revisions would configure the background firewalls
	 * properly hence allow everyone from a foreground as well but keep it
	 * unlocked for reconfigurability.
	 */
	u32 allow_all_perm = ((FWPRIVID_ALL << FWPRIVID_SHIFT) |
			      FWPERM_NON_SECURE_PRIV_RWCD |
			      FWPERM_NON_SECURE_USER_RWCD |
			      FWPERM_SECURE_PRIV_RWCD |
			      FWPERM_SECURE_USER_RWCD);

	struct fwl_config clec_region = {
		.data.name = "CLEC",
		.data.region = 3,
		.n_permission_regs = 3,
		.control = FWCTRL_EN | FWCTRL_CACHE,
		.permissions = {allow_all_perm, allow_all_perm, allow_all_perm},
		.start_address = (0x78000000),
		.end_address   = (0x7f000000)
	};

	/* Program the first port for CLEC region */
	clec_region.data.fwl_id = 5140;

	ret = configure_fwl_region(&clec_region);
	if (ret)
		printf("Failed to enable firewall region %u: %d\n",
		       clec_region.data.fwl_id, ret);

	/* Program the second port for CLEC region */
	clec_region.data.fwl_id = 5141;

	ret = configure_fwl_region(&clec_region);
	if (ret)
		printf("Failed to enable firewall region %u: %d\n",
		       clec_region.data.fwl_id, ret);
}

/* Execute and check results of BIST executed on MCU1_x and MCU4_O */
static void run_bist_j784s4(struct udevice *dev)
{
	struct bist_ops *ops;
	struct ti_sci_handle *handle;
	int ret;

	ops = (struct bist_ops *)device_get_ops(dev);
	handle = get_ti_sci_handle();

	/* get status of HW POST PBIST on MCU1_x */
	if (ops->run_pbist_post())
		panic("HW POST LBIST on MCU1_x failed\n");

	/* trigger PBIST tests on MCU4_0 */
	ret = prepare_pbist(handle);
	ret |= ops->run_pbist_neg();
	ret |= deprepare_pbist(handle);

	ret |= prepare_pbist(handle);
	ret |= ops->run_pbist();
	ret |= deprepare_pbist(handle);

	ret |= prepare_pbist(handle);
	ret |= ops->run_pbist_rom();
	ret |= deprepare_pbist(handle);

	if (ret)
		panic("PBIST on MCU4_0 failed: %d\n", ret);

	/* get status of HW POST PBIST on MCU1_x */
	if (ops->run_lbist_post())
		panic("HW POST LBIST on MCU1_x failed\n");

	/* trigger LBIST tests on MCU1_x */
	ret = prepare_lbist(handle);
	ret |= ops->run_lbist();
	ret |= deprepare_lbist(handle);
	if (ret)
		panic("LBIST on MCU4_0 failed: %d\n", ret);
}

/*
 * This uninitialized global variable would normal end up in the .bss section,
 * but the .bss is cleared between writing and reading this variable, so move
 * it to the .data section.
 */
u32 bootindex __section(".data");
static struct rom_extended_boot_data bootdata __section(".data");

static void store_boot_info_from_rom(void)
{
	bootindex = *(u32 *)(CONFIG_SYS_K3_BOOT_PARAM_TABLE_INDEX);
	memcpy(&bootdata, (uintptr_t *)ROM_EXTENDED_BOOT_DATA_INFO,
	       sizeof(struct rom_extended_boot_data));
}

void k3_spl_init(void)
{
	struct udevice *dev;
	int ret;

	/*
	 * Cannot delay this further as there is a chance that
	 * K3_BOOT_PARAM_TABLE_INDEX can be over written by SPL MALLOC section.
	 */
	store_boot_info_from_rom();

	/* Make all control module registers accessible */
	ctrl_mmr_unlock();

	if (IS_ENABLED(CONFIG_CPU_V7R)) {
		disable_linefill_optimization();
		setup_k3_mpu_regions();
	}

	/* Init DM early */
	ret = spl_early_init();

	/* Prepare console output */
	preloader_console_init();

	if (IS_ENABLED(CONFIG_CPU_V7R)) {
		/*
		 * Process pinctrl for the serial0 a.k.a. WKUP_UART0 module and continue
		 * regardless of the result of pinctrl. Do this without probing the
		 * device, but instead by searching the device that would request the
		 * given sequence number if probed. The UART will be used by the system
		 * firmware (TIFS) image for various purposes and TIFS depends on us
		 * to initialize its pin settings.
		 */
		ret = uclass_find_device_by_seq(UCLASS_SERIAL, 0, &dev);
		if (!ret)
			pinctrl_select_state(dev, "default");

		/*
		 * Load, start up, and configure system controller firmware. Provide
		 * the U-Boot console init function to the TIFS post-PM configuration
		 * callback hook, effectively switching on (or over) the console
		 * output.
		 */
		k3_sysfw_loader(is_rom_loaded_sysfw(&bootdata), NULL, NULL);

		if (IS_ENABLED(CONFIG_SPL_CLK_K3)) {
			/*
			 * Force probe of clk_k3 driver here to ensure basic default clock
			 * configuration is always done for enabling PM services.
			 */
			ret = uclass_get_device_by_driver(UCLASS_CLK,
							  DM_DRIVER_GET(ti_clk),
							  &dev);
			if (ret)
				panic("Failed to initialize clk-k3!\n");
		}

		remove_fwl_configs(cbass_hc_cfg0_fwls, ARRAY_SIZE(cbass_hc_cfg0_fwls));
		remove_fwl_configs(cbass_hc2_fwls, ARRAY_SIZE(cbass_hc2_fwls));
		remove_fwl_configs(cbass_rc_cfg0_fwls, ARRAY_SIZE(cbass_rc_cfg0_fwls));
		remove_fwl_configs(infra_cbass0_fwls, ARRAY_SIZE(infra_cbass0_fwls));
		remove_fwl_configs(mcu_cbass0_fwls, ARRAY_SIZE(mcu_cbass0_fwls));
		remove_fwl_configs(wkup_cbass0_fwls, ARRAY_SIZE(wkup_cbass0_fwls));
		remove_fwl_configs(navss_cbass0_fwls, ARRAY_SIZE(navss_cbass0_fwls));

		open_mpu_clec_firewalls();
	}

	writel(AUDIO_REFCLK1_DEFAULT, (uintptr_t)CTRL_MMR_CFG0_AUDIO_REFCLK1_CTRL);

	/* Shutdown MCU_R5 Core 1 in Split mode at A72 SPL Stage */
	if (IS_ENABLED(CONFIG_ARM64)) {
		ret = shutdown_mcu_r5_core1();
		if (ret)
			printf("Unable to shutdown MCU R5 core 1, %d\n", ret);
	}

	/* Output System Firmware version info */
	k3_sysfw_print_ver();

	/* Output DM Firmware version info */
	if (IS_ENABLED(CONFIG_ARM64))
		k3_dm_print_ver();
}

void lpm_process(void)
{
	int ret = 0;
	struct ti_sci_handle *ti_sci = get_ti_sci_handle();
	save_certificate();
	ret = ti_sci->ops.lpm_ops.lpm_save_addr(ti_sci, (u32)mem_addr_lpm.context_save_addr, mem_addr_lpm.size);
	if (ret)
		pr_err("TIFS lpm save addr fail\n");
}

int extract_lpm_region(void)
{
    ofnode node;
	u32 lpm_reg_addr, lpm_reg_size;

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
	mem_addr_lpm.atf_cert_addr =  (u32 *)((uintptr_t)mem_addr_lpm.context_save_addr + FW_IMAGE_SIZE);
	mem_addr_lpm.optee_cert_addr = (u32 *)((uintptr_t)mem_addr_lpm.atf_cert_addr + FW_IMAGE_SIZE);
	mem_addr_lpm.dm_save_addr = (u32 *)((uintptr_t)mem_addr_lpm.optee_cert_addr + (2*FW_IMAGE_SIZE));
	mem_addr_lpm.size = lpm_reg_size;
	return 0;
}

#define DDR_RET_VAL BIT(5)
#define GPIO_OUT_1 0x3D
#define PMIC_NSLEEP_REG 0x86

static void k3_deassert_DDR_RET(void)
{
	struct udevice *pmic;
	int regval;
	int err;

	err = uclass_get_device_by_name(UCLASS_PMIC,
					"pmic@48", &pmic);
	if (err) {
		printf("Getting PMIC init failed: %d\n", err);
		return;
	}

	/* Set DDR_RET Signal Low on PMIC B */
	regval = pmic_reg_read(pmic, GPIO_OUT_1) & ~DDR_RET_VAL;

	pmic_reg_write(pmic, GPIO_OUT_1, regval);
	pmic_reg_write(pmic, PMIC_NSLEEP_REG, 0x3);
}

void k3_mem_init(void)
{
	struct udevice *dev;
	int ret, ctrl = 0;

	if (IS_ENABLED(CONFIG_K3_J721E_DDRSS)) {
		struct udevice *devs[MAX_DDR_CONTROLLERS];
		struct k3_ddrss_regs regs[MAX_DDR_CONTROLLERS];

		ret = uclass_get_device(UCLASS_RAM, 0, &dev);
		if (ret)
			panic("DRAM 0 init failed: %d\n", ret);

		devs[0] = dev;
		ctrl++;

		while (ctrl < MAX_DDR_CONTROLLERS) {
			ret = uclass_next_device_err(&dev);
			if (ret == -ENODEV)
				break;

			if (ret)
				panic("DRAM %d init failed: %d\n", ctrl, ret);
			devs[ctrl] = dev;
			ctrl++;
		}

		if (board_is_resuming()) {
			typedef void __noreturn (*image_entry_noargs_t)(void);
			u32 loadaddr, size_int;
			void *image_addr;
			/* exit DDRs from retention */
			for (ctrl = 0; ctrl < MAX_DDR_CONTROLLERS; ctrl++) {
				k3_ddrss_lpddr4_exit_retention(devs[ctrl],
							       &regs[ctrl]);
			}

			/* de-assert DDR_RET pin */
			k3_deassert_DDR_RET();

			/* restore DDR max frequency */
			for (ctrl = 0; ctrl < MAX_DDR_CONTROLLERS; ctrl++)
				k3_ddrss_lpddr4_change_freq(devs[ctrl]);

			/* exit DDR from low power */
			for (ctrl = 0; ctrl < MAX_DDR_CONTROLLERS; ctrl++) {
				k3_ddrss_lpddr4_exit_low_power(devs[ctrl],
							       &regs[ctrl]);
			}
			printf("Initialized %d DRAM controllers\n", ctrl);
			ret = extract_lpm_region();
			if(ret)
				panic("Cannot find valid LPM address range..LPM resume failed \n");
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

			image_entry_noargs_t image_entry = (image_entry_noargs_t)loadaddr;
			image_entry();
		}
		printf("Initialized %d DRAM controllers\n", ctrl);
	}

	spl_enable_cache();
}

void board_init_f(ulong dummy)
{
	struct udevice *dev;
	int ret;
	/* init resume flag */
	gd_set_k3_resuming(-1);

	k3_spl_init();
	k3_mem_init();

	if (IS_ENABLED(CONFIG_CPU_V7R) && IS_ENABLED(CONFIG_K3_AVS0)) {
		ret = uclass_get_device_by_driver(UCLASS_MISC, DM_DRIVER_GET(k3_avs),
						  &dev);
		if (ret)
			printf("AVS init failed: %d\n", ret);
	}

	if (!IS_ENABLED(CONFIG_CPU_V7R) && IS_ENABLED(CONFIG_K3_BIST)) {
		ret = uclass_get_device_by_driver(UCLASS_MISC,
						  DM_DRIVER_GET(k3_bist),
						  &dev);
		if (ret)
			panic("Failed to get BIST device: %d\n", ret);
		run_bist_j784s4(dev);
	}

	if (IS_ENABLED(CONFIG_CPU_V7R))
		setup_navss_nb();

	setup_qos();

	if (IS_ENABLED(CONFIG_CPU_V7R)) {
		/*
		 * Errata ID i2437 SE Clock-Gating Turning Off Too Early
		 *
		 * A hardware bug is present in the C7120 Streaming Engine top level
		 * clock gating logic thatcan lead to the C7120 CPU hanging.

		 * Workaround: The DSP_<COREID>_DEBUG_CLKEN_OVERRIDE fields of the
		 * COMPUTE_CLUSTER_CFG_WRAP_0_CC_CNTRL register (where COREID is the
		 * name of the specific C7120 core) must be enabled before power-up
		 * of the C7120 core to override all clock-gating.
		 */

		/* Setup RAT mapping */
		debug("Errata i2437: Use RAT for COMPUTE_CLUSTER_CFG_WRAP_0_CC_CNTRL register\n");
		*(unsigned int *)(RAT_ERRATA2437_BASE_REGION0 + 0x24) = RAT_ERRATA2437_IN_ADDR;
		*(unsigned int *)(RAT_ERRATA2437_BASE_REGION0 + 0x28) = RAT_ERRATA2437_OUT_ADDR_L;
		*(unsigned int *)(RAT_ERRATA2437_BASE_REGION0 + 0x2c) = RAT_ERRATA2437_OUT_ADDR_U;
		*(unsigned int *)(RAT_ERRATA2437_BASE_REGION0 + 0x20) = RAT_ERRATA2437_CTRL;

		/* Enable DSP_X_DEBUG_CLKEN_OVERRIDE for C71x cores */
		*(unsigned int *)(RAT_ERRATA2437_IN_ADDR + 0x200) = 0x00000F00;

		/* Clear RAT mapping */
		*(unsigned int *)(RAT_ERRATA2437_BASE_REGION0 + 0x20) = 0x0;
		*(unsigned int *)(RAT_ERRATA2437_BASE_REGION0 + 0x24) = 0x0;
		*(unsigned int *)(RAT_ERRATA2437_BASE_REGION0 + 0x28) = 0x0;
		*(unsigned int *)(RAT_ERRATA2437_BASE_REGION0 + 0x2c) = 0x0;
	}
}

u32 spl_mmc_boot_mode(struct mmc *mmc, const u32 boot_device)
{
	switch (boot_device) {
	case BOOT_DEVICE_MMC1:
		if (IS_ENABLED(CONFIG_SUPPORT_EMMC_BOOT))
			return MMCSD_MODE_EMMCBOOT;
		if (IS_ENABLED(CONFIG_SPL_FS_FAT) || IS_ENABLED(CONFIG_SPL_FS_EXT4))
			return MMCSD_MODE_FS;
		return MMCSD_MODE_EMMCBOOT;
	case BOOT_DEVICE_MMC2:
		return MMCSD_MODE_FS;
	default:
		return MMCSD_MODE_RAW;
	}
}

static u32 __get_backup_bootmedia(u32 main_devstat)
{
	u32 bkup_boot = (main_devstat & MAIN_DEVSTAT_BKUP_BOOTMODE_MASK) >>
			MAIN_DEVSTAT_BKUP_BOOTMODE_SHIFT;

	switch (bkup_boot) {
	case BACKUP_BOOT_DEVICE_USB:
		return BOOT_DEVICE_DFU;
	case BACKUP_BOOT_DEVICE_UART:
		return BOOT_DEVICE_UART;
	case BACKUP_BOOT_DEVICE_ETHERNET:
		return BOOT_DEVICE_ETHERNET;
	case BACKUP_BOOT_DEVICE_MMC2:
	{
		u32 port = (main_devstat & MAIN_DEVSTAT_BKUP_MMC_PORT_MASK) >>
			    MAIN_DEVSTAT_BKUP_MMC_PORT_SHIFT;
		if (port == 0x0)
			return BOOT_DEVICE_MMC1;
		return BOOT_DEVICE_MMC2;
	}
	case BACKUP_BOOT_DEVICE_SPI:
		return BOOT_DEVICE_SPI;
	case BACKUP_BOOT_DEVICE_I2C:
		return BOOT_DEVICE_I2C;
	}

	return BOOT_DEVICE_RAM;
}

static u32 __get_primary_bootmedia(u32 main_devstat, u32 wkup_devstat)
{
	u32 bootmode = (wkup_devstat & WKUP_DEVSTAT_PRIMARY_BOOTMODE_MASK) >>
			WKUP_DEVSTAT_PRIMARY_BOOTMODE_SHIFT;

	bootmode |= (main_devstat & MAIN_DEVSTAT_BOOT_MODE_B_MASK) <<
			BOOT_MODE_B_SHIFT;

	if (bootmode == BOOT_DEVICE_OSPI || bootmode ==	BOOT_DEVICE_QSPI ||
	    bootmode == BOOT_DEVICE_XSPI)
		bootmode = BOOT_DEVICE_SPI;

	if (bootmode == BOOT_DEVICE_MMC2) {
		u32 port = (main_devstat &
			    MAIN_DEVSTAT_PRIM_BOOTMODE_MMC_PORT_MASK) >>
			   MAIN_DEVSTAT_PRIM_BOOTMODE_PORT_SHIFT;
		if (port == 0x0)
			bootmode = BOOT_DEVICE_MMC1;
	}

	return bootmode;
}

u32 spl_spi_boot_bus(void)
{
	u32 wkup_devstat = readl(CTRLMMR_WKUP_DEVSTAT);
	u32 main_devstat = readl(CTRLMMR_MAIN_DEVSTAT);
	u32 bootmode = ((wkup_devstat & WKUP_DEVSTAT_PRIMARY_BOOTMODE_MASK) >>
				WKUP_DEVSTAT_PRIMARY_BOOTMODE_SHIFT) |
			((main_devstat & MAIN_DEVSTAT_BOOT_MODE_B_MASK) << BOOT_MODE_B_SHIFT);

	return (bootmode == BOOT_DEVICE_QSPI) ? 1 : 0;
}

u32 spl_boot_device(void)
{
	u32 wkup_devstat = readl(CTRLMMR_WKUP_DEVSTAT);
	u32 main_devstat;

	if (wkup_devstat & WKUP_DEVSTAT_MCU_ONLY_MASK) {
		printf("ERROR: MCU only boot is not yet supported\n");
		return BOOT_DEVICE_RAM;
	}

	/* MAIN CTRL MMR can only be read if MCU ONLY is 0 */
	main_devstat = readl(CTRLMMR_MAIN_DEVSTAT);

	if (bootindex == K3_PRIMARY_BOOTMODE)
		return __get_primary_bootmedia(main_devstat, wkup_devstat);
	else
		return __get_backup_bootmedia(main_devstat);
}
