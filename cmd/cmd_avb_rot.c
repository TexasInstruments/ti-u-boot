// SPDX-License-Identifier: GPL-2.0
/*
 * U-Boot command for testing Root of Trust functionality
 * Provides commands to setup and verify Root of Trust implementation
 *
 * Copyright (C) 2025 Baylibre
 */

#include <command.h>
#include <avb_root_of_trust.h>

/*
 * Command to setup Root of Trust using integrated AVB verification
 */
static int do_avb_setup_rot(struct cmd_tbl *cmdtp, int flag,
			    int argc, char *const argv[])
{
	AvbOps *ops = NULL;
	AvbSlotVerifyData *slot_data = NULL;
	AvbSlotVerifyResult result;
	static const char * const requested_partitions[] = {"boot", NULL};
	int ret;

	printf("Setting up Root of Trust with AVB verification...\n");

	/* Initialize AVB operations */
	ops = avb_ops_alloc(CONFIG_FASTBOOT_FLASH_MMC_DEV);
	if (!ops) {
		printf("ERROR: Failed to allocate AVB operations\n");
		return CMD_RET_FAILURE;
	}

	/* Perform AVB verification */
	result = avb_slot_verify(ops, requested_partitions, "",
				 AVB_SLOT_VERIFY_FLAGS_NONE,
				AVB_HASHTREE_ERROR_MODE_RESTART_AND_INVALIDATE,
				&slot_data);

	/* Store Root of Trust from verification results */
	if (slot_data) {
		ret = avb_store_root_of_trust_from_verification(slot_data, result);
		if (ret) {
			printf("ERROR: Root of Trust storage failed: %d\n", ret);
			ret = CMD_RET_FAILURE;
		} else {
			printf("SUCCESS: Root of Trust stored from AVB verification\n");
			ret = CMD_RET_SUCCESS;
		}
	} else {
		printf("ERROR: No AVB verification data available\n");
		ret = CMD_RET_FAILURE;
	}

	/* Display verification result */
	switch (result) {
	case AVB_SLOT_VERIFY_RESULT_OK:
		printf("AVB Result: GREEN (verified and secure)\n");
		break;
	case AVB_SLOT_VERIFY_RESULT_ERROR_ROLLBACK_INDEX:
		printf("AVB Result: YELLOW (verified with rollback error)\n");
		break;
	default:
		printf("AVB Result: ORANGE/RED (verification failed)\n");
		break;
	}

	/* Cleanup */
	if (slot_data)
		avb_slot_verify_data_free(slot_data);
	avb_ops_free(ops);

	return ret;
}

/*
 * Command to setup Root of Trust using legacy method (for testing)
 */
static int do_avb_setup_rot_legacy(struct cmd_tbl *cmdtp, int flag,
				   int argc, char *const argv[])
{
	int ret;

	printf("Setting up Root of Trust (legacy method)...\n");

	ret = avb_setup_root_of_trust();
	if (ret) {
		printf("ERROR: Root of Trust setup failed: %d\n", ret);
		return CMD_RET_FAILURE;
	}

	printf("SUCCESS: Root of Trust setup completed\n");
	return CMD_RET_SUCCESS;
}

/*
 * Command to show Root of Trust information
 */
static int do_avb_show_rot_info(struct cmd_tbl *cmdtp, int flag,
				int argc, char *const argv[])
{
	avb_show_root_of_trust_info();
	return CMD_RET_SUCCESS;
}

/*
 * Main AVB Root of Trust command dispatcher
 */
static int do_avb_rot(struct cmd_tbl *cmdtp, int flag,
		      int argc, char *const argv[])
{
	if (argc < 2)
		return CMD_RET_USAGE;

	if (strcmp(argv[1], "setup") == 0)
		return do_avb_setup_rot(cmdtp, flag, argc - 1, argv + 1);
	else if (strcmp(argv[1], "legacy") == 0)
		return do_avb_setup_rot_legacy(cmdtp, flag, argc - 1, argv + 1);
	else if (strcmp(argv[1], "info") == 0)
		return do_avb_show_rot_info(cmdtp, flag, argc - 1, argv + 1);

	return CMD_RET_USAGE;
}

U_BOOT_CMD(avb_rot, 3, 0, do_avb_rot,
	   "Android Verified Boot Root of Trust commands",
	   "setup  - Setup Root of Trust from AVB verification (recommended)\n"
	   "avb_rot legacy - Setup Root of Trust using legacy method\n"
	   "avb_rot info   - Show Root of Trust implementation information"
);
