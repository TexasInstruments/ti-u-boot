// SPDX-License-Identifier: GPL-2.0
/*
 * Root of Trust implementation for Android Verified Boot
 * This module collects and stores Root of Trust information from AVB
 * verification for later retrieval by the KeyMint TA
 *
 * Copyright (C) 2025 Baylibre
 */

#include <avb_verify.h>
#include <tee.h>
#include <tee/optee_ta_avb.h>
#include <u-boot/sha256.h>
#include <stdio.h>
#include <string.h>
#include <env.h>
#include <time.h>
#include "libavb/libavb.h"

/* Root of Trust structure for Android Verified Boot */
struct avb_root_of_trust {
	u8 verified_boot_key[32];    /* AVB public key */
	u32 device_locked;           /* Device lock state */
	u32 verified_boot_state;     /* Verified boot state */
	u8 verified_boot_hash[32];   /* Boot image hash */
	u64 timestamp;               /* Boot timestamp */
} __packed;

/* Root of Trust persistent storage name for AVB TA */
#define ROT_PERSIST_NAME "avb_root_of_trust"

/*
 * Store Root of Trust data using AVB TA persistent storage
 * This function stores the complete Root of Trust structure in one call
 */
static int avb_store_root_of_trust_persist(const struct avb_root_of_trust *rot)
{
	struct tee_open_session_arg session_arg;
	struct tee_invoke_arg invoke_arg;
	struct tee_param param[4];
	struct tee_shm *shm_name, *shm_data;
	struct udevice *tee_dev;
	int ret;

	if (!rot) {
		printf("ERROR: Invalid Root of Trust data\n");
		return -EINVAL;
	}

	/* Find TEE device */
	tee_dev = tee_find_device(NULL, NULL, NULL, NULL);
	if (!tee_dev) {
		printf("ERROR: No TEE device found\n");
		return -ENODEV;
	}

	/* Open session with AVB TA */
	memset(&session_arg, 0, sizeof(session_arg));
	const struct tee_optee_ta_uuid uuid = TA_AVB_UUID;

	tee_optee_ta_uuid_to_octets(session_arg.uuid, &uuid);
	session_arg.clnt_login = TEE_LOGIN_PUBLIC;

	ret = tee_open_session(tee_dev, &session_arg, 0, NULL);
	if (ret || session_arg.ret) {
		printf("ERROR: [ROT] Failed to open AVB TA session: ret=%d, session_ret=0x%x\n",
		       ret, session_arg.ret);
		return -EIO;
	}

	/* Allocate shared memory for object name */
	ret = tee_shm_alloc(tee_dev, strlen(ROT_PERSIST_NAME) + 1,
			    TEE_SHM_ALLOC, &shm_name);
	if (ret) {
		printf("ERROR: Failed to allocate shared memory for name\n");
		goto cleanup_session;
	}

	/* Allocate shared memory for Root of Trust data */
	ret = tee_shm_alloc(tee_dev, sizeof(struct avb_root_of_trust),
			    TEE_SHM_ALLOC, &shm_data);
	if (ret) {
		printf("ERROR: Failed to allocate shared memory for data\n");
		goto cleanup_name;
	}

	/* Copy data to shared memory */
	strcpy((char *)shm_name->addr, ROT_PERSIST_NAME);
	memcpy(shm_data->addr, rot, sizeof(struct avb_root_of_trust));

	/* Prepare invocation parameters */
	memset(&invoke_arg, 0, sizeof(invoke_arg));
	memset(param, 0, sizeof(param));

	invoke_arg.func = TA_AVB_CMD_WRITE_PERSIST_VALUE;
	invoke_arg.session = session_arg.session;

	param[0].attr = TEE_PARAM_ATTR_TYPE_MEMREF_INPUT;
	param[0].u.memref.shm = shm_name;
	param[0].u.memref.size = strlen(ROT_PERSIST_NAME);

	param[1].attr = TEE_PARAM_ATTR_TYPE_MEMREF_INPUT;
	param[1].u.memref.shm = shm_data;
	param[1].u.memref.size = sizeof(struct avb_root_of_trust);

	/* Invoke AVB TA to store Root of Trust */
	ret = tee_invoke_func(tee_dev, &invoke_arg, 4, param);
	if (ret || invoke_arg.ret) {
		printf("ERROR: Failed to store Root of Trust: 0x%x\n", invoke_arg.ret);
		ret = -EIO;
	} else {
		printf("INFO: Root of Trust stored successfully in AVB persistent storage\n");
		ret = 0;
	}

	tee_shm_free(shm_data);
cleanup_name:
	tee_shm_free(shm_name);
cleanup_session:
	tee_close_session(tee_dev, session_arg.session);
	return ret;
}

/*
 * Extract and store Root of Trust from existing AVB verification data
 * This function is called after AVB verification is complete with the verification results
 * Optimized version with enhanced error handling and device state detection
 */
int avb_store_root_of_trust_from_verification(AvbSlotVerifyData *slot_data,
					      AvbSlotVerifyResult result)
{
	struct avb_root_of_trust rot;
	int ret;

	memset(&rot, 0, sizeof(rot));

	printf("INFO: Extracting Root of Trust from AVB verification (result=%d)...\n", result);

	/* Handle case where AVB verification failed or no data available */
	if (!slot_data) {
		printf("WARN: No AVB verification data available\n");
		rot.verified_boot_state = AVB_RED;
		/* Read actual lock state from AVB lib */
		bool is_unlocked = avb_is_device_unlocked();

		rot.device_locked = !is_unlocked;
		rot.timestamp = 0; /* Will be set by bootloader timestamp */
		goto store_rot;
	}

	/* Extract Root of Trust information from AVB verification data */
	if (slot_data->vbmeta_images && slot_data->num_vbmeta_images > 0) {
		AvbVBMetaData *vbmeta = &slot_data->vbmeta_images[0];

		/* Extract AVB public key from VBMeta header */
		if (vbmeta->vbmeta_data && vbmeta->vbmeta_size > 0) {
			/* Parse VBMeta header to extract public key */
			AvbVBMetaImageHeader header;

			avb_vbmeta_image_header_to_host_byte_order
				((const AvbVBMetaImageHeader *)
				 vbmeta->vbmeta_data, &header);
			if (header.public_key_size > 0 &&
			    header.public_key_offset < vbmeta->vbmeta_size) {
				u8 *public_key_data;

				public_key_data = vbmeta->vbmeta_data +
						  header.public_key_offset;

				if (header.public_key_size <=
				    sizeof(rot.verified_boot_key)) {
					memcpy(rot.verified_boot_key,
					       public_key_data,
					       header.public_key_size);
				} else {
					/* Hash large public keys to fit in 32 bytes */
					sha256_csum_wd(public_key_data,
						       header.public_key_size,
						       rot.verified_boot_key,
						       CHUNKSZ_SHA256);
				}
			} else {
				printf("WARN: No valid public key in VBMeta header\n");
			}
		} else {
			printf("WARN: No VBMeta data available\n");
		}

		/*
		 * Calculate vbmeta digest (hash of all vbmeta images combined)
		 * This is what Android VTS expects in verified_boot_hash field
		 * per the Android Verified Boot specification.
		 */
		avb_slot_verify_data_calculate_vbmeta_digest(slot_data,
							     AVB_DIGEST_TYPE_SHA256,
							     rot.verified_boot_hash);
	} else {
		printf("WARN: No VBMeta images available\n");
	}

	/* Determine device lock state from AVB lib */
	bool is_unlocked = avb_is_device_unlocked();

	rot.device_locked = !is_unlocked;

	/* Determine verified boot state based on AVB result and device state */
	switch (result) {
	case AVB_SLOT_VERIFY_RESULT_OK:
		if (!rot.device_locked)
			rot.verified_boot_state = AVB_ORANGE;
		else
			rot.verified_boot_state = AVB_GREEN;
		break;
	case AVB_SLOT_VERIFY_RESULT_ERROR_ROLLBACK_INDEX:
		rot.verified_boot_state = AVB_YELLOW;
		break;
	case AVB_SLOT_VERIFY_RESULT_ERROR_OOM:
	case AVB_SLOT_VERIFY_RESULT_ERROR_IO:
	case AVB_SLOT_VERIFY_RESULT_ERROR_VERIFICATION:
	case AVB_SLOT_VERIFY_RESULT_ERROR_PUBLIC_KEY_REJECTED:
	default:
		rot.verified_boot_state = AVB_RED;
		break;
	}

	/* Record boot timestamp */
	rot.timestamp = get_timer(0);

store_rot:
	/* Store Root of Trust via AVB TA persistent storage */
	ret = avb_store_root_of_trust_persist(&rot);

	printf("INFO: Root of Trust extraction %s (state=%s, locked=%s, result=%d)\n",
	       ret ? "FAILED" : "COMPLETED",
	       rot.verified_boot_state == AVB_GREEN ? "GREEN" :
	       rot.verified_boot_state == AVB_YELLOW ? "YELLOW" :
	       rot.verified_boot_state == AVB_ORANGE ? "ORANGE" : "RED",
	       rot.device_locked ? "YES" : "NO",
	       result);

	return ret;
}

/*
 * Legacy function for manual Root of Trust setup (deprecated)
 * Use avb_store_root_of_trust_from_verification() instead
 */
int avb_setup_root_of_trust(void)
{
	AvbSlotVerifyData *slot_data = NULL;
	AvbSlotVerifyResult result;
	AvbOps *ops;
	struct avb_root_of_trust rot;
	static const char * const requested_partitions[] = {"boot", NULL};
	int ret;

	memset(&rot, 0, sizeof(rot));

	printf("INFO: Setting up Root of Trust...\n");

	/* Initialize AVB operations */
	ops = avb_ops_alloc(CONFIG_FASTBOOT_FLASH_MMC_DEV);
	if (!ops) {
		printf("ERROR: Failed to allocate AVB operations\n");
		return -ENOMEM;
	}

	/* Perform AVB slot verification */
	result = avb_slot_verify(ops, requested_partitions, "", /* slot */
				 AVB_SLOT_VERIFY_FLAGS_NONE,
				 AVB_HASHTREE_ERROR_MODE_RESTART_AND_INVALIDATE,
				 &slot_data);

	/* Extract Root of Trust information from AVB verification */
	if (slot_data && slot_data->vbmeta_images) {
		AvbVBMetaData *vbmeta = &slot_data->vbmeta_images[0];

		/* Extract AVB public key from VBMeta header (simplified) */
		if (vbmeta->vbmeta_data && vbmeta->vbmeta_size > 0) {
			AvbVBMetaImageHeader header;

			avb_vbmeta_image_header_to_host_byte_order
				((const AvbVBMetaImageHeader *)
				 vbmeta->vbmeta_data, &header);
			if (header.public_key_size > 0 &&
			    header.public_key_offset < vbmeta->vbmeta_size) {
				u8 *public_key_data;

				public_key_data = vbmeta->vbmeta_data +
						  header.public_key_offset;

				if (header.public_key_size <=
				    sizeof(rot.verified_boot_key)) {
					memcpy(rot.verified_boot_key,
					       public_key_data,
					       header.public_key_size);
				} else {
					sha256_csum_wd(public_key_data,
						       header.public_key_size,
						       rot.verified_boot_key,
						       CHUNKSZ_SHA256);
				}
			}
		}

		/* Determine verified boot state based on AVB result */
		switch (result) {
		case AVB_SLOT_VERIFY_RESULT_OK:
			rot.verified_boot_state = AVB_GREEN;
			printf("INFO: Boot state: GREEN (verified and secure)\n");
			break;
		case AVB_SLOT_VERIFY_RESULT_ERROR_ROLLBACK_INDEX:
			rot.verified_boot_state = AVB_YELLOW;
			printf("INFO: Boot state: YELLOW (verified with rollback error)\n");
			break;
		default:
			rot.verified_boot_state = AVB_ORANGE;
			printf("WARN: Boot state: ORANGE (not verified)\n");
			break;
		}

		/* Get device lock state - FIXED: Read from AVB lib */
		bool is_unlocked = avb_is_device_unlocked();

		rot.device_locked = !is_unlocked;
		printf("INFO: Device locked: %s (from AVB lib)\n",
		       rot.device_locked ? "Yes" : "No");

		/* Calculate vbmeta digest (hash of all vbmeta images) */
		avb_slot_verify_data_calculate_vbmeta_digest(slot_data,
							     AVB_DIGEST_TYPE_SHA256,
							     rot.verified_boot_hash);
		printf("INFO: Calculated vbmeta digest\n");

		/* Record boot timestamp */
		rot.timestamp = get_timer(0);
		printf("INFO: Boot timestamp: %llu\n", rot.timestamp);
	} else {
		printf("WARN: No VBMeta data available, using default Root of Trust\n");
		rot.verified_boot_state = AVB_RED;
		rot.device_locked = 0;
		rot.timestamp = get_timer(0);
	}

	/* Store Root of Trust via AVB TA persistent storage */
	ret = avb_store_root_of_trust_persist(&rot);

	/* Cleanup AVB data structures */
	if (slot_data)
		avb_slot_verify_data_free(slot_data);
	avb_ops_free(ops);

	printf("WARN: Using legacy Root of Trust setup - consider integrating with AVB verification\n");
	printf("INFO: Root of Trust setup %s\n",
	       ret ? "FAILED" : "COMPLETED");
	return ret;
}

/*
 * Read and verify Root of Trust data from AVB TA persistent storage
 * This function reads back what was stored and displays it for verification
 */
static int avb_read_and_verify_root_of_trust(void)
{
	struct tee_open_session_arg session_arg;
	struct tee_invoke_arg invoke_arg;
	struct tee_param param[4];
	struct tee_shm *shm_name, *shm_data;
	struct udevice *tee_dev;
	struct avb_root_of_trust rot;
	int ret;
	int i;

	printf("INFO: Reading Root of Trust from AVB TA for verification...\n");

	/* Get TEE device */
	tee_dev = tee_find_device(NULL, NULL, NULL, NULL);
	if (!tee_dev) {
		printf("ERROR: Cannot find TEE device\n");
		return -ENODEV;
	}

	/* Allocate shared memory for storage name */
	ret = tee_shm_alloc(tee_dev, strlen(ROT_PERSIST_NAME) + 1, 0, &shm_name);
	if (ret) {
		printf("ERROR: Cannot allocate TEE shared memory for name (ret=%d)\n", ret);
		return -ENOMEM;
	}
	strcpy(shm_name->addr, ROT_PERSIST_NAME);

	/* Allocate shared memory for Root of Trust data */
	ret = tee_shm_alloc(tee_dev, sizeof(rot), 0, &shm_data);
	if (ret) {
		printf("ERROR: Cannot allocate TEE shared memory for data (ret=%d)\n", ret);
		tee_shm_free(shm_name);
		return -ENOMEM;
	}

	/* Open session with AVB TA */
	memset(&session_arg, 0, sizeof(session_arg));
	const struct tee_optee_ta_uuid uuid = TA_AVB_UUID;

	tee_optee_ta_uuid_to_octets(session_arg.uuid, &uuid);
	session_arg.clnt_login = TEE_LOGIN_REE_KERNEL;

	ret = tee_open_session(tee_dev, &session_arg, 0, NULL);
	if (ret) {
		printf("ERROR: Cannot open AVB TA session (ret=%d)\n", ret);
		goto cleanup_shm;
	}

	/* Setup parameters for reading persistent value */
	memset(param, 0, sizeof(param));
	param[0].attr = TEE_PARAM_ATTR_TYPE_MEMREF_INPUT;
	param[0].u.memref.shm = shm_name;
	param[0].u.memref.size = strlen(ROT_PERSIST_NAME);
	param[1].attr = TEE_PARAM_ATTR_TYPE_MEMREF_OUTPUT;
	param[1].u.memref.shm = shm_data;
	param[1].u.memref.size = sizeof(rot);

	/* Invoke read persistent value command */
	memset(&invoke_arg, 0, sizeof(invoke_arg));
	invoke_arg.func = TA_AVB_CMD_READ_PERSIST_VALUE;
	invoke_arg.session = session_arg.session;

	ret = tee_invoke_func(tee_dev, &invoke_arg, 4, param);
	if (ret || invoke_arg.ret) {
		printf("WARN: Failed to read Root of Trust from AVB TA (ret=%d, ta_ret=%d)\n",
		       ret, invoke_arg.ret);
		if (invoke_arg.ret == TEE_ERROR_ITEM_NOT_FOUND)
			printf("INFO: Root of Trust data was never stored in AVB TA\n");
		else if (invoke_arg.ret == TEE_ERROR_SECURITY)
			printf("INFO: Security error - AVB TA has no Root of Trust data\n");
		ret = -ENOENT;
		goto cleanup_session;
	}

	/* Copy data from shared memory */
	memcpy(&rot, shm_data->addr, sizeof(rot));

	printf("SUCCESS: Root of Trust data read from AVB TA:\n");
	printf("  Device Locked: %s (%u)\n",
	       rot.device_locked ? "YES" : "NO", rot.device_locked);
	printf("  Verified Boot State: %u ", rot.verified_boot_state);
	switch (rot.verified_boot_state) {
	case 0:
		printf("(GREEN - Verified)\n");
		break;
	case 1:
		printf("(YELLOW - Custom key)\n");
		break;
	case 2:
		printf("(ORANGE - Unverified)\n");
		break;
	case 3:
		printf("(RED - Failed)\n");
		break;
	default:
		printf("(UNKNOWN)\n");
		break;
	}
	printf("  Boot Timestamp: %llu\n", rot.timestamp);

	printf("  Verified Boot Key: ");
	for (i = 0; i < 32; i++) {
		printf("%02x", rot.verified_boot_key[i]);
		if (i == 15)
			printf("\n");
	}
	printf("\n");

	printf("  Verified Boot Hash: ");
	for (i = 0; i < 32; i++) {
		printf("%02x", rot.verified_boot_hash[i]);
		if (i == 15)
			printf("\n");
	}
	printf("\n");

	ret = 0;

cleanup_session:
	tee_close_session(tee_dev, session_arg.session);
cleanup_shm:
	tee_shm_free(shm_data);
	tee_shm_free(shm_name);

	return ret;
}

/*
 * Debug function to display Root of Trust information
 * This can be called from U-Boot command line for debugging
 */
void avb_show_root_of_trust_info(void)
{
	printf("Root of Trust Implementation Status:\n");
	printf("  - AVB integration: Enabled\n");
	printf("  - TEE storage: Enabled\n");
	printf("  - Persistent name: %s\n", ROT_PERSIST_NAME);
	printf("  - Structure size: %zu bytes\n",
	       sizeof(struct avb_root_of_trust));

	/* Try to read and display actual stored data */
	printf("\nAttempting to read stored Root of Trust data:\n");
	avb_read_and_verify_root_of_trust();
}
