/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Root of Trust header for Android Verified Boot
 * Defines interfaces for collecting and storing Root of Trust information
 *
 * Copyright (C) 2025 Baylibre
 */

#ifndef _AVB_ROOT_OF_TRUST_H_
#define _AVB_ROOT_OF_TRUST_H_

#include <avb_verify.h>

/**
 * Store Root of Trust from existing AVB verification data
 *
 * This function should be called immediately after AVB verification
 * with the verification results to store Root of Trust information
 * for later retrieval by the Android KeyMint service.
 *
 * @param slot_data AVB verification data containing VBMeta and boot images
 * @param result AVB verification result indicating boot state
 * @return 0 on success, negative error code on failure
 */
int avb_store_root_of_trust_from_verification(AvbSlotVerifyData *slot_data,
					      AvbSlotVerifyResult result);

/**
 * Legacy Root of Trust setup function (deprecated)
 *
 * This function performs its own AVB verification. Use
 * avb_store_root_of_trust_from_verification() instead for better
 * integration with existing AVB verification flows.
 *
 * @return 0 on success, negative error code on failure
 */
int avb_setup_root_of_trust(void);

/**
 * Display Root of Trust implementation information
 *
 * Debug function to show current Root of Trust configuration
 * and status. Useful for debugging and verification.
 */
void avb_show_root_of_trust_info(void);

/* Environment variable for device lock state */
#define AVB_DEVICE_LOCKED_ENV "avb_device_locked"

#endif /* _AVB_ROOT_OF_TRUST_H_ */
