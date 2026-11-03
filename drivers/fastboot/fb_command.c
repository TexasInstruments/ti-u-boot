// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (C) 2016 The Android Open Source Project
 */

#include <command.h>
#include <console.h>
#include <cli.h>
#include <env.h>
#include <fastboot.h>
#include <fastboot-internal.h>
#include <fb_mmc.h>
#include <fb_nand.h>
#include <fb_spi_flash.h>
#include <part.h>
#include <stdlib.h>
#include <vsprintf.h>
#include <stdbool.h>
#include <linux/printk.h>
#if (IS_ENABLED(CONFIG_AVB_VERIFY))
#include <avb_verify.h>
#endif

	/**
 * image_size - final fastboot image size
 */
	static u32 image_size;

/**
 * fastboot_bytes_received - number of bytes received in the current download
 */
static u32 fastboot_bytes_received;

/**
 * fastboot_bytes_expected - number of bytes expected in the current download
 */
static u32 fastboot_bytes_expected;

static void okay(char *, char *);
static void getvar(char *, char *);
static void download(char *, char *);
static void flash(char *, char *);
static void erase(char *, char *);
static void reboot_bootloader(char *, char *);
static void reboot_fastbootd(char *, char *);
static void reboot_recovery(char *, char *);
static void oem_format(char *, char *);
static void oem_partconf(char *, char *);
static void oem_bootbus(char *, char *);
static void oem_console(char *, char *);
static void oem_board(char *, char *);
static void flashing_lock(char *, char *);
static void flashing_unlock(char *, char *);
static void flashing_lock_critical(char *, char *);
static void flashing_unlock_critical(char *, char *);
static void flashing_get_unlock_ability(char *, char *);
static bool is_device_locked(void);
static bool is_critical_partition(const char *partition);
static bool is_device_critical_locked(void);
static void run_ucmd(char *, char *);
static void run_acmd(char *, char *);

static const struct {
	const char *command;
	void (*dispatch)(char *cmd_parameter, char *response);
} commands[FASTBOOT_COMMAND_COUNT] = {
	[FASTBOOT_COMMAND_GETVAR] = {
		.command = "getvar",
		.dispatch = getvar
	},
	[FASTBOOT_COMMAND_DOWNLOAD] = {
		.command = "download",
		.dispatch = download
	},
	[FASTBOOT_COMMAND_FLASH] =  {
		.command = "flash",
		.dispatch = CONFIG_IS_ENABLED(FASTBOOT_FLASH, (flash), (NULL))
	},
	[FASTBOOT_COMMAND_ERASE] =  {
		.command = "erase",
		.dispatch = CONFIG_IS_ENABLED(FASTBOOT_FLASH, (erase), (NULL))
	},
	[FASTBOOT_COMMAND_BOOT] =  {
		.command = "boot",
		.dispatch = okay
	},
	[FASTBOOT_COMMAND_CONTINUE] =  {
		.command = "continue",
		.dispatch = okay
	},
	[FASTBOOT_COMMAND_REBOOT] =  {
		.command = "reboot",
		.dispatch = okay
	},
	[FASTBOOT_COMMAND_REBOOT_BOOTLOADER] =  {
		.command = "reboot-bootloader",
		.dispatch = reboot_bootloader
	},
	[FASTBOOT_COMMAND_REBOOT_FASTBOOTD] =  {
		.command = "reboot-fastboot",
		.dispatch = reboot_fastbootd
	},
	[FASTBOOT_COMMAND_REBOOT_RECOVERY] =  {
		.command = "reboot-recovery",
		.dispatch = reboot_recovery
	},
	[FASTBOOT_COMMAND_SET_ACTIVE] =  {
		.command = "set_active",
		.dispatch = okay
	},
	[FASTBOOT_COMMAND_OEM_FORMAT] = {
		.command = "oem format",
		.dispatch = CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_FORMAT, (oem_format), (NULL))
	},
	[FASTBOOT_COMMAND_OEM_PARTCONF] = {
		.command = "oem partconf",
		.dispatch = CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_PARTCONF, (oem_partconf), (NULL))
	},
	[FASTBOOT_COMMAND_OEM_BOOTBUS] = {
		.command = "oem bootbus",
		.dispatch = CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_BOOTBUS, (oem_bootbus), (NULL))
	},
	[FASTBOOT_COMMAND_OEM_RUN] = {
		.command = "oem run",
		.dispatch = CONFIG_IS_ENABLED(FASTBOOT_OEM_RUN, (run_ucmd), (NULL))
	},
	[FASTBOOT_COMMAND_OEM_CONSOLE] = {
		.command = "oem console",
		.dispatch = CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_CONSOLE, (oem_console), (NULL))
	},
	[FASTBOOT_COMMAND_OEM_BOARD] = {
		.command = "oem board",
		.dispatch = CONFIG_IS_ENABLED(FASTBOOT_OEM_BOARD, (oem_board), (NULL))
	},
	[FASTBOOT_COMMAND_FLASHING_LOCK] = {
		.command = "flashing lock",
		.dispatch = CONFIG_IS_ENABLED(FASTBOOT_LOCKING, (flashing_lock), (NULL))
	},
	[FASTBOOT_COMMAND_FLASHING_UNLOCK] = {
		.command = "flashing unlock",
		.dispatch = CONFIG_IS_ENABLED(FASTBOOT_LOCKING, (flashing_unlock), (NULL))
	},
	[FASTBOOT_COMMAND_FLASHING_LOCK_CRITICAL] = {
		.command = "flashing lock_critical",
		.dispatch = CONFIG_IS_ENABLED(FASTBOOT_LOCKING, (flashing_lock_critical), (NULL))
	},
	[FASTBOOT_COMMAND_FLASHING_UNLOCK_CRITICAL] = {
		.command = "flashing unlock_critical",
		.dispatch = CONFIG_IS_ENABLED(FASTBOOT_LOCKING, (flashing_unlock_critical), (NULL))
	},
	[FASTBOOT_COMMAND_FLASHING_GET_UNLOCK_ABILITY] = {
		.command = "flashing get_unlock_ability",
		.dispatch = CONFIG_IS_ENABLED(FASTBOOT_LOCKING,
					      (flashing_get_unlock_ability), (NULL))
	},
	[FASTBOOT_COMMAND_UCMD] = {
		.command = "UCmd",
		.dispatch = CONFIG_IS_ENABLED(FASTBOOT_UUU_SUPPORT, (run_ucmd), (NULL))
	},
	[FASTBOOT_COMMAND_ACMD] = {
		.command = "ACmd",
		.dispatch = CONFIG_IS_ENABLED(FASTBOOT_UUU_SUPPORT, (run_acmd), (NULL))
	},
};

/**
 * fastboot_handle_command - Handle fastboot command
 *
 * @cmd_string: Pointer to command string
 * @response: Pointer to fastboot response buffer
 *
 * Return: Executed command, or -1 if not recognized
 */
int fastboot_handle_command(char *cmd_string, char *response)
{
	int i;
	char *cmd_parameter;

	cmd_parameter = cmd_string;
	strsep(&cmd_parameter, ":");

	for (i = 0; i < FASTBOOT_COMMAND_COUNT; i++) {
		if (!strcmp(commands[i].command, cmd_string)) {
			if (commands[i].dispatch) {
				commands[i].dispatch(cmd_parameter,
							response);
				return i;
			} else {
				pr_err("command %s not supported.\n", cmd_string);
				fastboot_fail("Unsupported command", response);
				return -1;
			}
		}
	}

	pr_err("command %s not recognized.\n", cmd_string);
	fastboot_fail("unrecognized command", response);
	return -1;
}

void fastboot_multiresponse(int cmd, char *response)
{
	switch (cmd) {
	case FASTBOOT_COMMAND_GETVAR:
		fastboot_getvar_all(response);
		break;
	case FASTBOOT_COMMAND_OEM_CONSOLE:
		if (CONFIG_IS_ENABLED(FASTBOOT_CMD_OEM_CONSOLE)) {
			char buf[FASTBOOT_RESPONSE_LEN] = { 0 };

			if (console_record_isempty()) {
				console_record_reset();
				fastboot_okay(NULL, response);
			} else {
				int ret = console_record_readline(buf, sizeof(buf) - 5);

				if (ret < 0)
					fastboot_fail("Error reading console", response);
				else
					fastboot_response("INFO", response, "%s", buf);
			}
			break;
		}
		fallthrough;
	default:
		fastboot_fail("Unknown multiresponse command", response);
		break;
	}
}

/**
 * okay() - Send bare OKAY response
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 *
 * Send a bare OKAY fastboot response. This is used where the command is
 * valid, but all the work is done after the response has been sent (e.g.
 * boot, reboot etc.)
 */
static void okay(char *cmd_parameter, char *response)
{
	fastboot_okay(NULL, response);
}

/**
 * getvar() - Read a config/version variable
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void getvar(char *cmd_parameter, char *response)
{
	fastboot_getvar(cmd_parameter, response);
}

/**
 * fastboot_download() - Start a download transfer from the client
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void download(char *cmd_parameter, char *response)
{
	char *tmp;

	if (!cmd_parameter) {
		fastboot_fail("Expected command parameter", response);
		return;
	}
	fastboot_bytes_received = 0;
	fastboot_bytes_expected = hextoul(cmd_parameter, &tmp);
	if (fastboot_bytes_expected == 0) {
		fastboot_fail("Expected nonzero image size", response);
		return;
	}
	/*
	 * Nothing to download yet. Response is of the form:
	 * [DATA|FAIL]$cmd_parameter
	 *
	 * where cmd_parameter is an 8 digit hexadecimal number
	 */
	if (fastboot_bytes_expected > fastboot_buf_size) {
		fastboot_fail(cmd_parameter, response);
	} else {
		printf("Starting download of %d bytes\n",
		       fastboot_bytes_expected);
		fastboot_response("DATA", response, "%s", cmd_parameter);
	}
}

/**
 * fastboot_data_remaining() - return bytes remaining in current transfer
 *
 * Return: Number of bytes left in the current download
 */
u32 fastboot_data_remaining(void)
{
	return fastboot_bytes_expected - fastboot_bytes_received;
}

/**
 * fastboot_data_download() - Copy image data to fastboot_buf_addr.
 *
 * @fastboot_data: Pointer to received fastboot data
 * @fastboot_data_len: Length of received fastboot data
 * @response: Pointer to fastboot response buffer
 *
 * Copies image data from fastboot_data to fastboot_buf_addr. Writes to
 * response. fastboot_bytes_received is updated to indicate the number
 * of bytes that have been transferred.
 *
 * On completion sets image_size and ${filesize} to the total size of the
 * downloaded image.
 */
void fastboot_data_download(const void *fastboot_data,
			    unsigned int fastboot_data_len,
			    char *response)
{
#define BYTES_PER_DOT	0x20000
	u32 pre_dot_num, now_dot_num;

	if (fastboot_data_len == 0 ||
	    (fastboot_bytes_received + fastboot_data_len) >
	    fastboot_bytes_expected) {
		fastboot_fail("Received invalid data length",
			      response);
		return;
	}
	/* Download data to fastboot_buf_addr */
	memcpy(fastboot_buf_addr + fastboot_bytes_received,
	       fastboot_data, fastboot_data_len);

	pre_dot_num = fastboot_bytes_received / BYTES_PER_DOT;
	fastboot_bytes_received += fastboot_data_len;
	now_dot_num = fastboot_bytes_received / BYTES_PER_DOT;

	if (pre_dot_num != now_dot_num) {
		putc('.');
		if (!(now_dot_num % 74))
			putc('\n');
	}
	*response = '\0';
}

/**
 * fastboot_data_complete() - Mark current transfer complete
 *
 * @response: Pointer to fastboot response buffer
 *
 * Set image_size and ${filesize} to the total size of the downloaded image.
 */
void fastboot_data_complete(char *response)
{
	/* Download complete. Respond with "OKAY" */
	fastboot_okay(NULL, response);
	printf("\ndownloading of %d bytes finished\n", fastboot_bytes_received);
	image_size = fastboot_bytes_received;
	env_set_hex("filesize", image_size);
	fastboot_bytes_expected = 0;
	fastboot_bytes_received = 0;
}

/**
 * flash() - write the downloaded image to the indicated partition.
 *
 * @cmd_parameter: Pointer to partition name
 * @response: Pointer to fastboot response buffer
 *
 * Writes the previously downloaded image to the partition indicated by
 * cmd_parameter. Writes to response.
 */
static void __maybe_unused flash(char *cmd_parameter, char *response)
{
	if (IS_ENABLED(CONFIG_FASTBOOT_LOCKING)) {
		/* Check if device is locked before allowing flash */
		if (is_device_locked()) {
			fastboot_fail("Device is locked. Cannot flash partitions", response);
			return;
		}

		/* Check if partition is critical and critical lock is enabled */
		if (is_critical_partition(cmd_parameter) && is_device_critical_locked()) {
			fastboot_fail("Critical partitions are locked. Cannot flash", response);
			return;
		}
	}

	if (IS_ENABLED(CONFIG_FASTBOOT_FLASH_MMC))
		fastboot_mmc_flash_write(cmd_parameter, fastboot_buf_addr,
					 image_size, response);

	if (IS_ENABLED(CONFIG_FASTBOOT_FLASH_NAND))
		fastboot_nand_flash_write(cmd_parameter, fastboot_buf_addr,
					  image_size, response);

	if (IS_ENABLED(CONFIG_FASTBOOT_FLASH_SPI))
		fastboot_spi_flash_write(cmd_parameter, fastboot_buf_addr,
					 image_size, response);
}

/**
 * erase() - erase the indicated partition.
 *
 * @cmd_parameter: Pointer to partition name
 * @response: Pointer to fastboot response buffer
 *
 * Erases the partition indicated by cmd_parameter (clear to 0x00s). Writes
 * to response.
 */
static void __maybe_unused erase(char *cmd_parameter, char *response)
{
	if (IS_ENABLED(CONFIG_FASTBOOT_LOCKING)) {
		/* Check if device is locked before allowing erase */
		if (is_device_locked()) {
			fastboot_fail("Device is locked. Cannot erase partitions",
				      response);
			return;
		}

		/* Check if partition is critical and critical lock is enabled */
		if (is_critical_partition(cmd_parameter) &&
		    is_device_critical_locked()) {
			fastboot_fail("Critical partitions are locked. Cannot erase",
				      response);
			return;
		}
	}

	if (IS_ENABLED(CONFIG_FASTBOOT_FLASH_MMC))
		fastboot_mmc_erase(cmd_parameter, response);

	if (IS_ENABLED(CONFIG_FASTBOOT_FLASH_NAND))
		fastboot_nand_erase(cmd_parameter, response);

	if (IS_ENABLED(CONFIG_FASTBOOT_FLASH_SPI))
		fastboot_spi_flash_erase(cmd_parameter, response);
}

/**
 * run_ucmd() - Execute the UCmd command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void __maybe_unused run_ucmd(char *cmd_parameter, char *response)
{
	if (!cmd_parameter) {
		pr_err("missing slot suffix\n");
		fastboot_fail("missing command", response);
		return;
	}

	if (run_command(cmd_parameter, 0))
		fastboot_fail("", response);
	else
		fastboot_okay(NULL, response);
}

static char g_a_cmd_buff[64];

void fastboot_acmd_complete(void)
{
	run_command(g_a_cmd_buff, 0);
}

/**
 * run_acmd() - Execute the ACmd command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void __maybe_unused run_acmd(char *cmd_parameter, char *response)
{
	if (!cmd_parameter) {
		pr_err("missing slot suffix\n");
		fastboot_fail("missing command", response);
		return;
	}

	if (strlen(cmd_parameter) >= sizeof(g_a_cmd_buff)) {
		pr_err("too long command\n");
		fastboot_fail("too long command", response);
		return;
	}

	strcpy(g_a_cmd_buff, cmd_parameter);
	fastboot_okay(NULL, response);
}

/**
 * reboot_bootloader() - Sets reboot bootloader flag.
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void reboot_bootloader(char *cmd_parameter, char *response)
{
	if (fastboot_set_reboot_flag(FASTBOOT_REBOOT_REASON_BOOTLOADER))
		fastboot_fail("Cannot set reboot flag", response);
	else
		fastboot_okay(NULL, response);
}

/**
 * reboot_fastbootd() - Sets reboot fastboot flag.
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void reboot_fastbootd(char *cmd_parameter, char *response)
{
	if (fastboot_set_reboot_flag(FASTBOOT_REBOOT_REASON_FASTBOOTD))
		fastboot_fail("Cannot set fastboot flag", response);
	else
		fastboot_okay(NULL, response);
}

/**
 * reboot_recovery() - Sets reboot recovery flag.
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void reboot_recovery(char *cmd_parameter, char *response)
{
	if (fastboot_set_reboot_flag(FASTBOOT_REBOOT_REASON_RECOVERY))
		fastboot_fail("Cannot set recovery flag", response);
	else
		fastboot_okay(NULL, response);
}

/**
 * oem_format() - Execute the OEM format command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void __maybe_unused oem_format(char *cmd_parameter, char *response)
{
	char cmdbuf[32];
	const int mmc_dev = config_opt_enabled(CONFIG_FASTBOOT_FLASH_MMC,
					       CONFIG_FASTBOOT_FLASH_MMC_DEV, -1);

	if (!env_get("partitions")) {
		fastboot_fail("partitions not set", response);
	} else {
		sprintf(cmdbuf, "gpt write mmc %x $partitions", mmc_dev);
		if (run_command(cmdbuf, 0))
			fastboot_fail("", response);
		else
			fastboot_okay(NULL, response);
	}
}

/**
 * oem_partconf() - Execute the OEM partconf command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void __maybe_unused oem_partconf(char *cmd_parameter, char *response)
{
	char cmdbuf[32];
	const int mmc_dev = config_opt_enabled(CONFIG_FASTBOOT_FLASH_MMC,
					       CONFIG_FASTBOOT_FLASH_MMC_DEV, -1);

	if (!cmd_parameter) {
		fastboot_fail("Expected command parameter", response);
		return;
	}

	/* execute 'mmc partconfg' command with cmd_parameter arguments*/
	snprintf(cmdbuf, sizeof(cmdbuf), "mmc partconf %x %s 0", mmc_dev, cmd_parameter);
	printf("Execute: %s\n", cmdbuf);
	if (run_command(cmdbuf, 0))
		fastboot_fail("Cannot set oem partconf", response);
	else
		fastboot_okay(NULL, response);
}

/**
 * oem_bootbus() - Execute the OEM bootbus command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void __maybe_unused oem_bootbus(char *cmd_parameter, char *response)
{
	char cmdbuf[32];
	const int mmc_dev = config_opt_enabled(CONFIG_FASTBOOT_FLASH_MMC,
					       CONFIG_FASTBOOT_FLASH_MMC_DEV, -1);

	if (!cmd_parameter) {
		fastboot_fail("Expected command parameter", response);
		return;
	}

	/* execute 'mmc bootbus' command with cmd_parameter arguments*/
	snprintf(cmdbuf, sizeof(cmdbuf), "mmc bootbus %x %s", mmc_dev, cmd_parameter);
	printf("Execute: %s\n", cmdbuf);
	if (run_command(cmdbuf, 0))
		fastboot_fail("Cannot set oem bootbus", response);
	else
		fastboot_okay(NULL, response);
}

/**
 * oem_console() - Execute the OEM console command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void __maybe_unused oem_console(char *cmd_parameter, char *response)
{
	if (cmd_parameter)
		console_in_puts(cmd_parameter);

	if (console_record_isempty())
		fastboot_fail("Empty console", response);
	else
		fastboot_response(FASTBOOT_MULTIRESPONSE_START, response, NULL);
}

/**
 * fastboot_oem_board() - Execute the OEM board command. This is default
 * weak implementation, which may be overwritten in board/ files.
 *
 * @cmd_parameter: Pointer to command parameter
 * @data: Pointer to fastboot input buffer
 * @size: Size of the fastboot input buffer
 * @response: Pointer to fastboot response buffer
 */
void __weak fastboot_oem_board(char *cmd_parameter, void *data, u32 size, char *response)
{
	fastboot_fail("oem board function not defined", response);
}

/**
 * oem_board() - Execute the OEM board command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void __maybe_unused oem_board(char *cmd_parameter, char *response)
{
	fastboot_oem_board(cmd_parameter, (void *)fastboot_buf_addr, image_size, response);
}

#if (IS_ENABLED(CONFIG_FASTBOOT_LOCKING))
/**
 * is_device_locked() - Check if device is locked
 *
 * Returns: true if device is locked, false if unlocked
 */
static bool is_device_locked(void)
{
	if (IS_ENABLED(CONFIG_AVB_VERIFY) && IS_ENABLED(CONFIG_OPTEE_TA_AVB)) {
		/* Use AVB helper function when all AVB locking components are available */
		return !avb_is_device_unlocked();
	} else {
		/* Use environment variable */
		char *device_unlocked = env_get("device_unlocked");

		return (!device_unlocked || strcmp(device_unlocked, "1") != 0);
	}
}

/**
 * is_critical_partition() - Check if partition is critical
 *
 * @partition: Partition name to check
 *
 * Return: true if partition is critical, false otherwise
 */
static bool is_critical_partition(const char *partition)
{
	static const char * const critical_partitions[] = {
		"bootloader",
		"radio",
		"sbl1",
		"sbl2",
		"sbl3",
		"tz",
		"rpm",
		"aboot",
		"xbl",
		"xbl_config",
		"tiboot3",
		"tispl",
		"u-boot",
		NULL
	};
	int i;

	for (i = 0; critical_partitions[i]; i++)
		if (strcmp(partition, critical_partitions[i]) == 0)
			return true;

	return false;
}

/**
 * is_device_critical_locked() - Check if device critical partitions are locked
 *
 * Return: true if critical partitions are locked, false if unlocked
 */
static bool is_device_critical_locked(void)
{
	if (IS_ENABLED(CONFIG_AVB_VERIFY)) {
		/* Use AVB helper function when AVB is available */
		return !avb_is_device_critical_unlocked();
	} else {
		/* Use environment variable */
		char *critical_unlocked = env_get("critical_unlocked");

		return (!critical_unlocked || strcmp(critical_unlocked, "1") != 0);
	}
}

/**
 * flashing_lock() - Execute the flashing lock command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void flashing_lock(char *cmd_parameter, char *response)
{
	if (IS_ENABLED(CONFIG_AVB_VERIFY) && IS_ENABLED(CONFIG_OPTEE_TA_AVB)) {
		AvbOps *avb_ops;
		AvbIOResult result;

		/* Use AVB infrastructure when all components are available */
		avb_ops = avb_ops_alloc(0);
		if (avb_ops) {
			result = write_is_device_unlocked(avb_ops, false);
			avb_ops_free(avb_ops);
			if (result == AVB_IO_RESULT_OK)
				fastboot_okay(NULL, response);
			else
				fastboot_fail("Failed to lock device", response);
			return;
		}
		fastboot_fail("AVB not available", response);
	} else {
		/* Use environment variable when AVB not available */
		env_set("device_unlocked", "0");
		env_save();
		fastboot_okay(NULL, response);
	}
}

/**
 * flashing_unlock() - Execute the flashing unlock command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void flashing_unlock(char *cmd_parameter, char *response)
{
	char *unlock_ability;
	char *confirm;
	char *auto_confirm;

	/* Check unlock_ability from environment */
	unlock_ability = env_get("fastboot.unlock_ability");
	if (!unlock_ability || strcmp(unlock_ability, "1") != 0) {
		fastboot_fail("Unlock not allowed", response);
		return;
	}

	/* If already unlocked, nothing to do */
	if (!is_device_locked()) {
		printf("Device is already unlocked\n");
		fastboot_okay(NULL, response);
		return;
	}

	/* Warning message */
	printf("\n");
	printf("WARNING: Unlocking the bootloader will void your warranty\n");
	printf("and may compromise the security of your device.\n");
	printf("This will also erase all user data.\n");
	printf("\n");
	printf("Type 'yes' to confirm unlock: ");

	/* Get user confirmation */
	confirm = malloc(16);
	if (!confirm) {
		fastboot_fail("Memory allocation failed", response);
		return;
	}

	/* For automated testing, check environment variable */
	auto_confirm = env_get("fastboot.unlock_confirm");
	if (auto_confirm && !strcmp(auto_confirm, "yes")) {
		strcpy(confirm, "yes");
		printf("yes (auto-confirmed)\n");
	} else {
		/* Read user input */
		if (!cli_readline_into_buffer("Confirm: ", confirm, 16)) {
			free(confirm);
			fastboot_fail("Unlock cancelled", response);
			return;
		}
	}

	if (strcmp(confirm, "yes") != 0) {
		free(confirm);
		printf("Unlock cancelled\n");
		fastboot_fail("Unlock not confirmed", response);
		return;
	}
	free(confirm);

	if (IS_ENABLED(CONFIG_AVB_VERIFY) && IS_ENABLED(CONFIG_OPTEE_TA_AVB)) {
		AvbOps *avb_ops;
		AvbIOResult result;

		/* Use AVB infrastructure when all components are available */
		avb_ops = avb_ops_alloc(0);
		if (avb_ops) {
			result = write_is_device_unlocked(avb_ops, true);
			avb_ops_free(avb_ops);
			if (result == AVB_IO_RESULT_OK) {
				printf("WARNING: Factory reset recommended\n");
				fastboot_okay(NULL, response);
			} else {
				fastboot_fail("Failed to unlock device",
					      response);
			}
			return;
		}
		fastboot_fail("AVB not available", response);
	} else {
		/* Use environment variable when AVB not available */
		env_set("device_unlocked", "1");
		env_save();
		printf("WARNING: Factory reset recommended\n");
		fastboot_okay(NULL, response);
	}
}

/**
 * flashing_lock_critical() - Execute the flashing lock_critical command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void flashing_lock_critical(char *cmd_parameter, char *response)
{
	if (IS_ENABLED(CONFIG_AVB_VERIFY)) {
		AvbOps *avb_ops;
		AvbIOResult result;

		/* Use AVB infrastructure when available */
		avb_ops = avb_ops_alloc(0);
		if (avb_ops) {
			result = write_is_device_critical_unlocked(avb_ops, false);
			avb_ops_free(avb_ops);
			if (result == AVB_IO_RESULT_OK)
				fastboot_okay(NULL, response);
			else
				fastboot_fail("Failed to lock critical partitions", response);
			return;
		}
		fastboot_fail("AVB not available", response);
	} else {
		/* Use environment variable when AVB not available */
		env_set("critical_unlocked", "0");
		env_save();
		fastboot_okay(NULL, response);
	}
}

/**
 * flashing_unlock_critical() - Execute the flashing unlock_critical command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void flashing_unlock_critical(char *cmd_parameter, char *response)
{
	char *unlock_ability;
	char *confirm;
	char *auto_confirm;

	/* Check unlock_ability from environment */
	unlock_ability = env_get("fastboot.unlock_ability");
	if (!unlock_ability || strcmp(unlock_ability, "1") != 0) {
		fastboot_fail("Unlock not allowed", response);
		return;
	}

	/* If already unlocked, nothing to do */
	if (!is_device_critical_locked()) {
		printf("Critical partitions are already unlocked\n");
		fastboot_okay(NULL, response);
		return;
	}

	/* Critical warning message */
	printf("\n");
	printf("CRITICAL WARNING: Unlocking critical partitions will\n");
	printf("allow modification of bootloader and radio firmware.\n");
	printf("This may PERMANENTLY DAMAGE your device and void warranty.\n");
	printf("This operation is intended for advanced development only.\n");
	printf("\n");
	printf("Type 'yes' to confirm critical unlock: ");

	/* Get user confirmation */
	confirm = malloc(16);
	if (!confirm) {
		fastboot_fail("Memory allocation failed", response);
		return;
	}

	/* For automated testing, check environment variable */
	auto_confirm = env_get("fastboot.unlock_critical_confirm");
	if (auto_confirm && !strcmp(auto_confirm, "yes")) {
		strcpy(confirm, "yes");
		printf("yes (auto-confirmed)\n");
	} else {
		/* Read user input */
		if (!cli_readline_into_buffer("Confirm: ", confirm, 16)) {
			free(confirm);
			fastboot_fail("Critical unlock cancelled", response);
			return;
		}
	}

	if (strcmp(confirm, "yes") != 0) {
		free(confirm);
		printf("Critical unlock cancelled\n");
		fastboot_fail("Critical unlock not confirmed", response);
		return;
	}
	free(confirm);

	if (IS_ENABLED(CONFIG_AVB_VERIFY)) {
		AvbOps *avb_ops;
		AvbIOResult result;

		/* Use AVB infrastructure when available */
		avb_ops = avb_ops_alloc(0);
		if (avb_ops) {
			result = write_is_device_critical_unlocked(avb_ops, true);
			avb_ops_free(avb_ops);
			if (result == AVB_IO_RESULT_OK) {
				printf("CRITICAL WARNING: Device may be damaged\n");
				fastboot_okay(NULL, response);
			} else {
				fastboot_fail("Failed to unlock critical", response);
			}
			return;
		}
		fastboot_fail("AVB not available", response);
	} else {
		/* Use environment variable when AVB not available */
		env_set("critical_unlocked", "1");
		env_save();
		printf("CRITICAL WARNING: Device may be permanently damaged\n");
		fastboot_okay(NULL, response);
	}
}

/**
 * flashing_get_unlock_ability() - Execute the flashing get_unlock_ability command
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 */
static void flashing_get_unlock_ability(char *cmd_parameter, char *response)
{
	char *unlock_ability;

	/* Check fastboot unlock ability from environment */
	/* This property is set by Android HAL OemLock */
	unlock_ability = env_get("fastboot.unlock_ability");
	if (unlock_ability && !strcmp(unlock_ability, "1"))
		fastboot_okay("1", response);
	else
		fastboot_okay("0", response);
}

#else /* !CONFIG_IS_ENABLED(FASTBOOT_LOCKING) */

/* Stub implementations when locking is disabled */
static bool __maybe_unused is_device_locked(void)
{
	return false;
}

static bool __maybe_unused is_critical_partition(const char *partition)
{
	return false;
}

static bool __maybe_unused is_device_critical_locked(void)
{
	return false;
}

static void __maybe_unused flashing_lock(char *cmd_parameter, char *response)
{
	fastboot_fail("Locking not supported", response);
}

static void __maybe_unused flashing_unlock(char *cmd_parameter, char *response)
{
	fastboot_fail("Unlocking not supported", response);
}

static void __maybe_unused flashing_lock_critical(char *cmd_parameter,
						  char *response)
{
	fastboot_fail("Critical locking not supported", response);
}

static void __maybe_unused flashing_unlock_critical(char *cmd_parameter,
						    char *response)
{
	fastboot_fail("Critical unlocking not supported", response);
}

static void __maybe_unused flashing_get_unlock_ability(char *cmd_parameter,
						       char *response)
{
	fastboot_okay("0", response);
}

#endif /* CONFIG_IS_ENABLED(FASTBOOT_LOCKING) */
