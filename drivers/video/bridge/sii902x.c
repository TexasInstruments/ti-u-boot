// SPDX-License-Identifier: GPL-2.0+
/*
 * SII902x RGB/HDMI bridge driver
 *
 * Copyright (C) 2026 Texas Instruments Incorporated - https://www.ti.com/
 *
 * Simplified u-boot port of the Linux kernel SII9022A driver,
 * based on drivers/gpu/drm/bridge/sii902x.c, which is
 *
 * Copyright (C) 2018 Renesas Electronics
 *
 * Copyright (C) 2016 Atmel
 *                    Bo Shen <voice.shen@atmel.com>
 *
 * Authors:           Bo Shen <voice.shen@atmel.com>
 *                    Boris Brezillon <boris.brezillon@free-electrons.com>
 *                    Wu, Songjun <Songjun.Wu@atmel.com>
 *
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * DDC bus acquisition and EDID reading are implemented, but not all features of the
 * Linux driver are supported. This driver is intended for use in U-Boot to
 * initialize the SII902x bridge and configure it for HDMI output.
 * The steps to acquire DDC bus and read EDID can be found in the datasheet.
 */

#include <dm.h>
#include <errno.h>
#include <edid.h>
#include <fdtdec.h>
#include <i2c.h>
#include <log.h>
#include <panel.h>
#include <video_bridge.h>
#include <linux/delay.h>
#include <time.h>
#include <dm/device_compat.h>
#include <dm/ofnode.h>
#include <linux/kernel.h>

#define SII902X_TPI_VIDEO_DATA			0x00

#define SII902X_TPI_PIXEL_REPETITION		0x08
#define SII902X_TPI_AVI_PIXEL_REP_BUS_24BIT	BIT(5)
#define SII902X_TPI_AVI_PIXEL_REP_RISING_EDGE	BIT(4)
#define SII902X_TPI_AVI_PIXEL_REP_NONE		0
#define SII902X_TPI_CLK_RATIO_1X		BIT(6)

#define SII902X_TPI_AVI_IN_FORMAT		0x09
#define SII902X_TPI_AVI_INPUT_RANGE_AUTO	(0 << 2)
#define SII902X_TPI_AVI_INPUT_COLORSPACE_RGB	(0 << 0)

#define SII902X_SYS_CTRL_DATA			0x1a
#define SII902X_SYS_CTRL_PWR_DWN		BIT(4)
#define SII902X_SYS_CTRL_AV_MUTE		BIT(3)
#define SII902X_SYS_CTRL_DDC_BUS_REQ		BIT(2)
#define SII902X_SYS_CTRL_DDC_BUS_GRTD		BIT(1)
#define SII902X_SYS_CTRL_OUTPUT_MODE		BIT(0)
#define SII902X_SYS_CTRL_OUTPUT_HDMI		1
#define SII902X_SYS_CTRL_OUTPUT_DVI		0

#define SII902X_REG_CHIPID(n)			(0x1b + (n))

#define SII902X_PWR_STATE_CTRL			0x1e
#define SII902X_AVI_POWER_STATE_MSK		GENMASK(1, 0)
#define SII902X_AVI_POWER_STATE_D(l)		((l) & SII902X_AVI_POWER_STATE_MSK)

#define SII902X_INT_ENABLE			0x3c
#define SII902X_INT_STATUS			0x3d
#define SII902X_HOTPLUG_EVENT			BIT(0)
#define SII902X_PLUGGED_STATUS			BIT(2)

#define SII902X_REG_TPI_RQB			0xc7

#define SII902X_I2C_BUS_ACQUISITION_TIMEOUT_MS	500
#define SII902X_DDC_ADDR			0x50

#define SII902X_MIN_PIXEL_CLOCK_KHZ		25000
#define SII902X_MAX_PIXEL_CLOCK_KHZ		165000

/* Fallback to these timings if pixel clocks are beyond the supported range */
static const struct display_timing default_timing = {
	.pixelclock.typ		= 74500,
	.hactive.typ		= 1280,
	.hfront_porch.typ	= 64,
	.hback_porch.typ	= 192,
	.hsync_len.typ		= 128,
	.vactive.typ		= 720,
	.vfront_porch.typ	= 3,
	.vback_porch.typ	= 20,
	.vsync_len.typ		= 5,
	.flags			= DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH,
};

struct sii902x_priv {
	u8 edid[EDID_EXT_SIZE];
	struct display_timing timing;
	bool edid_valid;  /* Flag to track if EDID has been successfully read */
};

static int sii902x_reg_read(struct udevice *dev, u8 reg)
{
	int ret = dm_i2c_reg_read(dev, reg);
	return ret;
}

static int sii902x_reg_write(struct udevice *dev, u8 reg, u8 val)
{
	int ret = dm_i2c_reg_write(dev, reg, val);
	return ret;
}

static int sii902x_update_bits(struct udevice *dev, u8 reg, u8 mask, u8 val)
{
	int ret, old_val, new_val;

	ret = sii902x_reg_read(dev, reg);
	if (ret < 0)
		return ret;

	old_val = ret;
	new_val = (ret & ~mask) | (val & mask);

	return sii902x_reg_write(dev, reg, new_val);
}

/**
 * sii902x_bridge_mode_set() - Configure display timings
 *
 * Internal function to configure the SII902X bridge with display timings.
 * This writes video timing data to specific TPI registers.
 *
 * @dev: Bridge device
 * @timing: Display timing configuration
 * @return: 0 on success, negative error code on failure
 */
static int sii902x_bridge_mode_set(struct udevice *dev,
				   const struct display_timing *timing)
{
	u8 buf[10];
	u16 pixel_clock_10khz;
	int ret;

	if (!timing) {
		dev_err(dev, "Invalid timing parameter\n");
		return -EINVAL;
	}

	/* Convert pixel clock from Hz to 10kHz units */
	pixel_clock_10khz = timing->pixelclock.typ / 10000;

	dev_info(dev, "Setting mode: %dx%d@%dkHz\n",
		 timing->hactive.typ, timing->vactive.typ,
		 timing->pixelclock.typ / 1000);

	/* Prepare TPI video data (registers 0x00-0x09) based on Linux kernel */
	buf[0] = pixel_clock_10khz & 0xff;		/* Pixel clock low byte */
	buf[1] = (pixel_clock_10khz >> 8) & 0xff;	/* Pixel clock high byte */
	buf[2] = 60;					/* Refresh rate (assume 60Hz) */
	buf[3] = 0x00;					/* Reserved */
	buf[4] = timing->hactive.typ & 0xff;		/* H active low byte */
	buf[5] = (timing->hactive.typ >> 8) & 0xff;	/* H active high byte */
	buf[6] = timing->vactive.typ & 0xff;		/* V active low byte */
	buf[7] = (timing->vactive.typ >> 8) & 0xff;	/* V active high byte */
	buf[8] = SII902X_TPI_CLK_RATIO_1X |
		 SII902X_TPI_AVI_PIXEL_REP_NONE |
		 SII902X_TPI_AVI_PIXEL_REP_BUS_24BIT;	/* TPI pixel repetition */
	buf[9] = SII902X_TPI_AVI_INPUT_RANGE_AUTO |
		 SII902X_TPI_AVI_INPUT_COLORSPACE_RGB;	/* TPI AVI input format */

	/* Write video timing data to TPI registers 0x00-0x09 */
	for (int i = 0; i < 10; i++) {
		ret = sii902x_reg_write(dev, SII902X_TPI_VIDEO_DATA + i, buf[i]);
		if (ret) {
			dev_err(dev, "Failed to write TPI video data[%d]: %d\n", i, ret);
			return ret;
		}
	}

	/* Validate timing constraints */
	if (timing->pixelclock.typ < SII902X_MIN_PIXEL_CLOCK_KHZ * 1000 ||
	    timing->pixelclock.typ > SII902X_MAX_PIXEL_CLOCK_KHZ * 1000) {
		dev_warn(dev, "Pixel clock %d Hz out of range (%d-%d kHz)\n",
			 timing->pixelclock.typ,
			 SII902X_MIN_PIXEL_CLOCK_KHZ,
			 SII902X_MAX_PIXEL_CLOCK_KHZ);
	}

	dev_info(dev, "Mode set complete: %dx%d@%dHz, pixel_clock=%dkHz\n",
		 timing->hactive.typ, timing->vactive.typ,
		 60, timing->pixelclock.typ / 1000);

	return 0;
}

/**
 * sii902x_bridge_init() - Initialize the SII902X bridge
 *
 * This function initializes the SII902X bridge by configuring the output mode
 * and power state registers.
 *
 * @dev: Bridge device
 * @return: 0 on success, negative error code on failure
 */
static int sii902x_bridge_init(struct udevice *dev)
{
	int ret;
	u8 output_mode = SII902X_SYS_CTRL_OUTPUT_HDMI;  /* Default to HDMI mode */

	/* Set output mode to HDMI */
	ret = sii902x_update_bits(dev, SII902X_SYS_CTRL_DATA,
				  SII902X_SYS_CTRL_OUTPUT_MODE, output_mode);
	if (ret) {
		dev_err(dev, "Failed to set output mode: %d\n", ret);
		return ret;
	}

	/* Power up the device (Power State D0) */
	ret = sii902x_update_bits(dev, SII902X_PWR_STATE_CTRL,
				  SII902X_AVI_POWER_STATE_MSK,
				  SII902X_AVI_POWER_STATE_D(0));
	if (ret) {
		dev_err(dev, "Failed to set power state: %d\n", ret);
		return ret;
	}

	/* Clear power down bit to enable the device */
	ret = sii902x_update_bits(dev, SII902X_SYS_CTRL_DATA,
				  SII902X_SYS_CTRL_PWR_DWN, 0);
	if (ret) {
		dev_err(dev, "Failed to clear power down: %d\n", ret);
		return ret;
	}

	return 0;
}

static int sii902x_ddc_bus_request(struct udevice *dev)
{
	u8 timeout_count = 0;
	int ret, status;
	uint saved_flags = 0;

	/* Save and set ignore-NAK flag for DDC bus request operations */
	i2c_get_chip_flags(dev, &saved_flags);
	i2c_set_chip_flags(dev, saved_flags | DM_I2C_CHIP_IGNORE_NAK);

	ret = sii902x_update_bits(dev, SII902X_SYS_CTRL_DATA,
				  SII902X_SYS_CTRL_DDC_BUS_REQ,
				  SII902X_SYS_CTRL_DDC_BUS_REQ);
	if (ret) {
		i2c_set_chip_flags(dev, saved_flags);
		return ret;
	}

	timeout_count = 0;
	while (timeout_count < 5) {
		status = sii902x_reg_read(dev, SII902X_SYS_CTRL_DATA);
		if (status < 0) {
			i2c_set_chip_flags(dev, saved_flags);
			return status;
		}

		if (status & SII902X_SYS_CTRL_DDC_BUS_GRTD) {
			/*
			 * Per datasheet: once granted, the host must write
			 * back bits[2:1]=11 (REQ|GRTD). This closes the
			 * switch and allows subsequent I2C accesses to flow
			 * out onto the DDC bus.
			 */
			ret = sii902x_update_bits(dev, SII902X_SYS_CTRL_DATA,
						  SII902X_SYS_CTRL_DDC_BUS_REQ |
						  SII902X_SYS_CTRL_DDC_BUS_GRTD,
						  SII902X_SYS_CTRL_DDC_BUS_REQ |
						  SII902X_SYS_CTRL_DDC_BUS_GRTD);
			return 0;
		}
		udelay(1000); /* Wait 1ms between attempts */
		timeout_count++;
	}

	i2c_set_chip_flags(dev, saved_flags);
	return -EBUSY;
}

static int sii902x_ddc_bus_release(struct udevice *dev)
{
	unsigned long start;
	int ret, status;
	uint saved_flags = 0;

	/* Save and set ignore-NAK flag for DDC bus release operations */
	i2c_get_chip_flags(dev, &saved_flags);
	i2c_set_chip_flags(dev, saved_flags | DM_I2C_CHIP_IGNORE_NAK);

	ret = sii902x_update_bits(dev, SII902X_SYS_CTRL_DATA,
				  SII902X_SYS_CTRL_DDC_BUS_REQ |
				  SII902X_SYS_CTRL_DDC_BUS_GRTD, 0);
	if (ret) {
		i2c_set_chip_flags(dev, saved_flags);
		return ret;
	}

	start = get_timer(0);
	do {
		status = sii902x_reg_read(dev, SII902X_SYS_CTRL_DATA);
		if (status < 0) {
			return status;
			i2c_set_chip_flags(dev, saved_flags);
		}

		if (!(status & (SII902X_SYS_CTRL_DDC_BUS_REQ |
				SII902X_SYS_CTRL_DDC_BUS_GRTD))) {
			i2c_set_chip_flags(dev, saved_flags);
			return 0;
		}

		/* Bits not yet clear - repeat the clearing write */
		ret = sii902x_update_bits(dev, SII902X_SYS_CTRL_DATA,
					  SII902X_SYS_CTRL_DDC_BUS_REQ |
					  SII902X_SYS_CTRL_DDC_BUS_GRTD, 0);
		udelay(1000);
	} while (get_timer(start) < SII902X_I2C_BUS_ACQUISITION_TIMEOUT_MS);

	dev_err(dev, "failed to release DDC bus\n");
	i2c_set_chip_flags(dev, saved_flags);
	return -ETIMEDOUT;
}

/**
 * sii902x_ddc_read_edid() - Read EDID from the HDMI sink via DDC
 *
 * The DDC device (address 0x50) shares the same physical I2C bus as
 * the bridge itself.  We read EDID by sending raw I2C messages to
 * the DDC address while the bus is granted to us.
 */
static int sii902x_ddc_read_edid(struct udevice *dev, u8 *buf, int size)
{
	struct udevice *i2c_bus = dev_get_parent(dev);
	struct udevice *ddc_dev;
	int ret;
	uint saved_flags = 0;

	if (!buf || size < 128)
		return -EINVAL;

	ret = i2c_get_chip(i2c_bus, SII902X_DDC_ADDR, 1, &ddc_dev);
	if (ret) {
		dev_err(dev, "cannot get DDC chip device: %d\n", ret);
		return ret;
	}

	/* Save and set ignore-NAK flag for DDC device EDID reads */
	i2c_get_chip_flags(ddc_dev, &saved_flags);
	i2c_set_chip_flags(ddc_dev, saved_flags | DM_I2C_CHIP_IGNORE_NAK);

	/* Read the base EDID block (128 bytes) */
	ret = dm_i2c_read(ddc_dev, 0, buf, EDID_SIZE);

	/* Restore original flags */
	i2c_set_chip_flags(ddc_dev, saved_flags);

	if (ret) {
		dev_err(dev, "failed to read base EDID block: %d\n", ret);
		return ret;
	}

	return EDID_SIZE;
}

static int sii902x_check_chipid(struct udevice *dev)
{
	int ret;
	u8 chipid[4];
	int i;

	for (i = 0; i < 4; i++) {
		ret = sii902x_reg_read(dev, SII902X_REG_CHIPID(i));
		if (ret < 0) {
			dev_err(dev, "failed to read chip ID[%d]: %d\n",
				i, ret);
			return ret;
		}
		chipid[i] = ret;
	}

	if (chipid[0] != 0xb0) {
		dev_err(dev, "invalid chip ID: %02x (expected 0xb0)\n",
			chipid[0]);
		return -ENODEV;
	}

	return 0;
}

static int sii902x_read_edid(struct udevice *dev, u8 *buf, int buf_size)
{
	struct sii902x_priv *priv = dev_get_priv(dev);
	uint saved_flags = 0;
	int ret, size;

	/* Check if EDID has already been successfully read and cached */
	if (priv->edid_valid) {
		size = min(128, buf_size);
		memcpy(buf, priv->edid, size);
		return size >= 128 ? 0 : -EIO;
	}

	/* Save and set ignore-NAK flag for entire EDID read sequence */
	i2c_get_chip_flags(dev, &saved_flags);
	i2c_set_chip_flags(dev, saved_flags | DM_I2C_CHIP_IGNORE_NAK);

	ret = sii902x_ddc_bus_request(dev);
	if (ret) {
		i2c_set_chip_flags(dev, saved_flags);
		return ret;
	}

	size = sii902x_ddc_read_edid(dev, priv->edid,
				     min(buf_size, (int)sizeof(priv->edid)));

	/* Always release the DDC bus, even on read failure */
	ret = sii902x_ddc_bus_release(dev);
	if (ret)
		return ret;

	/* Write 0x06 to SII902X_SYS_CTRL_DATA to enable EDID access */
	ret = sii902x_reg_write(dev, SII902X_SYS_CTRL_DATA, 0x06);
	if (ret) {
		sii902x_ddc_bus_release(dev);
		i2c_set_chip_flags(dev, saved_flags);
		return ret;
	}

	/* Restore original flags */
	i2c_set_chip_flags(dev, saved_flags);

	if (size <= 0)
		return size ? size : -EIO;

	/* Mark EDID as valid after successful read */
	priv->edid_valid = true;

	size = min(size, buf_size);
	memcpy(buf, priv->edid, size);

	return size >= 128 ? 0 : -EIO;
}

/**
 * sii902x_get_display_timings() - Get display timings
 *
 * This function searches for a panel device and extracts display timings
 * from either the panel device or directly uses the fallback default timings.
 *
 * @dev: SII902X bridge device
 * @timings: Pointer to display_timing structure to fill
 * @return: 0 on success, negative error code on failure
 */
static int sii902x_get_display_timings(struct udevice *dev, struct display_timing *timing)
{
	struct sii902x_priv *priv = dev_get_priv(dev);
	u8 edid[128];
	int bpc, ret;

	ret = sii902x_read_edid(dev, edid, sizeof(edid));
	if (ret) {
		memcpy(timing, &default_timing, sizeof(*timing));
		return 0;
	}

	ret = edid_get_timing(edid, sizeof(edid), timing, &bpc);
	if (ret) {
		memcpy(timing, &default_timing, sizeof(*timing));
		return 0;
	}

	/*
	 * edid_get_timing() returns pixelclock in Hz; U-Boot display_timing
	 * convention expects kHz (tidss does pclk * 1000 for clk_set_rate).
	 */
	timing->pixelclock.typ /= 1000;
	timing->pixelclock.min /= 1000;
	timing->pixelclock.max /= 1000;

	if (timing->pixelclock.typ < SII902X_MIN_PIXEL_CLOCK_KHZ ||
	    timing->pixelclock.typ > SII902X_MAX_PIXEL_CLOCK_KHZ) {
		memcpy(timing, &default_timing, sizeof(*timing));
		return 0;
	}

	priv->timing = *timing;
	return 0;
}

static int sii902x_enable(struct udevice *dev)
{
	struct display_timing timing;
	uint saved_flags = 0;
	int ret;

	/* Save and set ignore-NAK flag for entire enable sequence */
	i2c_get_chip_flags(dev, &saved_flags);
	i2c_set_chip_flags(dev, saved_flags | DM_I2C_CHIP_IGNORE_NAK);

	ret = sii902x_get_display_timings(dev, &timing);
	if (ret) {
		dev_err(dev, "Failed to get display timings: %d\n", ret);
		i2c_set_chip_flags(dev, saved_flags);
		return ret;
	}

	/* Configure the bridge with the retrieved timings */
	ret = sii902x_bridge_mode_set(dev, &timing);
	if (ret) {
		dev_err(dev, "Failed to set bridge mode: %d\n", ret);
		i2c_set_chip_flags(dev, saved_flags);
		return ret;
	}

	/* Restore original flags */
	i2c_set_chip_flags(dev, saved_flags);

	return 0;
}

static int sii902x_probe(struct udevice *dev)
{
	int ret;

	if (device_get_uclass_id(dev->parent) != UCLASS_I2C)
		return -EPROTONOSUPPORT;

	/* Enable TPI mode */
	ret = sii902x_reg_write(dev, SII902X_REG_TPI_RQB, 0x00);
	if (ret) {
		dev_err(dev, "failed to enable TPI mode: %d\n", ret);
		return ret;
	}

	ret = sii902x_check_chipid(dev);
	if (ret)
		return ret;

	return sii902x_bridge_init(dev);
}

static struct video_bridge_ops sii902x_ops = {
	.read_edid	= sii902x_read_edid,
	.get_display_timing = sii902x_get_display_timings,
	.enable		= sii902x_enable,
};

static const struct udevice_id sii902x_ids[] = {
	{ .compatible = "sil,sii9022", },
	{ }
};

U_BOOT_DRIVER(sii902x) = {
	.name	= "sii902x",
	.id	= UCLASS_VIDEO_BRIDGE,
	.of_match = sii902x_ids,
	.probe	= sii902x_probe,
	.ops	= &sii902x_ops,
	.priv_auto	= sizeof(struct sii902x_priv),
	.flags	= DM_FLAG_LEAVE_PD_ON,
};
