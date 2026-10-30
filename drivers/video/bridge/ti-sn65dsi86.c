// SPDX-License-Identifier: GPL-2.0+
/*
 * TI SN65DSI86 DSI-to-eDP/DP bridge driver for U-Boot.
 *
 * Based on Linux Kernel drivers/gpu/drm/bridge/ti-sn65dsi86.c
 * Ported to U-Boot by Rahul Sharma <r-sharma3@ti.com>
 *
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 * Copyright (C) 2026 Texas Instruments Incorporated - https://www.ti.com/
 */

#include <asm-generic/unaligned.h>
#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/ofnode_graph.h>
#include <dsi_host.h>
#include <edid.h>
#include <i2c.h>
#include <log.h>
#include <mipi_dsi.h>
#include <power/regulator.h>
#include <video_bridge.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/drm_dp_helper.h>
#include <asm/gpio.h>

/* Register definitions */
#define SN_DEVICE_ID_REGS			0x00
#define SN_SOFT_RESET_REG			0x09
#define  SOFT_RESET				BIT(0)
#define SN_DPPLL_SRC_REG			0x0A
#define  DPPLL_CLK_SRC_DSICLK			BIT(0)
#define  REFCLK_FREQ_MASK			GENMASK(3, 1)
#define  REFCLK_FREQ(x)				((x) << 1)
#define  DPPLL_SRC_DP_PLL_LOCK			BIT(7)
#define SN_PLL_ENABLE_REG			0x0D
#define SN_DSI_LANES_REG			0x10
#define  CHA_DSI_LANES_MASK			GENMASK(4, 3)
#define  CHA_DSI_LANES(x)			((x) << 3)
#define SN_DSIA_CLK_FREQ_REG			0x12
#define SN_CHA_ACTIVE_LINE_LENGTH_LOW_REG	0x20
#define SN_CHA_VERTICAL_DISPLAY_SIZE_LOW_REG	0x24
#define SN_CHA_HSYNC_PULSE_WIDTH_LOW_REG	0x2C
#define SN_CHA_HSYNC_PULSE_WIDTH_HIGH_REG	0x2D
#define  CHA_HSYNC_POLARITY			BIT(7)
#define SN_CHA_VSYNC_PULSE_WIDTH_LOW_REG	0x30
#define SN_CHA_VSYNC_PULSE_WIDTH_HIGH_REG	0x31
#define  CHA_VSYNC_POLARITY			BIT(7)
#define SN_CHA_HORIZONTAL_BACK_PORCH_REG	0x34
#define SN_CHA_VERTICAL_BACK_PORCH_REG		0x36
#define SN_CHA_HORIZONTAL_FRONT_PORCH_REG	0x38
#define SN_CHA_VERTICAL_FRONT_PORCH_REG	0x3A
#define SN_LN_ASSIGN_REG			0x59
#define  LN_ASSIGN_WIDTH			2
#define SN_ENH_FRAME_REG			0x5A
#define  VSTREAM_ENABLE				BIT(3)
#define  LN_POLRS_OFFSET			4
#define  LN_POLRS_MASK				0xf0
#define SN_DATA_FORMAT_REG			0x5B
#define  BPP_18_RGB				BIT(0)
#define SN_HPD_DISABLE_REG			0x5C
#define  HPD_DISABLE				BIT(0)
#define  HPD_DEBOUNCED_STATE			BIT(4)
#define SN_GPIO_CTRL_REG			0x5F
#define  SN_GPIO_MUX_SPECIAL			2
#define  SN_GPIO_MUX_MASK			0x3
#define  SN_PWM_GPIO_IDX			3	/* GPIO4 */
#define SN_AUX_WDATA_REG(x)			(0x64 + (x))
#define SN_AUX_ADDR_19_16_REG			0x74
#define SN_AUX_LENGTH_REG			0x77
#define SN_AUX_CMD_REG				0x78
#define  AUX_CMD_SEND				BIT(0)
#define  AUX_CMD_REQ(x)				((x) << 4)
#define SN_AUX_RDATA_REG(x)			(0x79 + (x))
#define SN_SSC_CONFIG_REG			0x93
#define  DP_NUM_LANES_MASK			GENMASK(5, 4)
#define  DP_NUM_LANES(x)			((x) << 4)
#define SN_DATARATE_CONFIG_REG			0x94
#define  DP_DATARATE_MASK			GENMASK(7, 5)
#define  DP_DATARATE(x)				((x) << 5)
#define SN_TRAINING_SETTING_REG			0x95
#define  SCRAMBLE_DISABLE			BIT(4)
#define SN_ML_TX_MODE_REG			0x96
#define  ML_TX_MAIN_LINK_OFF			0
#define  ML_TX_NORMAL_MODE			BIT(0)
#define SN_PWM_PRE_DIV_REG			0xA0
#define SN_BACKLIGHT_SCALE_REG			0xA1
#define SN_BACKLIGHT_REG			0xA3
#define SN_PWM_EN_INV_REG			0xA5
#define  SN_PWM_EN_MASK				BIT(1)
#define SN_AUX_CMD_STATUS_REG			0xF4
#define  AUX_IRQ_STATUS_AUX_RPLY_TOUT		BIT(3)
#define  AUX_IRQ_STATUS_AUX_SHORT		BIT(5)
#define  AUX_IRQ_STATUS_NAT_I2C_FAIL		BIT(6)

/* eDP DPCD addresses not in U-Boot's drm_dp_helper.h */
#define DP_EDP_DPCD_REV				0x700
#define  DP_EDP_11				0x00
#define  DP_EDP_14				0x03
#define DP_SUPPORTED_LINK_RATES			0x010
#define  DP_MAX_SUPPORTED_RATES			8
#define DP_ALTERNATE_SCRAMBLER_RESET_ENABLE	BIT(0)

#define MIN_DSI_CLK_FREQ_MHZ	40
#define DP_CLK_FUDGE_NUM	10
#define DP_CLK_FUDGE_DEN	8
#define SN_AUX_MAX_PAYLOAD_BYTES	16
#define SN_MAX_DP_LANES			4
#define SN_LINK_TRAINING_TRIES		10

static const struct display_timing default_timing = {
	.pixelclock.typ		= 148500000,
	.hactive.typ		= 1920,
	.hfront_porch.typ	= 88,
	.hback_porch.typ	= 148,
	.hsync_len.typ		= 44,
	.vactive.typ		= 1080,
	.vfront_porch.typ	= 4,
	.vback_porch.typ	= 36,
	.vsync_len.typ		= 5,
	.flags			= DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH,
};

/* Reference clock LUTs */
static const u32 ti_sn_refclk_lut[] = {
	12000000,
	19200000,
	26000000,
	27000000,
	38400000,
};

static const u32 ti_sn_dsiclk_lut[] = {
	468000000,
	384000000,
	416000000,
	486000000,
	460800000,
};

/* LUT index = register value; value = DP data rate in Mbps */
static const unsigned int ti_sn_dp_rate_lut[] = {
	0, 1620, 2160, 2430, 2700, 3240, 4320, 5400
};

struct ti_sn65dsi86_priv {
	struct udevice		*vcc;
	struct udevice		*vcca;
	struct udevice		*vccio;
	struct udevice		*vpll;
	struct gpio_desc	enable_gpio;
	struct clk		refclk;
	bool			has_refclk;
	bool			is_edp;
	int			dp_lanes;
	u8			ln_assign;
	u8			ln_polrs;
	int			dsi_lanes;
	struct udevice		*dsi_host;
	struct mipi_dsi_device	dsi_device;
	struct display_timing	timing;
};

/* ---- I2C helpers ---- */

static int ti_sn_rd(struct udevice *dev, u8 reg, u8 *val)
{
	return dm_i2c_read(dev, reg, val, 1);
}

static int ti_sn_wr(struct udevice *dev, u8 reg, u8 val)
{
	return dm_i2c_write(dev, reg, &val, 1);
}

static int ti_sn_rmw(struct udevice *dev, u8 reg, u8 mask, u8 val)
{
	u8 tmp;
	int ret;

	ret = ti_sn_rd(dev, reg, &tmp);
	if (ret)
		return ret;
	tmp = (tmp & ~mask) | (val & mask);
	return ti_sn_wr(dev, reg, tmp);
}

static void ti_sn_wr16(struct udevice *dev, u8 reg, u16 val)
{
	ti_sn_wr(dev, reg,     val & 0xff);
	ti_sn_wr(dev, reg + 1, val >> 8);
}

/* ---- AUX channel ---- */

static int ti_sn_aux_transfer(struct udevice *dev, u8 request,
			      u32 addr, u8 *buf, size_t len)
{
	u8 addr_len[4];
	u8 val;
	int i, ret;

	if (len > SN_AUX_MAX_PAYLOAD_BYTES)
		return -EINVAL;

	put_unaligned_be32(((addr & 0xfffff) << 8) | (len & 0xff), addr_len);

	ti_sn_wr(dev, SN_AUX_CMD_REG, AUX_CMD_REQ(request));
	for (i = 0; i < 4; i++)
		ti_sn_wr(dev, SN_AUX_ADDR_19_16_REG + i, addr_len[i]);

	if (!(request & 1)) {	/* write: bit 0 = 0 */
		for (i = 0; i < (int)len; i++)
			ti_sn_wr(dev, SN_AUX_WDATA_REG(i), buf[i]);
	}

	/* Clear stale status */
	ti_sn_wr(dev, SN_AUX_CMD_STATUS_REG,
		 AUX_IRQ_STATUS_NAT_I2C_FAIL |
		 AUX_IRQ_STATUS_AUX_RPLY_TOUT |
		 AUX_IRQ_STATUS_AUX_SHORT);

	ti_sn_wr(dev, SN_AUX_CMD_REG, AUX_CMD_REQ(request) | AUX_CMD_SEND);

	/* Poll up to 50ms */
	for (i = 0; i < 100; i++) {
		ret = ti_sn_rd(dev, SN_AUX_CMD_REG, &val);
		if (ret)
			return ret;
		if (!(val & AUX_CMD_SEND))
			break;
	}
	if (i == 100)
		return -ETIMEDOUT;

	ret = ti_sn_rd(dev, SN_AUX_CMD_STATUS_REG, &val);
	if (ret)
		return ret;
	if (val & AUX_IRQ_STATUS_AUX_RPLY_TOUT)
		return -ETIMEDOUT;
	if (val & AUX_IRQ_STATUS_NAT_I2C_FAIL)
		return -EIO;

	if (request & 1) {	/* read */
		for (i = 0; i < (int)len; i++)
			ti_sn_rd(dev, SN_AUX_RDATA_REG(i), &buf[i]);
	}

	return 0;
}

static int ti_sn_dpcd_read(struct udevice *dev, u32 reg, u8 *val)
{
	return ti_sn_aux_transfer(dev, DP_AUX_NATIVE_READ, reg, val, 1);
}

static int ti_sn_dpcd_write(struct udevice *dev, u32 reg, u8 val)
{
	return ti_sn_aux_transfer(dev, DP_AUX_NATIVE_WRITE, reg, &val, 1);
}

/* ---- EDID read + timing parse ---- */

static int ti_sn_read_edid(struct udevice *dev, u8 *buf, size_t buf_size)
{
	u8 offset = 0;
	size_t done = 0, chunk;
	int ret;

	if (buf_size < 128)
		return -EINVAL;

	ret = ti_sn_aux_transfer(dev, DP_AUX_I2C_WRITE | DP_AUX_I2C_MOT,
				 0x50, &offset, 1);
	if (ret)
		return ret;

	while (done < 128) {
		chunk = min_t(size_t, SN_AUX_MAX_PAYLOAD_BYTES, 128 - done);
		u8 req = (done + chunk < 128) ? (DP_AUX_I2C_READ | DP_AUX_I2C_MOT)
					      : DP_AUX_I2C_READ;

		ret = ti_sn_aux_transfer(dev, req, 0x50, buf + done, chunk);
		if (ret)
			return ret;
		done += chunk;
	}
	return 0;
}

/* Bridge enable helpers */
static void ti_sn_set_refclk_freq(struct udevice *dev)
{
	struct ti_sn65dsi86_priv *priv = dev_get_priv(dev);
	u32 refclk_rate;
	const u32 *lut;
	size_t lut_size;
	int i, best_idx = 1;

	if (priv->has_refclk) {
		refclk_rate = clk_get_rate(&priv->refclk);
		lut = ti_sn_refclk_lut;
		lut_size = ARRAY_SIZE(ti_sn_refclk_lut);

		for (i = 0; i < (int)lut_size; i++)
			if (lut[i] == refclk_rate)
				break;

		if (i >= (int)lut_size) {
			u32 best_diff = UINT_MAX;

			for (i = 0; i < (int)lut_size; i++) {
				u32 diff = (lut[i] > refclk_rate) ?
					   (lut[i] - refclk_rate) :
					   (refclk_rate - lut[i]);
				if (diff < best_diff) {
					best_diff = diff;
					best_idx = i;
				}
			}
			i = best_idx;
		}

		ti_sn_rmw(dev, SN_DPPLL_SRC_REG, REFCLK_FREQ_MASK, REFCLK_FREQ(i));
	} else {
		/* No external refclk: use DSI clock, find best match in DSI LUT */
		unsigned int bit_rate_mhz = (priv->timing.pixelclock.typ / 1000) * 24;
		unsigned int dsi_clk_hz = (bit_rate_mhz / (priv->dsi_lanes * 2)) * 1000000;

		lut = ti_sn_dsiclk_lut;
		lut_size = ARRAY_SIZE(ti_sn_dsiclk_lut);

		for (i = 0; i < (int)lut_size; i++)
			if (lut[i] == dsi_clk_hz)
				break;

		if (i >= (int)lut_size) {
			u32 best_diff = UINT_MAX;

			for (i = 0; i < (int)lut_size; i++) {
				u32 diff = (lut[i] > dsi_clk_hz) ?
					   (lut[i] - dsi_clk_hz) :
					   (dsi_clk_hz - lut[i]);
				if (diff < best_diff) {
					best_diff = diff;
					best_idx = i;
				}
			}
			i = best_idx;
		}

		ti_sn_rmw(dev, SN_DPPLL_SRC_REG,
			  REFCLK_FREQ_MASK | DPPLL_CLK_SRC_DSICLK,
			  REFCLK_FREQ(i) | DPPLL_CLK_SRC_DSICLK);
	}
}

/*
 * HPD is disabled only for eDP panels, not for DP connectors.
 */
static void ti_sn_enable_comms(struct udevice *dev)
{
	struct ti_sn65dsi86_priv *priv = dev_get_priv(dev);

	ti_sn_set_refclk_freq(dev);

	/*
	 * HPD debounce is 100-400 ms. For eDP panels disable it (the panel
	 * driver hardcodes its own delay). For plain DisplayPort connectors
	 * keep HPD enabled so hot-plug detection works.
	 */
	if (priv->is_edp)
		ti_sn_rmw(dev, SN_HPD_DISABLE_REG, HPD_DISABLE, HPD_DISABLE);
}

/*
 * Formula: val = (MIN_DSI_CLK_FREQ_MHZ / 5) + (((clk_mhz - MIN_DSI_CLK_FREQ_MHZ) / 5) & 0xFF)
 */
static void ti_sn_set_dsi_rate(struct udevice *dev, const struct display_timing *t)
{
	struct ti_sn65dsi86_priv *priv = dev_get_priv(dev);
	unsigned int bit_rate_mhz, clk_freq_mhz, val;

	/* bpp = 24 (RGB888), lanes = 2, DDR so /2 */
	bit_rate_mhz = (t->pixelclock.typ / 1000) * 24;
	clk_freq_mhz = bit_rate_mhz / (priv->dsi_lanes * 2);

	val = (MIN_DSI_CLK_FREQ_MHZ / 5) +
		(((clk_freq_mhz - MIN_DSI_CLK_FREQ_MHZ) / 5) & 0xFF);
	ti_sn_wr(dev, SN_DSIA_CLK_FREQ_REG, val);
}

/* Polarity is derived from DISPLAY_FLAGS (from EDID), not forced. */
static void ti_sn_set_video_timings(struct udevice *dev, const struct display_timing *t)
{
	u8 hsync_pol = 0, vsync_pol = 0;

	if (t->flags & DISPLAY_FLAGS_HSYNC_LOW)
		hsync_pol = CHA_HSYNC_POLARITY;
	if (t->flags & DISPLAY_FLAGS_VSYNC_LOW)
		vsync_pol = CHA_VSYNC_POLARITY;

	ti_sn_wr16(dev, SN_CHA_ACTIVE_LINE_LENGTH_LOW_REG, t->hactive.typ);
	ti_sn_wr16(dev, SN_CHA_VERTICAL_DISPLAY_SIZE_LOW_REG, t->vactive.typ);

	ti_sn_wr(dev, SN_CHA_HSYNC_PULSE_WIDTH_LOW_REG,
		 t->hsync_len.typ & 0xff);
	ti_sn_wr(dev, SN_CHA_HSYNC_PULSE_WIDTH_HIGH_REG,
		 ((t->hsync_len.typ >> 8) & 0x7f) | hsync_pol);

	ti_sn_wr(dev, SN_CHA_VSYNC_PULSE_WIDTH_LOW_REG,
		 t->vsync_len.typ & 0xff);
	ti_sn_wr(dev, SN_CHA_VSYNC_PULSE_WIDTH_HIGH_REG,
		 ((t->vsync_len.typ >> 8) & 0x7f) | vsync_pol);

	ti_sn_wr(dev, SN_CHA_HORIZONTAL_BACK_PORCH_REG,
		 t->hback_porch.typ & 0xff);
	ti_sn_wr(dev, SN_CHA_VERTICAL_BACK_PORCH_REG,
		 t->vback_porch.typ & 0xff);
	ti_sn_wr(dev, SN_CHA_HORIZONTAL_FRONT_PORCH_REG,
		 t->hfront_porch.typ & 0xff);
	ti_sn_wr(dev, SN_CHA_VERTICAL_FRONT_PORCH_REG,
		 t->vfront_porch.typ & 0xff);
}

static unsigned int ti_sn_read_valid_rates(struct udevice *dev)
{
	unsigned int valid_rates = 0;
	u8 dpcd_val;
	int ret, i, j;

	ret = ti_sn_dpcd_read(dev, DP_EDP_DPCD_REV, &dpcd_val);
	if (ret) {
		dev_warn(dev, "Can't read eDP revision, assuming 1.1\n");
		dpcd_val = DP_EDP_11;
	}

	if (dpcd_val >= DP_EDP_14) {
		__le16 sink_rates[DP_MAX_SUPPORTED_RATES];
		unsigned int rate_per_200khz, rate_mhz;

		ret = ti_sn_aux_transfer(dev, DP_AUX_NATIVE_READ,
					 DP_SUPPORTED_LINK_RATES,
					 (u8 *)sink_rates, sizeof(sink_rates));
		if (ret) {
			dev_warn(dev, "Can't read supported rate table\n");
			memset(sink_rates, 0, sizeof(sink_rates));
		}

		for (i = 0; i < DP_MAX_SUPPORTED_RATES; i++) {
			rate_per_200khz = le16_to_cpu(sink_rates[i]);
			if (!rate_per_200khz)
				break;
			rate_mhz = rate_per_200khz * 200 / 1000;
			for (j = 0; j < (int)ARRAY_SIZE(ti_sn_dp_rate_lut); j++)
				if (ti_sn_dp_rate_lut[j] == rate_mhz)
					valid_rates |= BIT(j);
		}

		for (i = 0; i < (int)ARRAY_SIZE(ti_sn_dp_rate_lut); i++)
			if (valid_rates & BIT(i))
				return valid_rates;

		dev_warn(dev, "No matching eDP rates in table; falling back\n");
	}

	ret = ti_sn_dpcd_read(dev, DP_MAX_LINK_RATE, &dpcd_val);
	if (ret) {
		dev_warn(dev, "Can't read max link rate; assuming 5.4 GHz\n");
		dpcd_val = DP_LINK_BW_5_4;
	}

	switch (dpcd_val) {
	default:
		dev_warn(dev, "Unexpected max rate 0x%x; assuming 5.4 GHz\n", dpcd_val);
		fallthrough;
	case DP_LINK_BW_5_4:
		valid_rates |= BIT(7);
		fallthrough;
	case DP_LINK_BW_2_7:
		valid_rates |= BIT(4);
		fallthrough;
	case DP_LINK_BW_1_62:
		valid_rates |= BIT(1);
		break;
	}

	return valid_rates;
}

static int ti_sn_calc_min_dp_rate_idx(const struct display_timing *t, int dp_lanes)
{
	unsigned int bit_rate_khz, dp_rate_mhz;
	int i;

	bit_rate_khz = t->pixelclock.typ * 24;	/* bpp=24 */
	dp_rate_mhz = DIV_ROUND_UP(bit_rate_khz * DP_CLK_FUDGE_NUM,
				   1000 * dp_lanes * DP_CLK_FUDGE_DEN);

	for (i = 1; i < (int)ARRAY_SIZE(ti_sn_dp_rate_lut) - 1; i++)
		if (ti_sn_dp_rate_lut[i] >= dp_rate_mhz)
			break;
	return i;
}

static int ti_sn_link_training(struct udevice *dev, int dp_rate_idx,
			       const char **last_err_str)
{
	u8 val;
	int i, j, ret = -EIO;

	ti_sn_rmw(dev, SN_DATARATE_CONFIG_REG,
		  DP_DATARATE_MASK, DP_DATARATE(dp_rate_idx));

	ti_sn_wr(dev, SN_PLL_ENABLE_REG, 1);

	/* Poll up to 50ms for DP PLL lock */
	for (i = 0; i < 50; i++) {
		ti_sn_rd(dev, SN_DPPLL_SRC_REG, &val);
		if (val & DPPLL_SRC_DP_PLL_LOCK)
			break;
	}
	if (!(val & DPPLL_SRC_DP_PLL_LOCK)) {
		*last_err_str = "DP PLL lock timed out";
		ret = -ETIMEDOUT;
		goto exit;
	}

	for (i = 0; i < SN_LINK_TRAINING_TRIES; i++) {
		ti_sn_wr(dev, SN_ML_TX_MODE_REG, 0x0A);

		for (j = 0; j < 500; j++) {
			ti_sn_rd(dev, SN_ML_TX_MODE_REG, &val);
			if (val == ML_TX_MAIN_LINK_OFF || val == ML_TX_NORMAL_MODE)
				break;
		}
		if (j == 500) {
			*last_err_str = "Link training polling timed out";
			ret = -ETIMEDOUT;
			continue;
		}
		if (val == ML_TX_MAIN_LINK_OFF) {
			*last_err_str = "Link training failed, link is off";
			ret = -EIO;
			continue;
		}
		ret = 0;
		break;
	}

exit:
	if (ret)
		ti_sn_wr(dev, SN_PLL_ENABLE_REG, 0);
	return ret;
}

static void ti_sn_parse_lanes(struct udevice *dev)
{
	struct ti_sn65dsi86_priv *priv = dev_get_priv(dev);
	u32 lane_assignments[SN_MAX_DP_LANES] = { 0, 1, 2, 3 };
	u32 lane_polarities[SN_MAX_DP_LANES] = { };
	ofnode endpoint;
	u8 ln_assign = 0, ln_polrs = 0;
	int dp_lanes, i;

	endpoint = ofnode_graph_get_endpoint_by_regs(dev_ofnode(dev), 1, -1);
	if (ofnode_valid(endpoint)) {
		const void *prop;
		int len;

		prop = ofnode_get_property(endpoint, "data-lanes", &len);
		if (prop && len > 0) {
			dp_lanes = min_t(int, len / (int)sizeof(u32), SN_MAX_DP_LANES);
			ofnode_read_u32_array(endpoint, "data-lanes",
					      lane_assignments, dp_lanes);
			ofnode_read_u32_array(endpoint, "lane-polarities",
					      lane_polarities, dp_lanes);
		} else {
			dp_lanes = SN_MAX_DP_LANES;
		}
	} else {
		dp_lanes = SN_MAX_DP_LANES;
	}

	for (i = SN_MAX_DP_LANES - 1; i >= 0; i--) {
		ln_assign = ln_assign << LN_ASSIGN_WIDTH | lane_assignments[i];
		ln_polrs = ln_polrs << 1 | lane_polarities[i];
	}

	priv->dp_lanes = dp_lanes;
	priv->ln_assign = ln_assign;
	priv->ln_polrs = ln_polrs;
}

/* ---- Power up sequence (Linux ti_sn65dsi86_resume equivalent) ---- */

static void ti_sn_power_up(struct udevice *dev)
{
	struct ti_sn65dsi86_priv *priv = dev_get_priv(dev);

	if (priv->vcc)
		regulator_set_enable(priv->vcc, true);
	if (priv->vcca)
		regulator_set_enable(priv->vcca, true);
	if (priv->vccio)
		regulator_set_enable(priv->vccio, true);
	if (priv->vpll)
		regulator_set_enable(priv->vpll, true);

	if (dm_gpio_is_valid(&priv->enable_gpio))
		dm_gpio_set_value(&priv->enable_gpio, 1);

	if (priv->has_refclk)
		clk_enable(&priv->refclk);
}

/* ---- video_bridge_ops ---- */

static int ti_sn65dsi86_attach(struct udevice *dev)
{
	struct ti_sn65dsi86_priv *priv = dev_get_priv(dev);

	if (!priv->dsi_host) {
		dev_err(dev, "DSI host not found\n");
		return -ENODEV;
	}
	return 0;
}

static int ti_sn65dsi86_get_display_timing(struct udevice *dev,
					   struct display_timing *timing)
{
	struct ti_sn65dsi86_priv *priv = dev_get_priv(dev);
	u8 edid[128];
	int bpc, ret;
	int timeout;
	u8 hpd;

	/* Return cached timing (with rounded pclk) if pre_enable already ran */
	if (priv->timing.hactive.typ && priv->timing.pixelclock.typ) {
		*timing = priv->timing;
		return 0;
	}

	/* Wait for HPD debounce */
	for (timeout = 500; timeout > 0; timeout--) {
		if (ti_sn_rd(dev, SN_HPD_DISABLE_REG, &hpd) == 0 &&
		    (hpd & HPD_DEBOUNCED_STATE))
			break;
	}

	ret = ti_sn_read_edid(dev, edid, sizeof(edid));
	if (ret) {
		dev_warn(dev, "EDID read failed (%d), using default timing\n",
			 ret);
		memcpy(timing, &default_timing, sizeof(*timing));
		return 0;
	}

	ret = edid_get_timing(edid, sizeof(edid), timing, &bpc);
	if (ret) {
		dev_warn(dev, "EDID parsing failed (%d), using default timing\n",
			 ret);
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

	priv->timing = *timing;
	return 0;
}

/*
 * pre_enable: Linux ti_sn_bridge_atomic_pre_enable equivalent.
 *
 * Brings up power, configures comms, then initialises the DSI host
 * (which starts the DPHY / HS clock) WITHOUT asserting VID_EN.
 * VID_EN is asserted only in enable(), after TIDSS VP is running and
 * link training is complete, so ERR_MISSING_HSYNC cannot latch.
 */
static int ti_sn65dsi86_pre_enable(struct udevice *dev)
{
	struct ti_sn65dsi86_priv *priv = dev_get_priv(dev);
	int ret;

	ti_sn_power_up(dev);

	/*
	 * Soft reset to ensure a clean state before configuring.
	 * After reset the chip re-samples GPIO3:1 for refclk source.
	 */
	ti_sn_wr(dev, SN_SOFT_RESET_REG, SOFT_RESET);

	/* Configure refclk frequency and conditionally disable HPD */
	ti_sn_enable_comms(dev);

	/*
	 * Initialise and enable the DSI host (sets VID_EN) BEFORE TIDSS VP
	 * starts.  The CDN DSI VSG must be armed here so it waits cleanly for
	 * the first VSYNC from the VP.  If VID_EN is asserted after the VP is
	 * already running the VSG sees a mid-frame DPI stream and immediately
	 * latches ERR_MISSING_HSYNC (VID_MODE_STS bit 2 / 0x4).
	 * This mirrors Linux cdns_dsi_bridge_atomic_pre_enable() which sets
	 * VID_EN in the pre_enable hook before the CRTC enable.
	 */
	if (priv->dsi_host) {
		ret = dsi_host_init(priv->dsi_host, &priv->dsi_device,
				    &priv->timing, priv->dsi_lanes, NULL);
		if (ret) {
			dev_err(dev, "dsi_host_init failed: %d\n", ret);
			return ret;
		}

		ret = dsi_host_enable(priv->dsi_host);
		if (ret) {
			dev_err(dev, "dsi_host_enable failed: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

/*
 * enable: Linux ti_sn_bridge_atomic_enable equivalent.
 *
 * At this point TIDSS VP is running (HSYNC is flowing into Cadence DSI).
 * VID_EN was already asserted in pre_enable so the CDN DSI VSG is armed
 * and waiting for HSYNC.  We do link training, program video timings,
 * enable VSTREAM, then call dsi_host_start_video to confirm VSG_RUN.
 */
static int ti_sn65dsi86_enable(struct udevice *dev)
{
	struct ti_sn65dsi86_priv *priv = dev_get_priv(dev);
	const char *last_err_str = "No supported DP rate";
	unsigned int valid_rates;
	u8 dpcd_val, val;
	int dp_rate_idx, max_dp_lanes;
	int ret = -EINVAL;

	if (ti_sn_dpcd_read(dev, DP_MAX_LANE_COUNT, &dpcd_val) == 0)
		max_dp_lanes = dpcd_val & DP_MAX_LANE_COUNT_MASK;
	else
		max_dp_lanes = SN_MAX_DP_LANES;
	priv->dp_lanes = min(priv->dp_lanes, max_dp_lanes);

	val = CHA_DSI_LANES(SN_MAX_DP_LANES - priv->dsi_lanes);
	ti_sn_rmw(dev, SN_DSI_LANES_REG, CHA_DSI_LANES_MASK, val);

	ti_sn_wr(dev, SN_LN_ASSIGN_REG, priv->ln_assign);
	ti_sn_rmw(dev, SN_ENH_FRAME_REG, LN_POLRS_MASK,
		  priv->ln_polrs << LN_POLRS_OFFSET);

	ti_sn_set_dsi_rate(dev, &priv->timing);

	if (priv->is_edp) {
		ti_sn_dpcd_write(dev, DP_EDP_CONFIGURATION_SET,
				 DP_ALTERNATE_SCRAMBLER_RESET_ENABLE);
		ti_sn_rmw(dev, SN_TRAINING_SETTING_REG, SCRAMBLE_DISABLE, 0);
	} else {
		ti_sn_rmw(dev, SN_TRAINING_SETTING_REG,
			  SCRAMBLE_DISABLE, SCRAMBLE_DISABLE);
	}

	ti_sn_rmw(dev, SN_DATA_FORMAT_REG, BPP_18_RGB, 0);

	ti_sn_rmw(dev, SN_SSC_CONFIG_REG, DP_NUM_LANES_MASK,
		  DP_NUM_LANES(min(priv->dp_lanes, 3)));

	valid_rates = ti_sn_read_valid_rates(dev);

	for (dp_rate_idx = ti_sn_calc_min_dp_rate_idx(&priv->timing, priv->dp_lanes);
	     dp_rate_idx < (int)ARRAY_SIZE(ti_sn_dp_rate_lut);
	     dp_rate_idx++) {
		if (!(valid_rates & BIT(dp_rate_idx)))
			continue;
		ret = ti_sn_link_training(dev, dp_rate_idx, &last_err_str);
		if (!ret)
			break;
	}
	if (ret) {
		dev_err(dev, "Link training failed: %s (%d)\n", last_err_str, ret);
		return ret;
	}

	ti_sn_set_video_timings(dev, &priv->timing);

	ti_sn_rmw(dev, SN_ENH_FRAME_REG, VSTREAM_ENABLE, VSTREAM_ENABLE);

	ti_sn_rmw(dev, SN_GPIO_CTRL_REG,
		  SN_GPIO_MUX_MASK << (2 * SN_PWM_GPIO_IDX),
		  SN_GPIO_MUX_SPECIAL << (2 * SN_PWM_GPIO_IDX));
	ti_sn_wr(dev, SN_PWM_PRE_DIV_REG, 1);
	ti_sn_wr(dev, SN_BACKLIGHT_SCALE_REG, 0xFF);
	ti_sn_wr(dev, SN_BACKLIGHT_SCALE_REG + 1, 0xFF);
	ti_sn_wr(dev, SN_BACKLIGHT_REG, 0xFF);
	ti_sn_wr(dev, SN_BACKLIGHT_REG + 1, 0xFF);
	ti_sn_wr(dev, SN_PWM_EN_INV_REG, SN_PWM_EN_MASK);

	if (priv->dsi_host) {
		ret = dsi_host_start_video(priv->dsi_host);
		if (ret)
			dev_warn(dev, "dsi_host_start_video: %d\n", ret);
	}

	return 0;
}

static const struct video_bridge_ops ti_sn65dsi86_ops = {
	.attach             = ti_sn65dsi86_attach,
	.pre_enable         = ti_sn65dsi86_pre_enable,
	.enable             = ti_sn65dsi86_enable,
	.get_display_timing = ti_sn65dsi86_get_display_timing,
};

static int ti_sn65dsi86_probe(struct udevice *dev)
{
	struct ti_sn65dsi86_priv *priv = dev_get_priv(dev);
	ofnode ep;
	u8 id_buf[8];
	char id_str[9];
	int ret;

	if (device_get_uclass_id(dev->parent) != UCLASS_I2C)
		return -EPROTONOSUPPORT;

	/* Optional regulators */
	device_get_supply_regulator(dev, "vcc-supply",   &priv->vcc);
	device_get_supply_regulator(dev, "vcca-supply",  &priv->vcca);
	device_get_supply_regulator(dev, "vccio-supply", &priv->vccio);
	device_get_supply_regulator(dev, "vpll-supply",  &priv->vpll);

	ret = gpio_request_by_name(dev, "enable-gpios", 0,
				   &priv->enable_gpio, GPIOD_IS_OUT);
	if (ret && ret != -ENOENT)
		return ret;

	ret = clk_get_by_name(dev, "refclk", &priv->refclk);
	priv->has_refclk = (ret == 0);

	/* Power up to read device ID */
	ti_sn_power_up(dev);

	ret = dm_i2c_read(dev, SN_DEVICE_ID_REGS, id_buf, sizeof(id_buf));
	if (ret) {
		dev_err(dev, "Failed to read device ID: %d\n", ret);
		return ret;
	}
	memcpy(id_str, id_buf, 8);
	id_str[8] = '\0';
	if (memcmp(id_buf, "68ISD   ", sizeof(id_buf))) {
		dev_err(dev, "Unrecognized device ID\n");
		return -ENODEV;
	}

	priv->dsi_lanes = 2;

	ti_sn_parse_lanes(dev);

	/*
	 * Detect connector type from port@1 remote node.
	 * "dp-connector" compatible Ã¢ÂÂ DP; "panel-edp" Ã¢ÂÂ eDP.
	 */
	priv->is_edp = false;
	ep = ofnode_graph_get_endpoint_by_regs(dev_ofnode(dev), 1, -1);
	if (ofnode_valid(ep)) {
		ofnode remote_ep = ofnode_graph_get_remote_endpoint(ep);

		if (ofnode_valid(remote_ep)) {
			ofnode conn = ofnode_graph_get_port_parent(remote_ep);

			if (ofnode_valid(conn) &&
			    ofnode_device_is_compatible(conn, "panel-edp"))
				priv->is_edp = true;
		}
	}

	/* Locate DSI host via OF graph port@0 */
	ep = ofnode_graph_get_endpoint_by_regs(dev_ofnode(dev), 0, -1);
	if (ofnode_valid(ep)) {
		ofnode remote_ep = ofnode_graph_get_remote_endpoint(ep);

		if (ofnode_valid(remote_ep)) {
			ofnode dsi_node = ofnode_graph_get_port_parent(remote_ep);

			if (ofnode_valid(dsi_node))
				uclass_get_device_by_ofnode(UCLASS_DSI_HOST,
							    dsi_node,
							    &priv->dsi_host);
		}
	}
	if (!priv->dsi_host)
		dev_warn(dev, "DSI host not found via OF graph\n");

	priv->dsi_device.dev = dev;
	priv->dsi_device.lanes = priv->dsi_lanes;
	priv->dsi_device.format = MIPI_DSI_FMT_RGB888;
	priv->dsi_device.mode_flags = MIPI_DSI_MODE_VIDEO |
				      MIPI_DSI_MODE_VIDEO_SYNC_PULSE;

	/*
	 * Read DPPLL_SRC to determine clock source. If DPPLL_CLK_SRC_DSICLK
	 * is clear the bridge uses an external refclk Ã¢ÂÂ non-continuous clock.
	 */
	if (priv->dsi_host) {
		u8 dppll_src = 0;

		ti_sn_enable_comms(dev);
		ti_sn_rd(dev, SN_DPPLL_SRC_REG, &dppll_src);
		if (!(dppll_src & DPPLL_CLK_SRC_DSICLK))
			priv->dsi_device.mode_flags |= MIPI_DSI_CLOCK_NON_CONTINUOUS;

		ret = dsi_host_attach(priv->dsi_host, &priv->dsi_device);
		if (ret)
			dev_warn(dev, "dsi_host_attach failed: %d\n", ret);
	}

	return 0;
}

/*
 * Uses the same compatible as the original driver so no DTS changes are
 * needed.  Enable CONFIG_VIDEO_BRIDGE_TI_SN65DSI86_NEW and disable
 * CONFIG_VIDEO_BRIDGE_TI_SN65DSI86 in defconfig to select this driver.
 */
static const struct udevice_id ti_sn65dsi86_ids[] = {
	{ .compatible = "ti,sn65dsi86" },
	{ }
};

U_BOOT_DRIVER(ti_sn65dsi86) = {
	.name		= "ti_sn65dsi86",
	.id		= UCLASS_VIDEO_BRIDGE,
	.of_match	= ti_sn65dsi86_ids,
	.probe		= ti_sn65dsi86_probe,
	.ops		= &ti_sn65dsi86_ops,
	.priv_auto	= sizeof(struct ti_sn65dsi86_priv),
};
