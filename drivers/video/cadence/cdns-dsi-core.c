// SPDX-License-Identifier: GPL-2.0
/*
 * Based on Linux Kernel drivers/gpu/drm/bridge/cadence/cdns-dsi-core.c
 * Ported to U-Boot by Rahul Sharma <r-sharma3@ti.com>
 *
 * Copyright: 2017 Cadence Design Systems, Inc.
 * Copyright (C) 2026 Texas Instruments Incorporated - https://www.ti.com/
 *
 * Author: Boris Brezillon <boris.brezillon@bootlin.com>
 */

#include <clk.h>
#include <dm.h>
#include <div64.h>
#include <linux/math64.h>
#include <dsi_host.h>
#include <generic-phy.h>
#include <panel.h>
#include <phy-mipi-dphy.h>
#include <reset.h>
#include <syscon.h>
#include <video_bridge.h>
#include <dm/device_compat.h>
#include <dm/lists.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/time.h>

#include "cdns-dsi-core.h"
#if defined(CONFIG_CDNS_DSI_J721E) || defined(CONFIG_SPL_CDNS_DSI_J721E)
#include "cdns-dsi-j721e.h"
#endif

#define IP_CONF				0x0
#define DIRCMD_FIFO_DEPTH(x)		(((x) & GENMASK(15, 13)) >> 13)
#define RX_FIFO_DEPTH(x)		((x) & GENMASK(5, 0))

#define MCTL_MAIN_DATA_CTL		0x4
#define HOST_EOT_GEN			BIT(17)
#define VID_EN				BIT(5)
#define IF_VID_SELECT(x)		((x) << 2)
#define IF_VID_SELECT_MASK		GENMASK(3, 2)
#define IF_VID_MODE			BIT(1)
#define BTA_EN				BIT(14)
#define READ_EN				BIT(13)
#define LINK_EN				BIT(0)

#define MCTL_MAIN_PHY_CTL		0x8
#define CLK_CONTINUOUS			BIT(4)
#define DATA_LANE_EN(x)			BIT((x) - 1)

#define MCTL_MAIN_EN			0xc
#define IF_EN(x)			BIT(13 + (x))
#define DATA_LANE_START(x)		BIT(4 + (x))
#define CLK_LANE_EN			BIT(3)
#define PLL_START			BIT(0)

#define MCTL_DPHY_CFG0			0x10
#define DPHY_C_RSTB			BIT(20)
#define DPHY_D_RSTB(x)			GENMASK(15 + (x), 16)
#define DPHY_PLL_PDN			BIT(10)
#define DPHY_CMN_PDN			BIT(9)
#define DPHY_C_PDN			BIT(8)
#define DPHY_ALL_D_PDN			GENMASK(7, 4)
#define DPHY_PLL_PSO			BIT(1)
#define DPHY_CMN_PSO			BIT(0)

#define MCTL_ULPOUT_TIME		0x1c
#define DATA_LANE_ULPOUT_TIME(x)	((x) << 9)
#define CLK_LANE_ULPOUT_TIME(x)		(x)

#define MCTL_DPHY_TIMEOUT1		0x14
#define HSTX_TIMEOUT(x)			((x) << 4)
#define HSTX_TIMEOUT_MAX		GENMASK(17, 0)
#define CLK_DIV(x)			(x)
#define CLK_DIV_MAX			GENMASK(3, 0)

#define MCTL_DPHY_TIMEOUT2		0x18
#define LPRX_TIMEOUT(x)			(x)

#define MCTL_MAIN_STS			0x24
#define HSTX_TIMEOUT_ERR		BIT(6)
#define LPRX_TIMEOUT_ERR		BIT(7)
#define CLK_LANE_RDY			BIT(1)
#define DATA_LANE_RDY(l)		BIT(2 + (l))
#define MCTL_MAIN_STS_CTL		0x130
#define MCTL_MAIN_STS_CLR		0x150
#define PLL_LOCKED			BIT(0)

#define MCTL_DPHY_ERR			0x28
#define ERR_CONT_LP(x, l)		BIT(18 + ((x) * 4) + (l))
#define ERR_CONTROL(l)			BIT(14 + (l))
#define ERR_SYNESC(l)			BIT(10 + (l))
#define ERR_ESC(l)			BIT(6 + (l))
#define MCTL_DPHY_ERR_CTL1		0x148

#define MCTL_LANE_STS			0x2c
#define PPI_C_TX_READY_HS		BIT(18)
#define DPHY_PLL_LOCK			BIT(17)
#define PPI_D_RX_ULPS_ESC(x)		(((x) & GENMASK(15, 12)) >> 12)
#define DATA_LANE_STATE(l, val)		(((val) >> (2 + (l) * 3)) & GENMASK(2, 0))
#define CLK_LANE_STATE_HS		2
#define CLK_LANE_STATE(val)		((val) & GENMASK(1, 0))

#define CMD_MODE_STS_CTL		0x134

#define DIRECT_CMD_SEND			0x80

#define DIRECT_CMD_MAIN_SETTINGS	0x84
#define CMD_LP_EN			BIT(24)
#define CMD_SIZE(x)			((x) << 16)
#define CMD_VCHAN_ID(x)			((x) << 14)
#define CMD_DATATYPE(x)			((x) << 8)
#define CMD_LONG			BIT(3)
#define READ_CMD			1
#define BTA_REQ				6

#define DIRECT_CMD_STS			0x88
#define DIRECT_CMD_STS_CTL		0x138
#define DIRECT_CMD_STS_CLR		0x158
#define DIRECT_CMD_STS_FLAG		0x178
#define READ_COMPLETED_WITH_ERR		BIT(10)
#define ACK_WITH_ERR_RCVD		BIT(5)
#define ACK_RCVD			BIT(4)
#define READ_COMPLETED			BIT(3)
#define WRITE_COMPLETED			BIT(1)

#define DIRECT_CMD_WRDATA		0x90

#define DIRECT_CMD_RDDATA		0xa0

#define DIRECT_CMD_RD_STS_CTL		0x13c

#define VID_MODE_STS			0xf0
#define VSG_RUNNING			BIT(0)
#define ERR_MISSING_DATA		BIT(1)
#define ERR_MISSING_HSYNC		BIT(2)
#define ERR_MISSING_VSYNC		BIT(3)
#define ERR_SMALL_LEN			BIT(4)
#define ERR_SMALL_HEIGHT		BIT(5)
#define ERR_BURST_WRITE			BIT(6)
#define ERR_LINE_WRITE			BIT(7)
#define ERR_LONG_READ			BIT(8)
#define ERR_VRS_WRONG_LEN		BIT(9)
#define VSG_RECOVERY			BIT(10)
#define VID_MODE_STS_CTL		0x140
#define VID_MODE_STS_CLR		0x160

#define DPI_IRQ_EN			0x1a0

#define VID_MAIN_CTL			0xb0
#define VID_IGNORE_MISS_VSYNC		BIT(31)
#define RECOVERY_MODE(x)		((x) << 25)
#define RECOVERY_MODE_NEXT_HSYNC	0
#define RECOVERY_MODE_NEXT_STOP_POINT	2
#define RECOVERY_MODE_NEXT_VSYNC	3
#define REG_BLKEOL_MODE(x)		((x) << 23)
#define REG_BLKLINE_MODE(x)		((x) << 21)
#define REG_BLK_MODE_BLANKING_PKT	1
#define SYNC_PULSE_HORIZONTAL		BIT(20)
#define SYNC_PULSE_ACTIVE		BIT(19)
#define VID_PIXEL_MODE_RGB565		(0 << 14)
#define VID_PIXEL_MODE_RGB666_PACKED	BIT(14)
#define VID_PIXEL_MODE_RGB666		(2 << 14)
#define VID_PIXEL_MODE_RGB888		(3 << 14)
#define VID_DATATYPE(x)			((x) << 8)

#define VID_VSIZE1			0xb4
#define VFP_LEN(x)			((x) << 12)
#define VBP_LEN(x)			((x) << 6)
#define VSA_LEN(x)			(x)

#define VID_VSIZE2			0xb8

#define VID_HSIZE1			0xc0
#define HBP_LEN(x)			((x) << 16)
#define HSA_LEN(x)			(x)

#define VID_HSIZE2			0xc4
#define HFP_LEN(x)			((x) << 16)
#define HACT_LEN(x)			(x)

#define VID_BLKSIZE1			0xcc
#define BLK_LINE_EVENT_PKT_LEN(x)	(x)

#define VID_BLKSIZE2			0xd0
#define BLK_LINE_PULSE_PKT_LEN(x)	(x)

#define VID_DPHY_TIME			0xdc
#define REG_WAKEUP_TIME(x)		((x) << 17)
#define REG_LINE_DURATION(x)		(x)

#define VID_VCA_SETTING2		0xf8
#define MAX_LINE_LIMIT(x)		((x) << 16)

#define ID_REG				0x1fc
#define REV_VENDOR_ID(x)		(((x) & GENMASK(31, 20)) >> 20)

#define DSI_HBP_FRAME_PULSE_OVERHEAD	12
#define DSI_HBP_FRAME_EVENT_OVERHEAD	16
#define DSI_HSA_FRAME_OVERHEAD		14
#define DSI_HFP_FRAME_OVERHEAD		6
#define DSI_HSS_VSS_VSE_FRAME_OVERHEAD	4
#define DSI_BLANKING_FRAME_OVERHEAD	6
#define DSI_NULL_FRAME_OVERHEAD		6
#define DSI_EOT_PKT_SIZE		4

static inline struct cdns_dsi *to_cdns_dsi(struct mipi_dsi_host *host)
{
	return container_of(host, struct cdns_dsi, host);
}

static unsigned int dpi_to_dsi_timing(unsigned int dpi_timing,
				      unsigned int dpi_bpp,
				      unsigned int dsi_pkt_overhead)
{
	unsigned int dsi_timing = DIV_ROUND_UP(dpi_timing * dpi_bpp, 8);

	if (dsi_timing < dsi_pkt_overhead)
		dsi_timing = 0;
	else
		dsi_timing -= dsi_pkt_overhead;

	return dsi_timing;
}

static int cdns_dsi_mode2cfg(struct cdns_dsi *dsi,
			     struct display_timing *timings,
			     struct cdns_dsi_cfg *dsi_cfg)
{
	struct cdns_dsi_output *output = &dsi->output;
	u32 dpi_hsa, dpi_hbp, dpi_hfp, dpi_hact;
	bool sync_pulse;
	int bpp;

	dpi_hsa = timings->hsync_len.typ;
	dpi_hbp = timings->hback_porch.typ;
	dpi_hfp = timings->hfront_porch.typ;
	dpi_hact = timings->hactive.typ;

	memset(dsi_cfg, 0, sizeof(*dsi_cfg));

	sync_pulse = output->dev->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE;

	bpp = mipi_dsi_pixel_format_to_bpp(output->dev->format);

	if (sync_pulse) {
		dsi_cfg->hbp = dpi_to_dsi_timing(dpi_hbp, bpp,
						 DSI_HBP_FRAME_PULSE_OVERHEAD);

		dsi_cfg->hsa = dpi_to_dsi_timing(dpi_hsa, bpp,
						 DSI_HSA_FRAME_OVERHEAD);
	} else {
		dsi_cfg->hbp = dpi_to_dsi_timing(dpi_hbp + dpi_hsa, bpp,
						 DSI_HBP_FRAME_EVENT_OVERHEAD);

		dsi_cfg->hsa = 0;
	}

	dsi_cfg->hact = dpi_to_dsi_timing(dpi_hact, bpp, 0);

	dsi_cfg->hfp = dpi_to_dsi_timing(dpi_hfp, bpp, DSI_HFP_FRAME_OVERHEAD);

	dsi_cfg->htotal = dsi_cfg->hact + dsi_cfg->hfp + DSI_HFP_FRAME_OVERHEAD;

	if (sync_pulse) {
		dsi_cfg->htotal += dsi_cfg->hbp + DSI_HBP_FRAME_PULSE_OVERHEAD;
		dsi_cfg->htotal += dsi_cfg->hsa + DSI_HSA_FRAME_OVERHEAD;
	} else {
		dsi_cfg->htotal += dsi_cfg->hbp + DSI_HBP_FRAME_EVENT_OVERHEAD;
	}

	return 0;
}

static int cdns_dsi_check_conf(struct cdns_dsi *dsi,
			       struct display_timing *timings,
			       struct cdns_dsi_cfg *dsi_cfg)
{
	struct cdns_dsi_output *output = &dsi->output;
	struct phy_configure_opts_mipi_dphy *phy_cfg = &output->phy_opts;
	unsigned int nlanes = output->dev->lanes;
	int ret;

	ret = cdns_dsi_mode2cfg(dsi, timings, dsi_cfg);
	if (ret)
		return ret;

	ret = phy_mipi_dphy_get_default_config(timings->pixelclock.typ * 1000,
					       mipi_dsi_pixel_format_to_bpp(output->dev->format),
					       nlanes, phy_cfg);
	if (ret) {
		pr_err("%s: phy_mipi_dphy_get_default_config failed: %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int cdns_dsi_hs_init(void *priv_data)
{
	struct mipi_dsi_device *device = priv_data;
	struct cdns_dsi *dsi = to_cdns_dsi(device->host);

	if (dsi->phy_initialized)
		return 0;

	writel(DPHY_CMN_PSO | DPHY_PLL_PSO | DPHY_ALL_D_PDN | DPHY_C_PDN |
	       DPHY_CMN_PDN | DPHY_PLL_PDN,
	       dsi->base + MCTL_DPHY_CFG0);

	generic_phy_init(&dsi->dphy);
	generic_phy_set_mode(&dsi->dphy, PHY_MODE_MIPI_DPHY, 0);

	dsi->phy_initialized = true;

	return 0;
}

static int cdns_dsi_get_lane_mbps(void *priv_data, struct display_timing *timings,
				  u32 lanes, u32 format, unsigned int *lane_mbps)
{
	struct mipi_dsi_device *device = priv_data;
	struct cdns_dsi *dsi = to_cdns_dsi(device->host);
	struct cdns_dsi_host_priv *hpriv = container_of(dsi, struct cdns_dsi_host_priv, dsi);
	struct phy_configure_opts_mipi_dphy *phy_cfg = &dsi->output.phy_opts;
	unsigned long orig_pclk_hz = timings->pixelclock.typ * 1000UL;
	unsigned long actual_pclk_hz;
	int bpp, ret;

	bpp = mipi_dsi_pixel_format_to_bpp(format);
	if (bpp < 0)
		return bpp;

	ret = phy_mipi_dphy_get_default_config(orig_pclk_hz, bpp, lanes, phy_cfg);
	if (ret) {
		pr_err("%s: failed to get DPHY default config: %d\n", __func__, ret);
		return ret;
	}

	ret = generic_phy_validate(&dsi->dphy, PHY_MODE_MIPI_DPHY, 0, phy_cfg);
	if (ret) {
		pr_err("%s: failed to validate DPHY config: %d\n", __func__, ret);
		return ret;
	}

	actual_pclk_hz = (unsigned long)div_u64((u64)phy_cfg->hs_clk_rate * lanes, bpp);
	timings->pixelclock.typ = actual_pclk_hz / 1000;
	hpriv->rounded_pclk_khz = timings->pixelclock.typ;

	ret = phy_mipi_dphy_get_default_config(actual_pclk_hz, bpp, lanes, phy_cfg);
	if (ret) {
		pr_err("%s: failed to recompute DPHY config for rounded pclk: %d\n", __func__, ret);
		return ret;
	}

	*lane_mbps = DIV_ROUND_UP(phy_cfg->hs_clk_rate, 1000000);

	return 0;
}

static const struct mipi_dsi_phy_ops dsi_cdns_phy_ops = {
	.init = cdns_dsi_hs_init,
	.get_lane_mbps = cdns_dsi_get_lane_mbps,
};

static int cdns_dsi_init_link(struct cdns_dsi *dsi)
{
	struct cdns_dsi_output *output = &dsi->output;
	unsigned long sysclk_rate, sysclk_period, ulpout;
	u32 val;
	int i;

	if (dsi->link_initialized)
		return 0;

	val = 0;
	for (i = 1; i < output->dev->lanes; i++)
		val |= DATA_LANE_EN(i);

	if (!(output->dev->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS))
		val |= CLK_CONTINUOUS;

	writel(val, dsi->base + MCTL_MAIN_PHY_CTL);

	/* ULPOUT should be set to 1ms and is expressed in sysclk cycles. */
	sysclk_rate = clk_get_rate(dsi->dsi_sys_clk);
	if (!sysclk_rate)
		return -EINVAL;

	sysclk_period = NSEC_PER_SEC / sysclk_rate;
	ulpout = DIV_ROUND_UP(NSEC_PER_MSEC, sysclk_period);
	writel(CLK_LANE_ULPOUT_TIME(ulpout) | DATA_LANE_ULPOUT_TIME(ulpout),
	       dsi->base + MCTL_ULPOUT_TIME);

	writel(LINK_EN, dsi->base + MCTL_MAIN_DATA_CTL);
	val = CLK_LANE_EN | PLL_START;
	for (i = 0; i < output->dev->lanes; i++)
		val |= DATA_LANE_START(i);

	writel(val, dsi->base + MCTL_MAIN_EN);
	dsi->link_initialized = true;

	return 0;
}

static int cdns_dsi_attach(struct mipi_dsi_host *host,
			   struct mipi_dsi_device *dev)
{
	struct cdns_dsi *dsi = to_cdns_dsi(host);
	struct cdns_dsi_output *output = &dsi->output;

	if (output->dev)
		return -EBUSY;

	output->dev = dev;
	dev->host = &dsi->host;

	return 0;
}

static int cdns_dsi_host_attach(struct udevice *dev,
				struct mipi_dsi_device *device)
{
	struct cdns_dsi_host_priv *priv = dev_get_priv(dev);
	struct cdns_dsi *dsi = &priv->dsi;

	return cdns_dsi_attach(&dsi->host, device);
}

static ssize_t cdns_dsi_transfer(struct mipi_dsi_host *host,
				 const struct mipi_dsi_msg *msg)
{
	struct cdns_dsi *dsi = to_cdns_dsi(host);
	u32 cmd, sts, val, wait = WRITE_COMPLETED, ctl = 0;
	struct mipi_dsi_packet packet;
	int ret, i, tx_len, rx_len;

	ret = cdns_dsi_init_link(dsi);
	if (ret)
		goto out;

	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret)
		goto out;

	tx_len = msg->tx_buf ? msg->tx_len : 0;
	rx_len = msg->rx_buf ? msg->rx_len : 0;

	/* For read operations, the maximum TX len is 2. */
	if (rx_len && tx_len > 2) {
		ret = -EOPNOTSUPP;
		goto out;
	}

	/* TX len is limited by the CMD FIFO depth. */
	if (tx_len > dsi->direct_cmd_fifo_depth) {
		ret = -EOPNOTSUPP;
		goto out;
	}

	/* RX len is limited by the RX FIFO depth. */
	if (rx_len > dsi->rx_fifo_depth) {
		ret = -EOPNOTSUPP;
		goto out;
	}

	cmd = CMD_SIZE(tx_len) | CMD_VCHAN_ID(msg->channel) |
	      CMD_DATATYPE(msg->type);

	if (msg->flags & MIPI_DSI_MSG_USE_LPM)
		cmd |= CMD_LP_EN;

	if (mipi_dsi_packet_format_is_long(msg->type))
		cmd |= CMD_LONG;

	if (rx_len) {
		cmd |= READ_CMD;
		wait = READ_COMPLETED_WITH_ERR | READ_COMPLETED;
		ctl = READ_EN | BTA_EN;
	} else if (msg->flags & MIPI_DSI_MSG_REQ_ACK) {
		cmd |= BTA_REQ;
		wait = ACK_WITH_ERR_RCVD | ACK_RCVD;
		ctl = BTA_EN;
	}

	writel(readl(dsi->base + MCTL_MAIN_DATA_CTL) | ctl,
	       dsi->base + MCTL_MAIN_DATA_CTL);

	writel(cmd, dsi->base + DIRECT_CMD_MAIN_SETTINGS);

	for (i = 0; i < tx_len; i += 4) {
		const u8 *buf = msg->tx_buf;
		int j;

		val = 0;
		for (j = 0; j < 4 && j + i < tx_len; j++)
			val |= (u32)buf[i + j] << (8 * j);

		writel(val, dsi->base + DIRECT_CMD_WRDATA);
	}

	/* Clear status flags before sending the command. */
	writel(wait, dsi->base + DIRECT_CMD_STS_CLR);
	writel(0, dsi->base + DIRECT_CMD_STS_CTL);
	writel(0, dsi->base + DIRECT_CMD_SEND);

	ret = readl_poll_timeout(dsi->base + DIRECT_CMD_STS, sts,
				 sts & wait, 1000000);
	if (ret) {
		ret = -ETIMEDOUT;
		goto out;
	}

	writel(wait, dsi->base + DIRECT_CMD_STS_CLR);
	writel(0, dsi->base + DIRECT_CMD_STS_CTL);

	writel(readl(dsi->base + MCTL_MAIN_DATA_CTL) & ~ctl,
	       dsi->base + MCTL_MAIN_DATA_CTL);

	/* 'READ' or 'WRITE with ACK' failed. */
	if (sts & (READ_COMPLETED_WITH_ERR | ACK_WITH_ERR_RCVD)) {
		ret = -EIO;
		goto out;
	}

	for (i = 0; i < rx_len; i += 4) {
		u8 *buf = msg->rx_buf;
		int j;

		val = readl(dsi->base + DIRECT_CMD_RDDATA);
		for (j = 0; j < 4 && j + i < rx_len; j++)
			buf[i + j] = val >> (8 * j);
	}

out:
	return ret;
}

static const struct mipi_dsi_host_ops cdns_dsi_host_ops = {
	.transfer = cdns_dsi_transfer,
};

static int cdns_dsi_host_init(struct udevice *dev,
			      struct mipi_dsi_device *device,
			      struct display_timing *timings,
			      unsigned int max_data_lanes,
			      const struct mipi_dsi_phy_ops *phy_ops)
{
	struct cdns_dsi_host_priv *priv = dev_get_priv(dev);
	struct cdns_dsi *dsi = &priv->dsi;
	int ret;

	if (!device)
		return -ENODEV;

	if (!timings || !max_data_lanes)
		return -EINVAL;

	if (!phy_ops)
		priv->phy_ops = &dsi_cdns_phy_ops;
	else
		priv->phy_ops = phy_ops;

	if (!priv->phy_ops->init || !priv->phy_ops->get_lane_mbps)
		return -EINVAL;

	priv->max_data_lanes = max_data_lanes;
	priv->timings = timings;
	priv->device = device;

	dsi->link_initialized = false;
	dsi->phy_initialized = false;

	writel(0, dsi->base + MCTL_MAIN_EN);
	writel(0, dsi->base + MCTL_MAIN_DATA_CTL);
	writel(0, dsi->base + MCTL_MAIN_PHY_CTL);
	writel(DPHY_CMN_PDN | DPHY_C_PDN | DPHY_ALL_D_PDN |
	       DPHY_PLL_PDN | DPHY_CMN_PSO | DPHY_PLL_PSO,
	       dsi->base + MCTL_DPHY_CFG0);

	/* CDN DSI DPI input only detects active-LOW HSYNC/VSYNC */
	timings->flags &= ~(DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH);
	timings->flags |= DISPLAY_FLAGS_HSYNC_LOW | DISPLAY_FLAGS_VSYNC_LOW;

	dsi->output.dev = device;
	device->host = &dsi->host;

	ret = cdns_dsi_check_conf(dsi, timings, &dsi->dsi_cfg);
	return ret;
}

static int cdns_dsi_setup_video_mode(struct cdns_dsi *dsi,
				     struct display_timing *timings)
{
	struct cdns_dsi_output *output = &dsi->output;
	struct cdns_dsi_cfg *dsi_cfg = &dsi->dsi_cfg;
	struct phy_configure_opts_mipi_dphy *phy_cfg = &output->phy_opts;
	unsigned long tx_byte_period;
	unsigned int nlanes = output->dev->lanes;
	u32 tmp, reg_wakeup, div, status;
	unsigned int vfp, vbp, vsa, vact;
	int ret;

	tmp = CLK_LANE_RDY;
	for (div = 0; div < nlanes; div++)
		tmp |= DATA_LANE_RDY(div);
	ret = readl_poll_timeout(dsi->base + MCTL_MAIN_STS, status,
				 (tmp == (status & tmp)), 500000);
	if (ret) {
		pr_err("%s: Timeout waiting for DSI lanes ready\n", __func__);
		pr_err("  status=0x%08x, expected=0x%08x\n", status, tmp);
		return -ETIMEDOUT;
	}

	writel(HBP_LEN(dsi_cfg->hbp) | HSA_LEN(dsi_cfg->hsa),
	       dsi->base + VID_HSIZE1);
	writel(HFP_LEN(dsi_cfg->hfp) | HACT_LEN(dsi_cfg->hact),
	       dsi->base + VID_HSIZE2);

	vsa = timings->vsync_len.typ;
	vbp = timings->vback_porch.typ;
	vfp = timings->vfront_porch.typ;
	vact = timings->vactive.typ;

	writel(VBP_LEN(vbp) | VFP_LEN(vfp) | VSA_LEN(vsa),
	       dsi->base + VID_VSIZE1);
	writel(vact, dsi->base + VID_VSIZE2);

	tmp = dsi_cfg->htotal -
	      (dsi_cfg->hsa + DSI_BLANKING_FRAME_OVERHEAD +
	       DSI_HSA_FRAME_OVERHEAD);
	writel(BLK_LINE_PULSE_PKT_LEN(tmp), dsi->base + VID_BLKSIZE2);
	if (output->dev->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
		writel(MAX_LINE_LIMIT(tmp - DSI_NULL_FRAME_OVERHEAD),
		       dsi->base + VID_VCA_SETTING2);

	tmp = dsi_cfg->htotal -
	      (DSI_HSS_VSS_VSE_FRAME_OVERHEAD + DSI_BLANKING_FRAME_OVERHEAD);
	writel(BLK_LINE_EVENT_PKT_LEN(tmp), dsi->base + VID_BLKSIZE1);
	if (!(output->dev->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE))
		writel(MAX_LINE_LIMIT(tmp - DSI_NULL_FRAME_OVERHEAD),
		       dsi->base + VID_VCA_SETTING2);

	tmp = DIV_ROUND_UP(dsi_cfg->htotal, nlanes) -
	      DIV_ROUND_UP(dsi_cfg->hsa, nlanes);
	if (!(output->dev->mode_flags & MIPI_DSI_MODE_EOT_PACKET))
		tmp -= DIV_ROUND_UP(DSI_EOT_PKT_SIZE, nlanes);

	tx_byte_period = DIV_ROUND_DOWN_ULL((u64)NSEC_PER_SEC * 8,
					    phy_cfg->hs_clk_rate);

	reg_wakeup = dsi_cfg->htotal / nlanes / 10;
	writel(REG_WAKEUP_TIME(reg_wakeup) | REG_LINE_DURATION(tmp),
	       dsi->base + VID_DPHY_TIME);

	/* HSTX/LPRX timeouts: at least one frame worth of TX byte clk cycles */
	tmp = DIV_ROUND_UP(NSEC_PER_SEC, (timings->pixelclock.typ * 1000) /
			   ((timings->hactive.typ + timings->hfront_porch.typ +
			     timings->hback_porch.typ + timings->hsync_len.typ) *
			    (timings->vactive.typ + timings->vfront_porch.typ +
			     timings->vback_porch.typ + timings->vsync_len.typ)));
	tmp /= tx_byte_period;

	for (div = 0; div <= CLK_DIV_MAX; div++) {
		if (tmp <= HSTX_TIMEOUT_MAX)
			break;
		tmp >>= 1;
	}
	if (tmp > HSTX_TIMEOUT_MAX)
		tmp = HSTX_TIMEOUT_MAX;

	writel(CLK_DIV(div) | HSTX_TIMEOUT(tmp), dsi->base + MCTL_DPHY_TIMEOUT1);
	writel(LPRX_TIMEOUT(tmp), dsi->base + MCTL_DPHY_TIMEOUT2);

	if (output->dev->mode_flags & MIPI_DSI_MODE_VIDEO) {
		switch (output->dev->format) {
		case MIPI_DSI_FMT_RGB888:
			tmp = VID_PIXEL_MODE_RGB888 |
			      VID_DATATYPE(MIPI_DSI_PACKED_PIXEL_STREAM_24);
			break;
		case MIPI_DSI_FMT_RGB666:
			tmp = VID_PIXEL_MODE_RGB666 |
			      VID_DATATYPE(MIPI_DSI_PIXEL_STREAM_3BYTE_18);
			break;
		case MIPI_DSI_FMT_RGB666_PACKED:
			tmp = VID_PIXEL_MODE_RGB666_PACKED |
			      VID_DATATYPE(MIPI_DSI_PACKED_PIXEL_STREAM_18);
			break;
		case MIPI_DSI_FMT_RGB565:
			tmp = VID_PIXEL_MODE_RGB565 |
			      VID_DATATYPE(MIPI_DSI_PACKED_PIXEL_STREAM_16);
			break;
		default:
			pr_err("%s: unsupported DSI format\n", __func__);
			return -EINVAL;
		}

		if (output->dev->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
			tmp |= SYNC_PULSE_ACTIVE | SYNC_PULSE_HORIZONTAL;

		tmp |= REG_BLKLINE_MODE(REG_BLK_MODE_BLANKING_PKT) |
		       REG_BLKEOL_MODE(REG_BLK_MODE_BLANKING_PKT) |
		       RECOVERY_MODE(RECOVERY_MODE_NEXT_HSYNC) |
		       VID_IGNORE_MISS_VSYNC;

		writel(tmp, dsi->base + VID_MAIN_CTL);
	}

	tmp = readl(dsi->base + MCTL_MAIN_DATA_CTL);
	tmp &= ~(IF_VID_SELECT_MASK | HOST_EOT_GEN | IF_VID_MODE);
	if (!(output->dev->mode_flags & MIPI_DSI_MODE_EOT_PACKET))
		tmp |= HOST_EOT_GEN;
	writel(tmp, dsi->base + MCTL_MAIN_DATA_CTL);

	return 0;
}

static int cdns_dsi_host_enable(struct udevice *dev)
{
	struct cdns_dsi_host_priv *priv = dev_get_priv(dev);
	struct cdns_dsi *dsi = &priv->dsi;
	struct cdns_dsi_output *output = &dsi->output;
	unsigned int lane_mbps;
	u32 tmp, status;
	int ret;

	ret = cdns_dsi_init_link(dsi);
	if (ret)
		return ret;

	ret = priv->phy_ops->init(priv->device);
	if (ret)
		return ret;

	ret = priv->phy_ops->get_lane_mbps(priv->device, priv->timings, priv->device->lanes,
					   priv->device->format, &lane_mbps);
	if (ret)
		return ret;

	ret = generic_phy_configure(&dsi->dphy, &dsi->output.phy_opts);
	if (ret)
		return ret;

	ret = generic_phy_power_on(&dsi->dphy);
	if (ret)
		return ret;

	/* Activate the PLL and wait until it's locked. */
	writel(PLL_LOCKED, dsi->base + MCTL_MAIN_STS_CLR);
	writel(DPHY_CMN_PSO | DPHY_ALL_D_PDN | DPHY_C_PDN | DPHY_CMN_PDN,
	       dsi->base + MCTL_DPHY_CFG0);

	ret = readl_poll_timeout(dsi->base + MCTL_MAIN_STS, status,
				 status & PLL_LOCKED, 100000);
	if (ret) {
		pr_err("%s: PLL lock timeout (MCTL_MAIN_STS=0x%08x)\n",
		       __func__, readl(dsi->base + MCTL_MAIN_STS));
		return -ETIMEDOUT;
	}

	/* De-assert data and clock reset lines. */
	writel(DPHY_CMN_PSO | DPHY_ALL_D_PDN | DPHY_C_PDN | DPHY_CMN_PDN |
	       DPHY_D_RSTB(dsi->output.dev->lanes) | DPHY_C_RSTB,
	       dsi->base + MCTL_DPHY_CFG0);

	ret = cdns_dsi_setup_video_mode(dsi, priv->timings);
	if (ret)
		return ret;

	if (dsi->platform_ops && dsi->platform_ops->enable)
		dsi->platform_ops->enable(dsi);

	if (output->dev->mode_flags & MIPI_DSI_MODE_VIDEO) {
		tmp = readl(dsi->base + MCTL_MAIN_DATA_CTL);
		tmp |= IF_VID_MODE | IF_VID_SELECT(dsi->input.id) | VID_EN;
		writel(tmp, dsi->base + MCTL_MAIN_DATA_CTL);

		/* Clear video mode status to start with a clean slate */
		writel(0xFFFFFFFF, dsi->base + VID_MODE_STS_CLR);
	}

	tmp = readl(dsi->base + MCTL_MAIN_EN) | IF_EN(dsi->input.id);
	writel(tmp, dsi->base + MCTL_MAIN_EN);

	return 0;
}

static struct dsi_host_ops cdns_dsi_ops = {
	.attach = cdns_dsi_host_attach,
	.init = cdns_dsi_host_init,
	.enable = cdns_dsi_host_enable,
};

static int cdns_dsi_of_to_plat(struct udevice *dev)
{
	struct cdns_dsi_host_priv *priv = dev_get_priv(dev);
	struct cdns_dsi *dsi = &priv->dsi;

	dsi->base = dev_read_addr_ptr(dev);
	if (!dsi->base) {
		dev_err(dev, "failed to get base address\n");
		return -EINVAL;
	}

	return 0;
}

static int cdns_dsi_probe(struct udevice *dev)
{
	struct cdns_dsi_host_priv *priv = dev_get_priv(dev);
	struct cdns_dsi *dsi = &priv->dsi;
	int ret;
	u32 val;

	dsi->dsi_p_clk = devm_clk_get(dev, "dsi_p_clk");
	if (IS_ERR(dsi->dsi_p_clk)) {
		dev_err(dev, "%s: failed to get dsi_p_clk\n", __func__);
		return PTR_ERR(dsi->dsi_p_clk);
	}

	ret = reset_get_by_name(dev, "dsi_p_rst", &dsi->dsi_p_rst);
	if (ret && ret != -ENOENT && ret != -EOPNOTSUPP && ret != -ENOTSUPP) {
		dev_err(dev, "%s: failed to get reset: %d\n", __func__, ret);
		return ret;
	}

	dsi->dsi_sys_clk = devm_clk_get(dev, "dsi_sys_clk");
	if (IS_ERR(dsi->dsi_sys_clk)) {
		dev_err(dev, "%s: failed to get dsi_sys_clk\n", __func__);
		return PTR_ERR(dsi->dsi_sys_clk);
	}

	ret = generic_phy_get_by_name(dev, "dphy", &dsi->dphy);
	if (ret) {
		dev_err(dev, "%s: failed to get dphy: %d\n", __func__, ret);
		return ret;
	}

	if (reset_valid(&dsi->dsi_p_rst))
		reset_assert(&dsi->dsi_p_rst);

	ret = clk_enable(dsi->dsi_p_clk);
	if (ret)
		goto err_no_clk;

	ret = clk_enable(dsi->dsi_sys_clk);
	if (ret)
		goto err_disable_p_clk;

	if (reset_valid(&dsi->dsi_p_rst))
		reset_deassert(&dsi->dsi_p_rst);

	val = readl(dsi->base + ID_REG);
	if (REV_VENDOR_ID(val) != 0xcad) {
		dev_err(dev, "invalid vendor id 0x%03lx\n",
			REV_VENDOR_ID(val));
		ret = -ENODEV;
		goto err_disable_sys_clk;
	}

	dsi->platform_ops = (const struct cdns_dsi_platform_ops *)dev_get_driver_data(dev);

	val = readl(dsi->base + IP_CONF);
	dsi->direct_cmd_fifo_depth = 1 << (DIRCMD_FIFO_DEPTH(val) + 2);
	dsi->rx_fifo_depth = RX_FIFO_DEPTH(val);

	writel(0, dsi->base + MCTL_MAIN_DATA_CTL);
	writel(0, dsi->base + MCTL_MAIN_EN);
	writel(0, dsi->base + MCTL_MAIN_PHY_CTL);

	dsi->input.id = CDNS_DPI_INPUT;

	/* Mask all interrupts before registering the IRQ handler. */
	writel(0, dsi->base + MCTL_MAIN_STS_CTL);
	writel(0, dsi->base + MCTL_DPHY_ERR_CTL1);
	writel(0, dsi->base + CMD_MODE_STS_CTL);
	writel(0, dsi->base + DIRECT_CMD_STS_CTL);
	writel(0, dsi->base + DIRECT_CMD_RD_STS_CTL);
	writel(0, dsi->base + VID_MODE_STS_CTL);
	writel(0, dsi->base + DPI_IRQ_EN);

	dsi->host.dev = (struct device *)dev;
	dsi->host.ops = &cdns_dsi_host_ops;

	if (dsi->platform_ops && dsi->platform_ops->init) {
		ret = dsi->platform_ops->init(dsi);
		if (ret) {
			dev_err(dev, "Platform initialization failed: %d\n", ret);
			goto err_disable_sys_clk;
		}
	}

	return 0;

err_disable_sys_clk:
	clk_disable(dsi->dsi_sys_clk);

err_disable_p_clk:
	clk_disable(dsi->dsi_p_clk);

err_no_clk:
	return ret;
}

static int cdns_dsi_remove(struct udevice *dev)
{
	struct cdns_dsi_host_priv *priv = dev_get_priv(dev);
	struct cdns_dsi *dsi = &priv->dsi;

	if (dsi->platform_ops) {
		if (dsi->platform_ops->disable)
			dsi->platform_ops->disable(dsi);

		if (dsi->platform_ops->deinit)
			dsi->platform_ops->deinit(dsi);
	}

	clk_disable(dsi->dsi_sys_clk);
	clk_disable(dsi->dsi_p_clk);

	return 0;
}

static const struct udevice_id cdns_dsi_of_match[] = {
	{ .compatible = "cdns,dsi" },
#if defined(CONFIG_CDNS_DSI_J721E) || defined(CONFIG_SPL_CDNS_DSI_J721E)
	{ .compatible = "ti,j721e-dsi", .data = (ulong)&dsi_ti_j721e_ops, },
#endif
	{ },
};

U_BOOT_DRIVER(cdns_dsi_core) = {
	.name				= "cdns-display-dsi",
	.id				= UCLASS_DSI_HOST,
	.of_match			= cdns_dsi_of_match,
	.bind				= dm_scan_fdt_dev,
	.probe				= cdns_dsi_probe,
	.remove				= cdns_dsi_remove,
	.of_to_plat			= cdns_dsi_of_to_plat,
	.ops				= &cdns_dsi_ops,
	.priv_auto			= sizeof(struct cdns_dsi_host_priv),
};
