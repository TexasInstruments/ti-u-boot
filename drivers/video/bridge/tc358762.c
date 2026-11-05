// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Marek Vasut <marex@denx.de>
 *
 * Based on tc358764.c by
 *  Andrzej Hajda <a.hajda@samsung.com>
 *  Maciej Purski <m.purski@samsung.com>
 *
 * Based on rpi_touchscreen.c by
 *  Eric Anholt <eric@anholt.net>
 *
 * Ported to U-Boot by Rahul Sharma <r-sharma3@ti.com>
 * Copyright (C) 2026 Texas Instruments Incorporated - https://www.ti.com/
 */

#include <asm/gpio.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/ofnode_graph.h>
#include <dsi_host.h>
#include <log.h>
#include <mipi_dsi.h>
#include <panel.h>
#include <power/regulator.h>
#include <video_bridge.h>
#include <linux/delay.h>

/* PPI layer registers */
#define PPI_STARTPPI		0x0104 /* START control bit */
#define PPI_LPTXTIMECNT		0x0114 /* LPTX timing signal */
#define PPI_D0S_ATMR		0x0144
#define PPI_D1S_ATMR		0x0148
#define PPI_D0S_CLRSIPOCOUNT	0x0164 /* Assertion timer for Lane 0 */
#define PPI_D1S_CLRSIPOCOUNT	0x0168 /* Assertion timer for Lane 1 */
#define PPI_START_FUNCTION	1

/* DSI layer registers */
#define DSI_STARTDSI		0x0204 /* START control bit of DSI-TX */
#define DSI_LANEENABLE		0x0210 /* Enables each lane */
#define DSI_RX_START		1

/* LCDC/DPI Host Registers */
#define LCDCTRL			0x0420 /* Video Path Control */
#define LCDCTRL_VTGEN		BIT(4) /* Use chip clock for timing */
#define LCDCTRL_UNK6		BIT(6) /* Unknown */
#define LCDCTRL_RGB888		BIT(8) /* RGB888 mode */
#define LCDCTRL_HSPOL		BIT(17) /* Polarity of HSYNC signal */
#define LCDCTRL_VSPOL		BIT(19) /* Polarity of VSYNC signal */
#define LCDCTRL_VSDELAY(v)	(((v) & 0xfff) << 20) /* VSYNC delay */

/* SPI Master Registers */
#define SPICMR			0x0450

/* System Controller Registers */
#define SYSCTRL			0x0464

/* System registers */
#define LPX_PERIOD		3

/* Lane enable PPI and DSI register bits */
#define LANEENABLE_CLEN		BIT(0)
#define LANEENABLE_L0EN		BIT(1)

struct tc358762_priv {
	struct mipi_dsi_device device;
	struct udevice *panel;
	struct display_timing timing;
	struct udevice *supply;
	struct gpio_desc led_en;
	int error;
};

static void tc358762_write(struct udevice *dev, u16 addr, u32 val)
{
	struct tc358762_priv *priv = dev_get_priv(dev);
	u8 data[6];
	ssize_t ret;

	if (priv->error)
		return;

	data[0] = addr;
	data[1] = addr >> 8;
	data[2] = val;
	data[3] = val >> 8;
	data[4] = val >> 16;
	data[5] = val >> 24;

	ret = mipi_dsi_generic_write(&priv->device, data, sizeof(data));
	if (ret < 0)
		priv->error = ret;
}

static int tc358762_clear_error(struct udevice *dev)
{
	struct tc358762_priv *priv = dev_get_priv(dev);
	int ret = priv->error;

	priv->error = 0;
	return ret;
}

static int tc358762_init(struct udevice *dev)
{
	u32 lcdctrl;

	tc358762_write(dev, DSI_LANEENABLE,
		       LANEENABLE_L0EN | LANEENABLE_CLEN);
	tc358762_write(dev, PPI_D0S_CLRSIPOCOUNT, 5);
	tc358762_write(dev, PPI_D1S_CLRSIPOCOUNT, 5);
	tc358762_write(dev, PPI_D0S_ATMR, 0);
	tc358762_write(dev, PPI_D1S_ATMR, 0);
	tc358762_write(dev, PPI_LPTXTIMECNT, LPX_PERIOD);

	tc358762_write(dev, SPICMR, 0x00);

	lcdctrl = LCDCTRL_VSDELAY(1) | LCDCTRL_RGB888 |
		  LCDCTRL_UNK6 | LCDCTRL_VTGEN;

	tc358762_write(dev, LCDCTRL, lcdctrl);

	tc358762_write(dev, SYSCTRL, 0x040f);
	mdelay(100);

	tc358762_write(dev, PPI_STARTPPI, PPI_START_FUNCTION);
	tc358762_write(dev, DSI_STARTDSI, DSI_RX_START);
	mdelay(100);

	return tc358762_clear_error(dev);
}

static int tc358762_attach(struct udevice *dev)
{
	struct tc358762_priv *priv = dev_get_priv(dev);
	struct video_bridge_priv *uc_priv = dev_get_uclass_priv(dev);
	int ret;

	if (dm_gpio_is_valid(&uc_priv->reset)) {
		dm_gpio_set_value(&uc_priv->reset, 1);
		mdelay(5);
	}

	ret = regulator_set_enable_if_allowed(priv->supply, true);
	if (ret) {
		log_debug("%s: error enabling supply (%d)\n", __func__, ret);
		return ret;
	}

	ret = dsi_host_init(dev->parent, &priv->device, &priv->timing, 1, NULL);
	if (ret) {
		dev_err(dev, "%s: dsi_host_init failed: %d\n", __func__, ret);
		return ret;
	}

	ret = dsi_host_enable(dev->parent);
	if (ret) {
		dev_err(dev, "%s: dsi_host_enable failed: %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int tc358762_enable(struct udevice *dev)
{
	struct tc358762_priv *priv = dev_get_priv(dev);
	int ret;

	ret = tc358762_init(dev);
	if (ret) {
		dev_err(dev, "%s: bridge init failed: %d\n", __func__, ret);
		return ret;
	}

	if (dm_gpio_is_valid(&priv->led_en))
		dm_gpio_set_value(&priv->led_en, 1);

	ret = panel_enable_backlight(priv->panel);
	if (ret == -ENOSYS)
		ret = 0;
	return ret;
}

static int tc358762_set_backlight(struct udevice *dev, int percent)
{
	struct tc358762_priv *priv = dev_get_priv(dev);

	return panel_set_backlight(priv->panel, percent);
}

static int tc358762_get_display_timing(struct udevice *dev,
				       struct display_timing *timing)
{
	struct tc358762_priv *priv = dev_get_priv(dev);

	memcpy(timing, &priv->timing, sizeof(*timing));
	return 0;
}

static int tc358762_get_panel(struct udevice *dev)
{
	struct tc358762_priv *priv = dev_get_priv(dev);
	int i, ret;
	u32 num;

	num = ofnode_graph_get_port_count(dev_ofnode(dev));

	for (i = 0; i < num; i++) {
		ofnode remote = ofnode_graph_get_remote_node(dev_ofnode(dev), i, -1);

		ret = uclass_get_device_by_ofnode(UCLASS_PANEL, remote, &priv->panel);
		if (!ret)
			return 0;
	}

	return -ENODEV;
}

static int tc358762_probe(struct udevice *dev)
{
	struct tc358762_priv *priv = dev_get_priv(dev);
	int ret;

	if (device_get_uclass_id(dev->parent) != UCLASS_DSI_HOST) {
		dev_err(dev, "%s: parent is not UCLASS_DSI_HOST\n", __func__);
		return -EPROTONOSUPPORT;
	}

	ret = tc358762_get_panel(dev);
	if (ret) {
		dev_err(dev, "%s: panel not found: %d\n", __func__, ret);
		return ret;
	}

	ret = panel_get_display_timing(priv->panel, &priv->timing);
	if (ret) {
		dev_err(dev, "%s: failed to get display timing: %d\n", __func__, ret);
		return ret;
	}

	priv->device.dev = dev;
	priv->device.lanes = 1;
	priv->device.format = MIPI_DSI_FMT_RGB888;
	priv->device.mode_flags = MIPI_DSI_MODE_VIDEO |
				   MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
				   MIPI_DSI_MODE_LPM |
				   MIPI_DSI_MODE_VIDEO_HSE;

	ret = device_get_supply_regulator(dev, "vddc-supply", &priv->supply);
	if (ret && ret != -ENOENT) {
		log_debug("%s: cannot get vddc supply: %d\n", __func__, ret);
		return ret;
	}

	gpio_request_by_name(dev, "led-gpios", 0, &priv->led_en, GPIOD_IS_OUT);

	return 0;
}

static const struct video_bridge_ops tc358762_ops = {
	.attach			= tc358762_attach,
	.enable			= tc358762_enable,
	.set_backlight		= tc358762_set_backlight,
	.get_display_timing	= tc358762_get_display_timing,
};

static const struct udevice_id tc358762_ids[] = {
	{ .compatible = "toshiba,tc358762" },
	{ }
};

U_BOOT_DRIVER(tc358762) = {
	.name		= "tc358762",
	.id		= UCLASS_VIDEO_BRIDGE,
	.of_match	= tc358762_ids,
	.ops		= &tc358762_ops,
	.bind		= dm_scan_fdt_dev,
	.probe		= tc358762_probe,
	.priv_auto	= sizeof(struct tc358762_priv),
};
