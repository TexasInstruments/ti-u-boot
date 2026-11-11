// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2012
 * Altera Corporation <www.altera.com>
 */

#include <clk.h>
#include <log.h>
#include <dm.h>
#include <dma.h>
#include <fdtdec.h>
#include <limits.h>
#include <malloc.h>
#include <reset.h>
#include <spi.h>
#include <spi-mem.h>
#include <dm/device_compat.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/sizes.h>
#include <linux/time.h>
#include <zynqmp_firmware.h>
#include "cadence_qspi.h"
#include <dt-bindings/power/xlnx-versal-power.h>

#define CQSPI_STIG_READ			0
#define CQSPI_STIG_WRITE		1
#define CQSPI_READ			2
#define CQSPI_WRITE			3

/* Quirks */
#define CQSPI_DISABLE_STIG_MODE		BIT(0)
#define CQSPI_NO_SUPPORT_WR_COMPLETION	BIT(2)

__weak int cadence_qspi_apb_dma_read(struct cadence_spi_priv *priv,
				     const struct spi_mem_op *op)
{
	return 0;
}

__weak int cadence_device_reset(struct udevice *dev)
{
	return 0;
}

__weak int cadence_qspi_flash_reset(struct udevice *dev)
{
	return 0;
}

__weak ofnode cadence_qspi_get_subnode(struct udevice *dev)
{
	return dev_read_first_subnode(dev);
}

static int cadence_spi_phy_apply_setting(struct cadence_spi_priv *priv,
					 struct phy_setting *phy)
{
	unsigned int reg;

	reg = readl(priv->regbase + CQSPI_REG_RD_DATA_CAPTURE);
	reg |= CQSPI_REG_RD_DATA_CAPTURE_SMPL_EDGE;
	writel(reg, priv->regbase + CQSPI_REG_RD_DATA_CAPTURE);

	cadence_qspi_apb_set_rx_dll(priv->regbase, phy->rx);
	cadence_qspi_apb_set_tx_dll(priv->regbase, phy->tx);
	priv->phy_read_delay = phy->read_delay;

	return cadence_qspi_apb_resync_dll(priv->regbase);
}

static int cadence_spi_phy_check_pattern(struct cadence_spi_priv *priv,
					 struct spi_slave *spi)
{
	struct spi_mem_op op = priv->phy_read_op;
	u8 *read_data;
	int ret;

	read_data = kmalloc(ARRAY_SIZE(phy_tuning_pattern), 0);
	if (!read_data)
		return -ENOMEM;

	op.data.buf.in = read_data;
	op.addr.val = priv->phy_pattern_start;
	op.data.nbytes = ARRAY_SIZE(phy_tuning_pattern);

	ret = spi_mem_exec_op(spi, &op);
	if (ret)
		goto out;

	if (memcmp(read_data, phy_tuning_pattern,
		   ARRAY_SIZE(phy_tuning_pattern))) {
		ret = -EAGAIN;
		goto out;
	}

	ret = 0;
out:
	kfree(read_data);
	return ret;
}

static int cadence_spi_find_rx_low_sdr(struct cadence_spi_priv *priv,
				       struct spi_slave *spi,
				       struct phy_setting *phy)
{
	int ret;

	phy->rx = 0;
	do {
		ret = cadence_spi_phy_apply_setting(priv, phy);
		if (!ret) {
			ret = cadence_spi_phy_check_pattern(priv, spi);
			if (!ret)
				return 0;
		}
		phy->rx++;
	} while (phy->rx < CQSPI_PHY_MAX_DELAY - 1);

	debug("Unable to find RX low\n");
	return -ENOENT;
}

static int cadence_spi_find_rx_high_sdr(struct cadence_spi_priv *priv,
					struct spi_slave *spi,
					struct phy_setting *phy,
					u8 lowerbound)
{
	int ret;

	phy->rx = CQSPI_PHY_MAX_DELAY;
	do {
		ret = cadence_spi_phy_apply_setting(priv, phy);
		if (!ret) {
			ret = cadence_spi_phy_check_pattern(priv, spi);
			if (!ret)
				return 0;
		}
		phy->rx--;
	} while (phy->rx > lowerbound);

	debug("Unable to find RX high\n");
	return -ENOENT;
}

static int cadence_spi_phy_check_consecutive_ddr(struct cadence_spi_priv *priv,
						 struct spi_slave *spi,
						 struct phy_setting start,
						 int target, bool match_pass,
						 bool upward)
{
	struct phy_setting probe = start;
	int step = upward ? 1 : -1;
	int count = 1;
	int ret;

	while (count < target) {
		if (upward ? (probe.tx >= CQSPI_PHY_MAX_DELAY ||
			      probe.rx >= CQSPI_PHY_MAX_DELAY) :
			     (probe.tx == 0 || probe.rx == 0))
			break;
		probe.tx += step;
		probe.rx += step;
		ret = cadence_spi_phy_apply_setting(priv, &probe);
		if (!ret)
			ret = cadence_spi_phy_check_pattern(priv, spi);
		if (!ret == match_pass)
			count++;
		else
			break;
	}

	return count;
}

static void cadence_spi_phy_find_next_match_ddr(struct cadence_spi_priv *priv,
						struct spi_slave *spi,
						struct phy_setting *search,
						const struct phy_setting *limit,
						bool match_pass, bool upward)
{
	int step = upward ? 1 : -1;
	int ret;

	while (upward ? (search->tx < limit->tx && search->rx < limit->rx) :
			(search->tx > limit->tx && search->rx > limit->rx)) {
		search->tx += step;
		search->rx += step;
		ret = cadence_spi_phy_apply_setting(priv, search);
		if (!ret)
			ret = cadence_spi_phy_check_pattern(priv, spi);
		if (!ret == match_pass)
			break;
	}
}

static void cadence_spi_phy_find_biggest_line_ddr(struct cadence_spi_priv *priv,
						  struct spi_slave *spi,
						  struct phy_setting *start,
						  struct phy_setting *end,
						  bool upward)
{
	struct phy_setting probe = *start;
	struct phy_setting best_start = *start;
	int step = upward ? 1 : -1;
	int cur_len = 1, best_len = 1;
	int ret;

	for (;;) {
		if (!upward && (probe.tx == 0 || probe.rx == 0))
			break;
		probe.tx += step;
		probe.rx += step;

		if (upward ? (probe.tx > end->tx || probe.rx > end->rx) :
			     (probe.tx < end->tx || probe.rx < end->rx)) {
			/* Overshot the search limit; rewind to the last tested tap */
			probe.tx -= step;
			probe.rx -= step;
			break;
		}

		ret = cadence_spi_phy_apply_setting(priv, &probe);
		if (!ret)
			ret = cadence_spi_phy_check_pattern(priv, spi);

		if (!ret) {
			cur_len++;
		} else {
			if (cur_len > best_len) {
				best_len = cur_len;
				best_start.tx = probe.tx - step * cur_len;
				best_start.rx = probe.rx - step * cur_len;
			}
			cur_len = 0;
			cadence_spi_phy_find_next_match_ddr(priv, spi, &probe,
							    end, true, upward);
			if (upward ? (probe.tx <= end->tx &&
				      probe.rx <= end->rx) :
				     (probe.tx >= end->tx &&
				      probe.rx >= end->rx))
				cur_len = 1;
		}
	}

	if (cur_len > best_len) {
		best_len = cur_len;
		best_start.tx = probe.tx - step * (cur_len - 1);
		best_start.rx = probe.rx - step * (cur_len - 1);
	}

	start->tx = best_start.tx;
	start->rx = best_start.rx;
	end->tx = start->tx + step * (best_len - 1);
	end->rx = start->rx + step * (best_len - 1);
}

static int cadence_spi_phy_rd_delay_valid_ddr(struct cadence_spi_priv *priv,
					      struct spi_slave *spi,
					      struct phy_setting probe,
					      struct phy_setting end)
{
	int ret = -ENOENT;

	while ((int)probe.tx <= (int)end.tx - CQSPI_PHY_DDR_RD_STEP &&
	       (int)probe.rx <= (int)end.rx - CQSPI_PHY_DDR_RD_STEP && ret) {
		ret = cadence_spi_phy_apply_setting(priv, &probe);
		if (!ret)
			ret = cadence_spi_phy_check_pattern(priv, spi);
		probe.tx += CQSPI_PHY_DDR_RD_STEP;
		probe.rx += CQSPI_PHY_DDR_RD_STEP;
	}

	return ret;
}

static int cadence_spi_phy_find_valid_rd_delays_ddr(struct cadence_spi_priv *priv,
						    struct spi_slave *spi,
						    struct phy_setting diag_start,
						    struct phy_setting diag_end,
						    u8 *min_rd, u8 *max_rd)
{
	struct phy_setting probe;
	u8 read_delay;

	*min_rd = CQSPI_PHY_RD_INVALID;
	*max_rd = CQSPI_PHY_RD_INVALID;

	for (read_delay = CQSPI_PHY_INIT_RD; read_delay <= CQSPI_PHY_MAX_RD;
	     read_delay++) {
		probe = diag_start;
		probe.read_delay = read_delay;
		if (!cadence_spi_phy_rd_delay_valid_ddr(priv, spi, probe,
							diag_end)) {
			if (*min_rd == CQSPI_PHY_RD_INVALID)
				*min_rd = read_delay;
			*max_rd = read_delay;
		}
	}

	return (*min_rd == CQSPI_PHY_RD_INVALID) ? -ENOENT : 0;
}

static int cadence_spi_phy_check_circle_ddr(struct cadence_spi_priv *priv,
					    struct spi_slave *spi,
					    const struct phy_setting *center)
{
	int radius = CQSPI_PHY_DDR_RADIUS;
	int radius_sq = radius * radius;
	int rx_min = (int)center->rx - radius;
	int rx_max = (int)center->rx + radius;
	int tx_min = (int)center->tx - radius;
	int tx_max = (int)center->tx + radius;
	struct phy_setting probe;
	int rx_delta, tx_delta;
	int rx, tx, ret;

	if (rx_max > CQSPI_PHY_MAX_DELAY || rx_min < 0 ||
	    tx_max > CQSPI_PHY_MAX_DELAY || tx_min < 0)
		return -ENOENT;

	/* Full disk of the given radius around center, not just two quadrants */
	for (rx = rx_min; rx <= rx_max; rx++) {
		rx_delta = rx - (int)center->rx;
		for (tx = tx_min; tx <= tx_max; tx++) {
			if (rx == (int)center->rx && tx == (int)center->tx)
				continue;
			tx_delta = tx - (int)center->tx;
			if (rx_delta * rx_delta + tx_delta * tx_delta >
			    radius_sq)
				continue;
			probe = (struct phy_setting){
				.rx = rx,
				.tx = tx,
				.read_delay = center->read_delay
			};
			ret = cadence_spi_phy_apply_setting(priv, &probe);
			if (!ret)
				ret = cadence_spi_phy_check_pattern(priv, spi);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int cadence_spi_phy_find_tuning_point_ddr(struct cadence_spi_priv *priv,
						 struct spi_slave *spi,
						 struct phy_setting *midpt)
{
	struct phy_setting top_left = *midpt, bot_right = *midpt;
	struct phy_setting anti_diag_mid;
	u8 prev_tx, prev_rx;
	int ret;

	/* Walk anti-diagonal top-left (tx--, rx++) until fail */
	while (top_left.tx > 0 && top_left.rx < CQSPI_PHY_MAX_DELAY) {
		prev_tx = top_left.tx;
		prev_rx = top_left.rx;
		top_left.tx--;
		top_left.rx++;
		ret = cadence_spi_phy_apply_setting(priv, &top_left);
		if (!ret)
			ret = cadence_spi_phy_check_pattern(priv, spi);
		if (ret) {
			top_left.tx = prev_tx;
			top_left.rx = prev_rx;
			break;
		}
	}

	/* Walk anti-diagonal bottom-right (tx++, rx--) until fail */
	while (bot_right.tx < CQSPI_PHY_MAX_DELAY && bot_right.rx > 0) {
		prev_tx = bot_right.tx;
		prev_rx = bot_right.rx;
		bot_right.tx++;
		bot_right.rx--;
		ret = cadence_spi_phy_apply_setting(priv, &bot_right);
		if (!ret)
			ret = cadence_spi_phy_check_pattern(priv, spi);
		if (ret) {
			bot_right.tx = prev_tx;
			bot_right.rx = prev_rx;
			break;
		}
	}

	/*
	 * Anti-diagonal span in taps. On this axis |delta_tx| == |delta_rx|
	 * for every step, so the tap count is the tx coordinate difference.
	 */
	if ((int)bot_right.tx - (int)top_left.tx < CQSPI_PHY_DDR_MIN_PASS_TAPS)
		return -ENOENT;

	midpt->tx = (top_left.tx + bot_right.tx) / 2;
	midpt->rx = (top_left.rx + bot_right.rx) / 2;
	anti_diag_mid = *midpt;

	ret = cadence_spi_phy_check_circle_ddr(priv, spi, midpt);
	if (!ret)
		return 0;

	/*
	 * Circle check failed at the anti-diagonal midpoint. Both boundary
	 * distances from anti_diag_mid are equal (or differ by one tap for
	 * odd spans), so try toward top-left first, then toward bottom-right.
	 */
	midpt->tx = (anti_diag_mid.tx + top_left.tx) / 2;
	midpt->rx = (anti_diag_mid.rx + top_left.rx) / 2;
	ret = cadence_spi_phy_check_circle_ddr(priv, spi, midpt);
	if (!ret)
		return 0;

	midpt->tx = (anti_diag_mid.tx + bot_right.tx) / 2;
	midpt->rx = (anti_diag_mid.rx + bot_right.rx) / 2;
	return cadence_spi_phy_check_circle_ddr(priv, spi, midpt);
}

static int cadence_spi_phy_search_diagonal_ddr(struct cadence_spi_priv *priv,
					       struct spi_slave *spi,
					       struct phy_setting *best,
					       struct phy_setting diag_start,
					       struct phy_setting diag_end)
{
	struct phy_setting gap_start = { .read_delay = CQSPI_PHY_RD_INVALID };
	struct phy_setting gap_end = { .read_delay = CQSPI_PHY_RD_INVALID };
	struct phy_setting midpoint = { .read_delay = CQSPI_PHY_RD_INVALID };
	struct phy_setting pass_start, pass_end;
	struct phy_setting low_midpt, high_midpt;
	struct phy_setting *first, *second;
	struct udevice *bus = spi->dev->parent;
	int pass_len, low_len, high_len;
	u8 min_rd, max_rd;
	int run, ret;

	ret = cadence_spi_phy_find_valid_rd_delays_ddr(priv, spi, diag_start,
						       diag_end, &min_rd,
						       &max_rd);
	if (ret)
		return ret;

	dev_dbg(bus, "DDR PHY diagonal: valid read delays %u..%u\n", min_rd,
		max_rd);

	/* Refine pass_start: walk forward at min_rd to first passing point */
	pass_start = diag_start;
	pass_start.read_delay = min_rd;
	for (;;) {
		ret = cadence_spi_phy_apply_setting(priv, &pass_start);
		if (!ret)
			ret = cadence_spi_phy_check_pattern(priv, spi);
		if (!ret)
			break;
		if (pass_start.tx >= CQSPI_PHY_MAX_DELAY ||
		    pass_start.rx >= CQSPI_PHY_MAX_DELAY)
			break;
		pass_start.tx++;
		pass_start.rx++;
	}

	/* Refine pass_end: walk backward at max_rd to first passing point */
	pass_end = diag_end;
	pass_end.read_delay = max_rd;
	for (;;) {
		ret = cadence_spi_phy_apply_setting(priv, &pass_end);
		if (!ret)
			ret = cadence_spi_phy_check_pattern(priv, spi);
		if (!ret)
			break;
		if (pass_end.tx == 0 || pass_end.rx == 0)
			break;
		pass_end.tx--;
		pass_end.rx--;
	}

	if (pass_start.read_delay == pass_end.read_delay) {
		/* Single region: find the longest run, take its midpoint */
		cadence_spi_phy_find_biggest_line_ddr(priv, spi, &pass_start,
						      &pass_end, true);

		/* tx and rx increase equally on the diagonal */
		pass_len = (int)pass_end.tx - (int)pass_start.tx;
		dev_dbg(bus, "DDR PHY diagonal: single region, %d taps\n",
			pass_len);
		if (pass_len < CQSPI_PHY_DDR_MIN_PASS_TAPS)
			return -ENOENT;

		midpoint.tx = (pass_start.tx + pass_end.tx) / 2;
		midpoint.rx = (pass_start.rx + pass_end.rx) / 2;
		midpoint.read_delay = pass_start.read_delay;

		ret = cadence_spi_phy_find_tuning_point_ddr(priv, spi,
							    &midpoint);
		if (!ret)
			*best = midpoint;
		return ret;
	}

	/* Dual regions: locate bottom passing region from pass_start upward */
	do {
		run = cadence_spi_phy_check_consecutive_ddr(priv, spi,
							    pass_start,
							    CQSPI_PHY_DDR_CONSEC_PASS,
							    true, true);
		if (run < CQSPI_PHY_DDR_CONSEC_PASS) {
			int adv = run + 1;

			if (pass_start.tx > CQSPI_PHY_MAX_DELAY - adv)
				adv = CQSPI_PHY_MAX_DELAY - pass_start.tx;
			if (pass_start.rx > CQSPI_PHY_MAX_DELAY - adv)
				adv = CQSPI_PHY_MAX_DELAY - pass_start.rx;

			pass_start.tx += adv;
			pass_start.rx += adv;
			cadence_spi_phy_find_next_match_ddr(priv, spi,
							    &pass_start,
							    &pass_end, true,
							    true);
		}
	} while (run < CQSPI_PHY_DDR_CONSEC_PASS &&
		 pass_start.tx < pass_end.tx);

	if (run < CQSPI_PHY_DDR_CONSEC_PASS) {
		pass_start.read_delay = CQSPI_PHY_RD_INVALID;
	} else {
		/*
		 * Walk past confirmed region to find gap start boundary.
		 * Clamp the jump so tx/rx never step past CQSPI_PHY_MAX_DELAY.
		 */
		int off = CQSPI_PHY_DDR_CONSEC_PASS + 1;

		if (pass_start.tx > CQSPI_PHY_MAX_DELAY - off)
			off = CQSPI_PHY_MAX_DELAY - pass_start.tx;
		if (pass_start.rx > CQSPI_PHY_MAX_DELAY - off)
			off = CQSPI_PHY_MAX_DELAY - pass_start.rx;

		gap_start.rx = pass_start.rx + off;
		gap_start.tx = pass_start.tx + off;
		gap_start.read_delay = pass_start.read_delay;

		for (;;) {
			ret = cadence_spi_phy_apply_setting(priv, &gap_start);
			if (!ret)
				ret = cadence_spi_phy_check_pattern(priv, spi);
			if (ret)
				break;
			if (gap_start.tx >= CQSPI_PHY_MAX_DELAY ||
			    gap_start.rx >= CQSPI_PHY_MAX_DELAY)
				break;
			gap_start.tx++;
			gap_start.rx++;
		}

		do {
			run = cadence_spi_phy_check_consecutive_ddr(priv, spi,
								    gap_start,
								    CQSPI_PHY_DDR_CONSEC_FAIL,
								    false, true);
			if (run < CQSPI_PHY_DDR_CONSEC_FAIL) {
				int adv = run + 1;

				if (gap_start.tx > CQSPI_PHY_MAX_DELAY - adv)
					adv = CQSPI_PHY_MAX_DELAY - gap_start.tx;
				if (gap_start.rx > CQSPI_PHY_MAX_DELAY - adv)
					adv = CQSPI_PHY_MAX_DELAY - gap_start.rx;

				gap_start.tx += adv;
				gap_start.rx += adv;
				cadence_spi_phy_find_next_match_ddr(priv, spi,
								    &gap_start,
								    &pass_end,
								    false,
								    true);
			}
		} while (run < CQSPI_PHY_DDR_CONSEC_FAIL &&
			 gap_start.tx < pass_end.tx);

		/* Narrow pass_start..gap_start to the best passing run */
		cadence_spi_phy_find_biggest_line_ddr(priv, spi, &pass_start,
						      &gap_start, true);
	}

	/* Locate top passing region from pass_end downward */
	do {
		run = cadence_spi_phy_check_consecutive_ddr(priv, spi,
							    pass_end,
							    CQSPI_PHY_DDR_CONSEC_PASS,
							    true, false);
		if (run < CQSPI_PHY_DDR_CONSEC_PASS) {
			int adv = run + 1;

			if (pass_end.tx < adv)
				adv = pass_end.tx;
			if (pass_end.rx < adv)
				adv = pass_end.rx;

			pass_end.tx -= adv;
			pass_end.rx -= adv;
			cadence_spi_phy_find_next_match_ddr(priv, spi,
							    &pass_end,
							    &pass_start, true,
							    false);
		}
	} while (run < CQSPI_PHY_DDR_CONSEC_PASS &&
		 pass_end.tx >= (pass_start.tx + CQSPI_PHY_DDR_CONSEC_PASS));

	if (run < CQSPI_PHY_DDR_CONSEC_PASS) {
		pass_end.read_delay = CQSPI_PHY_RD_INVALID;
	} else {
		/*
		 * Walk past confirmed region to find gap end boundary.
		 * Clamp the jump so tx/rx never step below zero.
		 */
		int off = CQSPI_PHY_DDR_CONSEC_PASS + 1;

		if (pass_end.tx < off)
			off = pass_end.tx;
		if (pass_end.rx < off)
			off = pass_end.rx;

		gap_end.rx = pass_end.rx - off;
		gap_end.tx = pass_end.tx - off;
		gap_end.read_delay = pass_end.read_delay;

		for (;;) {
			ret = cadence_spi_phy_apply_setting(priv, &gap_end);
			if (!ret)
				ret = cadence_spi_phy_check_pattern(priv, spi);
			if (ret)
				break;
			if (gap_end.tx == 0 || gap_end.rx == 0)
				break;
			gap_end.tx--;
			gap_end.rx--;
		}

		do {
			run = cadence_spi_phy_check_consecutive_ddr(priv, spi,
								    gap_end,
								    CQSPI_PHY_DDR_CONSEC_FAIL,
								    false, false);
			if (run < CQSPI_PHY_DDR_CONSEC_FAIL) {
				int adv = run + 1;

				if (gap_end.tx < adv)
					adv = gap_end.tx;
				if (gap_end.rx < adv)
					adv = gap_end.rx;

				gap_end.tx -= adv;
				gap_end.rx -= adv;
				cadence_spi_phy_find_next_match_ddr(priv, spi,
								    &gap_end,
								    &pass_start,
								    false,
								    false);
			}
		} while (run < CQSPI_PHY_DDR_CONSEC_FAIL &&
			 gap_end.tx >= pass_start.tx);

		/* Narrow gap_end..pass_end to the best passing run */
		cadence_spi_phy_find_biggest_line_ddr(priv, spi, &pass_end,
						      &gap_end, false);
	}

	if (pass_start.read_delay == CQSPI_PHY_RD_INVALID &&
	    pass_end.read_delay == CQSPI_PHY_RD_INVALID)
		return -ENOENT;

	low_len = 0;
	high_len = 0;

	if (pass_start.read_delay != CQSPI_PHY_RD_INVALID)
		low_len = (int)gap_start.tx - (int)pass_start.tx;

	if (pass_end.read_delay != CQSPI_PHY_RD_INVALID)
		high_len = (int)pass_end.tx - (int)gap_end.tx;

	dev_dbg(bus,
		"DDR PHY diagonal: dual region, low=%d taps, high=%d taps\n",
		low_len, high_len);

	if (low_len < CQSPI_PHY_DDR_MIN_PASS_TAPS &&
	    high_len < CQSPI_PHY_DDR_MIN_PASS_TAPS)
		return -ENOENT;

	low_midpt = (struct phy_setting){ .read_delay = CQSPI_PHY_RD_INVALID };
	high_midpt = (struct phy_setting){ .read_delay = CQSPI_PHY_RD_INVALID };

	if (high_len >= CQSPI_PHY_DDR_MIN_PASS_TAPS) {
		high_midpt.rx = (pass_end.rx + gap_end.rx) / 2;
		high_midpt.tx = (pass_end.tx + gap_end.tx) / 2;
		high_midpt.read_delay = pass_end.read_delay;
	}

	if (low_len >= CQSPI_PHY_DDR_MIN_PASS_TAPS) {
		low_midpt.rx = (pass_start.rx + gap_start.rx) / 2;
		low_midpt.tx = (pass_start.tx + gap_start.tx) / 2;
		low_midpt.read_delay = pass_start.read_delay;
	}

	/* Try the larger region first; fall back to the smaller one */
	if (high_len >= low_len) {
		first = high_midpt.read_delay != CQSPI_PHY_RD_INVALID ?
				&high_midpt :
				NULL;
		second = low_midpt.read_delay != CQSPI_PHY_RD_INVALID ?
				 &low_midpt :
				 NULL;
	} else {
		first = low_midpt.read_delay != CQSPI_PHY_RD_INVALID ?
				&low_midpt :
				NULL;
		second = high_midpt.read_delay != CQSPI_PHY_RD_INVALID ?
				 &high_midpt :
				 NULL;
	}

	if (first &&
	    !cadence_spi_phy_find_tuning_point_ddr(priv, spi, first)) {
		*best = *first;
		return 0;
	}

	if (second &&
	    !cadence_spi_phy_find_tuning_point_ddr(priv, spi, second)) {
		*best = *second;
		return 0;
	}

	return -ENOENT;
}

static int cadence_spi_phy_calibrate(struct cadence_spi_priv *priv,
				     struct spi_slave *spi)
{
	struct udevice *bus = spi->dev->parent;
	struct phy_setting diag_start, diag_end;
	struct phy_setting tuning_pt = { 0 };
	int shift, ret;

	priv->use_phy = true;

	diag_start = (struct phy_setting){ .tx = 0,
					   .rx = 0,
					   .read_delay = CQSPI_PHY_RD_INVALID };
	diag_end = (struct phy_setting){ .tx = CQSPI_PHY_MAX_DELAY,
					 .rx = CQSPI_PHY_MAX_DELAY,
					 .read_delay = CQSPI_PHY_RD_INVALID };

	dev_dbg(bus, "DDR PHY tuning: searching main diagonal\n");
	ret = cadence_spi_phy_search_diagonal_ddr(priv, spi, &tuning_pt,
						  diag_start, diag_end);

	for (shift = CQSPI_PHY_DDR_DIAG_SHIFT;
	     ret && shift <= CQSPI_PHY_DDR_MAX_DIAG_SHIFT;
	     shift += CQSPI_PHY_DDR_DIAG_SHIFT) {
		/* Up-shifted diagonal: rx-biased */
		dev_dbg(bus, "DDR PHY tuning: up-shifted diagonal, shift=%d\n",
			shift);
		diag_start = (struct phy_setting){
			.tx = 0, .rx = shift, .read_delay = CQSPI_PHY_RD_INVALID
		};
		diag_end = (struct phy_setting){
			.tx = CQSPI_PHY_MAX_DELAY - shift,
			.rx = CQSPI_PHY_MAX_DELAY,
			.read_delay = CQSPI_PHY_RD_INVALID
		};
		ret = cadence_spi_phy_search_diagonal_ddr(priv, spi,
							  &tuning_pt,
							  diag_start, diag_end);
		if (ret) {
			/* Right-shifted diagonal: tx-biased */
			dev_dbg(bus,
				"DDR PHY tuning: right-shifted diagonal, shift=%d\n",
				shift);
			diag_start = (struct phy_setting){
				.tx = shift,
				.rx = 0,
				.read_delay = CQSPI_PHY_RD_INVALID
			};
			diag_end = (struct phy_setting){
				.tx = CQSPI_PHY_MAX_DELAY,
				.rx = CQSPI_PHY_MAX_DELAY - shift,
				.read_delay = CQSPI_PHY_RD_INVALID
			};
			ret = cadence_spi_phy_search_diagonal_ddr(priv, spi,
								  &tuning_pt,
								  diag_start,
								  diag_end);
		}
	}

	if (ret) {
		dev_err(bus, "DDR PHY tuning failed to find valid configuration\n");
		goto out;
	}

	dev_dbg(bus, "DDR PHY tuning point: TX=%u RX=%u RD=%u\n",
		tuning_pt.tx, tuning_pt.rx, tuning_pt.read_delay);

	ret = cadence_spi_phy_apply_setting(priv, &tuning_pt);
	if (!ret)
		ret = cadence_spi_phy_check_pattern(priv, spi);
	if (ret) {
		dev_err(bus, "DDR PHY: pattern check failed at tuning point\n");
		ret = -EINVAL;
		goto out;
	}

	priv->phy_read_delay = tuning_pt.read_delay;
out:
	if (ret)
		priv->use_phy = false;
	return ret;
}

static void cadence_spi_phy_reset_setting(struct phy_setting *phy)
{
	phy->rx = 0;
	phy->tx = 127;
	phy->read_delay = 0;
}

static int cadence_spi_phy_calibrate_sdr(struct cadence_spi_priv *priv,
					 struct spi_slave *spi)
{
	struct udevice *bus = spi->dev->parent;
	struct phy_setting rxlow, rxhigh, first, second, final;
	char window1 = 0;
	char window2 = 0;
	int ret;

	priv->use_phy = true;
	cadence_spi_phy_reset_setting(&rxlow);
	cadence_spi_phy_reset_setting(&rxhigh);
	cadence_spi_phy_reset_setting(&first);

	do {
		ret = cadence_spi_find_rx_low_sdr(priv, spi, &rxlow);

		if (ret)
			rxlow.read_delay++;
	} while (ret && rxlow.read_delay <= CQSPI_PHY_MAX_RD);

	rxhigh.read_delay = rxlow.read_delay;
	ret = cadence_spi_find_rx_high_sdr(priv, spi, &rxhigh, rxlow.rx);
	if (ret)
		goto out;

	first.read_delay = rxlow.read_delay;
	window1 = rxhigh.rx - rxlow.rx;
	first.rx = rxlow.rx + (window1 / 2);

	dev_dbg(bus, "First tuning point: RX: %d TX: %d RD: %d\n", first.rx,
		first.tx, first.read_delay);
	ret = cadence_spi_phy_apply_setting(priv, &first);
	if (!ret)
		ret = cadence_spi_phy_check_pattern(priv, spi);

	if (ret || first.read_delay > CQSPI_PHY_MAX_RD)
		goto out;

	cadence_spi_phy_reset_setting(&rxlow);
	cadence_spi_phy_reset_setting(&rxhigh);
	cadence_spi_phy_reset_setting(&second);

	rxlow.read_delay = first.read_delay + 1;
	if (rxlow.read_delay > CQSPI_PHY_MAX_RD)
		goto compare;

	ret = cadence_spi_find_rx_low_sdr(priv, spi, &rxlow);

	if (ret)
		goto compare;

	rxhigh.read_delay = rxlow.read_delay;
	ret = cadence_spi_find_rx_high_sdr(priv, spi, &rxhigh, rxlow.rx);
	if (ret)
		goto compare;

	window2 = rxhigh.rx - rxlow.rx;
	second.rx = rxlow.rx + (window2 / 2);
	second.read_delay = rxlow.read_delay;

	dev_dbg(bus, "Second tuning point: RX: %d TX: %d RD: %d\n", second.rx,
		second.tx, second.read_delay);
	ret = cadence_spi_phy_apply_setting(priv, &second);
	if (!ret)
		ret = cadence_spi_phy_check_pattern(priv, spi);

	if (ret || second.read_delay > CQSPI_PHY_MAX_RD)
		window2 = 0;

compare:
	cadence_spi_phy_reset_setting(&final);
	if (window2 > window1) {
		final.rx = second.rx;
		final.read_delay = second.read_delay;
	} else {
		final.rx = first.rx;
		final.read_delay = first.read_delay;
	}

	dev_dbg(bus, "Final tuning point: RX: %d TX: %d RD: %d\n", final.rx,
		final.tx, final.read_delay);
	ret = cadence_spi_phy_apply_setting(priv, &final);
	if (!ret)
		ret = cadence_spi_phy_check_pattern(priv, spi);

	if (ret) {
		ret = -EINVAL;
		goto out;
	}

out:
	if (ret)
		priv->use_phy = false;

	return ret;
}

static int cadence_spi_write_speed(struct udevice *bus, uint hz)
{
	struct cadence_spi_priv *priv = dev_get_priv(bus);

	cadence_qspi_apb_config_baudrate_div(priv->regbase,
					     priv->ref_clk_hz, hz);

	/* Reconfigure delay timing if speed is changed. */
	cadence_qspi_apb_delay(priv->regbase, priv->ref_clk_hz, hz,
			       priv->tshsl_ns, priv->tsd2d_ns,
			       priv->tchsh_ns, priv->tslch_ns);

	return 0;
}

static int cadence_spi_read_id(struct cadence_spi_priv *priv, u8 len,
			       u8 *idcode)
{
	int err;

	struct spi_mem_op op = SPI_MEM_OP(SPI_MEM_OP_CMD(0x9F, 1),
					  SPI_MEM_OP_NO_ADDR,
					  SPI_MEM_OP_NO_DUMMY,
					  SPI_MEM_OP_DATA_IN(len, idcode, 1));

	err = cadence_qspi_apb_command_read_setup(priv, &op);
	if (!err)
		err = cadence_qspi_apb_command_read(priv, &op);

	return err;
}

/* Calibration sequence to determine the read data capture delay register */
static int spi_calibration(struct udevice *bus, uint hz)
{
	struct cadence_spi_priv *priv = dev_get_priv(bus);
	void *base = priv->regbase;
	unsigned int idcode = 0, temp = 0;
	int err = 0, i, range_lo = -1, range_hi = -1;

	/* start with slowest clock (1 MHz) */
	cadence_spi_write_speed(bus, 1000000);

	/* configure the read data capture delay register to 0 */
	cadence_qspi_apb_readdata_capture(base, 1, true, 0);

	/* Enable QSPI */
	cadence_qspi_apb_controller_enable(base);

	/* read the ID which will be our golden value */
	err = cadence_spi_read_id(priv, 3, (u8 *)&idcode);
	if (err) {
		puts("SF: Calibration failed (read)\n");
		return err;
	}

	/* use back the intended clock and find low range */
	cadence_spi_write_speed(bus, hz);
	for (i = 0; i < CQSPI_READ_CAPTURE_MAX_DELAY; i++) {
		/* Disable QSPI */
		cadence_qspi_apb_controller_disable(base);

		/* reconfigure the read data capture delay register */
		cadence_qspi_apb_readdata_capture(base, 1, true, i);

		/* Enable back QSPI */
		cadence_qspi_apb_controller_enable(base);

		/* issue a RDID to get the ID value */
		err = cadence_spi_read_id(priv, 3, (u8 *)&temp);
		if (err) {
			puts("SF: Calibration failed (read)\n");
			return err;
		}

		/* search for range lo */
		if (range_lo == -1 && temp == idcode) {
			range_lo = i;
			continue;
		}

		/* search for range hi */
		if (range_lo != -1 && temp != idcode) {
			range_hi = i - 1;
			break;
		}
		range_hi = i;
	}

	if (range_lo == -1) {
		puts("SF: Calibration failed (low range)\n");
		return -EIO;
	}

	/* Disable QSPI for subsequent initialization */
	cadence_qspi_apb_controller_disable(base);

	/* configure the final value for read data capture delay register */
	cadence_qspi_apb_readdata_capture(base, 1, true, (range_hi + range_lo) / 2);
	debug("SF: Read data capture delay calibrated to %i (%i - %i)\n",
	      (range_hi + range_lo) / 2, range_lo, range_hi);

	/* just to ensure we do once only when speed or chip select change */
	priv->qspi_calibrated_hz = hz;
	priv->qspi_calibrated_cs = spi_chip_select(bus);

	return 0;
}

static int cadence_spi_set_speed(struct udevice *bus, uint hz)
{
	struct cadence_spi_priv *priv = dev_get_priv(bus);
	int err;

	if (!hz || hz > priv->max_hz)
		hz = priv->max_hz;
	/* Disable QSPI */
	cadence_qspi_apb_controller_disable(priv->regbase);

	/*
	 * If the device tree already provides a read delay value, use that
	 * instead of calibrating.
	 */
	if (priv->read_delay >= 0) {
		cadence_spi_write_speed(bus, hz);
		cadence_qspi_apb_readdata_capture(priv->regbase, 1, true,
						  priv->read_delay);
	} else if (priv->previous_hz != hz ||
		   priv->qspi_calibrated_hz != hz ||
		   priv->qspi_calibrated_cs != spi_chip_select(bus)) {
		/*
		 * Calibration required for different current SCLK speed,
		 * requested SCLK speed or chip select
		 */
		err = spi_calibration(bus, hz);
		if (err)
			return err;

		/* prevent calibration run when same as previous request */
		priv->previous_hz = hz;
	}

	/* Enable QSPI */
	cadence_qspi_apb_controller_enable(priv->regbase);

	debug("%s: speed=%d\n", __func__, hz);

	return 0;
}

static int cadence_spi_probe(struct udevice *bus)
{
	struct cadence_spi_plat *plat = dev_get_plat(bus);
	struct cadence_spi_priv *priv = dev_get_priv(bus);
	struct clk clk;
	int ret;

	priv->regbase		= plat->regbase;
	priv->ahbbase		= plat->ahbbase;
	priv->is_dma		= plat->is_dma;
	priv->is_decoded_cs	= plat->is_decoded_cs;
	priv->fifo_depth	= plat->fifo_depth;
	priv->fifo_width	= plat->fifo_width;
	priv->trigger_address	= plat->trigger_address;
	priv->read_delay	= plat->read_delay;
	priv->phase_detect_selector = plat->phase_detect_selector;
	priv->phy_pattern_start = plat->phy_pattern_start;
	priv->phy_tx_start	= plat->phy_tx_start;
	priv->phy_tx_end	= plat->phy_tx_end;
	priv->ahbsize		= plat->ahbsize;
	priv->max_hz		= plat->max_hz;

	priv->page_size		= plat->page_size;
	priv->block_size	= plat->block_size;
	priv->tshsl_ns		= plat->tshsl_ns;
	priv->tsd2d_ns		= plat->tsd2d_ns;
	priv->tchsh_ns		= plat->tchsh_ns;
	priv->tslch_ns		= plat->tslch_ns;
	priv->quirks		= plat->quirks;

	/* write completion is supported by default */
	priv->wr_completion = true;

	if (priv->quirks & CQSPI_NO_SUPPORT_WR_COMPLETION)
		priv->wr_completion = false;

	if (IS_ENABLED(CONFIG_ZYNQMP_FIRMWARE))
		xilinx_pm_request(PM_REQUEST_NODE, PM_DEV_OSPI,
				  ZYNQMP_PM_CAPABILITY_ACCESS, ZYNQMP_PM_MAX_QOS,
				  ZYNQMP_PM_REQUEST_ACK_NO, 0, 0, NULL);

	if (priv->ref_clk_hz == 0) {
		ret = clk_get_by_index(bus, 0, &clk);
		if (ret) {
#ifdef CONFIG_HAS_CQSPI_REF_CLK
			priv->ref_clk_hz = CONFIG_CQSPI_REF_CLK;
#elif defined(CONFIG_ARCH_SOCFPGA)
			priv->ref_clk_hz = cm_get_qspi_controller_clk_hz();
#else
			return ret;
#endif
		} else {
			priv->ref_clk_hz = clk_get_rate(&clk);
			if (IS_ERR_VALUE(priv->ref_clk_hz))
				return priv->ref_clk_hz;
		}
	}

	priv->resets = devm_reset_bulk_get_optional(bus);
	if (priv->resets)
		reset_deassert_bulk(priv->resets);

	if (!priv->qspi_is_init) {
		cadence_qspi_apb_controller_init(priv);
		priv->qspi_is_init = 1;
	}

	priv->wr_delay = 50 * DIV_ROUND_UP(NSEC_PER_SEC, priv->ref_clk_hz);

	if (device_is_compatible(bus, "amd,versal2-ospi"))
		return cadence_device_reset(bus);

	/* Reset ospi flash device */
	return cadence_qspi_flash_reset(bus);

	return 0;
}

static int cadence_spi_remove(struct udevice *dev)
{
	struct cadence_spi_priv *priv = dev_get_priv(dev);
	int ret = 0;

	if (priv->resets)
		ret = reset_release_bulk(priv->resets);

	return ret;
}

static int cadence_spi_set_mode(struct udevice *bus, uint mode)
{
	struct cadence_spi_priv *priv = dev_get_priv(bus);

	/* Disable QSPI */
	cadence_qspi_apb_controller_disable(priv->regbase);

	/* Set SPI mode */
	cadence_qspi_apb_set_clk_mode(priv->regbase, mode);

	/* Enable Direct Access Controller */
	if (priv->use_dac_mode)
		cadence_qspi_apb_dac_mode_enable(priv->regbase);

	/* Enable QSPI */
	cadence_qspi_apb_controller_enable(priv->regbase);

	return 0;
}

static int cadence_spi_mem_exec_op(struct spi_slave *spi,
				   const struct spi_mem_op *op)
{
	struct udevice *bus = spi->dev->parent;
	struct cadence_spi_priv *priv = dev_get_priv(bus);
	void *base = priv->regbase;
	int err = 0;
	u32 mode;

	/* Set Chip select */
	cadence_qspi_apb_chipselect(base, spi_chip_select(spi->dev),
				    priv->is_decoded_cs);

	if (op->data.dir == SPI_MEM_DATA_IN && op->data.buf.in) {
		/*
		 * Performing reads in DAC mode forces to read minimum 4 bytes
		 * which is unsupported on some flash devices during register
		 * reads, prefer STIG mode for such small reads.
		 */
		if (!op->addr.nbytes ||
		    (op->data.nbytes <= CQSPI_STIG_DATA_LEN_MAX &&
		     !(priv->quirks & CQSPI_DISABLE_STIG_MODE)))
			mode = CQSPI_STIG_READ;
		else
			mode = CQSPI_READ;
	} else {
		if (!op->addr.nbytes || !op->data.buf.out ||
		    (op->data.nbytes <= CQSPI_STIG_DATA_LEN_MAX &&
		     !(priv->quirks & CQSPI_DISABLE_STIG_MODE)))
			mode = CQSPI_STIG_WRITE;
		else
			mode = CQSPI_WRITE;
	}

	switch (mode) {
	case CQSPI_STIG_READ:
		err = cadence_qspi_apb_command_read_setup(priv, op);
		if (!err)
			err = cadence_qspi_apb_command_read(priv, op);
		break;
	case CQSPI_STIG_WRITE:
		err = cadence_qspi_apb_command_write_setup(priv, op);
		if (!err)
			err = cadence_qspi_apb_command_write(priv, op);
		break;
	case CQSPI_READ:
		err = cadence_qspi_apb_read_setup(priv, op);
		if (!err) {
			if (priv->is_dma)
				err = cadence_qspi_apb_dma_read(priv, op);
			else
				err = cadence_qspi_apb_read_execute(priv, op);
		}
		break;
	case CQSPI_WRITE:
		err = cadence_qspi_apb_write_setup(priv, op);
		if (!err)
			err = cadence_qspi_apb_write_execute(priv, op);
		break;
	default:
		err = -1;
		break;
	}

	return err;
}

static bool cadence_spi_mem_supports_op(struct spi_slave *slave,
					const struct spi_mem_op *op)
{
	bool all_true, all_false;

	/*
	 * op->dummy.dtr is required for converting nbytes into ncycles.
	 * Also, don't check the dtr field of the op phase having zero nbytes.
	 */
	all_true = op->cmd.dtr &&
		   (!op->addr.nbytes || op->addr.dtr) &&
		   (!op->dummy.nbytes || op->dummy.dtr) &&
		   (!op->data.nbytes || op->data.dtr);

	all_false = !op->cmd.dtr && !op->addr.dtr && !op->dummy.dtr &&
		    !op->data.dtr;

	/* Mixed DTR modes not supported. */
	if (!(all_true || all_false))
		return false;

	if (all_true)
		return spi_mem_dtr_supports_op(slave, op);
	else
		return spi_mem_default_supports_op(slave, op);
}

static int cadence_spi_mem_do_calibration(struct spi_slave *spi,
					  struct spi_mem_op *op)
{
	struct udevice *bus = spi->dev->parent;
	struct cadence_spi_priv *priv = dev_get_priv(bus);
	struct udevice *dma_dev;
	int ret;

	priv->phy_read_op = *op;

	if (priv->phase_detect_selector >
	    CQSPI_REG_PHY_DLL_MASTER_DLY_ELMTS_LEN) {
		dev_warn(bus,
			 "Phase Detect Selector should be in between [0, 7]. Skipping Calibration\n");
		return 0;
	}

	ret = cadence_spi_phy_check_pattern(priv, spi);
	if (ret) {
		dev_warn(bus, "Pattern not found. Skipping calibration\n");
		return 0;
	}

	/*
	 * PHY-mode reads during calibration go through dma_memcpy() with no
	 * fallback. Without a mem-to-mem DMA device every probe in the
	 * search below is guaranteed to fail, so skip calibration entirely
	 * instead of paying for an exhaustive, doomed search.
	 */
	if (dma_get_device(DMA_SUPPORTS_MEM_TO_MEM, &dma_dev)) {
		dev_warn(bus, "No DMA device found. Skipping calibration\n");
		return 0;
	}

	if (cadence_qspi_apb_op_eligible(op)) {
		priv->use_dqs = true;

		cadence_qspi_apb_phy_pre_config(priv, 0, 1);

		ret = cadence_spi_phy_calibrate(priv, spi);
		if (ret)
			dev_warn(bus,
				 "PHY calibration failed: %d. Falling back to slower clock speeds.\n",
				 ret);
	} else if (cadence_qspi_apb_op_eligible_sdr(op)) {
		priv->use_dqs = false;

		cadence_qspi_apb_phy_pre_config(priv, 1, 0);

		ret = cadence_spi_phy_calibrate_sdr(priv, spi);
		if (ret)
			dev_warn(bus,
				 "PHY calibration failed: %d. Falling back to slower clock speeds.\n",
				 ret);

	} else {
		dev_warn(bus,
			 "Given read_op not eligible. Skipping Calibration.\n");
		return 0;
	}

	cadence_qspi_apb_phy_post_config(priv, priv->read_delay);
	return ret;
}

static int cadence_spi_ofdata_phy_pattern(ofnode flash_node)
{
	ofnode subnode;
	const char *label;
	u32 start;

	subnode = ofnode_find_subnode(flash_node, "partitions");
	if (!ofnode_valid(subnode))
		/*
		 * Maybe the node has legacy style partitions, listed directly
		 * under flash node.
		 */
		subnode = ofnode_first_subnode(flash_node);
	else
		subnode = ofnode_first_subnode(subnode);

	while (ofnode_valid(subnode)) {
		label = ofnode_read_string(subnode, "label");
		if (label && strstr(label, "phypattern")) {
			if (!ofnode_read_u32_array(subnode, "reg", &start, 1)) {
				if (ofnode_device_is_compatible(
					    flash_node, "jedec,spi-nor"))
					return start;
				if (ofnode_device_is_compatible(flash_node,
								"spi-nand"))
					return 0;
			}
			break;
		}

		subnode = ofnode_next_subnode(subnode);
	}

	return -ENOENT;
}

static int cadence_spi_of_to_plat(struct udevice *bus)
{
	struct cadence_spi_plat *plat = dev_get_plat(bus);
	struct cadence_spi_priv *priv = dev_get_priv(bus);
	ofnode subnode;
	int ret;

	plat->regbase = devfdt_get_addr_index_ptr(bus, 0);
	plat->ahbbase = devfdt_get_addr_size_index_ptr(bus, 1, &plat->ahbsize);
	plat->is_decoded_cs = dev_read_bool(bus, "cdns,is-decoded-cs");
	plat->fifo_depth = dev_read_u32_default(bus, "cdns,fifo-depth", 128);
	plat->fifo_width = dev_read_u32_default(bus, "cdns,fifo-width", 4);
	plat->trigger_address = dev_read_u32_default(bus,
						     "cdns,trigger-address",
						     0);
	plat->phase_detect_selector = dev_read_u32_default(bus,
							   "cdns,phase-detect-selector",
							   CQSPI_REG_PHY_DLL_MASTER_DLY_ELMTS_LEN + 1);
	/* Use DAC mode only when MMIO window is at least 8M wide */
	if (plat->ahbsize >= SZ_8M)
		priv->use_dac_mode = true;

	plat->is_dma = dev_read_bool(bus, "cdns,is-dma");

	/* All other parameters are embedded in the child node */
	subnode = cadence_qspi_get_subnode(bus);
	if (!ofnode_valid(subnode)) {
		printf("Error: subnode with SPI flash config missing!\n");
		return -ENODEV;
	}

	/* Use 500 KHz as a suitable default */
	plat->max_hz = ofnode_read_u32_default(subnode, "spi-max-frequency",
					       500000);

	/* Read other parameters from DT */
	plat->page_size = ofnode_read_u32_default(subnode, "page-size", 256);
	plat->block_size = ofnode_read_u32_default(subnode, "block-size", 16);
	plat->tshsl_ns = ofnode_read_u32_default(subnode, "cdns,tshsl-ns",
						 200);
	plat->tsd2d_ns = ofnode_read_u32_default(subnode, "cdns,tsd2d-ns",
						 255);
	plat->tchsh_ns = ofnode_read_u32_default(subnode, "cdns,tchsh-ns", 20);
	plat->tslch_ns = ofnode_read_u32_default(subnode, "cdns,tslch-ns", 20);
	/*
	 * Read delay should be an unsigned value but we use a signed integer
	 * so that negative values can indicate that the device tree did not
	 * specify any signed values and we need to perform the calibration
	 * sequence to find it out.
	 */
	plat->read_delay = ofnode_read_s32_default(subnode, "cdns,read-delay",
						   -1);

	const struct cqspi_driver_platdata *drvdata =
		(struct cqspi_driver_platdata *)dev_get_driver_data(bus);

	if (drvdata)
		plat->quirks = drvdata->quirks;
	else
		debug("%s: no driver data, quirks not set\n", __func__);

	plat->phy_tx_start = ofnode_read_u32_default(subnode,
						     "cdns,phy-tx-start",
						     16);
	plat->phy_tx_end = ofnode_read_u32_default(subnode,
						   "cdns,phy-tx-end",
						   48);

	/* Find the PHY tuning pattern partition. */
	ret = cadence_spi_ofdata_phy_pattern(subnode);
	if (ret < 0)
		dev_dbg(bus, "Unable to find PHY pattern partition\n");
	else
		plat->phy_pattern_start = ret;

	debug("%s: regbase=%p ahbbase=%p max-frequency=%d page-size=%d\n",
	      __func__, plat->regbase, plat->ahbbase, plat->max_hz,
	      plat->page_size);

	return 0;
}

static const struct spi_controller_mem_ops cadence_spi_mem_ops = {
	.exec_op = cadence_spi_mem_exec_op,
	.supports_op = cadence_spi_mem_supports_op,
	.do_calibration = cadence_spi_mem_do_calibration,
};

static const struct dm_spi_ops cadence_spi_ops = {
	.set_speed	= cadence_spi_set_speed,
	.set_mode	= cadence_spi_set_mode,
	.mem_ops	= &cadence_spi_mem_ops,
	/*
	 * cs_info is not needed, since we require all chip selects to be
	 * in the device tree explicitly
	 */
};

static const struct cqspi_driver_platdata cdns_qspi = {
	.quirks = CQSPI_DISABLE_STIG_MODE,
};

static const struct udevice_id cadence_spi_ids[] = {
	{
		.compatible = "cdns,qspi-nor",
		.data = (ulong)&cdns_qspi,
	},
	{
		.compatible = "ti,am654-ospi"
	},
	{
		.compatible = "amd,versal2-ospi"
	},
	{ }
};

U_BOOT_DRIVER(cadence_spi) = {
	.name = "cadence_spi",
	.id = UCLASS_SPI,
	.of_match = cadence_spi_ids,
	.ops = &cadence_spi_ops,
	.of_to_plat = cadence_spi_of_to_plat,
	.plat_auto	= sizeof(struct cadence_spi_plat),
	.priv_auto	= sizeof(struct cadence_spi_priv),
	.probe = cadence_spi_probe,
	.remove = cadence_spi_remove,
	.flags = DM_FLAG_OS_PREPARE,
};
