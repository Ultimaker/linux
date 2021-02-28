// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 * datasheet: http://www.ti.com/lit/ds/symlink/sn65dsi86.pdf
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#define SN_DEVICE_REV_REG			0x08
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
#define SN_CHA_VERTICAL_FRONT_PORCH_REG		0x3A
#define SN_ENH_FRAME_REG			0x5A
#define  VSTREAM_ENABLE				BIT(3)
#define SN_DATA_FORMAT_REG			0x5B
#define SN_HPD_DISABLE_REG			0x5C
#define  HPD_DISABLE				BIT(0)
#define SN_AUX_WDATA_REG(x)			(0x64 + (x))
#define SN_AUX_ADDR_19_16_REG			0x74
#define SN_AUX_ADDR_15_8_REG			0x75
#define SN_AUX_ADDR_7_0_REG			0x76
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
#define SN_ML_TX_MODE_REG			0x96
#define  ML_TX_MAIN_LINK_OFF			0
#define  ML_TX_NORMAL_MODE			BIT(0)
#define SN_AUX_CMD_STATUS_REG			0xF4
#define  AUX_IRQ_STATUS_AUX_RPLY_TOUT		BIT(3)
#define  AUX_IRQ_STATUS_AUX_SHORT		BIT(5)
#define  AUX_IRQ_STATUS_NAT_I2C_FAIL		BIT(6)
#define  PAGE_SELECT		0xFF
#define  ASSR_OVERRIDE		0x16
#define  ASSR_CONTROL_MASK			GENMASK(1, 0)
#define  STANDARD_DP_SEED 			0x0

/* Hot plug detection support */
#define SN_HPDLINE_REG (0x5C)
#define HPD_LINE_STATUS ((1) << 4)
#define HPD_LINE_ENABLE ((1) << 0)

#define SN_IRQEN_REG (0xE0)
#define SN_IRQHPD_EN_REG (0xE6)
#define SN_IRQHPD_STATUS_REG (0xF5)
#define HPD_EVENT_MASK (0x0F)
#define HPD_REMOVAL_IRQ_EN ((1) << 2)
#define HPD_INSERTION_IRQ_EN ((1) << 1)
#define HPD_REPLUG_IRQ_EN ((1) << 3)
#define IRQ_HPD_EN ((1) << 0)



#define MIN_DSI_CLK_FREQ_MHZ	40

/* fudge factor required to account for 8b/10b encoding */
#define DP_CLK_FUDGE_NUM	10
#define DP_CLK_FUDGE_DEN	8

/* Matches DP_AUX_MAX_PAYLOAD_BYTES (for now) */
#define SN_AUX_MAX_PAYLOAD_BYTES	16

#define SN_REGULATOR_SUPPLY_NUM		4

struct ti_sn_bridge {
	struct device				*dev;
	struct regmap				*regmap;
	struct drm_dp_aux			aux;
	struct drm_bridge			bridge;
	struct drm_connector		connector;
	struct dentry				*debugfs;
	struct device_node			*host_node;
	struct mipi_dsi_device		*dsi;
	struct clk					*refclk;
	struct drm_panel			*panel;
	struct gpio_desc			*enable_gpio;
	struct regulator_bulk_data	supplies[SN_REGULATOR_SUPPLY_NUM];
	unsigned int				dsi_freq;

	struct gpio_desc			*hpd_qpio;
	int 						hpd_irq;
	struct delayed_work 		mw;
	bool 						hpd_supported;
	bool						hpd_enabled;

	enum drm_connector_status	con_connected;
	bool						ddc_enabled;
	bool						edp_assr;
};

static const struct regmap_range ti_sn_bridge_volatile_ranges[] = {
	{ .range_min = 0, .range_max = 0xFF },
};

static const struct regmap_access_table ti_sn_bridge_volatile_table = {
	.yes_ranges = ti_sn_bridge_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(ti_sn_bridge_volatile_ranges),
};

static const struct regmap_config ti_sn_bridge_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_table = &ti_sn_bridge_volatile_table,
	.cache_type = REGCACHE_NONE,
};

static void ti_sn_bridge_write_u16(struct ti_sn_bridge *pdata,
				   unsigned int reg, u16 val)
{
	regmap_write(pdata->regmap, reg, val & 0xFF);
	regmap_write(pdata->regmap, reg + 1, val >> 8);
}

/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* Power management */

static int __maybe_unused ti_sn_bridge_resume(struct device *dev)
{
	struct ti_sn_bridge *pdata = dev_get_drvdata(dev);
	int ret;

	ret = regulator_bulk_enable(SN_REGULATOR_SUPPLY_NUM, pdata->supplies);
	if (ret) {
		DRM_ERROR("failed to enable supplies %d\n", ret);
		return ret;
	}

	gpiod_set_value(pdata->enable_gpio, 1);

	return ret;
}

static int __maybe_unused ti_sn_bridge_suspend(struct device *dev)
{
	struct ti_sn_bridge *pdata = dev_get_drvdata(dev);
	int ret;

	gpiod_set_value(pdata->enable_gpio, 0);

	ret = regulator_bulk_disable(SN_REGULATOR_SUPPLY_NUM, pdata->supplies);
	if (ret)
		DRM_ERROR("failed to disable supplies %d\n", ret);

	return ret;
}

static const struct dev_pm_ops ti_sn_bridge_pm_ops = {
	SET_RUNTIME_PM_OPS(ti_sn_bridge_suspend, ti_sn_bridge_resume, NULL)
};

/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* Driver debug */

static int status_show(struct seq_file *s, void *data)
{
	struct ti_sn_bridge *pdata = s->private;
	unsigned int reg, val;

	pm_runtime_get_sync(pdata->dev);

	seq_puts(s, "TI-SN65DSI86 DSI to Display Port Bridge Status\n");

	if (pdata->hpd_supported){
		seq_puts(s, "HPD (Hot Plug Detection): SUPPORTED\n");
		if (pdata->hpd_enabled) {
			seq_puts(s, "	HPD function : ENABLED\n");
			seq_printf(s, "	HPD interrupt no : %d\n", pdata->hpd_irq);
			regmap_read(pdata->regmap, 0x5c, &val);
			seq_printf(s, "	HPD status reg : 0x%08x\n", val);
		} else {
			seq_printf(s, "	HPD function : DISABLED\n");
		}
	} else {
		seq_puts(s, "HPD (Hot Plug Detection): NOT SUPPORTED\n");
	}

	seq_puts(s, "\n");

	seq_puts(s, "STATUS REGISTERS:\n");

	/* IRQ Status Registers, see Table 31 in datasheet */
	for (reg = 0xf0; reg <= 0xf8; reg++) {
		regmap_read(pdata->regmap, reg, &val);
		seq_printf(s, "[0x%02x] = 0x%08x\n", reg, val);
	}

	pm_runtime_put(pdata->dev);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(status);

static int connector_status_show(struct seq_file *s, void *data)
{
	struct ti_sn_bridge *pdata = s->private;

	seq_puts(s, "CONNECTOR STATUS: ");
	if (pdata->con_connected == connector_status_connected) {
		seq_puts(s, "connected\n");
	} else if (pdata->con_connected == connector_status_disconnected) {
		seq_puts(s, "disconnected\n");
	} else {
		seq_puts(s, "unknown\n");
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(connector_status);

static void ti_sn_debugfs_init(struct ti_sn_bridge *pdata)
{
	pdata->debugfs = debugfs_create_dir(dev_name(pdata->dev), NULL);

	debugfs_create_file("status", 0600, pdata->debugfs, pdata,
			&status_fops);
	debugfs_create_file("connector_status", 0600, pdata->debugfs, pdata,
			&connector_status_fops);
}

static void ti_sn_debugfs_remove(struct ti_sn_bridge *pdata)
{
	debugfs_remove_recursive(pdata->debugfs);
	pdata->debugfs = NULL;
}

/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* Configuration Helper Functions */

static u32 ti_sn_bridge_get_dsi_freq(struct ti_sn_bridge *pdata)
{
	return pdata->dsi_freq;
}

/* clk frequencies supported by bridge in Hz in case derived from REFCLK pin */
static const u32 ti_sn_bridge_refclk_lut[] = {
	12000000,
	19200000,
	26000000,
	27000000,
	38400000,
};

/* clk frequencies supported by bridge in Hz in case derived from DACP/N pin */
static const u32 ti_sn_bridge_dsiclk_lut[] = {
	468000000,
	384000000,
	416000000,
	486000000,
	460800000,
};

static void ti_sn_bridge_set_refclk_freq(struct ti_sn_bridge *pdata)
{
	int i;
	u32 refclk_rate;
	const u32 *refclk_lut;
	size_t refclk_lut_size;

	if (pdata->refclk) {
		refclk_rate = clk_get_rate(pdata->refclk);
		refclk_lut = ti_sn_bridge_refclk_lut;
		refclk_lut_size = ARRAY_SIZE(ti_sn_bridge_refclk_lut);
		clk_prepare_enable(pdata->refclk);
	} else {
		refclk_rate = ti_sn_bridge_get_dsi_freq(pdata) * 1000;
		refclk_lut = ti_sn_bridge_dsiclk_lut;
		refclk_lut_size = ARRAY_SIZE(ti_sn_bridge_dsiclk_lut);
	}

	/* for i equals to refclk_lut_size means default frequency */
	for (i = 0; i < refclk_lut_size; i++)
		if (refclk_lut[i] == refclk_rate)
			break;

	regmap_update_bits(pdata->regmap, SN_DPPLL_SRC_REG, REFCLK_FREQ_MASK,
			   REFCLK_FREQ(i));
}

/**
 * LUT index corresponds to register value and
 * LUT values corresponds to dp data rate supported
 * by the bridge in Mbps unit.
 */
static const unsigned int ti_sn_bridge_dp_rate_lut[] = {
	0, 1620, 2160, 2430, 2700, 3240, 4320, 5400
};

static void ti_sn_bridge_set_dsi_dp_rate(struct ti_sn_bridge *pdata)
{
	unsigned int bit_rate_mhz, clk_freq_mhz, dp_rate_mhz;
	unsigned int val, i;
	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;

	if ( ! pdata->dsi_freq ) {
		DRM_ERROR("Cannot obtain DSI frequency\n");
		return;
	}

	/* set DSIA clk frequency */
	bit_rate_mhz = (mode->clock / 1000) *
			mipi_dsi_pixel_format_to_bpp(pdata->dsi->format);

	clk_freq_mhz = pdata->dsi_freq / 1000;

	/* for each increment in val, frequency increases by 5MHz */
	val = (MIN_DSI_CLK_FREQ_MHZ / 5) +
		(((clk_freq_mhz - MIN_DSI_CLK_FREQ_MHZ) / 5) & 0xFF);
	regmap_write(pdata->regmap, SN_DSIA_CLK_FREQ_REG, val);

	/* set DP data rate */
	dp_rate_mhz = ((bit_rate_mhz / pdata->dsi->lanes) * DP_CLK_FUDGE_NUM) /
							DP_CLK_FUDGE_DEN;
	for (i = 0; i < ARRAY_SIZE(ti_sn_bridge_dp_rate_lut) - 1; i++)
		if (ti_sn_bridge_dp_rate_lut[i] > dp_rate_mhz)
			break;

	regmap_update_bits(pdata->regmap, SN_DATARATE_CONFIG_REG,
			   DP_DATARATE_MASK, DP_DATARATE(i));
}

static void ti_sn_bridge_set_video_timings(struct ti_sn_bridge *pdata)
{
	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;
	u8 hsync_polarity = 0, vsync_polarity = 0;

	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		hsync_polarity = CHA_HSYNC_POLARITY;
	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		vsync_polarity = CHA_VSYNC_POLARITY;

	ti_sn_bridge_write_u16(pdata, SN_CHA_ACTIVE_LINE_LENGTH_LOW_REG,
			       mode->hdisplay);
	ti_sn_bridge_write_u16(pdata, SN_CHA_VERTICAL_DISPLAY_SIZE_LOW_REG,
			       mode->vdisplay);
	regmap_write(pdata->regmap, SN_CHA_HSYNC_PULSE_WIDTH_LOW_REG,
		     (mode->hsync_end - mode->hsync_start) & 0xFF);
	regmap_write(pdata->regmap, SN_CHA_HSYNC_PULSE_WIDTH_HIGH_REG,
		     (((mode->hsync_end - mode->hsync_start) >> 8) & 0x7F) |
		     hsync_polarity);
	regmap_write(pdata->regmap, SN_CHA_VSYNC_PULSE_WIDTH_LOW_REG,
		     (mode->vsync_end - mode->vsync_start) & 0xFF);
	regmap_write(pdata->regmap, SN_CHA_VSYNC_PULSE_WIDTH_HIGH_REG,
		     (((mode->vsync_end - mode->vsync_start) >> 8) & 0x7F) |
		     vsync_polarity);

	regmap_write(pdata->regmap, SN_CHA_HORIZONTAL_BACK_PORCH_REG,
		     (mode->htotal - mode->hsync_end) & 0xFF);
	regmap_write(pdata->regmap, SN_CHA_VERTICAL_BACK_PORCH_REG,
		     (mode->vtotal - mode->vsync_end) & 0xFF);

	regmap_write(pdata->regmap, SN_CHA_HORIZONTAL_FRONT_PORCH_REG,
		     (mode->hsync_start - mode->hdisplay) & 0xFF);
	regmap_write(pdata->regmap, SN_CHA_VERTICAL_FRONT_PORCH_REG,
		     (mode->vsync_start - mode->vdisplay) & 0xFF);

	usleep_range(10000, 10500); /* 10ms delay recommended by spec */
}


static int ti_sn_bridge_enable_DP(struct ti_sn_bridge *pdata)
{
        unsigned int val = 0;
        int ret = 0;

        /* DSI_A lane config */
        val = CHA_DSI_LANES(4 - pdata->dsi->lanes);
        regmap_update_bits(pdata->regmap, SN_DSI_LANES_REG,
                           CHA_DSI_LANES_MASK, val);

        /* DP lane config */
        val = DP_NUM_LANES(pdata->dsi->lanes - 1);
        regmap_update_bits(pdata->regmap, SN_SSC_CONFIG_REG, DP_NUM_LANES_MASK,
                           val);

        /* set dsi/dp clk frequency value */
        ti_sn_bridge_set_dsi_dp_rate(pdata);

        /* enable DP PLL */
        regmap_write(pdata->regmap, SN_PLL_ENABLE_REG, 1);

        ret = regmap_read_poll_timeout(pdata->regmap, SN_DPPLL_SRC_REG, val,
                                       val & DPPLL_SRC_DP_PLL_LOCK, 1000,
                                       50 * 1000);
        if (ret) {
                DRM_ERROR("DP_PLL_LOCK polling failed (%d)\n", ret);
                return -ENODEV;
        }

        return 0;
}

static void ti_sn_bridge_disable_line(struct ti_sn_bridge *pdata)
{
        /* disable video stream */
        regmap_update_bits(pdata->regmap, SN_ENH_FRAME_REG, VSTREAM_ENABLE, 0);
        /* semi auto link training mode OFF */
        regmap_write(pdata->regmap, SN_ML_TX_MODE_REG, 0);
        /* disable DP PLL */
        regmap_write(pdata->regmap, SN_PLL_ENABLE_REG, 0);
}

static int ti_sn_bridge_enable_line(struct ti_sn_bridge *pdata)
{
        unsigned int val = 0;
        u8 assren = 0;
        int ret = 0;

        /**
         * The SN65DSI86 only supports ASSR Display Authentication method and
         * this method is enabled by default. An eDP panel must support this
         * authentication method. We need to enable this method in the eDP panel
         * at DisplayPort address 0x0010A prior to link training.
         */
        ret = drm_dp_dpcd_readb(&pdata->aux, DP_EDP_CONFIGURATION_CAP, &assren);
        if (ret < 0) {
                DRM_ERROR("AUX channel communication error, unable to check ASSR \n");
                return -ENODEV;
        } else {
                if (assren & 0x01) {
                        /* ASSR supported */
                        ret = drm_dp_dpcd_writeb(&pdata->aux, DP_EDP_CONFIGURATION_SET,
                                           DP_ALTERNATE_SCRAMBLER_RESET_ENABLE);
                        if (ret < 0) {
                                /* some communication error ?? */
                                DRM_ERROR("AUX channel communication error, unable to set ASSR \n");
                                return -ENODEV;
                        }
                        DRM_INFO("eDP capable monitor, set ASSR mode \n");
                } else {
                        /* ASSR not supported */
                        /* Disable ASSR; pin TEST2 must be set to 1p8V */
                        regmap_write(pdata->regmap, 0xff, 0b111); // select page 7
                        regmap_write(pdata->regmap, 0x16, 0x1); // ASSR OVERRIDE
                        regmap_write(pdata->regmap, 0xff, 0); // select page 0
                        regmap_update_bits(pdata->regmap, SN_ENH_FRAME_REG, GENMASK(1,0), 0); // DISABLE ASSR
                        DRM_INFO("DP only monitor, ASSR not supported, disable ASSR mode \n");
                }
        }

        /* Semi auto link training mode */
        regmap_write(pdata->regmap, SN_ML_TX_MODE_REG, 0x0A);
        ret = regmap_read_poll_timeout(pdata->regmap, SN_ML_TX_MODE_REG, val,
                                       val == ML_TX_MAIN_LINK_OFF ||
                                       val == ML_TX_NORMAL_MODE, 1000,
                                       500 * 1000);
        if (ret) {
                DRM_ERROR("Training complete polling failed (%d)\n", ret);
                return -ENODEV;
        } else if (val == ML_TX_MAIN_LINK_OFF) {
                DRM_ERROR("Link training failed, link is off\n");
                return -ENODEV;
        }

        return 0;
}



/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* HPD Helper Functions */
/* work thread + IRQ */

static struct ti_sn_bridge *
work_to_ti_sn_bridge(struct work_struct *work)
{
	return container_of(work, struct ti_sn_bridge, mw.work);
}

static void ti_sn_hpd_work_handler(struct work_struct *work)
{
	struct ti_sn_bridge *pdata = work_to_ti_sn_bridge(work);
	u32 val = 0;

	regmap_read(pdata->regmap, SN_HPDLINE_REG, &val);
	/* update connector status */
	if ((val & HPD_LINE_STATUS) != 0) {
		if (pdata->con_connected != connector_status_connected) {
			ti_sn_bridge_enable_line(pdata);
		}
		pdata->con_connected = connector_status_connected;
	} else {
		pdata->con_connected = connector_status_disconnected;
	}

	/* inform upper layers - will call connector detect */
	drm_helper_hpd_irq_event(pdata->connector.dev);

	/*re-enable IRQ */
	regmap_write(pdata->regmap, SN_IRQHPD_EN_REG, HPD_REMOVAL_IRQ_EN | HPD_INSERTION_IRQ_EN |
			     HPD_REPLUG_IRQ_EN);
	regmap_write(pdata->regmap, SN_IRQEN_REG, 0x01);
	enable_irq(pdata->hpd_irq);
}

/* IRQ HANDLER */
static irqreturn_t ti_sn_bridge_irq(int irq, void *dev_id)
{
	struct ti_sn_bridge *pdata = dev_id;
	unsigned int val = 0;
	int ret = 0;

	/* first disable IRQ */
	disable_irq_nosync(pdata->hpd_irq);

	/* read status register */
	ret = regmap_read(pdata->regmap, SN_IRQHPD_STATUS_REG, &val);
	regmap_write(pdata->regmap, SN_IRQHPD_STATUS_REG, val);

	/* Allow to re-debounce */
	/* Wake worker thread - do not stuck IRQ thread by long task */
	mod_delayed_work(system_wq, &pdata->mw, msecs_to_jiffies(300));

	return IRQ_HANDLED;
}

static void ti_sn_HPD_enable(struct ti_sn_bridge *pdata)
{
	int val = 0;

	/* enable HPD LINE */
	regmap_update_bits(pdata->regmap, SN_HPDLINE_REG, HPD_DISABLE, 0);
	/* wait for debounce */
	msleep(300);
	regmap_read(pdata->regmap, SN_HPDLINE_REG, &val);
	/* set connector status */
	if ((val & HPD_LINE_STATUS) != 0) {
		pdata->con_connected = connector_status_connected;
	} else {
		pdata->con_connected = connector_status_disconnected;
	}

	/* read IRQ status register */
	regmap_read(pdata->regmap, SN_IRQHPD_STATUS_REG, &val);
	/* if IRQ line is asserted - de-assert it */
	regmap_write(pdata->regmap, SN_IRQHPD_STATUS_REG, val);
	/*enable IRQ */
	regmap_write(pdata->regmap, SN_IRQHPD_EN_REG, HPD_REMOVAL_IRQ_EN | HPD_INSERTION_IRQ_EN |
			     HPD_REPLUG_IRQ_EN);
	regmap_write(pdata->regmap, SN_IRQEN_REG, 0x01);
	enable_irq(pdata->hpd_irq);

	pdata->hpd_enabled = true;
}

static void ti_sn_HPD_disable(struct ti_sn_bridge *pdata)
{
	int val = 0;

	/* read IRQ status register */
	regmap_read(pdata->regmap, SN_IRQHPD_STATUS_REG, &val);
	/* if IRQ line is asserted - de-assert it */
	regmap_write(pdata->regmap, SN_IRQHPD_STATUS_REG, val);
	/*disable IRQ */
	regmap_write(pdata->regmap, SN_IRQHPD_EN_REG, 0x00);
	regmap_write(pdata->regmap, SN_IRQEN_REG, 0x00);
	disable_irq(pdata->hpd_irq);

	pdata->hpd_enabled = false;
	/* connector status unknown */
	pdata->con_connected = connector_status_unknown;
}

static int ti_sn_HPD_init(struct ti_sn_bridge *pdata,
							struct i2c_client *client,
							const struct i2c_device_id *id)
{
	int ret = 0;

	/* HPD IRQ */
	pdata->hpd_irq = client->irq;
	ret = devm_request_threaded_irq(pdata->dev, pdata->hpd_irq, NULL,
						ti_sn_bridge_irq,
						IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
						dev_name(pdata->dev), pdata);
	if (ret < 0)
		return ret;
	disable_irq(pdata->hpd_irq);
	pdata->con_connected = connector_status_unknown;
	INIT_DELAYED_WORK(&pdata->mw, ti_sn_hpd_work_handler);

	return ret;
}


/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* Connector funcs */

static struct ti_sn_bridge *
connector_to_ti_sn_bridge(struct drm_connector *connector)
{
	return container_of(connector, struct ti_sn_bridge, connector);
}

static int ti_sn_bridge_connector_get_modes(struct drm_connector *connector)
{
	struct ti_sn_bridge *pdata = connector_to_ti_sn_bridge(connector);
	struct edid *pedid = NULL;
	int count = 0;
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;

	if (pdata->ddc_enabled) {
		pm_runtime_get_sync(pdata->dev);
		if (!pdata->hpd_enabled) {
			ti_sn_bridge_set_refclk_freq(pdata);
			msleep(300);
		}
		pedid = drm_get_edid(connector, &pdata->aux.ddc);
		pm_runtime_put_sync(pdata->dev);
		drm_connector_update_edid_property(connector, pedid);
		if (pedid) {
			count = drm_add_edid_modes(connector, pedid);
			kfree(pedid);
		}
		drm_display_info_set_bus_formats(&connector->display_info,
							 &bus_format, 1);
	}

	if (!count) {
		count = drm_panel_get_modes(pdata->panel);
	}

	return count;
}

static enum drm_mode_status
ti_sn_bridge_connector_mode_valid(struct drm_connector *connector,
				  struct drm_display_mode *mode)
{
	/* maximum supported resolution is 4K at 60 fps */
	if (mode->clock > 594000)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static struct drm_connector_helper_funcs ti_sn_bridge_connector_helper_funcs = {
	.get_modes = ti_sn_bridge_connector_get_modes,
	.mode_valid = ti_sn_bridge_connector_mode_valid,
};

static enum drm_connector_status
ti_sn_bridge_connector_detect(struct drm_connector *connector, bool force)
{
	struct ti_sn_bridge *pdata = connector_to_ti_sn_bridge(connector);
	u32 val;

	if (pdata->hpd_supported) {
		if (!pdata->hpd_enabled) {
			/* Not enabled yet - read that directly */
			pm_runtime_get_sync(pdata->dev);
			/* enable HPD LINE */
			regmap_update_bits(pdata->regmap, SN_HPDLINE_REG, HPD_DISABLE, 0);
			/* wait for debounce */
			msleep(300);
			regmap_read(pdata->regmap, SN_HPDLINE_REG, &val);
			/* set connector status */
			if ((val & HPD_LINE_STATUS) != 0) {
				pdata->con_connected = connector_status_connected;
			} else {
				pdata->con_connected = connector_status_disconnected;
			}
			pm_runtime_put_sync(pdata->dev);
		}
	} else {
		/* systems without HPD are considered always connected */
		pdata->con_connected = connector_status_connected;
	}

	return pdata->con_connected;
}

static const struct drm_connector_funcs ti_sn_bridge_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = ti_sn_bridge_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* Bridge funcs */

static struct ti_sn_bridge *bridge_to_ti_sn_bridge(struct drm_bridge *bridge)
{
	return container_of(bridge, struct ti_sn_bridge, bridge);
}

static int ti_sn_bridge_parse_regulators(struct ti_sn_bridge *pdata)
{
	unsigned int i;
	const char * const ti_sn_bridge_supply_names[] = {
		"vcca", "vcc", "vccio", "vpll",
	};

	for (i = 0; i < SN_REGULATOR_SUPPLY_NUM; i++)
		pdata->supplies[i].supply = ti_sn_bridge_supply_names[i];

	return devm_regulator_bulk_get(pdata->dev, SN_REGULATOR_SUPPLY_NUM,
				       pdata->supplies);
}

static int ti_sn_bridge_attach(struct drm_bridge *bridge)
{
	int ret;
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	const struct mipi_dsi_device_info info = { .type = "ti_sn_bridge",
						   .channel = 0,
						   .node = NULL,
						 };

	if (pdata->hpd_supported) {
		pdata->connector.polled = DRM_CONNECTOR_POLL_HPD;
	}

	if (pdata->edp_assr) {
		ret = drm_connector_init(bridge->dev, &pdata->connector,
					 &ti_sn_bridge_connector_funcs,
					 DRM_MODE_CONNECTOR_eDP);
	} else {
		ret = drm_connector_init(bridge->dev, &pdata->connector,
					 &ti_sn_bridge_connector_funcs,
					 DRM_MODE_CONNECTOR_DisplayPort);
	}
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(&pdata->connector,
				 &ti_sn_bridge_connector_helper_funcs);
	drm_connector_attach_encoder(&pdata->connector, bridge->encoder);

	/*
	 * TODO: ideally finding host resource and dsi dev registration needs
	 * to be done in bridge probe. But some existing DSI host drivers will
	 * wait for any of the drm_bridge/drm_panel to get added to the global
	 * bridge/panel list, before completing their probe. So if we do the
	 * dsi dev registration part in bridge probe, before populating in
	 * the global bridge list, then it will cause deadlock as dsi host probe
	 * will never complete, neither our bridge probe. So keeping it here
	 * will satisfy most of the existing host drivers. Once the host driver
	 * is fixed we can move the below code to bridge probe safely.
	 */
	host = of_find_mipi_dsi_host_by_node(pdata->host_node);
	if (!host) {
		DRM_ERROR("failed to find dsi host\n");
		ret = -ENODEV;
		goto err_dsi_host;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		DRM_ERROR("failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		goto err_dsi_host;
	}

	/* TODO: setting to 4 lanes always for now */
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE | MIPI_DSI_MODE_VIDEO_HFP |
	                  MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE;

	pdata->dsi_freq = 0;
	dsi->export_dsi_freq = &(pdata->dsi_freq);

	/* check if continuous dsi clock is required or not */
	/*pm_runtime_get_sync(pdata->dev);
	regmap_read(pdata->regmap, SN_DPPLL_SRC_REG, &val);
	pm_runtime_put(pdata->dev);
	if (!(val & DPPLL_CLK_SRC_DSICLK))
		dsi->mode_flags |= MIPI_DSI_CLOCK_NON_CONTINUOUS;*/

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		DRM_ERROR("failed to attach dsi to host\n");
		goto err_dsi_attach;
	}
	pdata->dsi = dsi;

	/* attach panel to bridge */
	drm_panel_attach(pdata->panel, &pdata->connector);

	return 0;

err_dsi_attach:
	mipi_dsi_device_unregister(dsi);
err_dsi_host:
	drm_connector_cleanup(&pdata->connector);
	return ret;
}

static void ti_sn_bridge_disable(struct drm_bridge *bridge)
{
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);

	drm_panel_disable(pdata->panel);

	ti_sn_bridge_disable_line(pdata);

	drm_panel_unprepare(pdata->panel);
}

static void ti_sn_bridge_enable(struct drm_bridge *bridge)
{
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);
	int ret;

	ret = ti_sn_bridge_enable_DP(pdata);
	if (ret) {
		ti_sn_bridge_disable(bridge);
		return;
	}

	ret = ti_sn_bridge_enable_line(pdata);
	if (ret) {
		ti_sn_bridge_disable(bridge);
		return;
	}

	/* config video parameters */
	ti_sn_bridge_set_video_timings(pdata);

	/* enable video stream */
	regmap_update_bits(pdata->regmap, SN_ENH_FRAME_REG, VSTREAM_ENABLE,
			   VSTREAM_ENABLE);

	drm_panel_enable(pdata->panel);
}

static void ti_sn_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);

	pm_runtime_get_sync(pdata->dev);

	/* configure bridge ref_clk */
	ti_sn_bridge_set_refclk_freq(pdata);


	//todo: need to add a condition for HPD
	ti_sn_HPD_enable(pdata);
	drm_panel_prepare(pdata->panel);
}

static void ti_sn_bridge_post_disable(struct drm_bridge *bridge)
{
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);

	ti_sn_HPD_disable(pdata);

	if (pdata->refclk)
		clk_disable_unprepare(pdata->refclk);

	pm_runtime_put_sync(pdata->dev);
}

static const struct drm_bridge_funcs ti_sn_bridge_funcs = {
	.attach = ti_sn_bridge_attach,
	.pre_enable = ti_sn_bridge_pre_enable,
	.enable = ti_sn_bridge_enable,
	.disable = ti_sn_bridge_disable,
	.post_disable = ti_sn_bridge_post_disable,
};

/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* AUX i2c transfer */

static struct ti_sn_bridge *aux_to_ti_sn_bridge(struct drm_dp_aux *aux)
{
	return container_of(aux, struct ti_sn_bridge, aux);
}

static ssize_t ti_sn_aux_transfer(struct drm_dp_aux *aux,
				  struct drm_dp_aux_msg *msg)
{
	struct ti_sn_bridge *pdata = aux_to_ti_sn_bridge(aux);
	u32 request = msg->request & ~DP_AUX_I2C_MOT;
	u32 request_val = AUX_CMD_REQ(msg->request);
	u8 *buf = (u8 *)msg->buffer;
	unsigned int val;
	int ret, i;

	if (msg->size > SN_AUX_MAX_PAYLOAD_BYTES)
		return -EINVAL;

	switch (request) {
	case DP_AUX_NATIVE_WRITE:
	case DP_AUX_I2C_WRITE:
	case DP_AUX_NATIVE_READ:
	case DP_AUX_I2C_READ:
		regmap_write(pdata->regmap, SN_AUX_CMD_REG, request_val);
		break;
	default:
		return -EINVAL;
	}

	regmap_write(pdata->regmap, SN_AUX_ADDR_19_16_REG,
		     (msg->address >> 16) & 0xF);
	regmap_write(pdata->regmap, SN_AUX_ADDR_15_8_REG,
		     (msg->address >> 8) & 0xFF);
	regmap_write(pdata->regmap, SN_AUX_ADDR_7_0_REG, msg->address & 0xFF);

	regmap_write(pdata->regmap, SN_AUX_LENGTH_REG, msg->size);

	if (request == DP_AUX_NATIVE_WRITE || request == DP_AUX_I2C_WRITE) {
		for (i = 0; i < msg->size; i++)
			regmap_write(pdata->regmap, SN_AUX_WDATA_REG(i),
				     buf[i]);
	}

	regmap_write(pdata->regmap, SN_AUX_CMD_REG, request_val | AUX_CMD_SEND);

	ret = regmap_read_poll_timeout(pdata->regmap, SN_AUX_CMD_REG, val,
				       !(val & AUX_CMD_SEND), 200,
				       50 * 1000);
	if (ret)
		return ret;

	ret = regmap_read(pdata->regmap, SN_AUX_CMD_STATUS_REG, &val);
	if (ret)
		return ret;
	else if ((val & AUX_IRQ_STATUS_NAT_I2C_FAIL)
		 || (val & AUX_IRQ_STATUS_AUX_RPLY_TOUT)
		 || (val & AUX_IRQ_STATUS_AUX_SHORT))
		return -ENXIO;

	if (request == DP_AUX_NATIVE_WRITE || request == DP_AUX_I2C_WRITE)
		return msg->size;

	for (i = 0; i < msg->size; i++) {
		unsigned int val;
		ret = regmap_read(pdata->regmap, SN_AUX_RDATA_REG(i),
				  &val);
		if (ret)
			return ret;

		WARN_ON(val & ~0xFF);
		buf[i] = (u8)(val & 0xFF);
	}

	return msg->size;
}

/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* Driver probe and remove */

static int ti_sn_bridge_parse_dsi_host(struct ti_sn_bridge *pdata)
{
	struct device_node *np = pdata->dev->of_node;

	pdata->host_node = of_graph_get_remote_node(np, 0, 0);

	if (!pdata->host_node) {
		DRM_ERROR("remote dsi host node not found\n");
		return -ENODEV;
	}

	return 0;
}

static int ti_sn_bridge_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct ti_sn_bridge *pdata;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DRM_ERROR("device doesn't support I2C\n");
		return -ENODEV;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(struct ti_sn_bridge),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->regmap = devm_regmap_init_i2c(client,
					     &ti_sn_bridge_regmap_config);
	if (IS_ERR(pdata->regmap)) {
		DRM_ERROR("regmap i2c init failed\n");
		return PTR_ERR(pdata->regmap);
	}

	pdata->dev = &client->dev;

	ret = drm_of_find_panel_or_bridge(pdata->dev->of_node, 1, 0,
					  &pdata->panel, NULL);
	if (ret) {
		DRM_ERROR("could not find any panel node\n");
		return ret;
	}

	dev_set_drvdata(&client->dev, pdata);

	pdata->enable_gpio = devm_gpiod_get(pdata->dev, "enable",
					    GPIOD_OUT_LOW);
	if (IS_ERR(pdata->enable_gpio)) {
		DRM_ERROR("failed to get enable gpio from DT\n");
		ret = PTR_ERR(pdata->enable_gpio);
		return ret;
	}

	ret = ti_sn_bridge_parse_regulators(pdata);
	if (ret) {
		DRM_ERROR("failed to parse regulators\n");
		return ret;
	}

	pdata->refclk = devm_clk_get(pdata->dev, "refclk");
	if (IS_ERR(pdata->refclk)) {
		ret = PTR_ERR(pdata->refclk);
		if (ret == -EPROBE_DEFER)
			return ret;
		DRM_DEBUG_KMS("refclk not found\n");
		pdata->refclk = NULL;
	}

	ret = ti_sn_bridge_parse_dsi_host(pdata);
	if (ret)
		return ret;

	pm_runtime_enable(pdata->dev);

	i2c_set_clientdata(client, pdata);

	pdata->aux.name = "ti-sn65dsi86-aux";
	pdata->aux.dev = pdata->dev;
	pdata->aux.transfer = ti_sn_aux_transfer;
	drm_dp_aux_register(&pdata->aux);

	/* other features: ddc - to read EDID from display, hpd - to detect display hotplug, assr - to enable eDP assr */
	/* NOTE: to use DDC it is necessary to have reference clock available */
	pdata->ddc_enabled = of_property_read_bool(pdata->dev->of_node, "ddc-edid-enable") && (pdata->refclk);
	pdata->hpd_supported = of_property_read_bool(pdata->dev->of_node, "hpd-enable");
	pdata->edp_assr = of_property_read_bool(pdata->dev->of_node, "edp-assr");

	DRM_INFO("TI_SN65DSI84: HPD_support: %d", pdata->hpd_supported);


	if (pdata->hpd_supported) {
		ret = ti_sn_HPD_init(pdata, client, id);
			if (ret) {
				pdata->hpd_supported = false;
			}
	}

	pdata->bridge.funcs = &ti_sn_bridge_funcs;
	pdata->bridge.of_node = client->dev.of_node;

	drm_bridge_add(&pdata->bridge);

	ti_sn_debugfs_init(pdata);

	return 0;
}

static int ti_sn_bridge_remove(struct i2c_client *client)
{
	struct ti_sn_bridge *pdata = i2c_get_clientdata(client);

	if (!pdata)
		return -EINVAL;

	ti_sn_debugfs_remove(pdata);

	of_node_put(pdata->host_node);

	pm_runtime_disable(pdata->dev);

	if (pdata->dsi) {
		mipi_dsi_detach(pdata->dsi);
		mipi_dsi_device_unregister(pdata->dsi);
	}

	drm_bridge_remove(&pdata->bridge);

	return 0;
}

static struct i2c_device_id ti_sn_bridge_id[] = {
	{ "ti,sn65dsi86", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ti_sn_bridge_id);

static const struct of_device_id ti_sn_bridge_match_table[] = {
	{.compatible = "ti,sn65dsi86"},
	{},
};
MODULE_DEVICE_TABLE(of, ti_sn_bridge_match_table);

static struct i2c_driver ti_sn_bridge_driver = {
	.driver = {
		.name = "ti_sn65dsi86",
		.of_match_table = ti_sn_bridge_match_table,
		.pm = &ti_sn_bridge_pm_ops,
	},
	.probe = ti_sn_bridge_probe,
	.remove = ti_sn_bridge_remove,
	.id_table = ti_sn_bridge_id,
};
module_i2c_driver(ti_sn_bridge_driver);

MODULE_AUTHOR("Sandeep Panda <spanda@codeaurora.org>");
MODULE_DESCRIPTION("sn65dsi86 DSI to eDP bridge driver");
MODULE_LICENSE("GPL v2");
