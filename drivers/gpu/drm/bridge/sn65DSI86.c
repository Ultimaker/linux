// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 */

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <video/videomode.h>

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

#define HPD_ENABLED
#ifdef HPD_ENABLED
	/* added to service IRQ, debug .... */
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
#endif //HPD_ENABLED

/* debugging */
// .....define this to enable debug prints:  #define TI_SN_DEBUG 1

#define MIN_DSI_CLK_FREQ_MHZ	40
#define DSI_LANES 4
#define DSI_BUS_CLK 594000000


/* fudge factor required to account for 8b/10b encoding */
#define DP_CLK_FUDGE_NUM	10
#define DP_CLK_FUDGE_DEN	8

/* Matches DP_AUX_MAX_PAYLOAD_BYTES (for now) */
#define SN_AUX_MAX_PAYLOAD_BYTES	16

#define SN_REGULATOR_SUPPLY_NUM		4

typedef enum {
	DP_EDID_NONE,
	DP_EDID_TRY,
	DP_EDID_USE
} dp_edid_t;

struct ti_sn_bridge {
	struct device			*dev;
	struct regmap			*regmap;
	struct drm_dp_aux		aux;
	struct drm_bridge		bridge;
	struct drm_connector	connector;
	bool enabled;
#ifdef HPD_ENABLED
	struct delayed_work mw;
	bool plugged;
	int hpd_irq;
#endif
	struct device_node		*host_node;
	struct mipi_dsi_device		*dsi;
	u32 forced_dsi_freq;
#ifdef SN65DSI86_USE_CLK
	struct clk			*refclk;
#endif
#ifdef SN65DSI86_USE_PANEL
	struct drm_panel		*panel;
#else
	struct videomode 	of_mode;
	struct edid 		*gen_edid;
	dp_edid_t 			edid_detect;
	bool 				edid_valid;
	bool 				mode_valid;
#endif
	struct gpio_desc	*enable_gpio;
#ifdef SN65DSI86_USE_REGULATORS
	struct regulator_bulk_data	supplies[SN_REGULATOR_SUPPLY_NUM];
#endif
};

/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
#ifdef CONFIG_DRM_TI_DEBUG
	#define TI_SN_SYSFS 1
	#ifdef CONFIG_DRM_TI_PRNDEBUG
		#define TI_SN_DBGPRN 1
	#endif
#endif

#ifdef TI_SN_SYSFS
typedef struct {
	struct videomode mode; 		// current video mode
	struct videomode of_mode;	// mode got from dtb
	bool of_mode_valid;			//
	struct edid edid;			// active edid
	bool edid_status;			// is edid read from monitor
	bool connector_status; 		// plug status
	bool bridge_status;    		// bridge state enabled/disabled
	u32 DSI_freq;				// forced DSI frequency
} ti_debug_t;

static ti_debug_t dbg = {};


static void ti_sn_dbg_dtb_mode_log(struct videomode *mode)
{
#ifdef TI_SN_DBGPRN
	pr_info("%s-%s: Mode read from dtb \n", __FILE__, __func__);
	pr_info("%s-%s: 	Pixel clock: %lu \n", __FILE__, __func__, (mode->pixelclock));
	pr_info("%s-%s: 	Horizontal:  %d \n", __FILE__, __func__, mode->hactive);
	pr_info("%s-%s: 	Vertical:    %d \n", __FILE__, __func__, mode->vactive);
#endif

	memcpy((void *)&dbg.of_mode, (void *) mode, sizeof(struct videomode));
	dbg.of_mode_valid = true;
}


static void ti_sn_dbg_mode_log(struct drm_display_mode *mode)
{
#ifdef TI_SN_DBGPRN
	pr_info("%s-%s: Bridge MODE SET \n", __FILE__, __func__);
	pr_info("%s-%s: Pixel clock: %d \n", __FILE__, __func__, (mode->clock * 1000));
	pr_info("%s-%s: Horizontal:  %d \n", __FILE__, __func__, mode->hdisplay);
	pr_info("%s-%s: Vertical:    %d \n", __FILE__, __func__, mode->vscan);
#endif

	dbg.mode.pixelclock = mode->clock * 1000;
	dbg.mode.hactive = mode->hdisplay;
	dbg.mode.hfront_porch = mode->hsync_start - mode->hdisplay;
	dbg.mode.hsync_len = mode->hsync_end - mode->hsync_start;
	dbg.mode.hback_porch = mode->htotal - mode->hsync_end;
	dbg.mode.vactive = mode->vdisplay;
	dbg.mode.vfront_porch = mode->vsync_start - mode->vdisplay;
	dbg.mode.vsync_len = mode->vsync_end - mode->vsync_start;
	dbg.mode.vback_porch = mode->vtotal - mode->vsync_end;
}

static void ti_sn_dbg_edid_log(struct edid *pedid, bool monitor)
{
#ifdef TI_SN_DBGPRN
	u32 hactive, vactive;

	if (monitor) {
		pr_info("%s-%s: Got EDID from monitor \n", __FILE__, __func__);
	} else {
		pr_info("%s-%s: EDID - default from DTB \n", __FILE__, __func__);
	}

	hactive = (((u32)(pedid->detailed_timings[0].data.pixel_data.hactive_hblank_hi & 0xf0)) << 4) | ((u32)(pedid->detailed_timings[0].data.pixel_data.hactive_lo));
	vactive = (((u32)(pedid->detailed_timings[0].data.pixel_data.vactive_vblank_hi & 0xf0)) << 4) | ((u32)(pedid->detailed_timings[0].data.pixel_data.vactive_lo));
	pr_info("%s-%s: Pixel clock: %d \n", __FILE__, __func__, (pedid->detailed_timings[0].pixel_clock * 10000));
	pr_info("%s-%s: Horizontal: %d \n", __FILE__, __func__, hactive);
	pr_info("%s-%s: Vertical  : %d \n", __FILE__, __func__, vactive);
#endif

	dbg.edid_status = monitor;
	memcpy((void *)&dbg.edid, (void *) pedid, sizeof(struct edid));
}

static void ti_sn_dbg_connector_status(struct ti_sn_bridge *pdata, bool hpd)
{
#ifdef TI_SN_DBGPRN
	if (hpd) {
		pr_info("%s-%s: connector HPD Event\n", __FILE__, __func__);
	} else {
		pr_info("%s-%s: connector query Event\n", __FILE__, __func__);
	}
	pr_info("%s-%s: connector status is - %d \n", __FILE__, __func__, pdata->plugged);
#endif
	dbg.connector_status = pdata->plugged;
}

static void ti_sn_dbg_bridge_status(struct ti_sn_bridge *pdata)
{
#ifdef TI_SN_DBGPRN
	if (dbg.bridge_status != pdata->enabled) {
		pr_info("%s-%s: bridge status Event\n", __FILE__, __func__);
		if (pdata->enabled) {
			pr_info("%s-%s: bridge going to be enabled\n", __FILE__, __func__);
		} else {
			pr_info("%s-%s: bridge going to be disabled\n", __FILE__, __func__);
		}
	} else {
		pr_info("%s-%s: bridge status query\n", __FILE__, __func__);
		if (dbg.bridge_status) {
			pr_info("%s-%s: bridge enabled\n", __FILE__, __func__);
		} else {
			pr_info("%s-%s: bridge disabled\n", __FILE__, __func__);
		}
	}
#endif

	dbg.bridge_status = pdata->enabled;
}

static void ti_sn_dbg_dsifreq_log(struct ti_sn_bridge *pdata)
{
#ifdef TI_SN_DBGPRN
	pr_info("%s-%s: FORCE DSI FREQ to %d \n", __FILE__, __func__, pdata->forced_dsi_freq);
#endif
	dbg.DSI_freq = pdata->forced_dsi_freq;
}

#else

static inline void ti_sn_dbg_dtb_mode_log(struct videomode *mode) {}
static inline void ti_sn_dbg_mode_log(struct drm_display_mode *mode) {}
static inline void ti_sn_dbg_edid_log(struct edid *pedid, bool monitor) {}
static inline void ti_sn_dbg_connector_status(struct ti_sn_bridge *pdata, bool hpd) {}
static inline void ti_sn_dbg_bridge_status(struct ti_sn_bridge *pdata) {}
static inline void ti_sn_dbg_dsifreq_log(struct ti_sn_bridge *pdata) {}

#endif //TI_SN_DEBUG
/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* regmap api */

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
/* Power management functions */

static int __maybe_unused ti_sn_bridge_resume(struct device *dev)
{
	struct ti_sn_bridge *pdata = dev_get_drvdata(dev);
	int ret = 0;

#ifdef SN65DSI86_USE_REGULATORS
	ret = regulator_bulk_enable(SN_REGULATOR_SUPPLY_NUM, pdata->supplies);
	if (ret) {
		DRM_ERROR("failed to enable supplies %d\n", ret);
		return ret;
	}
#endif

	gpiod_set_value(pdata->enable_gpio, 1);

	return ret;
}

static int __maybe_unused ti_sn_bridge_suspend(struct device *dev)
{
	struct ti_sn_bridge *pdata = dev_get_drvdata(dev);
	int ret = 0;

	gpiod_set_value(pdata->enable_gpio, 0);

#ifdef SN65DSI86_USE_REGULATORS
	ret = regulator_bulk_disable(SN_REGULATOR_SUPPLY_NUM, pdata->supplies);
	if (ret)
		DRM_ERROR("failed to disable supplies %d\n", ret);
#endif

	return ret;
}

static const struct dev_pm_ops ti_sn_bridge_pm_ops = {
	SET_RUNTIME_PM_OPS(ti_sn_bridge_suspend, ti_sn_bridge_resume, NULL)
};

/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* Helper functions  - EDID checksum calc., Pixel clock adjust, Force DSI clock */

/* calculate EDID checksum */
static void ti_sn_bridge_edid_chsum(struct edid *eptr)
{
    int i;
    u8 *ptr = (u8*) (eptr);
    u8 sum = 0;

    for (i = 0; i < (sizeof(struct edid) - 1); i++) {
            sum += *(ptr++);
    }
    eptr->checksum = 1 + (~sum);
}

#define MAX_UP_INC 2500000 /* allow increase of pixclk frequency about 2.5MHz max */
#define MIN_PIXCLK 25000000 /* minimal allowed pixel clock */

/* round pixel clock to be divisible from DSI_BUS_CLK */
static u32 ti_sn_bridge_adjust_pixclk(u32 oldpixclk)
{
	u32 divider, newpixclk;

	divider = DIV_ROUND_CLOSEST(DSI_BUS_CLK, oldpixclk);
	newpixclk = (DSI_BUS_CLK / divider);
	if ((newpixclk > oldpixclk) && ((newpixclk - oldpixclk) > MAX_UP_INC)) {
		divider ++;
		newpixclk = (DSI_BUS_CLK / divider);
	}

	if (newpixclk < (MIN_PIXCLK)) {
		divider --;
		newpixclk = (DSI_BUS_CLK / divider);
	}
	return newpixclk;
}


/* from sec-dsim.c; if not supported just do nothing  */
extern void sec_mipi_force_freq(u32 freq);
__attribute__((weak)) void sec_mipi_force_freq(u32 freq) 
{
        return;
}


static void ti_snbridge_force_DSI_freq(struct ti_sn_bridge *pdata)
{
	/* note:
	 * hardcoded 594MHz as LCDIF uses the same clk source,
	 * with that setting LCDIF out should be in sync with DSI
	 * with different setting DSI frequency introduces jitter */

	ti_sn_dbg_dsifreq_log(pdata);
	// pdata->forced_dsi_freq = 594000000;
	sec_mipi_force_freq(DIV_ROUND_UP(pdata->forced_dsi_freq, 1000) << 1); /* freq * 2 - DSI transfer two bits at one clock (on rising and falling edge)*/
}

/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* Display Port Configuration - enable/disable, line train */


#ifdef SN65DSI86_USE_CLK
static u32 ti_sn_get_dsi_freq(struct ti_sn_bridge *pdata)
{
	u32 bit_rate_khz, clk_freq_khz;
	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;

	bit_rate_khz = mode->clock *
			mipi_dsi_pixel_format_to_bpp(pdata->dsi->format);
	clk_freq_khz = bit_rate_khz / (pdata->dsi->lanes * 2);

	return clk_freq_khz;
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

static void ti_sn_set_refclk_freq(struct ti_sn_bridge *pdata)
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
		refclk_rate = ti_sn_get_dsi_freq(pdata) * 1000;
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

#else //SN65DSI86_USE_CLK

static void ti_sn_set_refclk_freq(struct ti_sn_bridge *pdata)
{
	/*ROPA: we have fixed 27MHz clock */
	regmap_update_bits(pdata->regmap, SN_DPPLL_SRC_REG, REFCLK_FREQ_MASK,
			   REFCLK_FREQ(3));
}

#endif //SN65DSI86_USE_CLK

/**
 * LUT index corresponds to register value and
 * LUT values corresponds to dp data rate supported
 * by the bridge in Mbps unit.
 */
static const unsigned int ti_sn_dp_rate_lut[] = {
	0, 1620, 2160, 2430, 2700, 3240, 4320, 5400
};

static void ti_sn_set_dsi_dp_rate(struct ti_sn_bridge *pdata)
{
	unsigned int bit_rate_mhz, clk_freq_mhz, dp_rate_mhz;
	unsigned int val, i;
	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;

	/* set DSIA clk frequency */
	/* + added 1MHz if clock has fraction of MHz */
	bit_rate_mhz = ((mode->clock / 1000) + ((mode->clock % 1000) ? 1 : 0)) *
			mipi_dsi_pixel_format_to_bpp(pdata->dsi->format);
	if (pdata->forced_dsi_freq > 0)
	{
		clk_freq_mhz = pdata->forced_dsi_freq / 1000000;  /*ROPA: forced_dsi_freq is in Hz; mode->clock is in kHz */
	} else {
		clk_freq_mhz = bit_rate_mhz / (pdata->dsi->lanes * 2);
	}

	/* for each increment in val, frequency increases by 5MHz */
	val = (MIN_DSI_CLK_FREQ_MHZ / 5) +
		(((clk_freq_mhz - MIN_DSI_CLK_FREQ_MHZ) / 5) & 0xFF);
	regmap_write(pdata->regmap, SN_DSIA_CLK_FREQ_REG, val);

	/* set DP data rate */
	dp_rate_mhz = ((bit_rate_mhz / pdata->dsi->lanes) * DP_CLK_FUDGE_NUM) /
							DP_CLK_FUDGE_DEN;
	for (i = 0; i < ARRAY_SIZE(ti_sn_dp_rate_lut) - 1; i++)
		if (ti_sn_dp_rate_lut[i] > dp_rate_mhz)
			break;

	regmap_update_bits(pdata->regmap, SN_DATARATE_CONFIG_REG,
			   DP_DATARATE_MASK, DP_DATARATE(i));
}


static int ti_sn_enable_DP(struct ti_sn_bridge *pdata)
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
	ti_sn_set_dsi_dp_rate(pdata);

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

static void ti_sn_disable_line(struct ti_sn_bridge *pdata)
{
	/* disable video stream */
	regmap_update_bits(pdata->regmap, SN_ENH_FRAME_REG, VSTREAM_ENABLE, 0);
	/* semi auto link training mode OFF */
	regmap_write(pdata->regmap, SN_ML_TX_MODE_REG, 0);
	/* disable DP PLL */
	regmap_write(pdata->regmap, SN_PLL_ENABLE_REG, 0);
}

static int ti_sn_enable_line(struct ti_sn_bridge *pdata)
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

static void ti_sn_set_video_timings(struct ti_sn_bridge *pdata)
{
	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;
	u8 hsync_polarity = 0, vsync_polarity = 0;

	if (pdata->of_mode.flags & DISPLAY_FLAGS_HSYNC_LOW) {
		hsync_polarity = CHA_HSYNC_POLARITY;
	}

	if (pdata->of_mode.flags & DISPLAY_FLAGS_VSYNC_LOW) {
		vsync_polarity = CHA_VSYNC_POLARITY;
	}

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

/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* DRM Display Port Auxiliary transfer functions */

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
	unsigned int val = 0;
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
/* HPD Helper Functions */
/* work thread + IRQ */
#ifdef HPD_ENABLED

static struct ti_sn_bridge *
work_to_ti_sn_bridge(struct work_struct *work)
{
	return container_of(work, struct ti_sn_bridge, mw.work);
}

static void ti_sn_bridge_enable(struct drm_bridge *bridge);
static void ti_sn_bridge_disable(struct drm_bridge *bridge);


static void ti_sn_hpd_work_handler(struct work_struct *work)
{
	struct ti_sn_bridge *pdata = work_to_ti_sn_bridge(work);
	u32 val = 0;

	regmap_read(pdata->regmap, SN_HPDLINE_REG, &val);
	/* update connector status */
	if (pdata->plugged != ((val & HPD_LINE_STATUS) != 0)) {
		/* changed */
		pdata->plugged = (val & HPD_LINE_STATUS);
		/* force enable or disable bridge - it seems that Wayland ignore
		 * connector status .... */
		if (pdata->plugged) {
			/* plugged enable DP and train line */
		    //ti_sn_bridge_enable(&pdata->bridge);
			ti_sn_enable_line(pdata);
		} else {
			/* unplugged disable DP */
			// ti_sn_bridge_disable(&pdata->bridge);
		}
	}

	ti_sn_dbg_connector_status(pdata, true);

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
	pdata->plugged = ((val & HPD_LINE_STATUS) != 0);

	/* read IRQ status register */
	regmap_read(pdata->regmap, SN_IRQHPD_STATUS_REG, &val);
	/* if IRQ line is asserted - de-assert it */
	regmap_write(pdata->regmap, SN_IRQHPD_STATUS_REG, val);
	/*enable IRQ */
	regmap_write(pdata->regmap, SN_IRQHPD_EN_REG, HPD_REMOVAL_IRQ_EN | HPD_INSERTION_IRQ_EN |
			     HPD_REPLUG_IRQ_EN);
	regmap_write(pdata->regmap, SN_IRQEN_REG, 0x01);
	enable_irq(pdata->hpd_irq);
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

	/* fix data */
	pdata->plugged = false;
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
	pdata->plugged = false;
	INIT_DELAYED_WORK(&pdata->mw, ti_sn_hpd_work_handler);

	return ret;
}


#endif //HPD_ENABLED
/****************************************************************************************/
/****************************************************************************************/
/*							DRM Layer													*/
/****************************************************************************************/
/****************************************************************************************/
/* DRM Connector funcs */

static struct ti_sn_bridge *
connector_to_ti_sn_bridge(struct drm_connector *connector)
{
	return container_of(connector, struct ti_sn_bridge, connector);
}


#ifdef SN65DSI86_USE_PANEL
static int ti_sn_bridge_connector_get_modes(struct drm_connector *connector)
{
	struct ti_sn_bridge *pdata = connector_to_ti_sn_bridge(connector);

	return drm_panel_get_modes(pdata->panel);
}
#else
static int ti_sn_bridge_connector_get_modes(struct drm_connector *connector)
{
	struct ti_sn_bridge *pdata = connector_to_ti_sn_bridge(connector);
	struct edid *pedid = NULL;
	u32 pixelclock = 0;
	unsigned count = 0;
	bool edid_detected = false;
	int ret = 0;
	/*ROPA: needed by DSI */
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;

	if (pdata->edid_detect != DP_EDID_NONE) {
		/* now we try to get EDID frmo display */
		pm_runtime_get_sync(pdata->dev);
		/* add time to setup chip */
		if (!pdata->enabled) {
			ti_sn_set_refclk_freq(pdata);
			msleep(300);
		}
		/* read EDID AUX transfer */
		pedid = drm_get_edid(connector, &pdata->aux.ddc);
		pm_runtime_put(pdata->dev);

		if (pedid) {
			/* GOT IT */
			/* first -> recalculate timing values */
			for (count = 0; count < 4; count++) {
				pixelclock = pedid->detailed_timings[count].pixel_clock * 10000UL;
				if (pixelclock) {
					pixelclock = ti_sn_bridge_adjust_pixclk(pixelclock);
					pedid->detailed_timings[count].pixel_clock = (__le16)(pixelclock / 10000);
				}
			}
			/* remove standard timings */
			for (count = 0; count < 8; count++) {
				pedid->standard_timings[count].hsize = 0;
				pedid->standard_timings[count].vfreq_aspect = 0;
			}
			/* fix checksum */
			ti_sn_bridge_edid_chsum(pedid);

			/* add to DRM layer */
			drm_mode_connector_update_edid_property(connector, (pedid));
			count = drm_add_edid_modes(connector, (pedid));

			if(count) {
				ti_sn_dbg_edid_log(pedid, true);
				edid_detected = true;
			}
			/* and free resources */
			kfree(pedid);
		}
	}

	if ((pdata->edid_detect != DP_EDID_USE) && (pdata->edid_valid) && (!edid_detected)) {
		/* we didn't get EDID from display, if it is not mandatory
		 * use DTB mode instead */
		pixelclock = pdata->gen_edid->detailed_timings[0].pixel_clock * 10000UL;
		drm_mode_connector_update_edid_property(connector, (pdata->gen_edid));
		count = drm_add_edid_modes(connector, (pdata->gen_edid));

		ti_sn_dbg_edid_log(pdata->gen_edid, false);
	}

	/*ROPA: needed by DSI */
	connector->display_info.bus_flags = DRM_BUS_FLAG_DE_LOW |
					    DRM_BUS_FLAG_PIXDATA_NEGEDGE;

	ret = drm_display_info_set_bus_formats(&connector->display_info,
					       &bus_format, 1);
	if (ret)
        count = 0;

	pdata->mode_valid = (count != 0);

	return count;
}
#endif


static enum drm_mode_status
ti_sn_bridge_connector_mode_valid(struct drm_connector *connector,
				  struct drm_display_mode *mode)
{
	struct ti_sn_bridge *pdata = connector_to_ti_sn_bridge(connector);

	if (!pdata->mode_valid)
		return MODE_ERROR;

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

#ifdef HPD_ENABLED

	if (!pdata->enabled) {
		/* if not enabled - enable and read HPD */
		pm_runtime_get_sync(pdata->dev);

		/* configure bridge ref_clk */
		ti_sn_set_refclk_freq(pdata);

		/* enable HPD LINE */
		regmap_update_bits(pdata->regmap, SN_HPDLINE_REG, HPD_DISABLE, 0);
		/* wait for debounce */
		msleep(300);
		regmap_read(pdata->regmap, SN_HPDLINE_REG, &val);
		/* set connector status */
		pdata->plugged = ((val & HPD_LINE_STATUS) != 0);

		pm_runtime_put_sync(pdata->dev);

		ti_sn_dbg_connector_status(pdata, false);
	}

	if (pdata->plugged) {
		return connector_status_connected;
	} else {
		return connector_status_disconnected;
	}
#else
	return connector_status_connected;
#endif //HPD_ENABLED
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
/* DRM Bridge */

static struct ti_sn_bridge *bridge_to_ti_sn_bridge(struct drm_bridge *bridge)
{
	return container_of(bridge, struct ti_sn_bridge, bridge);
}


#ifdef SN65DSI86_USE_REGULATORS
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
#endif


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

#ifdef HPD_ENABLED
	pdata->connector.polled = DRM_CONNECTOR_POLL_HPD;
	ret = drm_connector_init(bridge->dev, &pdata->connector,
				 &ti_sn_bridge_connector_funcs,
				 DRM_MODE_CONNECTOR_DisplayPort);
#else
	ret = drm_connector_init(bridge->dev, &pdata->connector,
				 &ti_sn_bridge_connector_funcs,
				 DRM_MODE_CONNECTOR_eDP);
#endif //HPD_ENABLED
	if (ret) {
		DRM_ERROR("TI_SN65DIS86: Failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(&pdata->connector,
				 &ti_sn_bridge_connector_helper_funcs);
	drm_mode_connector_attach_encoder(&pdata->connector, bridge->encoder);

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
		DRM_ERROR("TI_SN65DIS86: Failed to find dsi host\n");
		ret = -ENODEV;
		goto err_dsi_host;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		DRM_ERROR("TI_SN65DIS86: Failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		goto err_dsi_host;
	}

	/* TODO: setting to 4 lanes and RGB888 - always for now */
	dsi->lanes = DSI_LANES;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO_BURST; /* added burst*/

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		DRM_ERROR("TI_SN65DIS86: Failed to attach dsi to host\n");
		goto err_dsi_attach;
	}
	pdata->dsi = dsi;

#ifdef SN65DSI86_USE_PANEL
	/* attach panel to bridge */
	drm_panel_attach(pdata->panel, &pdata->connector);
#endif

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

#ifdef SN65DSI86_USE_PANEL
	drm_panel_disable(pdata->panel);
#endif

	ti_sn_disable_line(pdata);

	pdata->enabled = false;

	ti_sn_dbg_bridge_status(pdata);

#ifdef SN65DSI86_USE_PANEL
	drm_panel_unprepare(pdata->panel);
#endif
}


static void ti_sn_bridge_enable(struct drm_bridge *bridge)
{
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);
	int ret;

	if (pdata->enabled) {
		DRM_INFO("TI_SN65DSI86: %s:Bridge already enabled - exit \n",__func__);
		return;
	}

	ret = ti_sn_enable_DP(pdata);
	if (ret) {
		ti_sn_bridge_disable(bridge);
		return;
	}

	ret = ti_sn_enable_line(pdata);
	if (ret) {
		ti_sn_bridge_disable(bridge);
		return;
	}

	/* config video parameters */
	ti_sn_set_video_timings(pdata);

	/* TEST PATTERN*/
#ifdef SN65DSI86_TESTMODE
	regmap_write(pdata->regmap, 0x3C, 0x10);
#endif

	/* enable video stream */
	regmap_update_bits(pdata->regmap, SN_ENH_FRAME_REG, VSTREAM_ENABLE,
			   VSTREAM_ENABLE);

	pdata->enabled = true;

	ti_sn_dbg_bridge_status(pdata);

#ifdef SN65DSI86_USE_PANEL
	drm_panel_enable(pdata->panel);
#endif
}

static void ti_sn_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);

	pm_runtime_get_sync(pdata->dev);

	/* configure bridge ref_clk */
	ti_sn_set_refclk_freq(pdata);

#ifdef HPD_ENABLED
	ti_sn_HPD_enable(pdata);
#else //HPD_ENABLED
	/*
	 * HPD on this bridge chip is a bit useless.  This is an eDP bridge
	 * so the HPD is an internal signal that's only there to signal that
	 * the panel is done powering up.  ...but the bridge chip debounces
	 * this signal by between 100 ms and 400 ms (depending on process,
	 * voltage, and temperate--I measured it at about 200 ms).  One
	 * particular panel asserted HPD 84 ms after it was powered on meaning
	 * that we saw HPD 284 ms after power on.  ...but the same panel said
	 * that instead of looking at HPD you could just hardcode a delay of
	 * 200 ms.  We'll assume that the panel driver will have the hardcoded
	 * delay in its prepare and always disable HPD.
	 *
	 * If HPD somehow makes sense on some future panel we'll have to
	 * change this to be conditional on someone specifying that HPD should
	 * be used.
	 */
	regmap_update_bits(pdata->regmap, SN_HPD_DISABLE_REG, HPD_DISABLE,
			   HPD_DISABLE);
	msleep(200);
#endif //HPD_ENABLED

#ifdef SN65DSI86_USE_PANEL
	drm_panel_prepare(pdata->panel);
#endif
}

static void ti_sn_bridge_post_disable(struct drm_bridge *bridge)
{
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);

	/* we have static clk .... */
#ifdef SN65DSI86_USE_CLK
	if (pdata->refclk)
		clk_disable_unprepare(pdata->refclk);
#endif

#ifdef HPD_ENABLED
	ti_sn_HPD_disable(pdata);
#endif //HPD_ENABLED

	pm_runtime_put_sync(pdata->dev);
}

static void ti_sn_bridge_mode_set(struct drm_bridge *bridge,
	    struct drm_display_mode *orig_mode,
	    struct drm_display_mode *mode)
{
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);
	u32 pixelclock, value;

	/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
	/* freq * 2 - DSI transfer two times faster than necessary
	 * DSI output is DSI_bit_clk / 2; because DSI transfers data on rising and falling edge
	 * that is output frequency at DSI lane is half of bit rate
	 * I calculate dsi_clock as necessary bit rate, this force DSI to work at twice higher bit rate
	 * DSI_clk signal has dsi_clock frequency
	 * whith this setting I see less errors on sn64DSI84 ???????? */
	pixelclock = mode->clock * 1000;
	value = DIV_ROUND_UP(pixelclock * mipi_dsi_pixel_format_to_bpp(pdata->dsi->format), DSI_LANES);
	if (value > DSI_BUS_CLK) {
		value >>= 1; /* divide by two !!! as we reached maximum value */
	}
	if (value < (MIN_DSI_CLK_FREQ_MHZ * 1000 * 1000)) {
		value = MIN_DSI_CLK_FREQ_MHZ * 1000 * 1000;
	}
	pdata->forced_dsi_freq = value;
	ti_snbridge_force_DSI_freq(pdata);

	ti_sn_dbg_mode_log(mode);
}

static const struct drm_bridge_funcs ti_sn_bridge_funcs = {
	.attach = ti_sn_bridge_attach,
	.pre_enable = ti_sn_bridge_pre_enable,
	.enable = ti_sn_bridge_enable,
	.disable = ti_sn_bridge_disable,
	.post_disable = ti_sn_bridge_post_disable,
	.mode_set = ti_sn_bridge_mode_set,
};

/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* 						Driver initialization 											*/
/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* sysfs debug entries */
#ifdef TI_SN_SYSFS

static ssize_t ti_sn_mode_show(struct device *child, struct device_attribute *attr, char *buf)
{
	ssize_t i;

	i = sprintf(buf, "Mode: %dx%d, %luHz \n", dbg.mode.hactive, dbg.mode.vactive, dbg.mode.pixelclock);
	i += sprintf(buf + i, "timing: \n");
	i += sprintf(buf + i, "	  horizontal back porch:  %d\n", dbg.mode.hback_porch);
	i += sprintf(buf + i, "	  horizontal active:      %d\n", dbg.mode.hactive);
	i += sprintf(buf + i, "	  horizontal front porch: %d\n", dbg.mode.hfront_porch);
	i += sprintf(buf + i, "	  horizontal sync:        %d\n", dbg.mode.hsync_len);
	i += sprintf(buf + i, "	  vertical back porch:    %d\n", dbg.mode.vback_porch);
	i += sprintf(buf + i, "	  vertical active:        %d\n", dbg.mode.vactive);
	i += sprintf(buf + i, "	  vertical front porch:   %d\n", dbg.mode.vfront_porch);
	i += sprintf(buf + i, "	  vertical sync:          %d\n", dbg.mode.vsync_len);


	return i;
}
static DEVICE_ATTR(active_mode, 0444, ti_sn_mode_show, NULL);

static ssize_t ti_sn_edid_show(struct device *child, struct device_attribute *attr, char *buf)
{
	ssize_t i;
	u32 hactive, vactive;

	if (dbg.edid_status) {
		i = sprintf(buf, "EDID got from monitor \n");
	} else {
		i = sprintf(buf, "EDID defaults from DTB \n");
	}

	hactive = (((u32)(dbg.edid.detailed_timings[0].data.pixel_data.hactive_hblank_hi & 0xf0)) << 4) | ((u32)(dbg.edid.detailed_timings[0].data.pixel_data.hactive_lo));
	vactive = (((u32)(dbg.edid.detailed_timings[0].data.pixel_data.vactive_vblank_hi & 0xf0)) << 4) | ((u32)(dbg.edid.detailed_timings[0].data.pixel_data.vactive_lo));
	i += sprintf(buf + i, "Pixel clock: %d \n", (dbg.edid.detailed_timings[0].pixel_clock * 10000));
	i += sprintf(buf + i, "Horizontal: %d \n", hactive);
	i += sprintf(buf + i, "Vertical  : %d \n", vactive);

	return i;
}
static DEVICE_ATTR(edid_mode, 0444, ti_sn_edid_show, NULL);

static ssize_t ti_sn_of_mode_show(struct device *child, struct device_attribute *attr, char *buf)
{
	ssize_t i;

	if (dbg.of_mode_valid)
	{
		i = sprintf(buf, "Mode: %dx%d, %luHz \n", dbg.of_mode.hactive, dbg.of_mode.vactive, dbg.of_mode.pixelclock);
		i += sprintf(buf + i, "timing: \n");
		i += sprintf(buf + i, "	  horizontal back porch:  %d\n", dbg.of_mode.hback_porch);
		i += sprintf(buf + i, "	  horizontal active:      %d\n", dbg.of_mode.hactive);
		i += sprintf(buf + i, "	  horizontal front porch: %d\n", dbg.of_mode.hfront_porch);
		i += sprintf(buf + i, "	  horizontal sync:        %d\n", dbg.of_mode.hsync_len);
		i += sprintf(buf + i, "	  vertical back porch:    %d\n", dbg.of_mode.vback_porch);
		i += sprintf(buf + i, "	  vertical active:        %d\n", dbg.of_mode.vactive);
		i += sprintf(buf + i, "	  vertical front porch:   %d\n", dbg.of_mode.vfront_porch);
		i += sprintf(buf + i, "	  vertical sync:          %d\n", dbg.of_mode.vsync_len);
	} else {
		i = sprintf(buf,"NO MODE from DTB available\n");
	}

	return i;
}
static DEVICE_ATTR(of_mode, 0444, ti_sn_of_mode_show, NULL);

static ssize_t ti_sn_dsifreq_show(struct device *child, struct device_attribute *attr, char *buf)
{
	ssize_t i;

	i = sprintf(buf, "FORCED DSI frequency: %d\n", dbg.DSI_freq);
	return i;
}
static DEVICE_ATTR(dsi_freq, 0444, ti_sn_dsifreq_show, NULL);

static ssize_t ti_sn_connector_show(struct device *child, struct device_attribute *attr, char *buf)
{
	ssize_t i;

	if (dbg.connector_status) {
		i = sprintf(buf, "Connector connected\n");
	} else {
		i = sprintf(buf, "Connector disconnected\n");
	}
	return i;
}
static DEVICE_ATTR(connector_status, 0444, ti_sn_connector_show, NULL);

static ssize_t ti_sn_bridge_show(struct device *child, struct device_attribute *attr, char *buf)
{
	ssize_t i;

	if (dbg.bridge_status) {
		i = sprintf(buf, "Bridge enabled\n");
	} else {
		i = sprintf(buf, "Bridge disabled\n");
	}
	return i;
}
static DEVICE_ATTR(bridge_status, 0444, ti_sn_bridge_show, NULL);

ssize_t lcdif_dump_sysfs(char *buf);
static ssize_t lcdif_test_show(struct device *child, struct device_attribute *attr, char *buf)
{
	return lcdif_dump_sysfs(buf);
}
static DEVICE_ATTR(lcdif_regs, 0444, lcdif_test_show, NULL);


static void ti_sn_create_attribute(struct device *dev, struct ti_sn_bridge *pdata)
{

	device_create_file(dev, &dev_attr_active_mode);
	device_create_file(dev, &dev_attr_edid_mode);
	device_create_file(dev, &dev_attr_of_mode);
	device_create_file(dev, &dev_attr_dsi_freq);
	device_create_file(dev, &dev_attr_connector_status);
	device_create_file(dev, &dev_attr_bridge_status);

	device_create_file(dev, &dev_attr_lcdif_regs);
}

static void ti_sn_delete_attributes(struct device *dev)
{
	device_remove_file(dev, &dev_attr_active_mode);
	device_remove_file(dev, &dev_attr_edid_mode);
	device_remove_file(dev, &dev_attr_of_mode);
	device_remove_file(dev, &dev_attr_dsi_freq);
	device_remove_file(dev, &dev_attr_connector_status);
	device_remove_file(dev, &dev_attr_bridge_status);

	device_remove_file(dev, &dev_attr_lcdif_regs);
}



#else //TI_SN_SYSFS
/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* sysfs attributes */
static inline void ti_sn_create_attribute(struct device *dev, struct ti_sn_bridge *pdata) {}
static inline void ti_sn_delete_attributes(struct device *dev) {}

#endif //TI_SN_SYSFS
/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* 						Driver initialization 											*/
/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* artificial EDID - from DTB */

#ifndef SN65DSI86_USE_PANEL

static u8 default_edid[] = {
		/* 1024x768 60HZ David DATA */
		0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x31, 0xD8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x05, 0x16, 0x01, 0x03, 0x6D, 0x23, 0x1A, 0x78, 0xEA, 0x5E, 0xC0, 0xA4, 0x59, 0x4A, 0x98, 0x25,
		0x20, 0x50, 0x54, 0x00, 0x00, 0x00, 0x61, 0x40, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
		0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x64, 0x19, 0x00, 0x40, 0x41, 0x00, 0x26, 0x30, 0x28, 0xD0,
		0x44, 0x00, 0x63, 0x0A, 0x11, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x4C, 0x69, 0x6E,
		0x75, 0x78, 0x20, 0x23, 0x30, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD, 0x00, 0x3B,
		0x3D, 0x2F, 0x31, 0x07, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFC,
		0x00, 0x4C, 0x69, 0x6E, 0x75, 0x78, 0x20, 0x58, 0x47, 0x41, 0x0A, 0x20, 0x20, 0x20, 0x00, 0xE9,
};

static int ti_sn_bridge_make_edid(struct ti_sn_bridge *pdata)
{
	u8 val;
	u32 val32;
	struct edid *eptr;

	eptr = (struct edid *)devm_kzalloc(pdata->dev, sizeof(struct edid), GFP_KERNEL);
    if (!eptr)
            return -ENOMEM;

    memcpy((void *)eptr, (void *)default_edid, sizeof(default_edid));

	eptr->standard_timings[0].hsize = (pdata->of_mode.hactive / 8) - 31;

	eptr->detailed_timings[0].pixel_clock = pdata->of_mode.pixelclock / 10000;

	/* hactive + hblank */
	eptr->detailed_timings[0].data.pixel_data.hactive_lo = (u8)(pdata->of_mode.hactive & 0xff);
	val32 = pdata->of_mode.hfront_porch + pdata->of_mode.hback_porch + pdata->of_mode.hsync_len;
	eptr->detailed_timings[0].data.pixel_data.hblank_lo = (u8)((val32) & 0xff);
	val = (u8)(((pdata->of_mode.hactive >> 8) & 0x0f) << 4); // get high 4 bits and shift to upper nibble
	val |= (u8)((val32 >> 8) & 0x0f);
	eptr->detailed_timings[0].data.pixel_data.hactive_hblank_hi = val;

	/* vactive + vblank */
	eptr->detailed_timings[0].data.pixel_data.vactive_lo = (u8)(pdata->of_mode.vactive & 0xff);
	val32 = pdata->of_mode.vfront_porch + pdata->of_mode.vback_porch + pdata->of_mode.vsync_len;
	eptr->detailed_timings[0].data.pixel_data.vblank_lo = (u8)(val32 & 0xff);
	val = (u8)(((pdata->of_mode.vactive >> 8) & 0x0f) << 4);
	val |= (u8)((val32 >> 8) & 0x0f);
	eptr->detailed_timings[0].data.pixel_data.vactive_vblank_hi = val;

	/* hfrontp + hsync; lower 8 bits  */
	eptr->detailed_timings[0].data.pixel_data.hsync_offset_lo = (u8)(pdata->of_mode.hfront_porch & 0xff);
	eptr->detailed_timings[0].data.pixel_data.hsync_pulse_width_lo = (u8)(pdata->of_mode.hsync_len & 0xff);

	/* combined */
	// it was tested to have similar behavior as sn65dsi84 test mode //vfrontp++;
	val = (u8)((pdata->of_mode.vfront_porch & 0x0f) << 4); /*  vfrontp - lower 4 bits */
	val |= (u8)(pdata->of_mode.vsync_len & 0x0f); /* vsync - lower 4 bits  */
	eptr->detailed_timings[0].data.pixel_data.vsync_offset_pulse_width_lo = val;
	/* combined */
	val = (u8)(((pdata->of_mode.hfront_porch >> 8) & 0x3) << 6); /* nn000000 hfrontp - upper 2 bits */
	val |= (u8)(((pdata->of_mode.hsync_len >> 8) & 0x3) << 4); /* 00nn0000 hsync - upper 2 bits */
	val |= (u8)(((pdata->of_mode.vfront_porch >> 4) & 0x3) << 2); /* 0000nn00 vfrontp - upper 2 bits.... note value is 6 bits at all */
	val |= (u8)(((pdata->of_mode.vsync_len >> 4) & 0x03) << 0); /* 000000nn fsync - upper 2 bits ..... note value is 6 bits at all  */
	eptr->detailed_timings[0].data.pixel_data.hsync_vsync_offset_pulse_width_hi = val;

	/* Display dimmensions */
	eptr->detailed_timings[0].data.pixel_data.width_mm_lo = 0x63;
	eptr->detailed_timings[0].data.pixel_data.height_mm_lo = 0x0a;
	eptr->detailed_timings[0].data.pixel_data.width_height_mm_hi = 0x11;
	eptr->detailed_timings[0].data.pixel_data.hborder = 0x00;
	eptr->detailed_timings[0].data.pixel_data.vborder = 0x00;

	/* miscelanous */
	val = 0;

	/* interlaced not supported yet */
	val |= 0x18; /* Digital separate sync */

	/* to be fully compliant, but DSI layer set both to positive .... for LCDIF ?!*/
	if (pdata->of_mode.flags & DISPLAY_FLAGS_HSYNC_HIGH) {
		val |= (1 << 1); /* hsync polarity positive */
	}

	if (pdata->of_mode.flags & DISPLAY_FLAGS_VSYNC_HIGH) {
		val |= (1 << 2); /* vsync polarity positive */
	}

 	eptr->detailed_timings[0].data.pixel_data.misc = val;

 	ti_sn_bridge_edid_chsum(eptr);

 	pdata->gen_edid = eptr;
 	pdata->edid_valid = true;

 	return 0;
}


static int ti_sn_bridge_parse_video_mode(struct ti_sn_bridge *pdata)
{
	u32 value, bpp;

	/* check when this function is called and use adv instead ....... */
	if(of_property_read_u32(pdata->dev->of_node, "dp,pixelclock", &value)) return -EINVAL;
	if(value)
		pdata->of_mode.pixelclock = ti_sn_bridge_adjust_pixclk((unsigned long)value);
	/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
	/* freq * 2 - DSI transfer two times faster than necessary
	 * DSI output is DSI_bit_clk / 2; because DSI transfers data on rising and falling edge
	 * that is output frequency at DSI lane is half of bit rate
	 * I calculate dsi_clock as necessary bit rate, this force DSI to work at twice higher bit rate
	 * DSI_clk signal has dsi_clock frequency
	 * whith this setting I see less errors on sn64DSI84 ???????? */
	bpp = 24; // for now RGB888
	value = DIV_ROUND_UP(pdata->of_mode.pixelclock * bpp, DSI_LANES);
	if (value > DSI_BUS_CLK) {
		value >>= 1; /* divide by two !!! as we reached maximum value */
	}
	pdata->forced_dsi_freq = value;


	if(of_property_read_u32(pdata->dev->of_node, "dp,hactive", &value)) return -EINVAL;
	pdata->of_mode.hactive = value;
	if(of_property_read_u32(pdata->dev->of_node, "dp,vactive", &value)) return -EINVAL;
	pdata->of_mode.vactive = value;

	/* Horizonatal parameters */
	if(of_property_read_u32(pdata->dev->of_node, "dp,hfront_porch", &value)) return -EINVAL;
	pdata->of_mode.hfront_porch = value;
	if(of_property_read_u32(pdata->dev->of_node, "dp,hback_porch", &value)) return -EINVAL;
	pdata->of_mode.hback_porch = value;
	if(of_property_read_u32(pdata->dev->of_node, "dp,hsync_len", &value)) return -EINVAL;
	pdata->of_mode.hsync_len = value;

	/* Vertical parameters */
	if(of_property_read_u32(pdata->dev->of_node, "dp,vfront_porch", &value)) return -EINVAL;
	pdata->of_mode.vfront_porch = value;
	if(of_property_read_u32(pdata->dev->of_node, "dp,vback_porch", &value)) return -EINVAL;
	pdata->of_mode.vback_porch = value;
	if(of_property_read_u32(pdata->dev->of_node, "dp,vsync_len", &value)) return -EINVAL;
	pdata->of_mode.vsync_len = value;

	pdata->of_mode.flags = 0;
	if(of_property_read_bool(pdata->dev->of_node, "dp,hsyn_positive")) {
		pdata->of_mode.flags |= DISPLAY_FLAGS_HSYNC_HIGH;
	} else {
		pdata->of_mode.flags |= DISPLAY_FLAGS_HSYNC_LOW;
	}
	if(of_property_read_bool(pdata->dev->of_node, "dp,vsyn_positive")) {
		pdata->of_mode.flags |= DISPLAY_FLAGS_VSYNC_HIGH;
	} else {
		pdata->of_mode.flags |= DISPLAY_FLAGS_VSYNC_LOW;
	}
	if(of_property_read_bool(pdata->dev->of_node, "dp,de_positive")) {
		pdata->of_mode.flags |= DISPLAY_FLAGS_DE_HIGH;
	} else {
		pdata->of_mode.flags |= DISPLAY_FLAGS_DE_LOW;
	}

	ti_sn_dbg_dtb_mode_log(&pdata->of_mode);

	return ti_sn_bridge_make_edid(pdata);
}
#endif //SN65DSI86_USE_PANEL

/****************************************************************************************/
/* Driver initialization */

static int ti_sn_bridge_parse_dsi_host(struct ti_sn_bridge *pdata)
{
	struct device_node *np = pdata->dev->of_node;

	/*
	 * first parameter is my device node
	 * second parameter is port "reg" property - port@0 { ...
	 * 		possible definition of reg property is nodename@reg_num
	 * 		or inside node { ...   reg = <reg_num>;
	 * third parameter is endpoint "reg" property,
	 * 		usually defined as reg = <...>
	 * this call find port@0 endpoint 0 device, which is DSI
	 * see DTB
	 * */
	pdata->host_node = of_graph_get_remote_node(np, 0, 0);

	if (!pdata->host_node) {
		DRM_ERROR("TI_SN65DIS86: Remote dsi host node not found\n");
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
		DRM_ERROR("TI_SN65DIS86: Device doesn't support I2C\n");
		return -ENODEV;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(struct ti_sn_bridge),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->regmap = devm_regmap_init_i2c(client,
					     &ti_sn_bridge_regmap_config);
	if (IS_ERR(pdata->regmap)) {
		DRM_ERROR("TI_SN65DIS86: Regmap i2c init failed\n");
		return PTR_ERR(pdata->regmap);
	}

	pdata->dev = &client->dev;
	pdata->enabled = false;


	/* no panel supported in our BSP - find another soulution */
#ifdef SN65DSI86_USE_PANEL
	ret = drm_of_find_panel_or_bridge(pdata->dev->of_node, 1, 0,
					  &pdata->panel, NULL);
	if (ret) {
		DRM_ERROR("could not find any panel node\n");
		return ret;
	}
#else //SN65DSI86_USE_PANEL
	pdata->edid_valid = false;
	pdata->mode_valid = false;

	/* determine mode detection method */
	if (of_property_read_bool(pdata->dev->of_node, "dp,use-edid-data")) {
		pdata->edid_detect = DP_EDID_USE;
		DRM_INFO("TI_SN65DIS86: Mode determined by display \n");
	} else if (of_property_read_bool(pdata->dev->of_node, "dp,try-edid-data")) {
		pdata->edid_detect = DP_EDID_TRY;
		DRM_INFO("TI_SN65DIS86: Mode determined first by display or by DTB \n");
	} else {
		pdata->edid_detect = DP_EDID_NONE;
		DRM_INFO("TI_SN65DIS86: Mode determined DTB \n");
	}

	if (pdata->edid_detect != DP_EDID_USE) {
		if (ti_sn_bridge_parse_video_mode(pdata)) {
			if (pdata->edid_detect == DP_EDID_NONE) {
				DRM_ERROR("TI_SN65DIS86: No mode data provided\n");
				return -ENODEV;
			}
		}
	}
#endif //SN65DSI86_USE_PANEL

	dev_set_drvdata(&client->dev, pdata);

	/* used new gpio dt api -- apply to dtb */
	pdata->enable_gpio = devm_gpiod_get(pdata->dev, "enable",
					    GPIOD_OUT_LOW);
	if (IS_ERR(pdata->enable_gpio)) {
		DRM_ERROR("TI_SN65DIS86: Failed to get enable gpio from DT\n");
		ret = PTR_ERR(pdata->enable_gpio);
		return ret;
	}

#ifdef HPD_ENABLED
	ret = ti_sn_HPD_init(pdata, client, id);
	if (ret)
		return ret;
#endif //HPD_ENABLED


	/* we have static power - no regulator needed */
#ifdef SN65DSI86_USE_REGULATORS
	ret = ti_sn_bridge_parse_regulators(pdata);
	if (ret) {
		DRM_ERROR("failed to parse regulators\n");
		return ret;
	}
#endif //SN65DSI86_USE_REGULATORS

	/* we have static clk .... */
#ifdef SN65DSI86_USE_CLK
	pdata->refclk = devm_clk_get(pdata->dev, "refclk");
	if (IS_ERR(pdata->refclk)) {
		ret = PTR_ERR(pdata->refclk);
		if (ret == -EPROBE_DEFER)
			return ret;
		DRM_DEBUG_KMS("refclk not found\n");
		pdata->refclk = NULL;
	}
#endif //SN65DSI86_USE_CLK

	ret = ti_sn_bridge_parse_dsi_host(pdata);

	if (ret)
		return ret;

	pm_runtime_enable(pdata->dev);

	i2c_set_clientdata(client, pdata);

	pdata->aux.name = "ti-sn65dsi86-aux";
	pdata->aux.dev = pdata->dev;
	pdata->aux.transfer = ti_sn_aux_transfer;
	drm_dp_aux_register(&pdata->aux);

	pdata->bridge.funcs = &ti_sn_bridge_funcs;
	pdata->bridge.of_node = client->dev.of_node;
	drm_bridge_add(&pdata->bridge);

	ti_sn_create_attribute(&client->dev, pdata);
	DRM_INFO("TI_SN65DSI86: driver loaded\n");

	return 0;
}

static int ti_sn_bridge_remove(struct i2c_client *client)
{
	struct ti_sn_bridge *pdata = i2c_get_clientdata(client);

	if (!pdata)
		return -EINVAL;

	ti_sn_delete_attributes(&client->dev);

	of_node_put(pdata->host_node);

	pm_runtime_disable(pdata->dev);

	if (pdata->dsi) {
		mipi_dsi_detach(pdata->dsi);
		mipi_dsi_device_unregister(pdata->dsi);
	}

	DRM_INFO("TI_SN65DSI86: Driver removed\n");

	drm_bridge_remove(&pdata->bridge);

	return 0;
}

static struct i2c_device_id ti_sn_bridge_id[] = {
	{ "sn65dsi86", 0},
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


