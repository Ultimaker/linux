/*
 * Texas Instruments SN65DSI84 HDMI transmitter driver
 *
 * Updated by Robert Pasz; Congatec
 * Driver based on:
 * Marco Sandrelli <marco.sandrelli@seco.com>"
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/slab.h>

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>
#include <linux/of_graph.h>
#include <linux/of_gpio.h>
#include <video/videomode.h>
#include <drm/drm_panel.h>

#include "sn65dsi84.h"
#include "sn65dsi84_edid.h"

#ifdef CONFIG_DRM_SNDEBUG
#ifdef CONFIG_DRM_PRNDEBUG
	#define SN65DSI84_DBGPRN 1
#endif
	#define SNDEBUG 1
#endif

#define MAX_DSI_DIVIDER 25
#define MIN_LVDS_CLOCK 25000000
#define LOW_LVDS_CLOCK 40000000
#define BUS_CLK 594000000

/* Interface to DSI layer - driver should override DSI clock to achieve better functionality ... */
extern void sec_mipi_force_freq(u32 freq);
__attribute__((weak)) void sec_mipi_force_freq(u32 freq) 
{
        return;
}

/* -----------------------------------------------------------------------------
 * DEBUG SUPPORT
*/
#ifdef SNDEBUG

typedef struct {
	struct sn65dsi84 *adv;
	u32 divisor;
	u32 delay;
	u32 backporch;
	u32 frontporch;
	u32 sync;
	u32 active;
	u32 dsifreq;
	u32 dsi_m;
	u32 dsi_p;
	u32 dsi_s;
	u32 dsi_hfporch;
	u32 dsi_hbporch;
	u32 dsi_hsync;
	bool dsi_enabled;
} sn_debug_t;

static sn_debug_t sn_debug = {
		.adv = NULL,
		.dsi_enabled = false,
		.dsifreq = 0,
};

#endif
/* -----------------------------------------------------------------------------
   -----------------------------------------------------------------------------
 * Register access
 */


static const uint8_t sn65dsi84_register_defaults[] = { };

static bool sn65dsi84_register_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case	SN65DSI84_REG_CHIP_REVISION:
	case	SN65DSI84_REG_RESET:
	case	SN65DSI84_REG_LVDS_CLK:
	case	SN65DSI84_REG_CLK_DIV_MUL:
	case	SN65DSI84_REG_I2S_CONFG:
	case	SN65DSI84_REG_PLL_EN:
	case	SN65DSI84_REG_DSI_LANES:
	case	SN65DSI84_REG_DSI_EQ:
	case	SN65DSI84_REG_DSI_CLK_RANGE:
	case	SN65DSI84_REG_LVDS_PARAMETER:
	case	SN65DSI84_REG_LVDS_STRENGTH:
	case	SN65DSI84_REG_LVDS_SWAP:
	case	SN65DSI84_REG_LVDS_COMMON_MODE:
	case	SN65DSI84_REG_CHA_ACTIVE_LINE_LENGTH_LOW:
	case	SN65DSI84_REG_CHA_ACTIVE_LINE_LENGTH_HIGH:
	case	SN65DSI84_REG_CHA_VERTICAL_DISPLAY_SIZE_LOW:
	case	SN65DSI84_REG_CHA_VERTICAL_DISPLAY_SIZE_HIGH:
	case	SN65DSI84_REG_CHA_SYNC_DELAY_LOW:
	case	SN65DSI84_REG_CHA_SYNC_DELAY_HIGH:
	case	SN65DSI84_REG_CHA_HSYNC_PULSE_WIDTH_LOW:
	case	SN65DSI84_REG_CHA_HSYNC_PULSE_WIDTH_HIGH:
	case	SN65DSI84_REG_CHA_VSYNC_PULSE_WIDTH_LOW:
	case	SN65DSI84_REG_CHA_VSYNC_PULSE_WIDTH_HIGH:
	case	SN65DSI84_REG_CHA_HORIZONTAL_BACK_PORCH:
	case	SN65DSI84_REG_CHA_VERTICAL_BACK_PORCH:
	case	SN65DSI84_REG_CHA_HORIZONTAL_FRONT_PORCH:
	case	SN65DSI84_REG_CHA_VERTICAL_FRONT_PORCH:
	case	SN65DSI84_REG_CHA_TEST_PATTERN:
	case	SN65DSI84_REG_IRQ_EN:
	case	SN65DSI84_REG_ERR_EN:
	case	SN65DSI84_REG_DSI_PROTOCOL_ERR:
		return true;
	}

	return false;
}

static const struct regmap_config sn65dsi84_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0xff,
	.cache_type = REGCACHE_RBTREE,
	.reg_defaults_raw = sn65dsi84_register_defaults,
	.num_reg_defaults_raw = ARRAY_SIZE(sn65dsi84_register_defaults),

	.volatile_reg = sn65dsi84_register_volatile,
};
/* -----------------------------------------------------------------------------
 * EDID helpers
 *
 */

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

static void sn65dsi84_edid_chsum(struct edid *eptr)
{
    int i;
    u8 *ptr = (u8*) (eptr);
    u8 sum = 0;

    for (i = 0; i < (sizeof(struct edid) - 1); i++) {
            sum += *(ptr++);
    }
    eptr->checksum = 1 + (~sum);
}

static int sn65dsi84_make_edid(struct sn65dsi84 *adv)
{
	u32 hactive,vactive, pixclk;
	u32 hfrontp,hbackp,hsync;
	u32 vfrontp,vbackp,vsync;
	u8 val;
	u32 val32;
	int ret;
	struct edid *eptr = (struct edid *)(&default_edid);


	pixclk = adv->lvds_timing->pixelclock;

	hactive = adv->lvds_timing->hactive;
	vactive = adv->lvds_timing->vactive;
	/* Horizonatal parameters */
	hfrontp = adv->lvds_timing->hfront_porch;
	hbackp = adv->lvds_timing->hback_porch;
	hsync = adv->lvds_timing->hsync_len;
	/* Vertical parameters */
	vfrontp = adv->lvds_timing->vfront_porch;
	vbackp = adv->lvds_timing->vback_porch;
	vsync = adv->lvds_timing->vsync_len;

	eptr->standard_timings[0].hsize = (hactive / 8) - 31;

	eptr->detailed_timings[0].pixel_clock = pixclk / 10000;

	/* hactive + hblank */
	eptr->detailed_timings[0].data.pixel_data.hactive_lo = (u8)(hactive & 0xff);
	val32 = hfrontp + hbackp + hsync;
	eptr->detailed_timings[0].data.pixel_data.hblank_lo = (u8)((val32) & 0xff);
	val = (u8)(((hactive >> 8) & 0x0f) << 4); // get high 4 bits and shift to upper nibble
	val |= (u8)((val32 >> 8) & 0x0f);
	eptr->detailed_timings[0].data.pixel_data.hactive_hblank_hi = val;

	/* vactive + vblank */
	eptr->detailed_timings[0].data.pixel_data.vactive_lo = (u8)(vactive & 0xff);
	val32 = vfrontp + vbackp + vsync;
	eptr->detailed_timings[0].data.pixel_data.vblank_lo = (u8)(val32 & 0xff);
	val = (u8)(((vactive >> 8) & 0x0f) << 4);
	val |= (u8)((val32 >> 8) & 0x0f);
	eptr->detailed_timings[0].data.pixel_data.vactive_vblank_hi = val;

	/* hfrontp + hsync; lower 8 bits  */
	eptr->detailed_timings[0].data.pixel_data.hsync_offset_lo = (u8)(hfrontp & 0xff);
	eptr->detailed_timings[0].data.pixel_data.hsync_pulse_width_lo = (u8)(hsync & 0xff);

	/* combined */
	val = (u8)((vfrontp & 0x0f) << 4); /*  vfrontp - lower 4 bits */
	val |= (u8)(vsync & 0x0f); /* vsync - lower 4 bits  */
	eptr->detailed_timings[0].data.pixel_data.vsync_offset_pulse_width_lo = val;
	/* combined */
	val = (u8)(((hfrontp >> 8) & 0x3) << 6); /* nn000000 hfrontp - upper 2 bits */
	val |= (u8)(((hsync >> 8) & 0x3) << 4); /* 00nn0000 hsync - upper 2 bits */
	val |= (u8)(((vfrontp >> 4) & 0x3) << 2); /* 0000nn00 vfrontp - upper 2 bits.... note value is 6 bits at all */
	val |= (u8)(((vsync >> 4) & 0x03) << 0); /* 000000nn fsync - upper 2 bits ..... note value is 6 bits at all  */
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
#if 0
	if (of_property_read_bool(np, "lvds,interlaced"))
		val |= 0x80;
#endif

	val |= 0x18; /* Digital separate sync */

   	ret = 0;
	val32 = adv->hsync_polarity;
	if ((!ret) && val32) {
		val |= 0x01; /* hsync polarity positive */
	}
	val32 = adv->vsync_polarity;
	if ((!ret) && val32) {
		val |= 0x02; /* vsync polarity positive */
	}

 	eptr->detailed_timings[0].data.pixel_data.misc = val;

 	sn65dsi84_edid_chsum(eptr);

 	return 0;
}

/* -----------------------------------------------------------------------------
 * Hardware configuration
 */

static void sn65dsi84_set_link_config(struct sn65dsi84 *sn65dsi84,
				    const struct sn65dsi84_link_config *config)
{

}

static void sn65dsi84_power_on(struct sn65dsi84 *sn65dsi84)
{
	if (sn65dsi84->gpio_pd) {
				gpio_set_value(sn65dsi84->gpio_pd, 0);
				udelay(5);
				gpio_set_value(sn65dsi84->gpio_bckl, 1);
                gpio_set_value(sn65dsi84->gpio_pd, 1);
                msleep(1);
	}
        return;

}

static void sn65dsi84_power_off(struct sn65dsi84 *sn65dsi84)
{
	if (sn65dsi84->gpio_pd) {
                gpio_set_value(sn65dsi84->gpio_pd, 0);
				gpio_set_value(sn65dsi84->gpio_bckl, 1);
	}

	return;
}

/* -----------------------------------------------------------------------------
 * ADV75xx helpers
 */



static int sn65dsi84_get_valid_edid(struct sn65dsi84 *adv, int *edid, const char *edid_buf)
{
	for(*edid=0;*edid<sizeof(sn65dsi84_edid_id_list)/sizeof(sn65dsi84_edid_id_list[0]);(*edid)++ ) {
		if(!strcmp(edid_buf,sn65dsi84_edid_id_list[*edid]))
			return 0;
	}
	if((*edid) >= sizeof(sn65dsi84_edid_id_list)/sizeof(sn65dsi84_edid_id_list[0])) {
		DRM_ERROR("LVDS Resolution %s not found\n",edid_buf);
		DRM_ERROR("Selecting default resolution %s",sn65dsi84_edid_id_list[0]);
		return -EINVAL;
	}

	return 0;	
}

static int sn65dsi84_get_modes(struct sn65dsi84 *sn65dsi84,
			     struct drm_connector *connector)
{
	unsigned int count,edid_id = SN65DSI84_EDID_800x600;
	// ROPA - just test ..... u32 bus_format = MEDIA_BUS_FMT_RGB666_1X18;
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	int ret;

	const char *buf;

	if (!of_property_read_string(sn65dsi84->bridge.of_node, "lvds,edid", &(buf))) {
		/*returned 0 so we defined edid - lets try to find that in our list */
		ret = sn65dsi84_get_valid_edid(sn65dsi84,&edid_id,buf);
		if (!ret) {
			sn65dsi84->edid = (struct edid *) (sn65dsi84_edid_dsi_list[edid_id]);
			DRM_INFO("sn65: get_modes(): found defined EDID\n ");
		}
	} else {
		/* no edid defined - create edid from mode data */
		ret = sn65dsi84_make_edid(sn65dsi84);
		if (!ret) {
			sn65dsi84->edid = (struct edid *) (default_edid); 
			DRM_INFO("sn65: get_modes(): EDID made from mode data");
		}
	}


	if(ret < 0 ) {
		DRM_ERROR("sn65: get_modes(): No resolution found from dts!\n");
		DRM_ERROR("Select default resolution %s\n",sn65dsi84_edid_id_list[SN65DSI84_EDID_800x600]);
		edid_id = SN65DSI84_EDID_800x600;
		sn65dsi84->edid = (struct edid *) (sn65dsi84_edid_dsi_list[edid_id]);
	}
		
	DRM_DEBUG("EDID sn65dsi84 is valid return = %d\n",drm_edid_header_is_valid((u8 *)(sn65dsi84->edid)));


	drm_mode_connector_update_edid_property(connector, (sn65dsi84->edid));
	count = drm_add_edid_modes(connector, (sn65dsi84->edid));

#ifdef SN65DSI84_DBGPRN
	pr_info("DSI-sn65: get_modes() check pixel clock: %d\n", connector->display_info.pixel_clock);
#endif

	connector->display_info.bus_flags = DRM_BUS_FLAG_DE_LOW |
					    DRM_BUS_FLAG_PIXDATA_NEGEDGE;

	ret = drm_display_info_set_bus_formats(&connector->display_info,
					       &bus_format, 1);
	if (ret)
                return ret;

	return count;
}

static enum drm_connector_status
sn65dsi84_detect(struct sn65dsi84 *sn65dsi84, struct drm_connector *connector)
{
	enum drm_connector_status status;

	status = connector_status_connected;
	sn65dsi84->status = status;
	return connector_status_connected;
}

static int sn65dsi84_mode_valid(struct sn65dsi84 *sn65dsi84,
			      struct drm_display_mode *mode)
{
	DRM_DEBUG("sn65dsi84_mode_valid\n");
	if (mode->clock > 165000)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static void sn65dsi84_mipi_dsi_set(struct sn65dsi84 *adv, struct drm_display_mode *mode)
{
        struct mipi_dsi_device *dsi = adv->dsi;
        int lanes, ret;

        if (adv->num_dsi_lanes != 4)
                return;
        lanes = adv->num_dsi_lanes;

#ifdef SN65DSI84_DBGPRN
        pr_info("DSI-sn65: dsi_set() lanes = %d\n", lanes);
        pr_info("DSI-sn65: dsi_set() dsi->lanes = %d\n", dsi->lanes);
#endif

        if (lanes != dsi->lanes) {
                mipi_dsi_detach(dsi);
                dsi->lanes = lanes;
                ret = mipi_dsi_attach(dsi);
                if (ret)
                        dev_err(&dsi->dev, "failed to change host lanes\n");
        }
}

static int sn65dsi84_attach_dsi(struct sn65dsi84 *adv)
{
        struct device *dev = &adv->i2c_main->dev;
        struct mipi_dsi_host *host;
        struct mipi_dsi_device *dsi;
        int ret = 0;
        const struct mipi_dsi_device_info info = { .type = "sn65dsi84",
                                                   .channel = 0,
                                                   .node = NULL,
                                                 };

        host = of_find_mipi_dsi_host_by_node(adv->host_node);
        if (!host) {
                dev_err(dev, "failed to find dsi host\n");
                return -EPROBE_DEFER;
        }

        dsi = mipi_dsi_device_register_full(host, &info);
        if (IS_ERR(dsi)) {
                dev_err(dev, "failed to create dsi device\n");
                ret = PTR_ERR(dsi);
                goto err_dsi_device;
        }

        adv->dsi = dsi;

        dsi->lanes = adv->num_dsi_lanes;

	dsi->format = MIPI_DSI_FMT_RGB888;

        dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST | MIPI_DSI_MODE_VIDEO_HFP | MIPI_DSI_MODE_VIDEO_HBP |
        		          MIPI_DSI_MODE_VIDEO_HSA | MIPI_DSI_MODE_EOT_PACKET;

#ifdef SN65DSI84_DBGPRN
		pr_info("DSI-sn65: attach_dsi() adv_num... = %d\n", adv->num_dsi_lanes);
		pr_info("DSI-sn65: attach_dsi() dsi->lanes = %d\n", dsi->lanes);
#endif
        ret = mipi_dsi_attach(dsi);
        if (ret < 0) {
                dev_err(dev, "failed to attach dsi to host\n");
                goto err_dsi_attach;
        }

        return 0;

err_dsi_attach:
        mipi_dsi_device_unregister(dsi);
err_dsi_device:
        return ret;
}

static void sn65dsi84_mode_set(struct sn65dsi84 *sn65dsi84,
			     struct drm_display_mode *mode,
			     struct drm_display_mode *adj_mode)
{
	sn65dsi84_mipi_dsi_set(sn65dsi84, adj_mode);

	drm_mode_copy(&sn65dsi84->curr_mode, adj_mode);

	sn65dsi84->f_tmds = mode->clock;
}

/* Connector funcs */
static struct sn65dsi84 *connector_to_sn65dsi84(struct drm_connector *connector)
{
	return container_of(connector, struct sn65dsi84, connector);
}

static int sn65dsi84_connector_get_modes(struct drm_connector *connector)
{
	struct sn65dsi84 *adv = connector_to_sn65dsi84(connector);

	DRM_DEBUG("sn65dsi84_connector_get_modes\n");
	return sn65dsi84_get_modes(adv, connector);
}

static enum drm_mode_status
sn65dsi84_connector_mode_valid(struct drm_connector *connector,
			     struct drm_display_mode *mode)
{
	struct sn65dsi84 *adv = connector_to_sn65dsi84(connector);
	DRM_DEBUG("sn65dsi84_connector_mode_valid\n");
	return sn65dsi84_mode_valid(adv, mode);
}

static struct drm_connector_helper_funcs sn65dsi84_connector_helper_funcs = {
	.get_modes = sn65dsi84_connector_get_modes,
	.mode_valid = sn65dsi84_connector_mode_valid,
};

static enum drm_connector_status
sn65dsi84_connector_detect(struct drm_connector *connector, bool force)
{
	struct sn65dsi84 *adv = connector_to_sn65dsi84(connector);

	DRM_DEBUG("sn65dsi84_connector_detect\n");
	return sn65dsi84_detect(adv, connector);
}

static struct drm_connector_funcs sn65dsi84_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = sn65dsi84_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

/* Bridge funcs */
static struct sn65dsi84 *bridge_to_sn65dsi84(struct drm_bridge *bridge)
{
	return container_of(bridge, struct sn65dsi84, bridge);
}


static void sn65dsi84_calculate_clk_div_mul( u8 *clk_div_mul, unsigned int dsi_clk, unsigned int pixclk )
{
	unsigned int divider = MAX_DSI_DIVIDER;

	if(pixclk == 0) {
		DRM_INFO("sn65dsi84: Requested LVDS clock cannot be set\n");
		return;
	}

	// DSI clock divider calculation
	while ( ( (dsi_clk / divider ) < pixclk ) && ( divider > 0 ) )
	{
		divider--;
	}

	if (( divider == 0 ) || (pixclk < MIN_LVDS_CLOCK))
	{
		DRM_INFO("sn65dsi84: Requested LVDS clock cannot be set\n");
		return;
	}

	if ( pixclk < LOW_LVDS_CLOCK ) // some displays need higher frequency than computed at lower resolutions
		divider--;

	if( divider!= 0 && divider <= MIN_LVDS_CLOCK )
		*clk_div_mul = ((divider - 1) << 3);

	return;	
}

static void sn65dsi84_bridge_enable(struct drm_bridge *bridge)
{
	struct sn65dsi84 *adv = bridge_to_sn65dsi84(bridge);
	u8 val = 0;
	u32 val32;
	unsigned int dsi_clock = adv->dsi_clock;
	unsigned int lvds_clock = adv->lvds_timing->pixelclock;

	if (adv->lvds_dual_channel)
		lvds_clock = (lvds_clock >> 1); /* half of pixel clock */

	/*--------------------------------------------------------------------------------*/
	/* get DSI clock to calculate divisor */
	DRM_INFO("sn65dsi84_bridge_enable configuring bridge\n");
	/*--------------------------------------------------------------------------------*/
	/* Enable the chip power */
	sn65dsi84_power_on(adv);
	/* PLL Enable - Stop PLL */
	regmap_write(adv->regmap, SN65DSI84_REG_PLL_EN, 0x0);

	/*--------------------------------------------------------------------------------*/
	/* set divisor */
	val = 0;
	/* Calculate Multiplier and Divider for generate LVDS Clock from DSI Clock */
	sn65dsi84_calculate_clk_div_mul(&val, dsi_clock, lvds_clock);

	/* calculated, our display is very sensitive to timing */
#ifdef SN65DSI84_DBGPRN
	pr_info("DSI-sn65 - calculated divider for DSI to LVDS: 0x%x\n", val);
#endif
#ifdef SNDEBUG
	sn_debug.divisor = ((val >> 3) + 1);
	DRM_INFO("sn65dsi84_bridge_enable DSI CLOCK %d\n", dsi_clock);
	DRM_INFO("sn65dsi84_bridge_enable PIXEL_CLOCK %d\n", lvds_clock);
	DRM_INFO("sn65dsi84_bridge_enable DIVISOR %d\n", sn_debug.divisor);
#endif

	regmap_write(adv->regmap,SN65DSI84_REG_CLK_DIV_MUL, val);
	
	/*--------------------------------------------------------------------------------*/
	/* calculate LVDS frequency range */
	lvds_clock = dsi_clock / ((val >> 3) + 1); /* lvds_clock = dsi_clock / calculated_divider */
	if ( lvds_clock < 37500000 ) {
		val = 0x0;
	} else if ( lvds_clock < 62500000 ) {
		val = 0x1;
	} else if ( lvds_clock < 87500000 ) {
		val = 0x2;
	} else if ( lvds_clock < 112500000 ) {
		val = 0x3;
	} else if ( lvds_clock < 137500000 ) {
		val = 0x4;
	} else {
		val = 0x5;
	}
	/* shift to right position */
	val = val << 1;

	/* LVDS pixel clock derived from MIPI D-PHY channel A HS continuous */
	val |= 0x1;	 	
#ifdef SN65DSI84_DBGPRN
	pr_info("DSI-sn65 - calculated CLK RANGE for LVDS: 0x%x\n", val);
#endif
	regmap_write(adv->regmap,SN65DSI84_REG_LVDS_CLK, val);

	/*--------------------------------------------------------------------------------*/
	/* Enable DSI Lanes */
	if(adv->num_dsi_lanes == 1) regmap_write(adv->regmap, SN65DSI84_REG_DSI_LANES, ((0x1 << 5) | (0x3 << 3)));
	if(adv->num_dsi_lanes == 2) regmap_write(adv->regmap, SN65DSI84_REG_DSI_LANES, ((0x1 << 5) | (0x2 << 3)));
	if(adv->num_dsi_lanes == 3) regmap_write(adv->regmap, SN65DSI84_REG_DSI_LANES, ((0x1 << 5) | (0x1 << 3)));
	if(adv->num_dsi_lanes == 4) regmap_write(adv->regmap, SN65DSI84_REG_DSI_LANES, ((0x1 << 5) | (0x0 << 3)));
		
	/* DSI equalization - DSI clock and data equalization */
	regmap_write(adv->regmap,SN65DSI84_REG_DSI_EQ, /*0xCC*/0xCC);

#ifdef SN65DSI84_DBGPRN
	pr_info("DSI-sn65 - supposed dsi clock = %d\n", dsi_clock);
#endif
	regmap_write(adv->regmap, SN65DSI84_REG_DSI_CLK_RANGE, ((dsi_clock/(5*1000*1000)) + 0)); 
	

	/*--------------------------------------------------------------------------------*/
	/* set LVDS for single channel, 24 bit mode, HS/VS low, DE high */
	val = 0;
	if(!strcmp("rgb24",adv->lvds_color_depth)) {
		val |= 0x3 << 2;	
		DRM_INFO("LVDS color depth RGB24\n");
	} else {
		DRM_INFO("LVDS color depth RGB18\n");
	}

	if(!strcmp("jeida",adv->lvds_datamap)) {
                val |= 0x3;
                DRM_INFO("LVDS data mapping JEIDA\n");
	} else {
                DRM_INFO("LVDS data mapping VESA\n");
	}
	
	if(adv->lvds_dual_channel)
		val |= 0x0;
	else
		val |= (0x1 << 4);
	
    /* polarity setting */
	if(adv->hsync_polarity != SN65DSI84_SYNC_POLARITY_HIGH) {
		val |= (0x1 << 6);
#ifdef SN65DSI84_DBGPRN
		pr_info("DSI-sn65 - bridge_enable() HSYNC polarity set to low\n");
#endif
	} else {
#ifdef SN65DSI84_DBGPRN
		pr_info("DSI-sn65 - bridge_enable() HSYNC polarity set to high\n");
#endif
	}
	if(adv->vsync_polarity != SN65DSI84_SYNC_POLARITY_HIGH) {
		val |= (0x1 << 5);
#ifdef SN65DSI84_DBGPRN
		pr_info("DSI-sn65 - bridge_enable() VSYNC polarity set to low\n");
#endif
	} else {
#ifdef SN65DSI84_DBGPRN
		pr_info("DSI-sn65 - bridge_enable() VSYNC polarity set to high\n");
#endif
	}

#ifdef SN65DSI84_DBGPRN
	pr_info("DSI-sn65 - Configure reg 0x18: 0x%x\n",val);
#endif
	regmap_write(adv->regmap, SN65DSI84_REG_LVDS_PARAMETER,val);

	/*--------------------------------------------------------------------------------*/
	/*Channel Swap and Reverse options */
	val = 0;
	if(adv->lvds_channel_reverse)
		val |= (0x3 << 4);		
	if(adv->lvds_channel_swap)   
                val |= (0x1 << 6);
	/*--------------------------------------------------------------------------------*/
	regmap_write(adv->regmap, SN65DSI84_REG_LVDS_SWAP,val);

	/*--------------------------------------------------------------------------------*/
	/* adjust common mode */
	val = 0x30;
	regmap_write(adv->regmap, SN65DSI84_REG_LVDS_COMMON_MODE, val);

	/*--------------------------------------------------------------------------------*/
	/* Set resolution parameters */

	/* X resolution high/low */
	regmap_write(adv->regmap, SN65DSI84_REG_CHA_ACTIVE_LINE_LENGTH_LOW, adv->lvds_timing->hactive & 0x00ff );
	regmap_write(adv->regmap, SN65DSI84_REG_CHA_ACTIVE_LINE_LENGTH_HIGH, (adv->lvds_timing->hactive & 0xff00 )>>8 );
	/* Y resolution high/low */
	regmap_write(adv->regmap, SN65DSI84_REG_CHA_VERTICAL_DISPLAY_SIZE_LOW, adv->lvds_timing->vactive & 0x00ff );
	regmap_write(adv->regmap, SN65DSI84_REG_CHA_VERTICAL_DISPLAY_SIZE_HIGH,(adv->lvds_timing->vactive & 0xff00 )>>8);

	/* SYNC delay high/low */
	regmap_write(adv->regmap, SN65DSI84_REG_CHA_SYNC_DELAY_LOW, 0x20);
	regmap_write(adv->regmap, SN65DSI84_REG_CHA_SYNC_DELAY_HIGH, 0x0);
	/* HSYNC VSYNC width high/low */
	val32 = adv->lvds_timing->hsync_len;
	if(adv->lvds_dual_channel)
		val32 = val32 >> 1; /* half of this value */
	regmap_write(adv->regmap, SN65DSI84_REG_CHA_HSYNC_PULSE_WIDTH_LOW, val32 & 0x00ff);
	regmap_write(adv->regmap, SN65DSI84_REG_CHA_HSYNC_PULSE_WIDTH_HIGH,(val32 & 0xff00)>>8);
	
	regmap_write(adv->regmap, SN65DSI84_REG_CHA_VSYNC_PULSE_WIDTH_LOW,adv->lvds_timing->vsync_len & 0x00ff);
	regmap_write(adv->regmap, SN65DSI84_REG_CHA_VSYNC_PULSE_WIDTH_HIGH, (adv->lvds_timing->vsync_len & 0xff00)>>8);	

	val32 = adv->lvds_timing->hback_porch;
	if(adv->lvds_dual_channel)
		val32 = val32 >> 1; /* half of this value */
	regmap_write(adv->regmap, SN65DSI84_REG_CHA_HORIZONTAL_BACK_PORCH, val32);
	regmap_write(adv->regmap, SN65DSI84_REG_CHA_VERTICAL_BACK_PORCH, adv->lvds_timing->vback_porch);

#ifdef SNDEBUG
	sn_debug.delay = 0x20;
	sn_debug.sync = adv->lvds_timing->hsync_len;
	sn_debug.backporch = adv->lvds_timing->hback_porch;
	sn_debug.frontporch = adv->lvds_timing->hfront_porch;
	sn_debug.active = adv->lvds_timing->hactive;
#endif

	val32 = adv->lvds_timing->hfront_porch;
	if(adv->lvds_dual_channel)
		val32 = val32 >> 1; /* half of this value */
	regmap_write(adv->regmap, SN65DSI84_REG_CHA_HORIZONTAL_FRONT_PORCH, val32);
	regmap_write(adv->regmap, SN65DSI84_REG_CHA_VERTICAL_FRONT_PORCH, adv->lvds_timing->vfront_porch);

	/*--------------------------------------------------------------------------------*/
	/* Test Mode */
	if(adv->lvds_test_mode)
		regmap_write(adv->regmap, SN65DSI84_REG_CHA_TEST_PATTERN,0x10);
	else
		regmap_write(adv->regmap, SN65DSI84_REG_CHA_TEST_PATTERN,0x0);

	/*--------------------------------------------------------------------------------*/
	/* Reset the controller */
	regmap_write(adv->regmap, SN65DSI84_REG_RESET, 0x1);
	/*--------------------------------------------------------------------------------*/
	/* PLL Enable - Start PLL */    
	regmap_write(adv->regmap, SN65DSI84_REG_PLL_EN, 0x1);

	/*--------------------------------------------------------------------------------*/
	/* apply soft reset after a while */
	msleep(10);
	regmap_read(adv->regmap, SN65DSI84_REG_LVDS_CLK, &val32);
	val = (u8)(val32 & 0xff);
	val |= (1 << 7);
#ifdef SN65DSI84_DBGPRN
	pr_info("DSI-sn65 - bridge_enable() softreset val = %d\n", val);
#endif
	regmap_write(adv->regmap, SN65DSI84_REG_LVDS_CLK, val);
	msleep(1);
	/* read status after a while */
	regmap_write(adv->regmap, 0xE5, 0xFF);
	msleep(1);
	regmap_read(adv->regmap, 0xE5,&val32);
#ifdef SN65DSI84_DBGPRN
	pr_info("DSI-sn65 - bridge_enable() pixclock - %u\n", lvds_clock);
	pr_info("DSI-sn65 - bridge_enable() lanes - %d\n", adv->num_dsi_lanes);
	pr_info("DSI-sn65 - bridge_enable() read status: 0x%x\n",val32);
#endif

	return;
}

static void sn65dsi84_bridge_disable(struct drm_bridge *bridge)
{

	struct sn65dsi84 *adv = bridge_to_sn65dsi84(bridge);

	sn65dsi84_power_off(adv);

	return;
}

static void sn65dsi84_bridge_mode_set(struct drm_bridge *bridge,
				    struct drm_display_mode *mode,
				    struct drm_display_mode *adj_mode)
{
	struct sn65dsi84 *adv = bridge_to_sn65dsi84(bridge);

	DRM_INFO("sn65dsi84_bridge_mode_set\n");
	sn65dsi84_mode_set(adv, mode, adj_mode);
#ifdef SN65DSI84_DBGPRN
	/* ROPA - added info */
	pr_info("DSI-sn65 --------------------------------\n");
	pr_info("DSI-sn65 --------------------------------\n");

	pr_info("DSI-sn65 - bridge_mode_set()... called \n");
	pr_info("DSI-sn65 - bridge_mode_set() pixclock - %lu\n", adv->lvds_timing->pixelclock);
	pr_info("DSI-sn65 - bridge_mode_set() lanes - %d\n", adv->num_dsi_lanes);
#endif
}

static int sn65dsi84_bridge_attach(struct drm_bridge *bridge)
{
	struct sn65dsi84 *adv = bridge_to_sn65dsi84(bridge);
	int ret;

	DRM_INFO("sn65dsi84_bridge_attach\n");
	if (!bridge->encoder) {
		DRM_ERROR("Parent encoder object not found");
		return -ENODEV;
	}

	adv->connector.polled = DRM_CONNECTOR_POLL_HPD;


	ret = drm_connector_init(bridge->dev, &adv->connector,
				 &sn65dsi84_connector_funcs, DRM_MODE_CONNECTOR_LVDS);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}
	drm_connector_helper_add(&adv->connector,
				 &sn65dsi84_connector_helper_funcs);
	drm_mode_connector_attach_encoder(&adv->connector, bridge->encoder);

	ret = sn65dsi84_attach_dsi(adv);

	return ret;
}

static struct drm_bridge_funcs sn65dsi84_bridge_funcs = {
	.enable = sn65dsi84_bridge_enable,
	.disable = sn65dsi84_bridge_disable,
	.mode_set = sn65dsi84_bridge_mode_set,
	.attach = sn65dsi84_bridge_attach,
};
/* -----------------------------------------------------------------------------
 *  Attributes
 */
#ifdef SNDEBUG

static void sn65_reset(void)
{
	u8 val;
	u32 val32;

	/* Reset the controller */
	regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_RESET, 0x1);
	/* PLL Enable - Start PLL */
	regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_PLL_EN, 0x1);

	/* apply soft reset after a while */
	msleep(10);
	regmap_read(sn_debug.adv->regmap, SN65DSI84_REG_LVDS_CLK, &val32);
	val = (u8)(val32 & 0xff);
	val |= (1 << 7);
	pr_info("DSI-sn65 - bridge_enable() softreset val = %d\n", val);
	regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_LVDS_CLK, val);
	msleep(1);
	/* read status after a while */
	regmap_write(sn_debug.adv->regmap, 0xE5, 0xFF);
	msleep(1);
	regmap_read(sn_debug.adv->regmap, 0xE5,&val32);
	pr_info("DSI-sn65 - bridge_enable() pixclock - %lu\n", sn_debug.adv->lvds_timing->pixelclock);
	pr_info("DSI-sn65 - bridge_enable() lanes - %d\n", sn_debug.adv->num_dsi_lanes);
	pr_info("DSI-sn65 - bridge_enable() read status: 0x%x\n",val32);

}

static void update_divisor(void)
{
	if (sn_debug.adv) {
		/* suppoded that bridge is active - should be when file is accessible */
		/* PLL Enable - Stop PLL */
		regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_PLL_EN, 0x0);
		regmap_write(sn_debug.adv->regmap,SN65DSI84_REG_CLK_DIV_MUL, (((sn_debug.divisor - 1) << 3) & 0xfc));
		/* adjust dsi freq */
		regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_DSI_CLK_RANGE, ((sn_debug.dsifreq/(5*1000*1000)) + 0));
		sn65_reset();
	}
	return;
}

static void update_delay(void)
{
	u32 val = sn_debug.delay;

	if (sn_debug.adv) {
		/* suppoded that bridge is active - should be when file is accessible */
		/* PLL Enable - Stop PLL */
		regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_PLL_EN, 0x0);
		regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_CHA_SYNC_DELAY_LOW, (u8)(val & 0xff));
		regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_CHA_SYNC_DELAY_HIGH, (u8)((val >> 8) & 0x07));
		sn65_reset();
	}
	return;
}

static void update_porch(void)
{
	if (sn_debug.adv) {
		/* HSYNC VSYNC width high/low */
		if(sn_debug.adv->lvds_dual_channel) {
			regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_CHA_HSYNC_PULSE_WIDTH_LOW, (sn_debug.sync >> 1) & 0x00ff);
			regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_CHA_HSYNC_PULSE_WIDTH_HIGH,((sn_debug.sync >> 1) & 0xff00)>>8);
			regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_CHA_HORIZONTAL_BACK_PORCH, (sn_debug.backporch >> 1) & 0xff);
			regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_CHA_HORIZONTAL_FRONT_PORCH, (sn_debug.frontporch >> 1)& 0xff);
		} else {
			regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_CHA_HSYNC_PULSE_WIDTH_LOW, sn_debug.sync & 0x00ff);
			regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_CHA_HSYNC_PULSE_WIDTH_HIGH,(sn_debug.sync & 0xff00)>>8);
			regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_CHA_HORIZONTAL_BACK_PORCH, sn_debug.backporch & 0xff);
			regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_CHA_HORIZONTAL_FRONT_PORCH, sn_debug.frontporch & 0xff);
		}
		sn65_reset();
	}
	return;
}

static void update_active(void)
{
	if (sn_debug.adv) {
		/* X resolution high/low */
		regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_CHA_ACTIVE_LINE_LENGTH_LOW, sn_debug.active & 0x00ff );
		regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_CHA_ACTIVE_LINE_LENGTH_HIGH, (sn_debug.active & 0xff00 )>>8 );
		sn65_reset();

	}
 }

static ssize_t lvdshactive_show(struct device *child, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "LVDS horizontal active pixels = %d \n", sn_debug.active);
}
static ssize_t lvdshactive_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	/*ROPA: not too safe but ... */
	sscanf(buf, "%d", &value);
	sn_debug.active = (value & 0xffff);
	update_active();
	return count;
}
static DEVICE_ATTR(lvdshactive, 0664, lvdshactive_show, lvdshactive_store);

static ssize_t lvdshsync_show(struct device *child, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "LVDS horizontal sync length = %d \n", sn_debug.sync);
}
static ssize_t lvdshsync_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	/*ROPA: not too safe but ... */
	sscanf(buf, "%d", &value);
	sn_debug.sync = (value & 0xffff);

	update_porch();
	return count;
}
static DEVICE_ATTR(lvdshsync, 0664, lvdshsync_show, lvdshsync_store);

static ssize_t lvdshfporch_show(struct device *child, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "LVDS horizontal front porch = %d \n", sn_debug.frontporch);
}
static ssize_t lvdshfporch_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	/*ROPA: not too safe but ... */
	sscanf(buf, "%d", &value);
	sn_debug.frontporch = (value & 0xff);

	update_porch();
	return count;
}
static DEVICE_ATTR(lvdshfrontporch, 0664, lvdshfporch_show, lvdshfporch_store);


static ssize_t lvdshbporch_show(struct device *child, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "LVDS horizontal back porch = %d \n", sn_debug.backporch);
}
static ssize_t lvdshbporch_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	/*ROPA: not too safe but ... */
	sscanf(buf, "%d", &value);
	sn_debug.backporch = (value & 0xff);

	update_porch();
	return count;
}
static DEVICE_ATTR(lvdshbackporch, 0664, lvdshbporch_show, lvdshbporch_store);


static ssize_t lvdsdivisor_show(struct device *child, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "LVDS divisor = %d \n", sn_debug.divisor);
}
static ssize_t lvdsdivisor_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	/*ROPA: not too safe but ... */
	sscanf(buf, "%d", &value);
	sn_debug.divisor = (u8)(value & 0xff);

	update_divisor();
	return count;
}
static DEVICE_ATTR(lvdsdivisor, 0664, lvdsdivisor_show, lvdsdivisor_store);

static ssize_t lvdsdelay_show(struct device *child, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "LVDS delay = %d \n", sn_debug.delay);
}
static ssize_t lvdsdelay_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 value;
	/*ROPA: not too safe but ... */
	sscanf(buf, "%d", &value);
	sn_debug.delay = (value & 0x3ff);

	update_delay();
	return count;
}
static DEVICE_ATTR(lvdsdelay, 0664, lvdsdelay_show, lvdsdelay_store);

static void printbin(char *buf, u32 val32)
{
	int i = 7;
	unsigned char byte;

	byte = (val32 & 0xff);
	while (i >= 0) {
		buf[7 - i] = (byte & (1 << i)) ? '1':'0';
		i--;
	}
}

static ssize_t lvdsregisters_show(struct device *child, struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	u32 val32;
	u8 val8;
	char value[] = "0b00000000";

	/* read all mode related registers and show binary state */
	regmap_read(sn_debug.adv->regmap, 0x0A, &val32);
	printbin(value + 2, val32);
	val8 = (val32 & 0xff);
	count += sprintf(buf + count, "Register 0x0a ::  0x%02x - %s\n", val8, value);

	regmap_read(sn_debug.adv->regmap, 0x0B, &val32);
	printbin(value + 2, val32);
	val8 = (val32 & 0xff);
	count += sprintf(buf + count, "Register 0x0b ::  0x%02x - %s\n", val8, value);

	regmap_read(sn_debug.adv->regmap, 0x10, &val32);
	printbin(value + 2, val32);
	val8 = (val32 & 0xff);
	count += sprintf(buf + count, "Register 0x10 ::  0x%02x - %s\n", val8, value);

	regmap_read(sn_debug.adv->regmap, 0x11, &val32);
	printbin(value + 2, val32);
	val8 = (val32 & 0xff);
	count += sprintf(buf + count, "Register 0x11 ::  0x%02x - %s\n", val8, value);

	regmap_read(sn_debug.adv->regmap, 0x12, &val32);
	printbin(value + 2, val32);
	val8 = (val32 & 0xff);
	count += sprintf(buf + count, "Register 0x12 ::  0x%02x - %s\n", val8, value);

	regmap_read(sn_debug.adv->regmap, 0x18, &val32);
	printbin(value + 2, val32);
	val8 = (val32 & 0xff);
	count += sprintf(buf + count, "Register 0x18 ::  0x%02x - %s\n", val8, value);

	regmap_read(sn_debug.adv->regmap, 0x20, &val32);
	printbin(value + 2, val32);
	val8 = (val32 & 0xff);
	count += sprintf(buf + count, "Register 0x20 ::  0x%02x - %s\n", val8, value);

	regmap_read(sn_debug.adv->regmap, 0x21, &val32);
	printbin(value + 2, val32);
	val8 = (val32 & 0xff);
	count += sprintf(buf + count, "Register 0x21 ::  0x%02x - %s\n", val8, value);

	regmap_read(sn_debug.adv->regmap, 0x28, &val32);
	printbin(value + 2, val32);
	val8 = (val32 & 0xff);
	count += sprintf(buf + count, "Register 0x28 ::  0x%02x - %s\n", val8, value);

	regmap_read(sn_debug.adv->regmap, 0x29, &val32);
	printbin(value + 2, val32);
	val8 = (val32 & 0xff);
	count += sprintf(buf + count, "Register 0x29 ::  0x%02x - %s\n", val8, value);

	regmap_read(sn_debug.adv->regmap, 0x2c, &val32);
	printbin(value + 2, val32);
	val8 = (val32 & 0xff);
	count += sprintf(buf + count, "Register 0x2c ::  0x%02x - %s\n", val8, value);

	regmap_read(sn_debug.adv->regmap, 0x2d, &val32);
	printbin(value + 2, val32);
	val8 = (val32 & 0xff);
	count += sprintf(buf + count, "Register 0x2d ::  0x%02x - %s\n", val8, value);

	regmap_read(sn_debug.adv->regmap, 0x30, &val32);
	printbin(value + 2, val32);
	val8 = (val32 & 0xff);
	count += sprintf(buf + count, "Register 0x30 ::  0x%02x - %s\n", val8, value);

	regmap_read(sn_debug.adv->regmap, 0x31, &val32);
	printbin(value + 2, val32);
	val8 = (val32 & 0xff);
	count += sprintf(buf + count, "Register 0x31 ::  0x%02x - %s\n", val8, value);

	regmap_read(sn_debug.adv->regmap, 0x34, &val32);
	printbin(value + 2, val32);
	val8 = (val32 & 0xff);
	count += sprintf(buf + count, "Register 0x34 ::  0x%02x - %s\n", val8, value);

	regmap_read(sn_debug.adv->regmap, 0xE1, &val32);
	printbin(value + 2, val32);
	val8 = (val32 & 0xff);
	count += sprintf(buf + count, "Register 0xE1 ::  0x%02x - %s\n", val8, value);

	regmap_read(sn_debug.adv->regmap, 0xE5, &val32);
	regmap_write(sn_debug.adv->regmap, 0xE5, 0xff);
	printbin(value + 2, val32);
	val8 = (val32 & 0xff);
	count += sprintf(buf + count, "Register 0xE5 ::  0x%02x - %s\n", val8, value);

	return count;
}
static DEVICE_ATTR(lvdsregisters, 0444, lvdsregisters_show, NULL);

extern void update_dsiHW(void);
__attribute__((weak)) void update_dsiHW(void)
{
    return;
}

static ssize_t dsi_hfporch_show(struct device *child, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "DSI horizontal front porch = %d \n", sn_debug.dsi_hfporch);
}
static ssize_t dsi_hfporch_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	/*ROPA: not too safe but ... */
	sscanf(buf, "%d", &value);
	sn_debug.dsi_hfporch = value;

	update_dsiHW();
	update_divisor();
	return count;
}
static DEVICE_ATTR(dsi_hfporch, 0664, dsi_hfporch_show, dsi_hfporch_store);

static ssize_t dsi_hbporch_show(struct device *child, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "DSI horizontal back porch = %d \n", sn_debug.dsi_hbporch);
}
static ssize_t dsi_hbporch_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	/*ROPA: not too safe but ... */
	sscanf(buf, "%d", &value);
	sn_debug.dsi_hbporch = value;

	update_dsiHW();
	update_divisor();
	return count;
}
static DEVICE_ATTR(dsi_hbporch, 0664, dsi_hbporch_show, dsi_hbporch_store);

static ssize_t dsi_hsync_show(struct device *child, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "DSI horizontal sync = %d \n", sn_debug.dsi_hsync & 0xffff);
}
static ssize_t dsi_hsync_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	/*ROPA: not too safe but ... */
	sscanf(buf, "%d", &value);
	sn_debug.dsi_hsync |= (value & 0xffff);

	update_dsiHW();
	update_divisor();
	return count;
}
static DEVICE_ATTR(dsi_hsync, 0664, dsi_hsync_show, dsi_hsync_store);

static ssize_t dsifreq_show(struct device *child, struct device_attribute *attr, char *buf)
{
	ssize_t count;
	count =  sprintf(buf, "calculated DSI frequency = %d \n", sn_debug.dsifreq);
	count += sprintf(buf + count, "current    DSI frequency = %d \n", sn_debug.adv->dsi_clock);
	return count;
}
static ssize_t dsifreq_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	/*ROPA: not too safe but ... */
	sscanf(buf, "%d", &value);
	sn_debug.dsifreq = value;
	sn_debug.adv->dsi_clock = value;

	sec_mipi_force_freq(DIV_ROUND_UP(sn_debug.dsifreq, 1000) << 1); /* freq * 2 - DSI transfer two bits at one clock (on rising and falling edge)*/
	update_dsiHW();
	update_divisor();

	return count;
}
static DEVICE_ATTR(dsifreq, 0664, dsifreq_show, dsifreq_store);

static ssize_t dsi_pll_show(struct device *child, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "DSI PLL:\n m=%d\n p=%d\n s=%d\n ", sn_debug.dsi_m, sn_debug.dsi_p, sn_debug.dsi_s);
}
static DEVICE_ATTR(dsipll, 0444, dsi_pll_show, NULL);


static ssize_t dsi_regs_show(struct device *child, struct device_attribute *attr, char *buf)
{
	u32 *ptr;
	ssize_t i;

	ptr = (u32*) ioremap(0x0000000032e10000,100); // ROPA: dirty but functional
	i = sprintf(buf, "DSI: ptr mapped to virt_addr = 0x%016llx \n", (u64)ptr);
	i += sprintf(buf + i,"DSI: DSI_CLKCTRL setting register = 0x%08x \n", *(ptr + 4));
	i += sprintf(buf + i,"DSI: DSI_TIMEOUT setting register = 0x%08x \n", *(ptr + 5));
	i += sprintf(buf + i,"DSI: DSI_CONFIG setting register = 0x%08x \n", *(ptr + 6));
	i += sprintf(buf + i,"DSI: DSI_ESCMODE setting register = 0x%08x \n", *(ptr + 7));
	i += sprintf(buf + i,"DSI: DSI_MDRESOL setting register = 0x%08x \n", *(ptr + 8));
	i += sprintf(buf + i,"DSI: DSI_MVPORCH setting register = 0x%08x \n", *(ptr + 9));
	i += sprintf(buf + i,"DSI: DSI_MHPORCH setting register = 0x%08x \n", *(ptr + 10));
	i += sprintf(buf + i,"DSI: DSI_MSYNC setting register = 0x%08x \n", *(ptr + 11));

	iounmap((void*)((unsigned long)ptr));

	return i;
}
static DEVICE_ATTR(dsi_regs, 0444, dsi_regs_show, NULL);


static ssize_t sn65dsi84_status_show(struct device *child, struct device_attribute *attr, char *buf)
{
	u8 value;
	u32 val32;

	regmap_read(sn_debug.adv->regmap, 0xE5,&val32);
	value = (u8)(val32 & 0xff);

	return sprintf(buf, "Status register 0xe5 = 0x%02X \n", value);
}
static DEVICE_ATTR(sn65dsi84_status, 0444, sn65dsi84_status_show, NULL);

static ssize_t sn65dsi84_test_show(struct device *child, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "SN65DSI84: write 1 to set test mode and 0 to disable that \n");
}

static ssize_t sn65dsi84_test_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d", &value);
	if (value) {
		regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_CHA_TEST_PATTERN,0x10);
	} else {
		regmap_write(sn_debug.adv->regmap, SN65DSI84_REG_CHA_TEST_PATTERN,0x0);
	}
	return count;
}
static DEVICE_ATTR(sn65dsi84_test, 0644, sn65dsi84_test_show, sn65dsi84_test_store);

extern ssize_t lcdif_dump_sysfs(char *buf);
__attribute__((weak)) ssize_t lcdif_dump_sysfs(char *buf)
{
    return sprintf(buf, "SN65DSI84: NOT SUPPORTED \n");
}

static ssize_t lcdif_test_show(struct device *child, struct device_attribute *attr, char *buf)
{
	return lcdif_dump_sysfs(buf);
}
static DEVICE_ATTR(lcdif_regs, 0444, lcdif_test_show, NULL);

static ssize_t lcdif_mux_show(struct device *child, struct device_attribute *attr, char *buf)
{
	u32 *ptr;
	ssize_t i;

	ptr = (u32*) ioremap(0x000000003038a500,1); // ROPA: dirty but functional
	i = sprintf(buf, "LCDIF: ptr mapped to virt_addr = 0x%016llx \n", (u64)ptr);
	i += sprintf(buf + i,"LCDIF: CCM_TARGET setting register = 0x%08x \n", *ptr);
	iounmap((void*)((unsigned long)ptr));

	return i;
}
static DEVICE_ATTR(lcdif_mux, 0444, lcdif_mux_show, NULL);

static ssize_t lcdif_lodiv_show(struct device *child, struct device_attribute *attr, char *buf)
{
	u32 *ptr;
	ssize_t i;

	ptr = (u32*) ioremap(0x000000003038a500,1); // ROPA: dirty but functional
	i = sprintf(buf, "LCDIF: ptr mapped to virt_addr = 0x%016llx \n", (u64)ptr);
	i += sprintf(buf + i,"LCDIF: CCM_TARGET setting register = 0x%08x \n", *ptr);
	iounmap((void*)((unsigned long)ptr));

	return i;
}
static ssize_t lcdif_lodiv_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 value;
	u32 *ptr;

	ptr = (u32*) ioremap(0x000000003038a500,1); // ROPA: dirty but functional
	sscanf(buf, "%ud", &value);
	*ptr = (*ptr & ~(0x3f)) | (value & (0x3f));
	iounmap((void*)((unsigned long)ptr));

	return count;
}
static DEVICE_ATTR(lcdif_lodiv, 0644, lcdif_lodiv_show, lcdif_lodiv_store);


static ssize_t lcdif_pll_show(struct device *child, struct device_attribute *attr, char *buf)
{
	u32 *ptr;
	ssize_t i;

	ptr = (u32*) ioremap(0x0000000030360028,20); // ROPA: dirty but functional
	i = sprintf(buf, "LCDIF: ptr mapped to virt_addr = 0x%016llx \n", (u64)ptr);
	i += sprintf(buf + i,"LCDIF: PLL_GEN_CTRL setting register = 0x%08x \n", *ptr);
	i += sprintf(buf + i,"LCDIF: PLL_GEN_FDIV0 setting register = 0x%08x \n", *(ptr + 1));
	i += sprintf(buf + i,"LCDIF: PLL_GEN_FDIV1 setting register = 0x%08x \n", *(ptr + 2));
	i += sprintf(buf + i,"LCDIF: PLL_GEN_SSCG setting register = 0x%08x \n", *(ptr + 3));
	i += sprintf(buf + i,"LCDIF: PLL_GEN_MNIT setting register = 0x%08x \n", *(ptr + 4));

	iounmap((void*)((unsigned long)ptr));

	return i;
}
static DEVICE_ATTR(lcdif_pll, 0444, lcdif_pll_show, NULL);


static void sn65dsi84_create_attribute(struct device *dev, struct sn65dsi84 *adv)
{
	if (!sn_debug.dsifreq)
	{
		/* set to known value */
		sn_debug.dsifreq = adv->dsi_clock;
	}

	sn_debug.adv = adv;
	device_create_file(dev, &dev_attr_lvdsdivisor);
	device_create_file(dev, &dev_attr_lvdsdelay);
	device_create_file(dev, &dev_attr_dsifreq);
	device_create_file(dev, &dev_attr_dsipll);
	device_create_file(dev, &dev_attr_dsi_regs);
	device_create_file(dev, &dev_attr_sn65dsi84_status);
	device_create_file(dev, &dev_attr_sn65dsi84_test);
	device_create_file(dev, &dev_attr_lvdshsync);
	device_create_file(dev, &dev_attr_lvdshbackporch);
	device_create_file(dev, &dev_attr_lvdshfrontporch);
	device_create_file(dev, &dev_attr_lvdshactive);
	device_create_file(dev, &dev_attr_lvdsregisters);
	device_create_file(dev, &dev_attr_dsi_hbporch);
	device_create_file(dev, &dev_attr_dsi_hfporch);
	device_create_file(dev, &dev_attr_dsi_hsync);
	device_create_file(dev, &dev_attr_lcdif_regs);
	device_create_file(dev, &dev_attr_lcdif_mux);
	device_create_file(dev, &dev_attr_lcdif_pll);
	device_create_file(dev, &dev_attr_lcdif_lodiv);
}

static void sn65dsi84_delete_attributes(struct device *dev)
{
	device_remove_file(dev, &dev_attr_lvdsdivisor);
	device_remove_file(dev, &dev_attr_lvdsdelay);
	device_remove_file(dev, &dev_attr_dsifreq);
	device_remove_file(dev, &dev_attr_dsipll);
	device_remove_file(dev, &dev_attr_dsi_regs);
	device_remove_file(dev, &dev_attr_sn65dsi84_status);
	device_remove_file(dev, &dev_attr_sn65dsi84_test);
	device_remove_file(dev, &dev_attr_lvdshsync);
	device_remove_file(dev, &dev_attr_lvdshbackporch);
	device_remove_file(dev, &dev_attr_lvdshfrontporch);
	device_remove_file(dev, &dev_attr_lvdshactive);
	device_remove_file(dev, &dev_attr_lvdsregisters);
	device_remove_file(dev, &dev_attr_dsi_hbporch);
	device_remove_file(dev, &dev_attr_dsi_hfporch);
	device_remove_file(dev, &dev_attr_dsi_hsync);
	device_remove_file(dev, &dev_attr_lcdif_regs);
	device_remove_file(dev, &dev_attr_lcdif_mux);
	device_remove_file(dev, &dev_attr_lcdif_pll);
	device_remove_file(dev, &dev_attr_lcdif_lodiv);
}

int sn65dsi84_get_DSI_freq(void)
{
	return (DIV_ROUND_UP(sn_debug.dsifreq, 1000) << 1); /* freq * 2 - DSI transfer two bits at one clock (on rising and falling edge)*/
}
EXPORT_SYMBOL(sn65dsi84_get_DSI_freq);

void sn65dsi84_set_pll(int m, int p, int s, int freq)
{
	sn_debug.dsi_m = m;
	sn_debug.dsi_p = p;
	sn_debug.dsi_s = s;
	sn_debug.dsifreq = (freq >> 1) * 1000; /* get half of bit_rate value */
}
EXPORT_SYMBOL(sn65dsi84_set_pll);

void sn65dsi84_set_dsi_params(u32 hfp, u32 hbp, u32 hsyn)
{
	sn_debug.dsi_hfporch = hfp;
	sn_debug.dsi_hbporch = hbp;
	sn_debug.dsi_hsync = hsyn;
}
EXPORT_SYMBOL(sn65dsi84_set_dsi_params);

u32 sn65dsi84_get_hfporch(void)
{
	return sn_debug.dsi_hfporch;
}
EXPORT_SYMBOL(sn65dsi84_get_hfporch);

u32 sn65dsi84_get_hbporch(void)
{
	return sn_debug.dsi_hbporch;
}
EXPORT_SYMBOL(sn65dsi84_get_hbporch);

u32 sn65dsi84_get_hsyn(void)
{
	return sn_debug.dsi_hsync;
}
EXPORT_SYMBOL(sn65dsi84_get_hsyn);

/* -----------------------------------------------------------------------------*/

#else // SNDEBUG
static void sn65dsi84_create_attribute(struct device *dev, struct sn65dsi84 *adv)
{
}

static void sn65dsi84_delete_attributes(struct device *dev)
{
}
#endif // SNDEBUG
/* -----------------------------------------------------------------------------*/
/* -----------------------------------------------------------------------------
 * DTB
 */
/* Calculate dsi frequency on my own - override dsi */
static void sn65dsi84_force_DSI_freq(struct sn65dsi84 *adv)
{
	sec_mipi_force_freq(DIV_ROUND_UP(adv->dsi_clock, 1000) << 1);
	/* DSI output is DSI_bit_clk / 2; because DSI transfers data on rising and falling edge
	 * so as dsi_clock is demanded output DSI lane frequency, I need to set DSI clock at dsi_clock * 2 == DSI_bit_clk
	 */
}

#define MAX_UP_INC 2500000 /* allow increase of pixclk frequency about 2.5MHz max */

static u32 sn65dsi84_adjust_pixclk(u32 oldpixclk)
{
	u32 divider, newpixclk;

	divider = DIV_ROUND_CLOSEST(BUS_CLK, oldpixclk);
	newpixclk = (BUS_CLK / divider);
	if ((newpixclk > oldpixclk) && ((newpixclk - oldpixclk) > MAX_UP_INC)) {
		divider ++;
		newpixclk = (BUS_CLK / divider);
	}

	if (newpixclk < MIN_LVDS_CLOCK) {
		divider --;
		newpixclk = (BUS_CLK / divider);
	}
	return newpixclk;
}

static bool sn65dsi84_videomode_parse_dt(struct device_node *np, struct sn65dsi84 *adv)
{
	u32 value, bpp, ret, num_lanes;

	if(of_property_read_u32(np, "lvds,pixelclock", &value))  return false;
	// adjust pixel clock
	if (value < MIN_LVDS_CLOCK) value = MIN_LVDS_CLOCK;

	adv->lvds_timing->pixelclock = sn65dsi84_adjust_pixclk(value);

	/* Horizonatal parameters */
	if(of_property_read_u32(np, "lvds,hactive", &value)) return false;
	adv->lvds_timing->hactive = value;
	if(of_property_read_u32(np, "lvds,hfront_porch", &value)) return false;
	adv->lvds_timing->hfront_porch = value;
	if(of_property_read_u32(np, "lvds,hback_porch", &value)) return false;
	adv->lvds_timing->hback_porch = value;
	if(of_property_read_u32(np, "lvds,hsync_len", &value)) return false;
	adv->lvds_timing->hsync_len = value;

	/* Vertical parameters */
	if(of_property_read_u32(np, "lvds,vactive", &value)) return false;
	adv->lvds_timing->vactive = value;
	if(of_property_read_u32(np, "lvds,vfront_porch", &value)) return false;
	adv->lvds_timing->vfront_porch = value;
	if(of_property_read_u32(np, "lvds,vback_porch", &value)) return false;
	adv->lvds_timing->vback_porch = value;
	if(of_property_read_u32(np, "lvds,vsync_len", &value)) return false;
	adv->lvds_timing->vsync_len = value;
	if(of_property_read_string(np, "lvds,color_depth", &(adv->lvds_color_depth))) return false;
	if(of_property_read_string(np, "lvds,datamap", &(adv->lvds_datamap))) return false;

	/* LVDS parameters */
	adv->lvds_dual_channel = of_property_read_bool(np, "lvds,dual-channel");
	adv->lvds_channel_reverse = of_property_read_bool(np, "lvds,channel-reverse");
	adv->lvds_channel_swap = of_property_read_bool(np, "lvds,channel-swap");
	adv->lvds_test_mode = of_property_read_bool(np, "lvds,test-mode");

	ret = of_property_read_u32(np, "lvds,hsync_pol", &value);
	if ((!ret) && value) {
		adv->hsync_polarity = SN65DSI84_SYNC_POLARITY_HIGH;
	} else {
		adv->hsync_polarity = SN65DSI84_SYNC_POLARITY_LOW;
	}
	ret = of_property_read_u32(np, "lvds,vsync_pol", &value);
	if ((!ret) && value) {
		adv->vsync_polarity = SN65DSI84_SYNC_POLARITY_HIGH;
	} else {
		adv->vsync_polarity = SN65DSI84_SYNC_POLARITY_LOW;
	}

	/* DSI parameters */
    of_property_read_u32(np, "adi,dsi-lanes", &num_lanes);

    if (num_lanes < 1 || num_lanes > 4)
              return false;

    adv->num_dsi_lanes = num_lanes;

#if 0  // for now ignore
	ret = of_property_read_u32(np, "adi,dsi-clock", &value);
	if (ret < 0){
		bpp = 24; /* only supported DSI bit format MIPI_DSI_FMT_RGB888 */
		value = DIV_ROUND_UP(adv->lvds_timing->pixelclock * bpp, adv->num_dsi_lanes);
	}
#endif
	/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
	/* freq * 2 - DSI transfer two times faster than necessary
	 * DSI output is DSI_bit_clk / 2; because DSI transfers data on rising and falling edge
	 * that is output frequency at DSI lane is half of bit rate
	 * I calculate dsi_clock as necessary bit rate, this force DSI to work at twice higher bit rate
	 * DSI_clk signal has dsi_clock frequency
	 * whith this setting I see less errors on sn64DSI84 ???????? */
	bpp = 24; // for now RGB888
	value = DIV_ROUND_UP(adv->lvds_timing->pixelclock * bpp, adv->num_dsi_lanes);
	if (value > BUS_CLK) {
		value >>= 1; /* divide by two !!! as we reached maximum value */
	}
	adv->dsi_clock = value;

	return true;
}

static int sn65dsi84_parse_dt(struct device_node *np, struct sn65dsi84 *adv)
{
    struct device_node *endpoint;
	int ret;

	if(!sn65dsi84_videomode_parse_dt(np, adv)) {
			return -ENODEV;
	}

    adv->gpio_pd = of_get_named_gpio(np, "pd", 0);
    if (!gpio_is_valid(adv->gpio_pd))
             DRM_INFO("No valid Power Gpio Found\n");
	else {
		ret = gpio_request(adv->gpio_pd, "sn65dsi84_pwr");  
		if (ret < 0)
       			return -EINVAL;
		ret = gpio_direction_output(adv->gpio_pd, 0);
        if (ret < 0)
                return -EINVAL;
    }

    adv->gpio_bckl = of_get_named_gpio(np, "bckl", 0);
    if (!gpio_is_valid(adv->gpio_bckl))
             DRM_INFO("No valid Power Gpio Found\n");
	else {
		ret = gpio_request(adv->gpio_bckl, "sn65dsi84_pwr");
		if (ret < 0)
       			return -EINVAL;
		ret = gpio_direction_output(adv->gpio_bckl, 0);
        if (ret < 0)
                return -EINVAL;
    }

    endpoint = of_graph_get_next_endpoint(np, NULL);
    if (!endpoint)
            return -ENODEV;

	adv->host_node = of_graph_get_remote_port_parent(endpoint);
	if (!adv->host_node) {
			of_node_put(endpoint);
			return -ENODEV;
	}

	of_node_put(endpoint);
	of_node_put(adv->host_node);

	adv->use_timing_gen = !of_property_read_bool(np,
											"adi,disable-timing-generator");

	adv->rgb = true;
	adv->embedded_sync = false;

	return 0;
}

/* -----------------------------------------------------------------------------
 * probe + remove
 */


static int sn65dsi84_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct sn65dsi84_link_config link_config;
	struct sn65dsi84 *sn65dsi84;
	struct videomode *vmode;
	struct device *dev = &i2c->dev;
	unsigned int val;
	int ret;

	if (!dev->of_node)
		return -EINVAL;

	sn65dsi84 = devm_kzalloc(dev, sizeof(*sn65dsi84), GFP_KERNEL);
	if (!sn65dsi84)
		return -ENOMEM;

	vmode = devm_kzalloc(dev, sizeof(*vmode), GFP_KERNEL);
        if (!vmode)
                return -ENOMEM;

    sn65dsi84->lvds_timing = vmode;

	sn65dsi84->powered = false;
	sn65dsi84->status = connector_status_disconnected;


	memset(&link_config, 0, sizeof(link_config));

	ret = sn65dsi84_parse_dt(dev->of_node, sn65dsi84);
	if (ret) {
		dev_info(dev,"error parsing DT\n");
		return ret;
	}

	/*
	 * The power down GPIO is optional. If present, toggle it from active to
	 * inactive to wake up the encoder.
	 */

	sn65dsi84_power_on(sn65dsi84);


	sn65dsi84->regmap = devm_regmap_init_i2c(i2c, &sn65dsi84_regmap_config);
	if (IS_ERR(sn65dsi84->regmap))
		return PTR_ERR(sn65dsi84->regmap);


	ret = regmap_read(sn65dsi84->regmap, SN65DSI84_REG_DSI_PROTOCOL_ERR, &val);

	if (ret) {
		dev_info(dev, "physical bridge not found\n");
		return -ENODEV;
	}
	if (val)
		dev_info(dev, "physical bridge found\n");
	else
		return -ENODEV;


	sn65dsi84_power_off(sn65dsi84);



	i2c_set_clientdata(i2c, sn65dsi84);

	sn65dsi84_set_link_config(sn65dsi84, &link_config);

	sn65dsi84->bridge.funcs = &sn65dsi84_bridge_funcs;
	sn65dsi84->bridge.of_node = dev->of_node;
	ret = drm_bridge_add(&sn65dsi84->bridge);
	if (ret) {
		dev_err(dev, "failed to add sn65dsi84 bridge\n");
		goto err_i2c_unregister_edid;
	}

	sn65dsi84_force_DSI_freq(sn65dsi84);

	sn65dsi84_create_attribute(&i2c->dev, sn65dsi84);

	return 0;

err_i2c_unregister_edid:
	return ret;
}

static int sn65dsi84_remove(struct i2c_client *i2c)
{
	struct sn65dsi84 *sn65dsi84 = i2c_get_clientdata(i2c);

	drm_bridge_remove(&sn65dsi84->bridge);

	i2c_unregister_device(sn65dsi84->i2c_edid);

	kfree(sn65dsi84->edid);

	sn65dsi84_delete_attributes(&i2c->dev);

	return 0;
}

static const struct i2c_device_id sn65dsi84_i2c_ids[] = {
	{ "sn65dsi84", SN65DSI84 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sn65dsi84_i2c_ids);

static const struct of_device_id sn65dsi84_of_ids[] = {
	{ .compatible = "ti,sn65dsi84", .data = (void *)SN65DSI84 },
	{ }
};
MODULE_DEVICE_TABLE(of, sn65dsi84_of_ids);

static struct mipi_dsi_driver adv7533_dsi_driver = {
	.driver.name = "sn65dsi84",
};

static struct i2c_driver sn65dsi84_driver = {
	.driver = {
		.name = "sn65dsi84",
		.of_match_table = sn65dsi84_of_ids,
	},
	.id_table = sn65dsi84_i2c_ids,
	.probe = sn65dsi84_probe,
	.remove = sn65dsi84_remove,
};

static int __init sn65dsi84_init(void)
{
	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
		mipi_dsi_driver_register(&adv7533_dsi_driver);

	return i2c_add_driver(&sn65dsi84_driver);
}
module_init(sn65dsi84_init);

static void __exit sn65dsi84_exit(void)
{
	i2c_del_driver(&sn65dsi84_driver);

	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
		mipi_dsi_driver_unregister(&adv7533_dsi_driver);

}
module_exit(sn65dsi84_exit);

MODULE_AUTHOR("Robert Pasz <robert.pasz@congatec.com>");
MODULE_DESCRIPTION("SN65DSI84 LVDS transmitter driver");
MODULE_LICENSE("GPL");
