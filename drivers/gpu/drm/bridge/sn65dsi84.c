// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020, congatec a.g.
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

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>


#define MAX_DSI_DIVIDER 		25
#define MIN_LVDS_CLOCK 			24000

/* Registers */
#define SOFT_RESET_REG					0x09

#define PLL_REG						0x0A
#define HS_CLK_SRC_BIT				BIT(0)
#define LVDS_CLK_RANGE_MASK			GENMASK(3, 1)
#define LVDS_CLK_RANGE_SHIFT(x)		((x) << 1)
#define PLL_EN_STAT_BIT				BIT(7)

#define CLK_REG						0x0B
#define DSI_CLK_DIVIDER_MASK		GENMASK(7, 3)
#define DSI_CLK_DIVIDER_SHIFT(x)	((x) << 3)
#define REFCLK_MULTIPLIER_MASK		GENMASK(1, 0)
#define REFCLK_MULTIPLIER_SHIFT(x)	((x) << 0)

#define PLL_EN_REG					0x0D

#define CHA_DSI_CFG_REG				0x10
#define CHA_DSI_LANES_MASK			GENMASK(4, 3)
#define CHA_DSI_LANES_SHIFT(x)		((x) << 3)

#define CHA_DSI_CLK_RANGE_REG		0x12

#define LVDS_CFG_REG				0x18
#define CHB_24BPP_FORMAT1_BIT		BIT(0)
#define CHA_24BPP_FORMAT1_BIT		BIT(1)
#define CHB_24BPP_MODE_BIT			BIT(2)
#define CHA_24BPP_MODE_BIT			BIT(3)
#define LVDS_LINK_CFG_BIT			BIT(4)
#define VS_NEG_POLARITY_BIT			BIT(5)
#define HS_NEG_POLARITY_BIT			BIT(6)
#define DE_NEG_POLARITY_BIT			BIT(7)

#define LVDS18h_CHB_24BPP_FORMAT1_OFFSET        0
#define LVDS18h_CHA_24BPP_FORMAT1_OFFSET        1
#define LVDS18h_CHB_24BPP_MODE_OFFSET           2
#define LVDS18h_CHA_24BPP_MODE_OFFSET           3
#define LVDS18h_LVDS_LINK_CFG_OFFSET            4
#define LVDS18h_VS_NEG_POLARITY_OFFSET          5
#define LVDS18h_HS_NEG_POLARITY_OFFSET          6
#define LVDS18h_DE_NEG_POLARITY_OFFSET          7

#define CHA_ACTIVE_LINE_LENGTH_LOW_REG	0x20
#define CHA_ACTIVE_LINE_LENGTH_HIGH_REG	0x21

#define CHA_VERTICAL_DISPLAY_SIZE_LOW_REG	0x24
#define CHA_VERTICAL_DISPLAY_SIZE_HIGH_REG	0x25

#define CHA_SYNC_DELAY_LOW_REG		0x28
#define CHA_SYNC_DELAY_HIGH_REG		0x29

#define CHA_HSYNC_PULSE_WIDTH_LOW_REG	0x2C
#define CHA_HSYNC_PULSE_WIDTH_HIGH_REG	0x2D

#define CHA_VSYNC_PULSE_WIDTH_LOW_REG	0x30
#define CHA_VSYNC_PULSE_WIDTH_HIGH_REG	0x31

#define CHA_HORIZONTAL_BACK_PORCH_REG	0x34

#define CHA_VERTICAL_BACK_PORCH_REG		0x36

#define CHA_HORIZONTAL_FRONT_PORCH_REG	0x38

#define CHA_VERTICAL_FRONT_PORCH_REG	0x3A

struct sn65dsi84_bridge {
	struct device			*dev;
	struct regmap			*regmap;
	struct drm_bridge		bridge;
	struct drm_connector		connector;
	struct dentry			*debugfs;
	struct device_node		*host_node;
	struct mipi_dsi_device		*dsi;
	struct drm_panel		*panel;
	struct gpio_desc		*enable_gpio;
	unsigned int			dsi_freq;
	unsigned int			lvds_use_jeida;
	unsigned int			lvds_is_two_channel;
	unsigned int			lvds_bpp_is_18;
	unsigned int			de_pol;
	unsigned int			hsync_pol;
	unsigned int			vsync_pol;
};

static const struct regmap_range sn65dsi84_bridge_volatile_ranges[] = {
	{ .range_min = 0, .range_max = 0xE5 },
};

static const struct regmap_access_table sn65dsi84_bridge_volatile_table = {
	.yes_ranges = sn65dsi84_bridge_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(sn65dsi84_bridge_volatile_ranges),
};

static const struct regmap_config sn65dsi84_bridge_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_table = &sn65dsi84_bridge_volatile_table,
	.cache_type = REGCACHE_NONE,
};

static int __maybe_unused sn65dsi84_bridge_resume(struct device *dev)
{
	struct sn65dsi84_bridge *pdata = dev_get_drvdata(dev);

	gpiod_set_value(pdata->enable_gpio, 1);

	return 0;
}

static int __maybe_unused sn65dsi84_bridge_suspend(struct device *dev)
{
	struct sn65dsi84_bridge *pdata = dev_get_drvdata(dev);

	gpiod_set_value(pdata->enable_gpio, 0);

	return 0;
}
static const struct dev_pm_ops sn65dsi84_bridge_pm_ops = {
	SET_RUNTIME_PM_OPS(sn65dsi84_bridge_suspend, sn65dsi84_bridge_resume, NULL)
};

static int status_show(struct seq_file *s, void *data)
{
	struct sn65dsi84_bridge *pdata = s->private;
	unsigned int reg, val;

	seq_puts(s, "ALL REGISTERS:\n");

	pm_runtime_get_sync(pdata->dev);

	/* list registers */
	for (reg = 0x0; reg <= 0x3E; reg++) {
		regmap_read(pdata->regmap, reg, &val);
		seq_printf(s, "[0x%02x] = 0x%02x\n", reg, val);
	}

	pm_runtime_put(pdata->dev);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(status);

static void sn65dsi84_debugfs_init(struct sn65dsi84_bridge *pdata)
{
	pdata->debugfs = debugfs_create_dir(dev_name(pdata->dev), NULL);

	debugfs_create_file("status", 0600, pdata->debugfs, pdata,
			&status_fops);
}

static void sn65dsi84_debugfs_remove(struct sn65dsi84_bridge *pdata)
{
	debugfs_remove_recursive(pdata->debugfs);
	pdata->debugfs = NULL;
}

/* Connector funcs */
static struct sn65dsi84_bridge *
connector_to_sn65dsi84_bridge(struct drm_connector *connector)
{
	return container_of(connector, struct sn65dsi84_bridge, connector);
}

static int sn65dsi84_bridge_connector_get_modes(struct drm_connector *connector)
{
	struct sn65dsi84_bridge *pdata = connector_to_sn65dsi84_bridge(connector);

	return drm_panel_get_modes(pdata->panel);
}

static enum drm_mode_status
sn65dsi84_bridge_connector_mode_valid(struct drm_connector *connector,
				  struct drm_display_mode *mode)
{
	/* max is 1600x1200 @60 */
	if (mode->clock > 170000)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static struct drm_connector_helper_funcs sn65dsi84_bridge_connector_helper_funcs = {
	.get_modes = sn65dsi84_bridge_connector_get_modes,
	.mode_valid = sn65dsi84_bridge_connector_mode_valid,
};

static enum drm_connector_status
sn65dsi84_bridge_connector_detect(struct drm_connector *connector, bool force)
{
	/**
	 * TODO: Currently if drm_panel is present, then always
	 * return the status as connected. Need to add support to detect
	 * device state for hot pluggable scenarios.
	 */
	return connector_status_connected;
}

static const struct drm_connector_funcs sn65dsi84_bridge_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = sn65dsi84_bridge_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static struct sn65dsi84_bridge *bridge_to_sn65dsi84_bridge(struct drm_bridge *bridge)
{
	return container_of(bridge, struct sn65dsi84_bridge, bridge);
}

static int sn65dsi84_bridge_attach(struct drm_bridge *bridge)
{
	int ret;
	struct sn65dsi84_bridge *pdata = bridge_to_sn65dsi84_bridge(bridge);
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	const struct mipi_dsi_device_info info = { .type = "sn65dsi84_bridge",
						   .channel = 0,
						   .node = NULL,
						 };

	ret = drm_connector_init(bridge->dev, &pdata->connector,
				 &sn65dsi84_bridge_connector_funcs,
				 DRM_MODE_CONNECTOR_LVDS);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(&pdata->connector,
				 &sn65dsi84_bridge_connector_helper_funcs);
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

	regmap_write( pdata->regmap, PLL_EN_REG, 0x0);

	/* TODO: setting to 4 lanes always for now */
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE | MIPI_DSI_MODE_VIDEO_HFP |
	                  MIPI_DSI_MODE_VIDEO_AUTO_VERT;

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

static void sn65dsi84_bridge_disable(struct drm_bridge *bridge)
{
	struct sn65dsi84_bridge *pdata = bridge_to_sn65dsi84_bridge(bridge);

	drm_panel_disable(pdata->panel);

	/* disable PLL */
	regmap_write(pdata->regmap, PLL_EN_REG, 0);

	drm_panel_unprepare(pdata->panel);
}


static void sn65dsi84_bridge_set_dsi_lvds_rate(struct sn65dsi84_bridge *pdata)
{
	unsigned int val;
	unsigned int sndiv = MAX_DSI_DIVIDER;
	unsigned int lvdsclock_khz, pixdiv, pix_freq;
	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;

	if ( ! pdata->dsi_freq ) {
		DRM_ERROR("Cannot obtain DSI frequency ( => cannot set LVDS clock divider)\n");
		return;
	}

	pixdiv = DIV_ROUND_CLOSEST(594000, mode->clock);
	pix_freq = 594000 / pixdiv;

	/* for each increment in val, frequency increases by 5MHz */
	val = pdata->dsi_freq / 5000;
	regmap_write(pdata->regmap, CHA_DSI_CLK_RANGE_REG, val & 0xFF );

	/* DSI clock divider calculation */
	while ( ( ( pdata->dsi_freq / sndiv ) < pix_freq ) && ( sndiv > 0 ) )
		sndiv--;

	if (pdata->lvds_is_two_channel)
		sndiv<<=1;

	if (( sndiv == 0 ) || (pix_freq < MIN_LVDS_CLOCK))
	{
		DRM_ERROR("Requested LVDS clock cannot be set\n");
		return;
	}

    regmap_update_bits(pdata->regmap, CLK_REG, DSI_CLK_DIVIDER_MASK, DSI_CLK_DIVIDER_SHIFT( sndiv - 1 ));

    /* calculation and setting of the LVDS clock range */
    lvdsclock_khz = pdata->dsi_freq / sndiv;

    if ( lvdsclock_khz < 37500 )
    	regmap_update_bits(pdata->regmap, PLL_REG, LVDS_CLK_RANGE_MASK, LVDS_CLK_RANGE_SHIFT(0x0));
    else if ( lvdsclock_khz < 62500 )
    	regmap_update_bits(pdata->regmap, PLL_REG, LVDS_CLK_RANGE_MASK, LVDS_CLK_RANGE_SHIFT(0x1));
    else if ( lvdsclock_khz < 87500 )
    {
    	regmap_update_bits(pdata->regmap, PLL_REG, LVDS_CLK_RANGE_MASK, LVDS_CLK_RANGE_SHIFT(0x2));
    }
    else if ( lvdsclock_khz < 112500 )
    	regmap_update_bits(pdata->regmap, PLL_REG, LVDS_CLK_RANGE_MASK, LVDS_CLK_RANGE_SHIFT(0x3));
    else if ( lvdsclock_khz < 137500 )
    	regmap_update_bits(pdata->regmap, PLL_REG, LVDS_CLK_RANGE_MASK, LVDS_CLK_RANGE_SHIFT(0x4));
    else
    	regmap_update_bits(pdata->regmap, PLL_REG, LVDS_CLK_RANGE_MASK, LVDS_CLK_RANGE_SHIFT(0x5));

}

static void sn65dsi84_bridge_set_video_timings(struct sn65dsi84_bridge *pdata)
{
	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;
	unsigned int val32;

    /* LVDS configuration setting */
	regmap_write(pdata->regmap, LVDS_CFG_REG,
                    ( ( pdata->hsync_pol ? 0 : 1 << LVDS18h_HS_NEG_POLARITY_OFFSET ) |
                      ( pdata->vsync_pol ? 0 : 1 << LVDS18h_VS_NEG_POLARITY_OFFSET ) |
                      ( pdata->de_pol ? 0 : 1 << LVDS18h_DE_NEG_POLARITY_OFFSET ) |
                      ( pdata->lvds_use_jeida ? 1 << LVDS18h_CHA_24BPP_FORMAT1_OFFSET : 0 ) |
                      ( pdata->lvds_use_jeida ? 1 << LVDS18h_CHB_24BPP_FORMAT1_OFFSET : 0 ) |
                      ( pdata->lvds_bpp_is_18 ? 0 : 1 << LVDS18h_CHA_24BPP_MODE_OFFSET ) |
                      ( pdata->lvds_bpp_is_18 ? 0 : 1 << LVDS18h_CHB_24BPP_MODE_OFFSET ) |
                      ( pdata->lvds_is_two_channel ? 0 : 1 << LVDS18h_LVDS_LINK_CFG_OFFSET )
                    )
                 );

    /* display parameters setting */
	regmap_write( pdata->regmap, CHA_ACTIVE_LINE_LENGTH_LOW_REG, mode->hdisplay & 0xFF ); /* Line Len - lower 8 bits of the 12 */
	regmap_write( pdata->regmap, CHA_ACTIVE_LINE_LENGTH_HIGH_REG, ( mode->hdisplay >> 8 ) & 0xFF ); /* Line Len -   high 4 bits of the 12 */

	regmap_write( pdata->regmap, CHA_SYNC_DELAY_LOW_REG, 0x20); /* sync delay, hardcoded to 0x20, as per the datasheet */
	regmap_write( pdata->regmap, CHA_SYNC_DELAY_HIGH_REG, 0x0);

	val32 = mode->hsync_end - mode->hsync_start;

	if ( pdata->lvds_is_two_channel )
	{
		val32 = val32 >> 1;
	}

	regmap_write( pdata->regmap, CHA_HSYNC_PULSE_WIDTH_LOW_REG, val32 & 0xFF ); /* hsync pulse width - lower 8 bits of the 12 */
	regmap_write( pdata->regmap, CHA_HSYNC_PULSE_WIDTH_HIGH_REG, ( val32 >> 8 ) & 0x7F ); /* ... high 4 bits of the 12 */

	regmap_write( pdata->regmap, CHA_VSYNC_PULSE_WIDTH_LOW_REG, ( mode->vsync_end - mode->vsync_start ) & 0xFF); /* vsync pulse width - lower 8 bits of the 12 */
	regmap_write( pdata->regmap, CHA_VSYNC_PULSE_WIDTH_HIGH_REG, ( ( mode->vsync_end - mode->vsync_start ) >> 8) & 0x7F); /* ... high 4 bits of the 12 */

	val32 = mode->htotal - mode->hsync_end;

	if ( pdata->lvds_is_two_channel )
	{
		val32 = val32 >> 1;
	}

    regmap_write( pdata->regmap, CHA_HORIZONTAL_BACK_PORCH_REG, val32 & 0xFF );
}

static void sn65dsi84_bridge_enable(struct drm_bridge *bridge)
{
	struct sn65dsi84_bridge *pdata = bridge_to_sn65dsi84_bridge(bridge);

	regmap_update_bits(pdata->regmap, CHA_DSI_CFG_REG,
			CHA_DSI_LANES_MASK, CHA_DSI_LANES_SHIFT(4 - pdata->dsi->lanes));

	/* set dsi/lvds clk frequency value */
	sn65dsi84_bridge_set_dsi_lvds_rate(pdata);

	/* config video parameters */
	sn65dsi84_bridge_set_video_timings(pdata);

	/* enable video stream */
    /* PLL enable */
	regmap_write( pdata->regmap, PLL_EN_REG, 0x1);

    /* TBD - PLL check */

    usleep_range( 15000, 15500 ); /* 15ms delay recommended by spec */

    /* Set the soft reset bit (SCR 0x09.0) */
    regmap_write( pdata->regmap, SOFT_RESET_REG, 0x1);

    usleep_range( 15000, 15500 ); /* 15ms delay recommended by spec */

    drm_panel_enable(pdata->panel);
}

static void sn65dsi84_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct sn65dsi84_bridge *pdata = bridge_to_sn65dsi84_bridge(bridge);

	pm_runtime_get_sync(pdata->dev);

	gpiod_set_value(pdata->enable_gpio, 0);

	usleep_range( 15000, 15500 );

	gpiod_set_value(pdata->enable_gpio, 1);

	usleep_range( 15000, 15500 );

	/* configure bridge ref_clk */
	regmap_update_bits(pdata->regmap, PLL_REG, HS_CLK_SRC_BIT, HS_CLK_SRC_BIT);

	drm_panel_prepare(pdata->panel);
}

static void sn65dsi84_bridge_post_disable(struct drm_bridge *bridge)
{
	struct sn65dsi84_bridge *pdata = bridge_to_sn65dsi84_bridge(bridge);

	pm_runtime_put_sync(pdata->dev);
}

static const struct drm_bridge_funcs sn65dsi84_bridge_funcs = {
	.attach = sn65dsi84_bridge_attach,
	.pre_enable = sn65dsi84_bridge_pre_enable,
	.enable = sn65dsi84_bridge_enable,
	.disable = sn65dsi84_bridge_disable,
	.post_disable = sn65dsi84_bridge_post_disable,
};

static int sn65dsi84_bridge_parse_dsi_host(struct sn65dsi84_bridge *pdata)
{
	struct device_node *np = pdata->dev->of_node;

	pdata->host_node = of_graph_get_remote_node(np, 0, 0);

	if (!pdata->host_node) {
		DRM_ERROR("remote dsi host node not found\n");
		return -ENODEV;
	}

	return 0;
}

static int sn65dsi84_bridge_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct sn65dsi84_bridge *pdata;
	int ret;
	unsigned int val;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DRM_ERROR("device doesn't support I2C\n");
		return -ENODEV;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(struct sn65dsi84_bridge),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->regmap = devm_regmap_init_i2c(client,
					     &sn65dsi84_bridge_regmap_config);
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

	if ( ! of_property_read_u32(pdata->panel->dev->of_node, "de_pol", &val) && (val == 1))
	{
		pdata->de_pol = 1;
	}
	if ( ! of_property_read_u32(pdata->panel->dev->of_node, "hsync_pol", &val) && (val == 1))
	{
		pdata->hsync_pol = 1;
	}

	if ( ! of_property_read_u32(pdata->panel->dev->of_node, "vsync_pol", &val) && (val == 1))
	{
		pdata->vsync_pol = 1;
	}

	if (of_property_read_bool(pdata->panel->dev->of_node, "lvds-use-jeida"))
			pdata->lvds_use_jeida = 1;
			
	if (of_property_read_bool(pdata->panel->dev->of_node, "lvds-is-two-channel"))
		pdata->lvds_is_two_channel = 1;
		
	if (of_property_read_bool(pdata->panel->dev->of_node, "lvds-use-18bpp"))
		pdata->lvds_bpp_is_18 = 1;

	dev_set_drvdata(&client->dev, pdata);

	pdata->enable_gpio = devm_gpiod_get(pdata->dev, "enable",
					    GPIOD_OUT_LOW);
	if (IS_ERR(pdata->enable_gpio)) {
		DRM_ERROR("failed to get enable gpio from DT\n");
		ret = PTR_ERR(pdata->enable_gpio);
		return ret;
	}

	ret = sn65dsi84_bridge_parse_dsi_host(pdata);
	if (ret)
		return ret;

	pm_runtime_enable(pdata->dev);

	i2c_set_clientdata(client, pdata);

	pdata->bridge.funcs = &sn65dsi84_bridge_funcs;
	pdata->bridge.of_node = client->dev.of_node;

	drm_bridge_add(&pdata->bridge);

	sn65dsi84_debugfs_init(pdata);

	return 0;
}

static int sn65dsi84_bridge_remove(struct i2c_client *client)
{
	struct sn65dsi84_bridge *pdata = i2c_get_clientdata(client);

	if (!pdata)
		return -EINVAL;

	sn65dsi84_debugfs_remove(pdata);

	of_node_put(pdata->host_node);

	pm_runtime_disable(pdata->dev);

	if (pdata->dsi) {
		mipi_dsi_detach(pdata->dsi);
		mipi_dsi_device_unregister(pdata->dsi);
	}

	drm_bridge_remove(&pdata->bridge);

	return 0;
}

static struct i2c_device_id sn65dsi84_bridge_id[] = {
	{ "sn65dsi84", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, sn65dsi84_bridge_id);

static const struct of_device_id sn65dsi84_bridge_match_table[] = {
	{.compatible = "sn65dsi84"},
	{},
};
MODULE_DEVICE_TABLE(of, sn65dsi84_bridge_match_table);

static struct i2c_driver sn65dsi84_bridge_driver = {
	.driver = {
		.name = "sn65dsi84_bridge",
		.of_match_table = sn65dsi84_bridge_match_table,
		.pm = &sn65dsi84_bridge_pm_ops,
	},
	.probe = sn65dsi84_bridge_probe,
	.remove = sn65dsi84_bridge_remove,
	.id_table = sn65dsi84_bridge_id,
};
module_i2c_driver(sn65dsi84_bridge_driver);

MODULE_AUTHOR("Lukas Posadka <lukas.posadka@congate.com>");
MODULE_DESCRIPTION("sn65dsi84 DSI to LVDS bridge driver");
MODULE_LICENSE("GPL v2");
