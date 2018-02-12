/*
 * Copyright (C) 2016 Maxime Ripard
 *
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/component.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include "sun4i_backend.h"
#include "sun4i_crtc.h"
#include "sun4i_drv.h"
#include "sun4i_hdmi.h"
#include "sun4i_hdmi_i2c_drv.h"
#include "sun4i_tcon.h"

static inline struct sun4i_hdmi *
drm_encoder_to_sun4i_hdmi(struct drm_encoder *encoder)
{
	return container_of(encoder, struct sun4i_hdmi,
			    encoder);
}

static inline struct sun4i_hdmi *
drm_connector_to_sun4i_hdmi(struct drm_connector *connector)
{
	return container_of(connector, struct sun4i_hdmi,
			    connector);
}

static int sun4i_hdmi_setup_avi_infoframes(struct sun4i_hdmi *hdmi,
					   struct drm_display_mode *mode)
{
	struct hdmi_avi_infoframe frame;
	u8 buffer[17];
	int i, ret;

	ret = drm_hdmi_avi_infoframe_from_display_mode(&frame, mode, false);
	if (ret < 0) {
		DRM_ERROR("Failed to get infoframes from mode\n");
		return ret;
	}

	ret = hdmi_avi_infoframe_pack(&frame, buffer, sizeof(buffer));
	if (ret < 0) {
		DRM_ERROR("Failed to pack infoframes\n");
		return ret;
	}

	for (i = 0; i < sizeof(buffer); i++)
		writeb(buffer[i], hdmi->base + SUN4I_HDMI_AVI_INFOFRAME_REG(i));

	return 0;
}

static int sun4i_hdmi_atomic_check(struct drm_encoder *encoder,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *conn_state)
{
	struct drm_display_mode *mode = &crtc_state->mode;

	if (mode->flags & DRM_MODE_FLAG_DBLCLK)
		return -EINVAL;

	return 0;
}

static void sun4i_hdmi_disable(struct drm_encoder *encoder)
{
	struct sun4i_hdmi *hdmi = drm_encoder_to_sun4i_hdmi(encoder);
	u32 val;

	DRM_DEBUG_DRIVER("Disabling the HDMI Output\n");

	val = readl(hdmi->base + SUN4I_HDMI_VID_CTRL_REG);
	val &= ~SUN4I_HDMI_VID_CTRL_ENABLE;
	writel(val, hdmi->base + SUN4I_HDMI_VID_CTRL_REG);
}

static void sun4i_hdmi_enable(struct drm_encoder *encoder)
{
	struct drm_display_mode *mode = &encoder->crtc->state->adjusted_mode;
	struct sun4i_hdmi *hdmi = drm_encoder_to_sun4i_hdmi(encoder);
	u32 val = 0;

	DRM_DEBUG_DRIVER("Enabling the HDMI Output\n");

	sun4i_hdmi_setup_avi_infoframes(hdmi, mode);
	val |= SUN4I_HDMI_PKT_CTRL_TYPE(0, SUN4I_HDMI_PKT_AVI);
	val |= SUN4I_HDMI_PKT_CTRL_TYPE(1, SUN4I_HDMI_PKT_END);
	writel(val, hdmi->base + SUN4I_HDMI_PKT_CTRL_REG(0));

	val = SUN4I_HDMI_VID_CTRL_ENABLE;
	if (hdmi->hdmi_monitor)
		val |= SUN4I_HDMI_VID_CTRL_HDMI_MODE;

	writel(val, hdmi->base + SUN4I_HDMI_VID_CTRL_REG);
}

static void sun4i_hdmi_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct sun4i_hdmi *hdmi = drm_encoder_to_sun4i_hdmi(encoder);
	unsigned int x, y;
	u32 val;

	clk_set_rate(hdmi->mod_clk, mode->crtc_clock * 1000);
	clk_set_rate(hdmi->tmds_clk, mode->crtc_clock * 1000);

	/* Set input sync enable */
	writel(SUN4I_HDMI_UNKNOWN_INPUT_SYNC,
	       hdmi->base + SUN4I_HDMI_UNKNOWN_REG);

	/*
	 * Setup output pad (?) controls
	 *
	 * This is done here instead of at probe/bind time because
	 * the controller seems to toggle some of the bits on its own.
	 *
	 * We can't just initialize the register there, we need to
	 * protect the clock bits that have already been read out and
	 * cached by the clock framework.
	 */
	val = readl(hdmi->base + SUN4I_HDMI_PAD_CTRL1_REG);
	val &= SUN4I_HDMI_PAD_CTRL1_HALVE_CLK;
	val |= hdmi->variant->pad_ctrl1_init_val;
	writel(val, hdmi->base + SUN4I_HDMI_PAD_CTRL1_REG);
	val = readl(hdmi->base + SUN4I_HDMI_PAD_CTRL1_REG);

	/* Setup timing registers */
	writel(SUN4I_HDMI_VID_TIMING_X(mode->hdisplay) |
	       SUN4I_HDMI_VID_TIMING_Y(mode->vdisplay),
	       hdmi->base + SUN4I_HDMI_VID_TIMING_ACT_REG);

	x = mode->htotal - mode->hsync_start;
	y = mode->vtotal - mode->vsync_start;
	writel(SUN4I_HDMI_VID_TIMING_X(x) | SUN4I_HDMI_VID_TIMING_Y(y),
	       hdmi->base + SUN4I_HDMI_VID_TIMING_BP_REG);

	x = mode->hsync_start - mode->hdisplay;
	y = mode->vsync_start - mode->vdisplay;
	writel(SUN4I_HDMI_VID_TIMING_X(x) | SUN4I_HDMI_VID_TIMING_Y(y),
	       hdmi->base + SUN4I_HDMI_VID_TIMING_FP_REG);

	x = mode->hsync_end - mode->hsync_start;
	y = mode->vsync_end - mode->vsync_start;
	writel(SUN4I_HDMI_VID_TIMING_X(x) | SUN4I_HDMI_VID_TIMING_Y(y),
	       hdmi->base + SUN4I_HDMI_VID_TIMING_SPW_REG);

	val = SUN4I_HDMI_VID_TIMING_POL_TX_CLK;
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		val |= SUN4I_HDMI_VID_TIMING_POL_HSYNC;

	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		val |= SUN4I_HDMI_VID_TIMING_POL_VSYNC;

	writel(val, hdmi->base + SUN4I_HDMI_VID_TIMING_POL_REG);
}

static const struct drm_encoder_helper_funcs sun4i_hdmi_helper_funcs = {
	.atomic_check	= sun4i_hdmi_atomic_check,
	.disable	= sun4i_hdmi_disable,
	.enable		= sun4i_hdmi_enable,
	.mode_set	= sun4i_hdmi_mode_set,
};

static const struct drm_encoder_funcs sun4i_hdmi_funcs = {
	.destroy	= drm_encoder_cleanup,
};

static int sun4i_hdmi_get_modes(struct drm_connector *connector)
{
	struct sun4i_hdmi *hdmi = drm_connector_to_sun4i_hdmi(connector);
	struct edid *edid;
	int ret;

	edid = drm_get_edid(connector, hdmi->i2c);
	if (!edid)
		return 0;

	hdmi->hdmi_monitor = drm_detect_hdmi_monitor(edid);
	DRM_DEBUG_DRIVER("Monitor is %s monitor\n",
			 hdmi->hdmi_monitor ? "an HDMI" : "a DVI");

	drm_mode_connector_update_edid_property(connector, edid);
	cec_s_phys_addr_from_edid(hdmi->cec_adap, edid);
	ret = drm_add_edid_modes(connector, edid);
	kfree(edid);

	return ret;
}

static const struct drm_connector_helper_funcs sun4i_hdmi_connector_helper_funcs = {
	.get_modes	= sun4i_hdmi_get_modes,
};

static enum drm_connector_status
sun4i_hdmi_connector_detect(struct drm_connector *connector, bool force)
{
	struct sun4i_hdmi *hdmi = drm_connector_to_sun4i_hdmi(connector);
	unsigned long reg;

	if (readl_poll_timeout(hdmi->base + SUN4I_HDMI_HPD_REG, reg,
			       reg & SUN4I_HDMI_HPD_HIGH,
			       0, 500000)) {
		cec_phys_addr_invalidate(hdmi->cec_adap);
		return connector_status_disconnected;
	}

	return connector_status_connected;
}

static const struct drm_connector_funcs sun4i_hdmi_connector_funcs = {
	.detect			= sun4i_hdmi_connector_detect,
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.destroy		= drm_connector_cleanup,
	.reset			= drm_atomic_helper_connector_reset,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
};

#ifdef CONFIG_DRM_SUN4I_HDMI_CEC
static bool sun4i_hdmi_cec_pin_read(struct cec_adapter *adap)
{
	struct sun4i_hdmi *hdmi = cec_get_drvdata(adap);

	return readl(hdmi->base + SUN4I_HDMI_CEC) & SUN4I_HDMI_CEC_RX;
}

static void sun4i_hdmi_cec_pin_low(struct cec_adapter *adap)
{
	struct sun4i_hdmi *hdmi = cec_get_drvdata(adap);

	/* Start driving the CEC pin low */
	writel(SUN4I_HDMI_CEC_ENABLE, hdmi->base + SUN4I_HDMI_CEC);
}

static void sun4i_hdmi_cec_pin_high(struct cec_adapter *adap)
{
	struct sun4i_hdmi *hdmi = cec_get_drvdata(adap);

	/*
	 * Stop driving the CEC pin, the pull up will take over
	 * unless another CEC device is driving the pin low.
	 */
	writel(0, hdmi->base + SUN4I_HDMI_CEC);
}

static const struct cec_pin_ops sun4i_hdmi_cec_pin_ops = {
	.read = sun4i_hdmi_cec_pin_read,
	.low = sun4i_hdmi_cec_pin_low,
	.high = sun4i_hdmi_cec_pin_high,
};
#endif

#define SUN4I_HDMI_PAD_CTRL1_MASK	(GENMASK(24, 7) | GENMASK(5, 0))
#define SUN4I_HDMI_PLL_CTRL_MASK	(GENMASK(31, 8) | GENMASK(3, 0))

/* Only difference from sun5i is AMP is 4 instead of 6 */
static const struct sun4i_hdmi_variant sun4i_variant = {
	.pad_ctrl0_init_val	= SUN4I_HDMI_PAD_CTRL0_TXEN |
				  SUN4I_HDMI_PAD_CTRL0_CKEN |
				  SUN4I_HDMI_PAD_CTRL0_PWENG |
				  SUN4I_HDMI_PAD_CTRL0_PWEND |
				  SUN4I_HDMI_PAD_CTRL0_PWENC |
				  SUN4I_HDMI_PAD_CTRL0_LDODEN |
				  SUN4I_HDMI_PAD_CTRL0_LDOCEN |
				  SUN4I_HDMI_PAD_CTRL0_BIASEN,
	.pad_ctrl1_init_val	= SUN4I_HDMI_PAD_CTRL1_REG_AMP(4) |
				  SUN4I_HDMI_PAD_CTRL1_REG_EMP(2) |
				  SUN4I_HDMI_PAD_CTRL1_REG_DENCK |
				  SUN4I_HDMI_PAD_CTRL1_REG_DEN |
				  SUN4I_HDMI_PAD_CTRL1_EMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_EMP_OPT |
				  SUN4I_HDMI_PAD_CTRL1_AMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_AMP_OPT,
	.pll_ctrl_init_val	= SUN4I_HDMI_PLL_CTRL_VCO_S(8) |
				  SUN4I_HDMI_PLL_CTRL_CS(7) |
				  SUN4I_HDMI_PLL_CTRL_CP_S(15) |
				  SUN4I_HDMI_PLL_CTRL_S(7) |
				  SUN4I_HDMI_PLL_CTRL_VCO_GAIN(4) |
				  SUN4I_HDMI_PLL_CTRL_SDIV2 |
				  SUN4I_HDMI_PLL_CTRL_LDO2_EN |
				  SUN4I_HDMI_PLL_CTRL_LDO1_EN |
				  SUN4I_HDMI_PLL_CTRL_HV_IS_33 |
				  SUN4I_HDMI_PLL_CTRL_BWS |
				  SUN4I_HDMI_PLL_CTRL_PLL_EN,
};

static const struct sun4i_hdmi_variant sun5i_variant = {
	.pad_ctrl0_init_val	= SUN4I_HDMI_PAD_CTRL0_TXEN |
				  SUN4I_HDMI_PAD_CTRL0_CKEN |
				  SUN4I_HDMI_PAD_CTRL0_PWENG |
				  SUN4I_HDMI_PAD_CTRL0_PWEND |
				  SUN4I_HDMI_PAD_CTRL0_PWENC |
				  SUN4I_HDMI_PAD_CTRL0_LDODEN |
				  SUN4I_HDMI_PAD_CTRL0_LDOCEN |
				  SUN4I_HDMI_PAD_CTRL0_BIASEN,
	.pad_ctrl1_init_val	= SUN4I_HDMI_PAD_CTRL1_REG_AMP(6) |
				  SUN4I_HDMI_PAD_CTRL1_REG_EMP(2) |
				  SUN4I_HDMI_PAD_CTRL1_REG_DENCK |
				  SUN4I_HDMI_PAD_CTRL1_REG_DEN |
				  SUN4I_HDMI_PAD_CTRL1_EMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_EMP_OPT |
				  SUN4I_HDMI_PAD_CTRL1_AMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_AMP_OPT,
	.pll_ctrl_init_val	= SUN4I_HDMI_PLL_CTRL_VCO_S(8) |
				  SUN4I_HDMI_PLL_CTRL_CS(7) |
				  SUN4I_HDMI_PLL_CTRL_CP_S(15) |
				  SUN4I_HDMI_PLL_CTRL_S(7) |
				  SUN4I_HDMI_PLL_CTRL_VCO_GAIN(4) |
				  SUN4I_HDMI_PLL_CTRL_SDIV2 |
				  SUN4I_HDMI_PLL_CTRL_LDO2_EN |
				  SUN4I_HDMI_PLL_CTRL_LDO1_EN |
				  SUN4I_HDMI_PLL_CTRL_HV_IS_33 |
				  SUN4I_HDMI_PLL_CTRL_BWS |
				  SUN4I_HDMI_PLL_CTRL_PLL_EN,
};

static const struct sun4i_hdmi_variant sun6i_variant = {
	.has_reset_control	= true,
	.pad_ctrl0_init_val	= 0xff |
				  SUN4I_HDMI_PAD_CTRL0_TXEN |
				  SUN4I_HDMI_PAD_CTRL0_CKEN |
				  SUN4I_HDMI_PAD_CTRL0_PWENG |
				  SUN4I_HDMI_PAD_CTRL0_PWEND |
				  SUN4I_HDMI_PAD_CTRL0_PWENC |
				  SUN4I_HDMI_PAD_CTRL0_LDODEN |
				  SUN4I_HDMI_PAD_CTRL0_LDOCEN,
	.pad_ctrl1_init_val	= SUN4I_HDMI_PAD_CTRL1_REG_AMP(6) |
				  SUN4I_HDMI_PAD_CTRL1_REG_EMP(4) |
				  SUN4I_HDMI_PAD_CTRL1_REG_DENCK |
				  SUN4I_HDMI_PAD_CTRL1_REG_DEN |
				  SUN4I_HDMI_PAD_CTRL1_EMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_EMP_OPT |
				  SUN4I_HDMI_PAD_CTRL1_PWSDT |
				  SUN4I_HDMI_PAD_CTRL1_PWSCK |
				  SUN4I_HDMI_PAD_CTRL1_AMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_AMP_OPT |
				  SUN4I_HDMI_PAD_CTRL1_UNKNOWN,
	.pll_ctrl_init_val	= SUN4I_HDMI_PLL_CTRL_VCO_S(8) |
				  SUN4I_HDMI_PLL_CTRL_CS(3) |
				  SUN4I_HDMI_PLL_CTRL_CP_S(10) |
				  SUN4I_HDMI_PLL_CTRL_S(4) |
				  SUN4I_HDMI_PLL_CTRL_VCO_GAIN(4) |
				  SUN4I_HDMI_PLL_CTRL_SDIV2 |
				  SUN4I_HDMI_PLL_CTRL_LDO2_EN |
				  SUN4I_HDMI_PLL_CTRL_LDO1_EN |
				  SUN4I_HDMI_PLL_CTRL_HV_IS_33 |
				  SUN4I_HDMI_PLL_CTRL_PLL_EN,

	.tmds_clk_div_offset	= 1,
};

static int sun4i_hdmi_bind(struct device *dev, struct device *master,
			   void *data)
{
	struct device_node *i2c_np;
	struct drm_device *drm = data;
	struct sun4i_drv *drv = drm->dev_private;
	struct sun4i_hdmi *hdmi;
	u32 reg;
	int ret;

	hdmi = dev_get_drvdata(dev);
	if (!hdmi) {
		pr_err("hdmi tmds clk not initialized\n");
		return -ENODEV;
	}

	hdmi->drv = drv;

	clk_prepare_enable(hdmi->bus_clk);
	clk_prepare_enable(hdmi->mod_clk);

	writel(SUN4I_HDMI_CTRL_ENABLE, hdmi->base + SUN4I_HDMI_CTRL_REG);

	writel(hdmi->variant->pad_ctrl0_init_val,
	       hdmi->base + SUN4I_HDMI_PAD_CTRL0_REG);

	reg = readl(hdmi->base + SUN4I_HDMI_PLL_CTRL_REG);
	reg &= SUN4I_HDMI_PLL_CTRL_DIV_MASK;
	reg |= hdmi->variant->pll_ctrl_init_val;
	writel(reg, hdmi->base + SUN4I_HDMI_PLL_CTRL_REG);

	i2c_np = of_parse_phandle(dev->of_node, "ddc-i2c-bus", 0);
	if (!i2c_np) {
		dev_warn(dev, "Missing ddc-i2c-bus node\n");

		/* legacy devicetree's do not have the hdmi-i2c node */
		hdmi->i2c_drv = sun4i_hdmi_i2c_setup(dev, hdmi->base);
		if (IS_ERR(hdmi->i2c_drv)) {
			if (PTR_ERR(hdmi->i2c_drv) != -EPROBE_DEFER)
				dev_err(dev, "Couldn't setup HDMI I2C driver\n");
			ret = PTR_ERR(hdmi->i2c_drv);
			goto err_disable_clks;
		}

		hdmi->i2c = &hdmi->i2c_drv->adap;
	} else {
		hdmi->i2c = of_find_i2c_adapter_by_node(i2c_np);
	}
	if (!hdmi->i2c) {
		dev_err(dev, "Couldn't create the HDMI I2C adapter\n");
		ret = -ENODEV;
		goto err_i2c_adap;
	}

	drm_encoder_helper_add(&hdmi->encoder,
			       &sun4i_hdmi_helper_funcs);
	ret = drm_encoder_init(drm,
			       &hdmi->encoder,
			       &sun4i_hdmi_funcs,
			       DRM_MODE_ENCODER_TMDS,
			       NULL);
	if (ret) {
		dev_err(dev, "Couldn't initialise the HDMI encoder\n");
		ret = -ENODEV;
		goto err_i2c_adap;
	}

	hdmi->encoder.possible_crtcs = drm_of_find_possible_crtcs(drm,
								  dev->of_node);
	if (!hdmi->encoder.possible_crtcs) {
		ret = -EPROBE_DEFER;
		goto err_i2c_adap;
	}

#ifdef CONFIG_DRM_SUN4I_HDMI_CEC
	hdmi->cec_adap = cec_pin_allocate_adapter(&sun4i_hdmi_cec_pin_ops,
		hdmi, "sun4i", CEC_CAP_TRANSMIT | CEC_CAP_LOG_ADDRS |
		CEC_CAP_PASSTHROUGH | CEC_CAP_RC);
	ret = PTR_ERR_OR_ZERO(hdmi->cec_adap);
	if (ret < 0)
		goto err_cleanup_connector;
	writel(readl(hdmi->base + SUN4I_HDMI_CEC) & ~SUN4I_HDMI_CEC_TX,
	       hdmi->base + SUN4I_HDMI_CEC);
#endif

	drm_connector_helper_add(&hdmi->connector,
				 &sun4i_hdmi_connector_helper_funcs);
	ret = drm_connector_init(drm, &hdmi->connector,
				 &sun4i_hdmi_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);
	if (ret) {
		dev_err(dev,
			"Couldn't initialise the HDMI connector\n");
		goto err_cleanup_connector;
	}

	/* There is no HPD interrupt, so we need to poll the controller */
	hdmi->connector.polled = DRM_CONNECTOR_POLL_CONNECT |
		DRM_CONNECTOR_POLL_DISCONNECT;

	ret = cec_register_adapter(hdmi->cec_adap, dev);
	if (ret < 0)
		goto err_cleanup_connector;
	drm_mode_connector_attach_encoder(&hdmi->connector, &hdmi->encoder);

	return 0;

err_cleanup_connector:
	cec_delete_adapter(hdmi->cec_adap);
	drm_encoder_cleanup(&hdmi->encoder);
err_i2c_adap:
	if (!hdmi->i2c_drv)
		put_device(&hdmi->i2c->dev);
err_disable_clks:
	clk_disable_unprepare(hdmi->mod_clk);
	clk_disable_unprepare(hdmi->bus_clk);
	return ret;
}

static void sun4i_hdmi_unbind(struct device *dev, struct device *master,
			    void *data)
{
	struct sun4i_hdmi *hdmi = dev_get_drvdata(dev);

	if (!hdmi->i2c_drv)
		put_device(&hdmi->i2c->dev);
	sun4i_hdmi_i2c_fini(hdmi->i2c_drv);
	cec_unregister_adapter(hdmi->cec_adap);
	drm_connector_cleanup(&hdmi->connector);
	drm_encoder_cleanup(&hdmi->encoder);
	clk_disable_unprepare(hdmi->mod_clk);
	clk_disable_unprepare(hdmi->bus_clk);
}

static const struct component_ops sun4i_hdmi_ops = {
	.bind	= sun4i_hdmi_bind,
	.unbind	= sun4i_hdmi_unbind,
};

struct clk_core {
	const char		*name;
	const struct clk_ops	*ops;
	struct clk_hw		*hw;
	struct module		*owner;
	struct device		*dev;
	struct clk_core		*parent;
	const char		**parent_names;
	struct clk_core		**parents;
	u8			num_parents;
	u8			new_parent_index;
	unsigned long		rate;
	unsigned long		req_rate;
	unsigned long		new_rate;
	struct clk_core		*new_parent;
	struct clk_core		*new_child;
	unsigned long		flags;
	bool			orphan;
	unsigned int		enable_count;
	unsigned int		prepare_count;
	unsigned long		min_rate;
	unsigned long		max_rate;
	unsigned long		accuracy;
	int			phase;
	struct hlist_head	children;
	struct hlist_node	child_node;
	struct hlist_head	clks;
	unsigned int		notifier_count;
#ifdef CONFIG_DEBUG_FS
	struct dentry		*dentry;
	struct hlist_node	debug_node;
#endif
	struct kref		ref;
};
struct clk {
	struct clk_core	*core;
	const char *dev_id;
	const char *con_id;
	unsigned long min_rate;
	unsigned long max_rate;
	struct hlist_node clks_node;
};

static int sun4i_hdmi_tmds_clk_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev_of_node(dev);
	struct resource *res;
	struct sun4i_hdmi *hdmi;
	int ret;

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	hdmi->dev = dev;

	hdmi->variant = of_device_get_match_data(dev);
	if (!hdmi->variant) {
		dev_err(dev, "hdmi_tmds_clk: couldn't find matching device\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hdmi->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(hdmi->base)) {
		dev_err(dev, "couldn't map the HDMI encoder registers\n");
		return PTR_ERR(hdmi->base);
	}

	if (hdmi->variant->has_reset_control) {
		hdmi->reset = devm_reset_control_get(dev, NULL);
		if (IS_ERR(hdmi->reset)) {
			dev_err(dev, "couldn't get the HDMI reset control\n");
			return PTR_ERR(hdmi->reset);
		}

		ret = reset_control_deassert(hdmi->reset);
		if (ret) {
			dev_err(dev, "couldn't deassert HDMI reset\n");
			goto err_assert_reset;
		}
	}

	hdmi->bus_clk = devm_clk_get(dev, "ahb");
	if (IS_ERR(hdmi->bus_clk)) {
		dev_err(dev, "couldn't get the HDMI bus clock\n");
		ret = PTR_ERR(hdmi->bus_clk);
		goto err_assert_reset;
	}
	clk_prepare_enable(hdmi->bus_clk);

	hdmi->mod_clk = of_clk_get_by_name(node, "mod");
	if (IS_ERR(hdmi->mod_clk)) {
		dev_err(dev, "couldn't get the HDMI mod clock\n");
		ret = PTR_ERR(hdmi->mod_clk);
		goto err_disable_bus_clk;
	}
	clk_prepare_enable(hdmi->mod_clk);

	hdmi->pll0_clk = devm_clk_get(dev, "pll-0");
	if (IS_ERR(hdmi->pll0_clk)) {
		pr_err("couldn't get the HDMI PLL 0 clock\n");
		ret = PTR_ERR(hdmi->pll0_clk);
		goto err_disable_mod_clk;
	}

	hdmi->pll1_clk = of_clk_get_by_name(node, "pll-1");
	if (IS_ERR(hdmi->pll1_clk)) {
		dev_err(dev, "couldn't get the HDMI PLL 1 clock\n");
		ret = PTR_ERR(hdmi->pll1_clk);
		goto err_disable_mod_clk;
	}

	ret = of_property_read_string(node, "clock-output-names",
				      &hdmi->tmds_clk_name);
	if (ret) {
		/* Deal with old/incomplete DTs */
		hdmi->tmds_clk_name = "hdmi-tmds";
		dev_warn(dev, "no 'clock-output-names', falling back to: %s\n",
			 hdmi->tmds_clk_name);
	}

	ret = sun4i_tmds_create(hdmi);
	if (ret) {
		dev_err(dev, "couldn't create the TMDS clock\n");
		goto err_disable_mod_clk;
	}
	ret = of_clk_add_provider(node, of_clk_src_simple_get,
				  hdmi->tmds_clk);
	// TODO devm_of_clk_add_provider()
	if (ret) {
		dev_err(dev, "couldn't register the TMDS clock\n");
		goto err_disable_mod_clk;
	}

	dev_set_drvdata(dev, hdmi);

	return ret;

err_disable_mod_clk:
	clk_disable_unprepare(hdmi->mod_clk);
err_disable_bus_clk:
	clk_disable_unprepare(hdmi->bus_clk);
err_assert_reset:
	reset_control_assert(hdmi->reset);

	return ret;
}

static int sun4i_hdmi_probe(struct platform_device *pdev)
{
	int ret;

	ret = sun4i_hdmi_tmds_clk_init(pdev);
	if (ret)
		return ret;

	return component_add(&pdev->dev, &sun4i_hdmi_ops);
}

static int sun4i_hdmi_remove(struct platform_device *pdev)
{
	struct device_node *node = dev_of_node(&pdev->dev);

	component_del(&pdev->dev, &sun4i_hdmi_ops);
	of_clk_del_provider(node);

	return 0;
}

static const struct of_device_id sun4i_hdmi_of_table[] = {
	{ .compatible = "allwinner,sun4i-a10-hdmi", .data = &sun4i_variant, },
	{ .compatible = "allwinner,sun5i-a10s-hdmi", .data = &sun5i_variant, },
	{ .compatible = "allwinner,sun6i-a31-hdmi", .data = &sun6i_variant, },
	{ .compatible = "allwinner,sun7i-a20-hdmi", .data = &sun5i_variant, },
	{ }
};
MODULE_DEVICE_TABLE(of, sun4i_hdmi_of_table);

static struct platform_driver sun4i_hdmi_driver = {
	.probe		= sun4i_hdmi_probe,
	.remove		= sun4i_hdmi_remove,
	.driver		= {
		.name		= "sun4i-hdmi",
		.of_match_table	= sun4i_hdmi_of_table,
	},
};
module_platform_driver(sun4i_hdmi_driver);

MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_DESCRIPTION("Allwinner A10 HDMI Driver");
MODULE_LICENSE("GPL");
