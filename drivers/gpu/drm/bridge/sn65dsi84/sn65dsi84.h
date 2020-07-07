/*
 * TI SN65DSI84 LVDS transmitter driver
 *
 * Copyright 2018 Marco Sandrelli Seco s.r.l.
 *
 * Licensed under the GPL-2.
 */

#ifndef __DRM_I2C_SN65DSI84_H__
#define __DRM_I2C_SN65DSI84_H__

#include <linux/hdmi.h>
#include <linux/i2c.h>
#include <linux/regmap.h>

#include <drm/drm_crtc_helper.h>
#include <drm/drm_mipi_dsi.h>

#define SN65DSI84_REG_CHIP_REVISION			0x00
#define SN65DSI84_REG_RESET				0x09
#define SN65DSI84_REG_LVDS_CLK				0x0a
#define SN65DSI84_REG_CLK_DIV_MUL			0x0b
#define SN65DSI84_REG_I2S_CONFG				0x0c
#define SN65DSI84_REG_PLL_EN				0x0d
#define SN65DSI84_REG_DSI_LANES				0x10
#define SN65DSI84_REG_DSI_EQ				0x11
#define SN65DSI84_REG_DSI_CLK_RANGE			0x12
#define SN65DSI84_REG_LVDS_PARAMETER			0x18
#define SN65DSI84_REG_LVDS_STRENGTH			0x19
#define SN65DSI84_REG_LVDS_SWAP				0x1a
#define SN65DSI84_REG_LVDS_COMMON_MODE			0x1b
#define SN65DSI84_REG_CHA_ACTIVE_LINE_LENGTH_LOW	0x20
#define SN65DSI84_REG_CHA_ACTIVE_LINE_LENGTH_HIGH	0x21
#define SN65DSI84_REG_CHA_VERTICAL_DISPLAY_SIZE_LOW	0x24
#define SN65DSI84_REG_CHA_VERTICAL_DISPLAY_SIZE_HIGH	0x25
#define SN65DSI84_REG_CHA_SYNC_DELAY_LOW		0x28
#define SN65DSI84_REG_CHA_SYNC_DELAY_HIGH		0x29
#define SN65DSI84_REG_CHA_HSYNC_PULSE_WIDTH_LOW		0x2c
#define SN65DSI84_REG_CHA_HSYNC_PULSE_WIDTH_HIGH	0x2d
#define SN65DSI84_REG_CHA_VSYNC_PULSE_WIDTH_LOW		0x30
#define SN65DSI84_REG_CHA_VSYNC_PULSE_WIDTH_HIGH	0x31
#define SN65DSI84_REG_CHA_HORIZONTAL_BACK_PORCH		0x34
#define SN65DSI84_REG_CHA_VERTICAL_BACK_PORCH		0x36
#define SN65DSI84_REG_CHA_HORIZONTAL_FRONT_PORCH	0x38
#define SN65DSI84_REG_CHA_VERTICAL_FRONT_PORCH		0x3a
/* Test Pattern */
#define SN65DSI84_REG_CHA_TEST_PATTERN			0x3c
/* Enable */
#define SN65DSI84_REG_IRQ_EN				0xe0
#define SN65DSI84_REG_ERR_EN                            0xe1
#define SN65DSI84_REG_DSI_PROTOCOL_ERR			0xe5


enum sn65dsi84_input_clock {
	SN65DSI84_INPUT_CLOCK_1X,
	SN65DSI84_INPUT_CLOCK_2X,
	SN65DSI84_INPUT_CLOCK_DDR,
};

enum sn65dsi84_input_justification {
	SN65DSI84_INPUT_JUSTIFICATION_EVENLY = 0,
	SN65DSI84_INPUT_JUSTIFICATION_RIGHT = 1,
	SN65DSI84_INPUT_JUSTIFICATION_LEFT = 2,
};

enum sn65dsi84_input_sync_pulse {
	SN65DSI84_INPUT_SYNC_PULSE_DE = 0,
	SN65DSI84_INPUT_SYNC_PULSE_HSYNC = 1,
	SN65DSI84_INPUT_SYNC_PULSE_VSYNC = 2,
	SN65DSI84_INPUT_SYNC_PULSE_NONE = 3,
};

/**
 * enum sn65dsi84_sync_polarity - Polarity for the input sync signals
 * @SN65DSI84_SYNC_POLARITY_PASSTHROUGH:  Sync polarity matches that of
 *				       the currently configured mode.
 * @SN65DSI84_SYNC_POLARITY_LOW:	    Sync polarity is low
 * @SN65DSI84_SYNC_POLARITY_HIGH:	    Sync polarity is high
 *
 * If the polarity is set to either LOW or HIGH the driver will configure the
 * SN65DSI84 to internally invert the sync signal if required to match the sync
 * polarity setting for the currently selected output mode.
 *
 * If the polarity is set to PASSTHROUGH, the SN65DSI84 will route the signal
 * unchanged. This is used when the upstream graphics core already generates
 * the sync signals with the correct polarity.
 */
enum sn65dsi84_sync_polarity {
	SN65DSI84_SYNC_POLARITY_PASSTHROUGH,
	SN65DSI84_SYNC_POLARITY_LOW,
	SN65DSI84_SYNC_POLARITY_HIGH,
};

/**
 * struct sn65dsi84_link_config - Describes sn65dsi84 hardware configuration
 * @input_color_depth:		Number of bits per color component (8, 10 or 12)
 * @input_colorspace:		The input colorspace (RGB, YUV444, YUV422)
 * @input_clock:		The input video clock style (1x, 2x, DDR)
 * @input_style:		The input component arrangement variant
 * @input_justification:	Video input format bit justification
 * @clock_delay:		Clock delay for the input clock (in ps)
 * @embedded_sync:		Video input uses BT.656-style embedded sync
 * @sync_pulse:			Select the sync pulse
 * @vsync_polarity:		vsync input signal configuration
 * @hsync_polarity:		hsync input signal configuration
 */
struct sn65dsi84_link_config {
	unsigned int input_color_depth;
	enum hdmi_colorspace input_colorspace;
	enum sn65dsi84_input_clock input_clock;
	unsigned int input_style;
	enum sn65dsi84_input_justification input_justification;

	int clock_delay;

	bool embedded_sync;
	enum sn65dsi84_input_sync_pulse sync_pulse;
	enum sn65dsi84_sync_polarity vsync_polarity;
	enum sn65dsi84_sync_polarity hsync_polarity;
};

/**
 * enum sn65dsi84_csc_scaling - Scaling factor for the SN65DSI84 CSC
 * @SN65DSI84_CSC_SCALING_1: CSC results are not scaled
 * @SN65DSI84_CSC_SCALING_2: CSC results are scaled by a factor of two
 * @SN65DSI84_CSC_SCALING_4: CSC results are scalled by a factor of four
 */
enum sn65dsi84_csc_scaling {
	SN65DSI84_CSC_SCALING_1 = 0,
	SN65DSI84_CSC_SCALING_2 = 1,
	SN65DSI84_CSC_SCALING_4 = 2,
};

/**
 * struct sn65dsi84_video_config - Describes sn65dsi84 hardware configuration
 * @csc_enable:			Whether to enable color space conversion
 * @csc_scaling_factor:		Color space conversion scaling factor
 * @csc_coefficents:		Color space conversion coefficents
 * @hdmi_mode:			Whether to use HDMI or DVI output mode
 * @avi_infoframe:		HDMI infoframe
 */
struct sn65dsi84_video_config {
	bool csc_enable;
	enum sn65dsi84_csc_scaling csc_scaling_factor;
	const uint16_t *csc_coefficents;

	bool hdmi_mode;
	struct hdmi_avi_infoframe avi_infoframe;
};

enum sn65dsi84_type {
	SN65DSI84,
};

struct sn65dsi84 {
	struct i2c_client *i2c_main;
	struct i2c_client *i2c_edid;
	struct i2c_client *i2c_cec;

	struct regmap *regmap;
	struct regmap *regmap_cec;
	enum drm_connector_status status;
	bool powered;

	struct drm_display_mode curr_mode;
	
	bool mode_valid;
	struct videomode *lvds_timing;
	const char *lvds_color_depth;
	const char *lvds_datamap;
	
	bool lvds_dual_channel;
	bool lvds_channel_swap;
	bool lvds_channel_reverse;
	bool lvds_test_mode;
	
	unsigned int f_tmds;

	unsigned int current_edid_segment;
	uint8_t edid_buf[256];
	bool edid_read;

	wait_queue_head_t wq;
	struct work_struct hpd_work;

	struct drm_bridge bridge;
	struct drm_connector connector;

	bool embedded_sync;
	enum sn65dsi84_sync_polarity vsync_polarity;
	enum sn65dsi84_sync_polarity hsync_polarity;
	bool rgb;

	struct edid *edid;

	int		gpio_pd;

	/* TODO ROPA: added for test, remove after back light pwm driver added  */
	int		gpio_bckl;

	/* ADV7533 DSI RX related params */
	struct device_node *host_node;
	struct mipi_dsi_device *dsi;
	u8 num_dsi_lanes;
	u32 dsi_clock; /*TODO ROPA: added */
	bool use_timing_gen;

	enum sn65dsi84_type type;
};


#endif /* __DRM_I2C_SN65DSI84_H__ */
