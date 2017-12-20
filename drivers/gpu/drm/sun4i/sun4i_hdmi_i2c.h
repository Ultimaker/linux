/*
 * Copyright (C) 2016 Maxime Ripard
 * Copyright (C) 2017 Chen-Yu Tsai
 * Copyirght (C) 2017 Jonathan Liu
 * Copyright (C) 2017 Olliver Schinagl
 *
 * Chen-Yu Tsai <wens@csie.org>
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 * Jonathan Liu <net147@gmail.com>
 * Olliver Schinagl <oliver@schinagl.nl>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#ifndef _SUN4I_HDMI_I2C_H_
#define _SUN4I_HDMI_I2C_H_

#define SUN4I_HDMI_DDC_CTRL_REG		0x00
#define SUN4I_HDMI_DDC_CTRL_ENABLE		BIT(31)
#define SUN4I_HDMI_DDC_CTRL_START_CMD		BIT(30)
#define SUN4I_HDMI_DDC_CTRL_FIFO_DIR_MASK	BIT(8)
#define SUN4I_HDMI_DDC_CTRL_FIFO_DIR_WRITE	(1 << 8)
#define SUN4I_HDMI_DDC_CTRL_FIFO_DIR_READ	(0 << 8)
#define SUN4I_HDMI_DDC_CTRL_RESET		BIT(0)

#define SUN4I_HDMI_DDC_ADDR_REG		0x04
#define SUN4I_HDMI_DDC_ADDR_SEGMENT(seg)	(((seg) & 0xff) << 24)
#define SUN4I_HDMI_DDC_ADDR_EDDC(addr)		(((addr) & 0xff) << 16)
#define SUN4I_HDMI_DDC_ADDR_OFFSET(off)		(((off) & 0xff) << 8)
#define SUN4I_HDMI_DDC_ADDR_SLAVE(addr)		((addr) & 0xff)

#define SUN4I_HDMI_DDC_INT_STATUS_REG		0x0c
#define SUN4I_HDMI_DDC_INT_STATUS_ILLEGAL_FIFO_OPERATION	BIT(7)
#define SUN4I_HDMI_DDC_INT_STATUS_DDC_RX_FIFO_UNDERFLOW		BIT(6)
#define SUN4I_HDMI_DDC_INT_STATUS_DDC_TX_FIFO_OVERFLOW		BIT(5)
#define SUN4I_HDMI_DDC_INT_STATUS_FIFO_REQUEST			BIT(4)
#define SUN4I_HDMI_DDC_INT_STATUS_ARBITRATION_ERROR		BIT(3)
#define SUN4I_HDMI_DDC_INT_STATUS_ACK_ERROR			BIT(2)
#define SUN4I_HDMI_DDC_INT_STATUS_BUS_ERROR			BIT(1)
#define SUN4I_HDMI_DDC_INT_STATUS_TRANSFER_COMPLETE		BIT(0)

#define SUN4I_HDMI_DDC_INT_STATUS_ERROR_MASK ( \
	SUN4I_HDMI_DDC_INT_STATUS_ILLEGAL_FIFO_OPERATION | \
	SUN4I_HDMI_DDC_INT_STATUS_DDC_RX_FIFO_UNDERFLOW | \
	SUN4I_HDMI_DDC_INT_STATUS_DDC_TX_FIFO_OVERFLOW | \
	SUN4I_HDMI_DDC_INT_STATUS_ARBITRATION_ERROR | \
	SUN4I_HDMI_DDC_INT_STATUS_ACK_ERROR | \
	SUN4I_HDMI_DDC_INT_STATUS_BUS_ERROR \
)

#define SUN4I_HDMI_DDC_FIFO_CTRL_REG	0x10
#define SUN4I_HDMI_DDC_FIFO_CTRL_CLEAR		BIT(31)
#define SUN4I_HDMI_DDC_FIFO_CTRL_RX_THRES(n)	(((n) & 0xf) << 4)
#define SUN4I_HDMI_DDC_FIFO_CTRL_RX_THRES_MASK	GENMASK(7, 4)
#define SUN4I_HDMI_DDC_FIFO_CTRL_RX_THRES_MAX	(BIT(4) - 1)
#define SUN4I_HDMI_DDC_FIFO_CTRL_TX_THRES(n)	((n) & 0xf)
#define SUN4I_HDMI_DDC_FIFO_CTRL_TX_THRES_MASK	GENMASK(3, 0)
#define SUN4I_HDMI_DDC_FIFO_CTRL_TX_THRES_MAX	(BIT(4) - 1)

#define SUN4I_HDMI_DDC_FIFO_DATA_REG	0x18

#define SUN4I_HDMI_DDC_BYTE_COUNT_REG	0x1c
#define SUN4I_HDMI_DDC_BYTE_COUNT_MAX		(BIT(10) - 1)

#define SUN4I_HDMI_DDC_CMD_REG		0x20
#define SUN4I_HDMI_DDC_CMD_EXPLICIT_EDDC_READ	6
#define SUN4I_HDMI_DDC_CMD_IMPLICIT_READ	5
#define SUN4I_HDMI_DDC_CMD_IMPLICIT_WRITE	3

#define SUN4I_HDMI_DDC_CLK_REG		0x28
#define SUN4I_HDMI_DDC_CLK_M(m)			(((m) & 0x7) << 3)
#define SUN4I_HDMI_DDC_CLK_N(n)			((n) & 0x7)

#define SUN4I_HDMI_DDC_LINE_CTRL_REG	0x40
#define SUN4I_HDMI_DDC_LINE_CTRL_SDA_ENABLE	BIT(9)
#define SUN4I_HDMI_DDC_LINE_CTRL_SCL_ENABLE	BIT(8)

#define SUN4I_HDMI_DDC_FIFO_SIZE	16

/* A31 specific */
#define SUN6I_HDMI_DDC_CTRL_REG		0x00
#define SUN6I_HDMI_DDC_CTRL_RESET		BIT(31)
#define SUN6I_HDMI_DDC_CTRL_START_CMD		BIT(27)
#define SUN6I_HDMI_DDC_CTRL_SDA_ENABLE		BIT(6)
#define SUN6I_HDMI_DDC_CTRL_SCL_ENABLE		BIT(4)
#define SUN6I_HDMI_DDC_CTRL_ENABLE		BIT(0)

#define SUN6I_HDMI_DDC_CMD_REG		0x08
#define SUN6I_HDMI_DDC_CMD_BYTE_COUNT(count)	((count) << 16)
/* command types in lower 3 bits are the same as sun4i */

#define SUN6I_HDMI_DDC_ADDR_REG		0x0c
#define SUN6I_HDMI_DDC_ADDR_SEGMENT(seg)	(((seg) & 0xff) << 24)
#define SUN6I_HDMI_DDC_ADDR_EDDC(addr)		(((addr) & 0xff) << 16)
#define SUN6I_HDMI_DDC_ADDR_OFFSET(off)		(((off) & 0xff) << 8)
#define SUN6I_HDMI_DDC_ADDR_SLAVE(addr)		(((addr) & 0xff) << 1)

#define SUN6I_HDMI_DDC_INT_STATUS_REG	0x14
#define SUN6I_HDMI_DDC_INT_STATUS_TIMEOUT	BIT(8)
/* lower 8 bits are the same as sun4i */

#define SUN6I_HDMI_DDC_FIFO_CTRL_REG	0x18
#define SUN6I_HDMI_DDC_FIFO_CTRL_CLEAR		BIT(15)
/* lower 9 bits are the same as sun4i */

#define SUN6I_HDMI_DDC_CLK_REG		0x20
/* DDC CLK bit fields are the same, but the formula is not */

#define SUN6I_HDMI_DDC_FIFO_DATA_REG	0x80

#endif
