/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018 Olliver Schinagl
 * Copyright (C) 2018 Ultimaker B.V.
 *
 * Olliver Schinagl <oliver@schinagl.nl>
 *
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/elf.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/remoteproc.h>
#include <linux/spi/spi.h>
#include <linux/byteorder/generic.h>

#include "remoteproc_internal.h"

#define E_MACHINE_AVR		0x53

#define AVR_PUS_RESET_PULSE	1000
#define AVR_PUS_START_WAIT	20

/* Store AVR opcodes in a single, wire-order, 32 bit integer */
#define AVR_GEN_OPCODE(b1, b2, b3, b4) \
	(htonl(((b1) << 24 | (b2) << 16 | (b3) << 8 | (b4))))

#define AVR_SYNC_CMD(sync)	((ntohl(sync) & GENMASK(23, 16)) >> 16)
#define AVR_SYNC_BYTE(sync)	((ntohl(sync) & GENMASK(15, 8)) >> 8)

/* Main instructions */
#define AVR_PROGRAMMING_ENABLE	AVR_GEN_OPCODE(0xac, 0x53, 0x00, 0x00)
#define AVR_CHIP_ERASE		AVR_GEN_OPCODE(0xac, 0x80, 0x00, 0x00)
#define AVR_POLL_RDY		AVR_GEN_OPCODE(0xf0, 0x00, 0x00, 0x00)

/* Read instructions */
#define HI 1
#define LO 0
#define AVR_READ_FLASH_BYTES(hilo, addr) \
	AVR_GEN_OPCODE(0x20 | (((hilo) < 3) & 0x8), (addr) & 0xff, ((addr) >> 8) & 0xff, 0x00)

#define _AVR_READ_FLASH_HIGH	AVR_GEN_OPCODE(0x28, 00, 00, 00)


#define AVR_READ_SIG_BYTES_CNT	3
#define _AVR_READ_SIG_BYTES	AVR_GEN_OPCODE(0x30, 0x00, 0x00, 0x00)
#define AVR_READ_SIG_BYTES(num)	(htonl(((num) % AVR_READ_SIG_BYTES_CNT) << 8) | _AVR_READ_SIG_BYTES)
#define AVR_GET_SIG_BYTES(num)	(ntohl(num) & GENMASK(7, 0))

#define AVR_BYTES_PER_WORD(bits_per_word) \
	((bits_per_word) >> 3)
#define AVR_MEMORY_SIZE(m) \
	(AVR_BYTES_PER_WORD(m.bits_per_word) * (m.page_size) * (m.page_count))

static char *avr_fw_name;
module_param(avr_fw_name, charp, S_IRUGO);
MODULE_PARM_DESC(avr_fw_name,
		 "Name of firmware file in /lib/firmware (if not specified defaults to 'avr-rproc-fw')");
/* TODO read from dts? */


struct avr_memory {
	u8 bits_per_word;
	u16 page_size;
	u16 page_count;
};

struct avr_data {
	struct avr_memory eeprom;
	struct avr_memory flash;
	u8 signature[AVR_READ_SIG_BYTES_CNT];
	u32 max_speed_hz;
};

struct avr_rproc {
	spinlock_t *spinlock;
	struct spi_device *spi;
	struct gpio_desc *power;
	struct gpio_desc *reset;
	const u8 *firmware_data;
	const struct avr_data *avr_data;
};

static int
avr_elf_load_segments(struct rproc *rproc, const struct firmware *fw)
{
	struct avr_rproc *avr_rproc = rproc->priv;
	struct device *dev = &rproc->dev;
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	struct elf32_shdr *shdr;
	int i, ret = 0;
	const u8 *elf_data = fw->data;
	const char *name_table;


	ehdr = (struct elf32_hdr *)elf_data;
	shdr = (struct elf32_shdr *)(elf_data + ehdr->e_shoff);
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);
	name_table = elf_data + shdr[ehdr->e_shstrndx].sh_offset;

	dev_err(dev, "ehdr: machine: 0x%02x entry: 0x%02x\n", ehdr->e_machine, ehdr->e_entry);
	if (ehdr->e_machine != E_MACHINE_AVR) {
		dev_err(dev, "0x%02x is not an avr8 elf firmware\n", ehdr->e_machine);
		return -EINVAL;
	}

	for (i = 0; i < ehdr->e_shnum; i++, shdr++) {
		dev_err(dev, "section name_table: %s @ 0x%02x\n", name_table + shdr->sh_name, shdr->sh_offset);
	}

	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		u32 da = phdr->p_paddr;
		u32 memsz = phdr->p_memsz;
		u32 filesz = phdr->p_filesz;
		u32 offset = phdr->p_offset;

		if (phdr->p_type != PT_LOAD)
			continue;

		dev_err(dev, "phdr: type %d da 0x%x va 0x%x memsz 0x%x filesz 0x%x offset 0x%02x\n",
			phdr->p_type, da, phdr->p_vaddr, memsz, filesz, phdr->p_offset);

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%x memsz 0x%x\n",
				filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > fw->size) {
			dev_err(dev, "truncated fw: need 0x%x avail 0x%zx\n",
				offset + filesz, fw->size);
			ret = -EINVAL;
			break;
		}

		if (phdr->p_filesz) {
			avr_rproc->firmware_data = elf_data + phdr->p_offset;
			return 0;
		}
	}

	return ret;
};

static int avr_verify_firmware(struct avr_rproc *avr_rproc)
{
	struct spi_device *spi = avr_rproc->spi;
	struct device *dev = &spi->dev;
	u32 flash_size = AVR_MEMORY_SIZE(avr_rproc->avr_data->flash);
	u8 fw_cmd[flash_size];
	u8 fw_buf[flash_size];
	struct spi_transfer xfer = {
		.tx_buf = fw_cmd,
		.rx_buf = fw_buf,
		.len = sizeof(fw_buf),
	};
	int i, j;
	int ret;

	dev_err(dev, "for: page_count: %d flash_size: %d\n", avr_rproc->avr_data->flash.page_count, flash_size);
	j = 0;
	for (i = 0;
	     i < avr_rproc->avr_data->flash.page_count;
	     i++) {
		//fw_cmd[j] = AVR_READ_FLASH_BYTES(HI, i);
		//fw_cmd[j + 1] = AVR_READ_FLASH_BYTES(LO, i);
		//dev_err(dev, "hi: 0x%08x lo: 0x%08x\n", fw_cmd[j], fw_cmd[j + 1]);
		dev_err(dev, "i: %d i*2: %d j: %d j+1; %d\n", i, i << 1, j, j + 1);
	}
	return 0;

	ret = spi_sync_transfer(spi, &xfer, 1);
	if (ret) {
		dev_err(dev, "spi transfer error\n");
		return ret;
	}

	return 0;
}

static int avr_check_signature(struct avr_rproc *avr_rproc)
{
	struct spi_device *spi = avr_rproc->spi;
	struct device *dev = &spi->dev;
	const u32 cmd[] = {
		AVR_READ_SIG_BYTES(0),
		AVR_READ_SIG_BYTES(1),
		AVR_READ_SIG_BYTES(2),
	};
	u32 signature[ARRAY_SIZE(cmd)];
	struct spi_transfer xfer[] = {
		{
			.tx_buf = &cmd[0],
			.rx_buf = &signature[0],
			.len = sizeof(cmd[0]),
		}, {
			.tx_buf = &cmd[1],
			.rx_buf = &signature[1],
			.len = sizeof(cmd[1]),
		}, {
			.tx_buf = &cmd[2],
			.rx_buf = &signature[2],
			.len = sizeof(cmd[2]),
		},
	};
	int ret;

	ret = spi_sync_transfer(spi, xfer, ARRAY_SIZE(xfer));
	if (ret)
		return ret;

	if ((avr_rproc->avr_data->signature[0] != AVR_GET_SIG_BYTES(signature[0])) ||
	    (avr_rproc->avr_data->signature[1] != AVR_GET_SIG_BYTES(signature[1])) ||
	    (avr_rproc->avr_data->signature[2] != AVR_GET_SIG_BYTES(signature[2]))) {
		dev_err(dev, "signature missmatch: 0x%02lx 0x%02lx 0x%02lx != 0x%02x 0x%02x 0x%02x\n",
			AVR_GET_SIG_BYTES(signature[0]),
			AVR_GET_SIG_BYTES(signature[1]),
			AVR_GET_SIG_BYTES(signature[2]),
			avr_rproc->avr_data->signature[0],
			avr_rproc->avr_data->signature[1],
			avr_rproc->avr_data->signature[2]);
		return -EINVAL;
	}

	return 0; //((AVR_SYNC_BYTE(avr_rproc->avr_data->signature[0]) !=
		 //AVR_SYNC_CMD(signature[0])) ||
		//(AVR_SYNC_BYTE(avr_rproc->avr_data->signature[1]) !=
		 //AVR_SYNC_CMD(signature[1])) ||
		//(AVR_SYNC_BYTE(avr_rproc->avr_data->signature[2]) !=
		 //AVR_SYNC_CMD(signature[2])));
}

static int avr_enable_serial_download(struct avr_rproc *avr_rproc)
{
	struct spi_device *spi = avr_rproc->spi;
	const u32 cmd = AVR_PROGRAMMING_ENABLE;
	u32 sync = 0xff;
	struct spi_transfer xfer[] = {
		{
			.tx_buf = &cmd,
			.rx_buf = &sync,
			.len = sizeof(cmd),
		},
	};
	int ret;

	ret = spi_sync_transfer(spi, xfer, ARRAY_SIZE(xfer));
	if (ret)
		return ret;

	return (AVR_SYNC_BYTE(sync) != AVR_SYNC_CMD(cmd));
}

static int avr_rproc_start(struct rproc *rproc)
{
	struct device *dev = &rproc->dev;
	struct avr_rproc *avr_rproc = rproc->priv;
	int ret;

	dev_err(dev, "Starting avr-rproc\n");

	/* power up sequence */
	gpiod_set_value(avr_rproc->reset, 1);
	gpiod_set_value(avr_rproc->power, 1);

	gpiod_set_value(avr_rproc->reset, 0);
	udelay(AVR_PUS_RESET_PULSE);
	gpiod_set_value(avr_rproc->reset, 1);
	msleep(AVR_PUS_START_WAIT);

	if (avr_enable_serial_download(avr_rproc)) {
		dev_err(dev, "sync-error enabling serial download mode\n");
		return -EIO;
	}

	ret = avr_check_signature(avr_rproc);
	if (ret) {
		dev_err(dev, "signature check failed\n");
		return ret;
	}

	ret = avr_verify_firmware(avr_rproc);
	if (ret) {
		dev_err(dev, "firmware verification failed\n");
		return ret;
	}

	gpiod_set_value(avr_rproc->reset, 0);

	ret = 1;
	if (ret)
		dev_err(dev, "Failed to power avr!\n");

	return 0;
}

static int avr_rproc_stop(struct rproc *rproc)
{
	struct device *dev = &rproc->dev;
	struct avr_rproc *avr_rproc = rproc->priv;
	int ret;

	dev_err(dev, "Stopping avr\n");

	gpiod_set_value(avr_rproc->reset, 1);
	gpiod_set_value(avr_rproc->power, 0);

	ret = 1;
	if (ret)
		dev_err(dev, "Failed to stop avr!\n");

	return 0;
}

static const struct rproc_ops avr_rproc_ops = {
	.start		= avr_rproc_start,
	.stop		= avr_rproc_stop,
};

static struct rproc_fw_ops avr_fw_ops = {
	.load = avr_elf_load_segments,
};

static int avr_rproc_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct avr_rproc *avr_rproc;
	struct rproc *rproc;
	int ret;
	const struct rproc_fw_ops *cloned_elf_ops;


	dev_err(dev, "avr_rproc_probe\n");

	avr_rproc = devm_kzalloc(dev, sizeof(*avr_rproc), GFP_KERNEL);
	if (!avr_rproc)
		return -ENOMEM;

	avr_rproc->avr_data = of_device_get_match_data(dev);
	if (!avr_rproc->avr_data)
		return -EINVAL;

	avr_rproc->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(avr_rproc->reset))
		return PTR_ERR(avr_rproc->reset);

	avr_rproc->power = devm_gpiod_get_optional(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(avr_rproc->power))
		return PTR_ERR(avr_rproc->power);

	dev_err(dev, "setting up spi\n");
	spi->mode = SPI_MODE_0;
	spi->max_speed_hz = 100000;
	ret = spi_setup(spi);
	if (ret)
		return ret;
	avr_rproc->spi = spi;

	dev_err(dev, "allocating rproc\n");
	rproc = rproc_alloc(dev, "avr", &avr_rproc_ops,
			    NULL, sizeof(*avr_rproc));
	if (!rproc) {
		dev_err(dev, "unable to allocate remoteproc\n");
		return -ENOMEM;
	}
	rproc->priv = avr_rproc;

	rproc->auto_boot = false;

	cloned_elf_ops = rproc->fw_ops;
	avr_fw_ops.find_rsc_table = cloned_elf_ops->find_rsc_table;
	avr_fw_ops.find_loaded_rsc_table = cloned_elf_ops->find_loaded_rsc_table;
	avr_fw_ops.sanity_check = cloned_elf_ops->sanity_check;
	avr_fw_ops.get_boot_addr = cloned_elf_ops->get_boot_addr;
	rproc->fw_ops = &avr_fw_ops;

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "rproc add failed\n");

		goto err_put_rproc;
	}
	spi_set_drvdata(spi, rproc);
	dev_err(dev, "rproc %p avr_rproc %p spi %p avr->spi %p\n", rproc, avr_rproc, spi, avr_rproc->spi);

	dev_err(dev, "init done\n");
	return 0;

err_put_rproc:
	rproc_free(rproc);

	return ret;
}

static int avr_rproc_remove(struct spi_device *spi)
{
	struct rproc *rproc = spi_get_drvdata(spi);

	if (rproc) {
		rproc_del(rproc);
		rproc_free(rproc);
	}

	return 0;
}

static struct avr_data atmega2560 = {
	.signature = { 0x1e, 0x98, 0x01 },
	.max_speed_hz = 10000,
	.flash = {
		.bits_per_word = 16,
		.page_size = 128,
		.page_count = 1024,
	},
	.eeprom = {
		.bits_per_word = 8,
		.page_size = 8,
		.page_count = 512,
	},
};

static const struct spi_device_id avr_ids[] = {
	{"atmega2560-rproc", 0},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(spi, avr_ids);

static const struct of_device_id avr_rproc_match[] = {
	{ .compatible = "at,atmega2560-rproc", .data = &atmega2560 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, avr_rproc_match);

static struct spi_driver avr_rproc_driver = {
	.probe = avr_rproc_probe,
	.remove = avr_rproc_remove,
	.id_table = avr_ids,
	.driver = {
		.name = "avr-rproc",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(avr_rproc_match),
	},
};
module_spi_driver(avr_rproc_driver);

MODULE_DESCRIPTION("Atmel AVR Remote Processor Control Driver");
MODULE_AUTHOR("Olliver Schinagl <oliver@schinagl.nl>");
MODULE_LICENSE("GPL");
