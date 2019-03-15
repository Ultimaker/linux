/*
 * Copyright (C) 2016 MSC Technologies
 *       <www.msc-technologies.eu>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/idr.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/kdev_t.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
//#include <linux/regmap.h>
#include <linux/crc32.h>
#include <linux/of_device.h>
#include <linux/string.h>

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

static struct class boarddata_class;

#define BI_CONTENT_MAX 128


#define BI_MAJOR_VERSION    1
#define BI_MINOR_VERSION    1
#define BI_NAME_LEN         31
#define BI_VARIANT_LEN      3
#define BI_FEATURE_LEN      3
#define BI_SERIAL_LEN       10
#define BI_REVISION_LEN     2

struct s_eeprom_content_head
{
	uint8_t     magic[4];
	uint8_t     version_maj;
	uint8_t     version_min;
	uint16_t    chksum;
	uint32_t    reserved[2];
};

struct s_boarddata {
	struct i2c_client *client;

	unsigned char header_offset;
	struct s_eeprom_content_head eeprom_content_head;

	unsigned char content_offset;
	unsigned char content_length;
	unsigned char content[BI_CONTENT_MAX];

	char serial_number[32];
	char product_name[32];
	char variant_key[32];
	char feature_key[32];
	char revision[32];

	unsigned char serial_number_offset;
	unsigned char product_name_offset;
	unsigned char variant_key_offset;
	unsigned char feature_key_offset;
	unsigned char revision_offset;
	unsigned char checksum_offset;
	bool is_q7;
};


static int boarddata_entry_get(struct s_boarddata *boarddata, unsigned int offset,int len, char * buf)
{
	unsigned char ofs[2];
	int ret = 0;

	if (boarddata->is_q7) {
		ofs[0] = offset & 0xff;
		ret = i2c_master_send(boarddata->client, ofs, 1);
	} else {
		ofs[0] = offset >> 8;
		ofs[1] = offset & 0xff;
		ret = i2c_master_send(boarddata->client, ofs, 2);
	}
	//	printk("%s %s %d ret=%d\n",__FILE__,__FUNCTION__,__LINE__, ret );
	if (ret<0)
		return ret;

	ret = i2c_master_recv(boarddata->client, buf, len);
	//	printk("%s %s %d ret=%d\n",__FILE__,__FUNCTION__,__LINE__, ret );
	if (ret < 0)
		return ret;
#if 0
	{
		unsigned char n;
		for(n=0;n<len;n++)
			printk("%s %s %d 0x%02x:[%02X]\n",__FILE__,__FUNCTION__,__LINE__,n, buf[n]);
	}
#endif
	buf[len] = 0; // force string termination

	return len;
}

static int boardinfo_check_magic(struct s_eeprom_content_head *head)
{
	if (head == NULL)
		return 1;

	if (head->magic[0] != 'm' || head->magic[1] != 's' || head->magic[2] != 'c' )
		return 1;

	return 0;
}

static int boardinfo_check_version(struct s_eeprom_content_head *head)
{
	if (head == NULL)
		return 1;

	if (head->version_maj != BI_MAJOR_VERSION ||
			head->version_min > BI_MINOR_VERSION)
		return 1;

	return 0;
}

int boardinfo_calc_checksum(unsigned char * info, unsigned char length, uint16_t *chksum)
{
	int i;
	unsigned char *ptr;

	if (info == NULL)
		return 1;

	ptr = (unsigned char *)info;
	*chksum = 0;
	for (i = 0; i < length; i++)
		*chksum += ptr[i];

	return 0;
}

static int boardinfo_check_checksum(unsigned char * info, unsigned char length, uint16_t chksum)
{
	uint16_t chksum_temp;
	int ret;

	ret = boardinfo_calc_checksum(info, length, &chksum_temp);
	if (ret)
		return ret;

	if (chksum_temp != chksum)
	{
		printk("%s %s %d :  length=%d, calc=%d, given=%d \n",__FILENAME__,__FUNCTION__,__LINE__,length,chksum_temp,chksum);
		return 1;
	}

	return 0;
}

static int boardinfo_check_q7_checksum(unsigned char * info, unsigned char length, uint32_t chksum)
{
	uint32_t chksum_temp;

	chksum_temp  = ~(uint32_t)crc32(~0, (void*)info, (int)length);

	if (chksum_temp != chksum) {
		printk("%s %s %d :  length=%d, calc=0x%x, given=0x%x \n",__FILENAME__,__FUNCTION__,__LINE__,length,chksum_temp,chksum);
		return 1;
	}
	return 0;
}

static ssize_t serial_number_show(struct class *class, struct class_attribute *attr, char *buf)
{
	struct s_boarddata *boarddata = (struct s_boarddata *)class->p;
	return sprintf(buf, "%s\n", boarddata->serial_number);
}

static ssize_t product_name_show(struct class *class, struct class_attribute *attr,char *buf)
{
	struct s_boarddata *boarddata = (struct s_boarddata *)class->p;
	return sprintf(buf, "%s\n", boarddata->product_name);
}

static ssize_t variant_key_show(struct class *class, struct class_attribute *attr, char *buf)
{
	struct s_boarddata *boarddata = (struct s_boarddata *)class->p;
	return sprintf(buf, "%s\n", boarddata->variant_key);
}

static ssize_t feature_key_show(struct class *class, struct class_attribute *attr, char *buf)
{
	struct s_boarddata *boarddata = (struct s_boarddata *)class->p;
	return sprintf(buf, "%s\n", boarddata->feature_key);
}

static ssize_t revision_show(struct class *class, struct class_attribute *attr, char *buf)
{
	struct s_boarddata *boarddata = (struct s_boarddata *)class->p;
	return sprintf(buf, "%s\n", boarddata->revision);
}

static struct class_attribute boarddata_class_attrs[] = {
	__ATTR(product_name,  0444, product_name_show,  NULL),
	__ATTR(variant_key,   0444, variant_key_show,   NULL),
	__ATTR(feature_key,   0444, feature_key_show,   NULL),
	__ATTR(revision,      0444, revision_show,      NULL),
	__ATTR(serial_number, 0444, serial_number_show, NULL),
	__ATTR_NULL,
};

static struct class boarddata_class = {
	.name =	"boarddata",
	.owner =	THIS_MODULE,

	.class_attrs =	boarddata_class_attrs,
};


static int boarddata_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct s_boarddata *boarddata;
	struct device	*dev = &client->dev;
	const char	*boardtype;
	int		status;
	unsigned int	val;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
	{
		dev_err(&client->dev, "no device found!\n");
		return -ENODEV;
	}

	boarddata = devm_kzalloc(&client->dev, sizeof(struct s_boarddata), GFP_KERNEL);
	if (!boarddata) {
		dev_err(&client->dev, "out of memory!\n");
		return -ENOMEM;
	}

	boarddata->client = client;
	i2c_set_clientdata(client, boarddata);

	status = class_register(&boarddata_class);
	if (status < 0)
		return status;

	boarddata_class.p = (void*)boarddata;

	boarddata->is_q7 = false;

	if ( !of_property_read_string(dev->of_node, "board-type", &boardtype) ) {
		//printk( "boarddata: boardtype=%s\n", boardtype);
		if ( !strcmp( boardtype, "msc-Q7-imx6" ) )
			boarddata->is_q7 = true;
	}
	//printk( "boarddata: boarddata->is_q7=%d\n", boarddata->is_q7 );

	if (!boarddata->is_q7) {
		if (!of_property_read_u32(dev->of_node, "header-offset", &val)) {

			boarddata->header_offset = val;
			// read head
			if ( 0 >= boarddata_entry_get(boarddata,boarddata->header_offset,sizeof(struct s_eeprom_content_head),(char *)&boarddata->eeprom_content_head)) {
				dev_err(&client->dev, "header not readable\n");
				return (-1000);
			}

			if (boardinfo_check_magic(&boarddata->eeprom_content_head)) {
				dev_err(&client->dev, "no header found\n");
				return (-1001);
			}

			if (boardinfo_check_version(&boarddata->eeprom_content_head)) {
				dev_err(&client->dev, "boarddata version failed\n");
				return (-1002);
			}
		} else {
			dev_err(&client->dev, "header-offset must be defined in DT\n");
			return (-1010);
		}
	}

	if (!of_property_read_u32(dev->of_node, "content-offset", &val)) {
		boarddata->content_offset = val;
	} else {
		dev_err(&client->dev, "content-offset must be defined in DT\n");
		return (-1011);
	}

	if (!of_property_read_u32(dev->of_node, "content-length", &val)) {
		boarddata->content_length = val;
		if ((boarddata->content_length < 1) || (boarddata->content_length > BI_CONTENT_MAX)) {
			dev_err(&client->dev, "content-length out of range\n");
			return (-1013);
		}
	} else {
		dev_err(&client->dev, "content-length must be defined in DT\n");
		return (-1012);
	}

	// read content
	if ( 0 >= boarddata_entry_get(boarddata,boarddata->content_offset,boarddata->content_length,boarddata->content)) {
		dev_err(&client->dev, "content not readable\n");
		return (-1020);
	}

	if (boarddata->is_q7) {
		if (!of_property_read_u32(dev->of_node, "checksum-offset", &val)) {
			boarddata->checksum_offset = val;
		} else {
			dev_err(&client->dev, "checksum-offset must be defined in DT\n");
			return (-1121);
		}
		if (boardinfo_check_q7_checksum(boarddata->content, boarddata->content_length-4,
						*(int*)&(boarddata->content[boarddata->checksum_offset]))) {
			dev_err(&client->dev, "Checksum check failed. \n");
			return (-1021);
		}
	} else {
		if (boardinfo_check_checksum(boarddata->content, boarddata->content_length,
					     boarddata->eeprom_content_head.chksum)) {
			dev_err(&client->dev, "Checksum check failed. \n");
			return (-1021);
		}
	}
	if (!of_property_read_u32(dev->of_node, "serial-number-offset", &val)) {
		boarddata->serial_number_offset = val;
		if ( 0 >= boarddata_entry_get(boarddata,boarddata->serial_number_offset,BI_SERIAL_LEN,boarddata->serial_number)) {
			sprintf(boarddata->serial_number, "not defined\n");
		}
		printk("%s serial_number=%s\n",__FILENAME__,boarddata->serial_number);
	} else {
		sprintf(boarddata->serial_number, "not defined in DT\n");
	}

	if (!of_property_read_u32(dev->of_node, "product-name-offset", &val)) {
		boarddata->product_name_offset = val;
		if ( 0 >= boarddata_entry_get(boarddata,boarddata->product_name_offset,BI_NAME_LEN,boarddata->product_name)) {
			sprintf(boarddata->product_name, "not defined\n");
		}
		printk("%s product_name=%s\n",__FILENAME__,boarddata->product_name);
	} else {
		sprintf(boarddata->product_name, "not defined in DT\n");
	}

	if (!of_property_read_u32(dev->of_node, "variant-key-offset", &val)) {
		boarddata->variant_key_offset = val;
		if ( 0 >= boarddata_entry_get(boarddata,boarddata->variant_key_offset,BI_VARIANT_LEN,boarddata->variant_key)) {
			sprintf(boarddata->variant_key, "not defined\n");
		}
		printk("%s variant_key=%s\n",__FILENAME__,boarddata->variant_key);
	} else {
		sprintf(boarddata->variant_key, "not defined in DT\n");
	}

	if (!of_property_read_u32(dev->of_node, "feature-key-offset", &val)) {
		boarddata->feature_key_offset = val;
		if ( 0 >= boarddata_entry_get(boarddata,boarddata->feature_key_offset,BI_FEATURE_LEN,boarddata->feature_key)) {
			sprintf(boarddata->feature_key, "not defined\n");
		}
		printk("%s feature_key=%s\n",__FILENAME__,boarddata->feature_key);
	} else {
		sprintf(boarddata->feature_key, "not defined in DT\n");
	}

	if (!of_property_read_u32(dev->of_node, "revision-offset", &val)) {
		boarddata->revision_offset = val;
		if ( 0 >= boarddata_entry_get(boarddata,boarddata->revision_offset,BI_REVISION_LEN,boarddata->revision)) {
			sprintf(boarddata->revision, "not defined\n");
		}
		printk("%s revision=%s\n",__FILENAME__,boarddata->revision);
	} else {
		sprintf(boarddata->revision, "not defined in DT\n");
	}

	return 0;
}

#ifdef CONFIG_PM
static int boarddata_suspend(struct device *dev)
{
	return 0;
}

static int boarddata_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops boarddata_dev_pm_ops = {
	.suspend	= boarddata_suspend,
	.resume	= boarddata_resume,
};

#define BOARDDATA_DEV_PM_OPS (&boarddata_dev_pm_ops)
#else
#define	BOARDDATA_DEV_PM_OPS NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id boarddata_id[] = {
	{ "boarddata", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, boarddata_id);

static struct i2c_driver boarddata_driver = {
	.driver = {
		.name	= "boarddata",
		.pm	= BOARDDATA_DEV_PM_OPS,
	},
	.probe		= boarddata_probe,
	.id_table	= boarddata_id,
};

module_i2c_driver(boarddata_driver);

MODULE_AUTHOR("Dirk Servos <dirk.servos@msc-technologies.eu>");
MODULE_DESCRIPTION("MSC-Technologies Boarddata driver");
MODULE_LICENSE("GPL");

