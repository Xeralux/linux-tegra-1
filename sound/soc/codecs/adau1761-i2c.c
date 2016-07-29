/*
 * Driver for ADAU1761/ADAU1461/ADAU1761/ADAU1961 codec
 *
 * Copyright 2014 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <sound/soc.h>

#include "adau1761.h"

static const struct of_device_id adau1761_of_match[] = {
	{ .compatible = "ad,adau1361", .data = (void *)ADAU1361 },
	{ .compatible = "ad,adau1461", .data = (void *)ADAU1761 },
	{ .compatible = "ad,adau1761", .data = (void *)ADAU1761 },
	{ .compatible = "ad,adau1961", .data = (void *)ADAU1361 },
	{ }
};

MODULE_DEVICE_TABLE(of, adau1761_of_match);

static int adau1761_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	enum adau17x1_type type = id->driver_data;
	struct regmap_config config;

	if (np) {
		const struct of_device_id *of_id;
		of_id = of_match_device(adau1761_of_match, &client->dev);
		if (of_id)
			type = (enum adau17x1_type) of_id->data;
	}
	config = adau1761_regmap_config;
	config.val_bits = 8;
	config.reg_bits = 16;

	dev_info(&client->dev, "probing codec type %u\n", type);

	return adau1761_probe(&client->dev,
		devm_regmap_init_i2c(client, &config),
		type, NULL);
}

static int adau1761_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct i2c_device_id adau1761_i2c_ids[] = {
	{ "adau1361", ADAU1361 },
	{ "adau1461", ADAU1761 },
	{ "adau1761", ADAU1761 },
	{ "adau1961", ADAU1361 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adau1761_i2c_ids);

static struct i2c_driver adau1761_i2c_driver = {
	.driver = {
		.name = "adau1761",
		.owner = THIS_MODULE,
		.of_match_table = adau1761_of_match,
	},
	.probe = adau1761_i2c_probe,
	.remove = adau1761_i2c_remove,
	.id_table = adau1761_i2c_ids,
};
module_i2c_driver(adau1761_i2c_driver);

MODULE_DESCRIPTION("ASoC ADAU1361/ADAU1461/ADAU1761/ADAU1961 CODEC I2C driver");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_LICENSE("GPL");
