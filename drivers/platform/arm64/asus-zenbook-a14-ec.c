// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASUS Zenbook A14 UX3407QA Embedded Controller driver
 *
 * The EC exposes its mailbox at I2C address 0x5b. The mailbox uses raw
 * register and compound operations for fan and temperature readings.
 */
// allow: SIZE_OK - I2C lifecycle and hwmon callbacks share EC state.

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>

#define DRV_NAME		"asus-zenbook-a14-ec"
#define HWMON_NAME		"asus_zenbook_a14_ec"

#define EC_I2C_ADDR		0x5b

/* EC protocol opcodes */
#define EC_OP_ADDR		0x10
#define EC_OP_DATA		0x11

/* Compound-op mailboxes at major 0xc4 */
#define EC_CC_BUSY		0x30
#define EC_CC_REGSEL		0x31
#define EC_CC_DATA		0x32

#define EC_SETTLE_INTERVAL_US	50000
#define EC_SETTLE_TIMEOUT_MS	2000

/* hwmon-exposed registers */
#define EC_REG_FAN_MODE_MAJ	0x01
#define EC_REG_FAN_MODE_RMIN	0x02	/* read:  0=auto, 2=manual */
#define EC_REG_FAN_MODE_WMIN	0x82	/* write: 0=auto, 2=manual */
#define EC_REG_FAN_TACH_MAJ	0x01
#define EC_REG_FAN_TACH_MIN	0x09	/* RPM = value × 88 */
#define EC_REG_PWM_MAJ		0x01
#define EC_REG_PWM_RMIN		0x0a	/* read:  0-255 */
#define EC_REG_FAN_SEL_WMIN	0x8c	/* fan-id selector for PWM/tach */
#define EC_REG_TEMP_MAJ		0x05
#define EC_REG_TEMP_MIN		0x02	/* °C */

#define EC_NUM_FANS		2

/* Tach-to-RPM conversion: empirically RPM ≈ tach × 88. */
#define EC_TACH_RPM_MULT	88

/* Fan mode encoding observed in (0x01, 0x02). */
#define EC_FAN_MODE_AUTO	0
#define EC_FAN_MODE_MANUAL	2

struct asus_ec {
	struct device		*dev;
	struct i2c_client	*client;
	struct mutex		lock;		/* serialises EC compound access */
	struct mutex		mode_lock;

	/* DMA-safe scratch (kmalloc-backed via devm_kzalloc) */
	u8			tx[3];
	u8			rx[1];
};

/* ------------------------------------------------------------------ */
/* EC primitives — caller MUST hold ec->lock                          */
/* ------------------------------------------------------------------ */

static int __ec_rb(struct asus_ec *ec, u8 maj, u8 min, u8 *out)
{
	struct i2c_msg msgs[3];
	static const u8 op_data = EC_OP_DATA;
	int ret;

	ec->tx[0] = EC_OP_ADDR;
	ec->tx[1] = maj;
	ec->tx[2] = min;

	msgs[0].addr  = EC_I2C_ADDR;
	msgs[0].flags = 0;
	msgs[0].len   = 3;
	msgs[0].buf   = ec->tx;

	msgs[1].addr  = EC_I2C_ADDR;
	msgs[1].flags = 0;
	msgs[1].len   = 1;
	msgs[1].buf   = (u8 *)&op_data;

	msgs[2].addr  = EC_I2C_ADDR;
	msgs[2].flags = I2C_M_RD;
	msgs[2].len   = 1;
	msgs[2].buf   = ec->rx;

	ret = i2c_transfer(ec->client->adapter, msgs, 3);
	if (ret < 0) {
		dev_dbg(ec->dev, "ecrb(0x%02x,0x%02x) failed: %d\n",
			maj, min, ret);
		return ret;
	}
	if (ret != 3)
		return -EIO;

	*out = ec->rx[0];
	return 0;
}

static int __ec_wb(struct asus_ec *ec, u8 maj, u8 min, u8 val)
{
	struct i2c_msg msgs[2];
	u8 data[2];
	int ret;

	ec->tx[0] = EC_OP_ADDR;
	ec->tx[1] = maj;
	ec->tx[2] = min;

	data[0] = EC_OP_DATA;
	data[1] = val;

	msgs[0].addr  = EC_I2C_ADDR;
	msgs[0].flags = 0;
	msgs[0].len   = 3;
	msgs[0].buf   = ec->tx;

	msgs[1].addr  = EC_I2C_ADDR;
	msgs[1].flags = 0;
	msgs[1].len   = 2;
	msgs[1].buf   = data;

	ret = i2c_transfer(ec->client->adapter, msgs, 2);
	if (ret < 0) {
		dev_dbg(ec->dev, "ecwb(0x%02x,0x%02x,0x%02x) failed: %d\n",
			maj, min, val, ret);
		return ret;
	}
	if (ret != 2)
		return -EIO;
	return 0;
}

static int __ec_settle(struct asus_ec *ec)
{
	unsigned long deadline;
	u8 v;
	int ret;

	deadline = jiffies + msecs_to_jiffies(EC_SETTLE_TIMEOUT_MS);

	for (;;) {
		ret = __ec_rb(ec, 0xc4, EC_CC_BUSY, &v);
		if (ret)
			return ret;
		if (v == 0)
			return 0;
		if (time_after(jiffies, deadline)) {
			dev_warn(ec->dev,
				 "ec_settle timeout (last busy=0x%02x)\n", v);
			return -ETIMEDOUT;
		}
		usleep_range(EC_SETTLE_INTERVAL_US,
			     EC_SETTLE_INTERVAL_US + 10000);
	}
}

static int __ec_cr(struct asus_ec *ec, u8 a1, u8 a2, u8 *out)
{
	int ret;
	u8 v;

	if (a2 >= 0x80) {
		dev_err(ec->dev,
			"eccr refused: a2=0x%02x >= 0x80 destructive\n", a2);
		return -EINVAL;
	}

	ret = __ec_settle(ec);
	if (ret)
		return ret;

	ret = __ec_wb(ec, 0xc4, EC_CC_REGSEL, a2);
	if (ret)
		return ret;
	ret = __ec_wb(ec, 0xc4, EC_CC_BUSY, a1);
	if (ret)
		return ret;

	ret = __ec_settle(ec);
	if (ret)
		return ret;

	ret = __ec_rb(ec, 0xc4, EC_CC_DATA, &v);
	if (ret)
		return ret;

	ret = __ec_wb(ec, 0xc4, EC_CC_DATA, 0x00);
	if (ret)
		return ret;

	*out = v;
	return 0;
}

static int __ec_cw(struct asus_ec *ec, u8 a1, u8 a2, u8 val)
{
	int ret;

	ret = __ec_settle(ec);
	if (ret)
		return ret;

	ret = __ec_wb(ec, 0xc4, EC_CC_REGSEL, a2);
	if (ret)
		return ret;
	ret = __ec_wb(ec, 0xc4, EC_CC_DATA, val);
	if (ret)
		return ret;
	ret = __ec_wb(ec, 0xc4, EC_CC_BUSY, a1);
	if (ret)
		return ret;

	return __ec_settle(ec);
}

static int asus_ec_read_reg(struct asus_ec *ec, u8 maj, u8 min, u8 *out)
{
	int ret;

	mutex_lock(&ec->lock);
	ret = __ec_cr(ec, maj, min, out);
	mutex_unlock(&ec->lock);
	return ret;
}

static int asus_ec_write_reg(struct asus_ec *ec, u8 maj, u8 min, u8 val)
{
	int ret;

	mutex_lock(&ec->lock);
	ret = __ec_cw(ec, maj, min, val);
	mutex_unlock(&ec->lock);
	return ret;
}

static int asus_ec_set_fan_mode(struct asus_ec *ec, u8 mode)
{
	return asus_ec_write_reg(ec, EC_REG_FAN_MODE_MAJ,
				 EC_REG_FAN_MODE_WMIN, mode);
}

static int asus_ec_read_fan_reg(struct asus_ec *ec, int fan_id,
				 u8 reg_min, u8 *out)
{
	int ret;

	mutex_lock(&ec->lock);
	ret = __ec_cw(ec, EC_REG_PWM_MAJ, EC_REG_FAN_SEL_WMIN, fan_id);
	if (ret)
		goto out;
	ret = __ec_cr(ec, EC_REG_FAN_TACH_MAJ, reg_min, out);
out:
	mutex_unlock(&ec->lock);
	return ret;
}

/* Caller MUST hold ec->mode_lock. */
static int asus_ec_leave_manual(struct asus_ec *ec)
{
	int ret;

	ret = asus_ec_set_fan_mode(ec, EC_FAN_MODE_AUTO);
	if (ret) {
		dev_err(ec->dev, "failed to restore auto fan mode: %d\n", ret);
		return ret;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* hwmon callbacks                                                    */
/* ------------------------------------------------------------------ */

static umode_t asus_ec_hwmon_is_visible(const void *drvdata,
					enum hwmon_sensor_types type,
					u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		if (channel >= EC_NUM_FANS)
			return 0;
		switch (attr) {
		case hwmon_fan_input:
		case hwmon_fan_label:
			return 0444;
		default:
			return 0;
		}
	case hwmon_pwm:
		if (channel != 0)
			return 0;
		switch (attr) {
		case hwmon_pwm_input:
		case hwmon_pwm_enable:
			return 0444;
		default:
			return 0;
		}
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
		case hwmon_temp_label:
			return 0444;
		default:
			return 0;
		}
	default:
		return 0;
	}
}

static int asus_ec_hwmon_read(struct device *dev,
			      enum hwmon_sensor_types type,
			      u32 attr, int channel, long *val)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	u8 v;
	int ret;

	switch (type) {
	case hwmon_fan:
		if (attr != hwmon_fan_input || channel >= EC_NUM_FANS)
			return -EOPNOTSUPP;
		ret = asus_ec_read_fan_reg(ec, channel, EC_REG_FAN_TACH_MIN, &v);
		if (ret)
			return ret;
		*val = (long)v * EC_TACH_RPM_MULT;
		return 0;

	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			if (channel >= EC_NUM_FANS)
				return -EOPNOTSUPP;
			ret = asus_ec_read_fan_reg(ec, channel, EC_REG_PWM_RMIN, &v);
			if (ret)
				return ret;
			*val = v;
			return 0;
		case hwmon_pwm_enable:
			if (channel != 0)
				return -EOPNOTSUPP;
			ret = asus_ec_read_reg(ec, EC_REG_FAN_MODE_MAJ,
					       EC_REG_FAN_MODE_RMIN, &v);
			if (ret)
				return ret;
			if (v == EC_FAN_MODE_MANUAL)
				*val = 1;
			else if (v == EC_FAN_MODE_AUTO)
				*val = 2;
			else
				*val = 0;
			return 0;
		default:
			return -EOPNOTSUPP;
		}

	case hwmon_temp:
		if (attr != hwmon_temp_input)
			return -EOPNOTSUPP;
		ret = asus_ec_read_reg(ec, EC_REG_TEMP_MAJ, EC_REG_TEMP_MIN, &v);
		if (ret)
			return ret;
		*val = (long)v * 1000;
		return 0;

	default:
		return -EOPNOTSUPP;
	}
}

static int asus_ec_hwmon_write(struct device *dev,
			       enum hwmon_sensor_types type,
			       u32 attr, int channel, long val)
{
	return -EOPNOTSUPP;
}

static const char * const fan_labels[] = { "left", "right" };

static int asus_ec_hwmon_read_string(struct device *dev,
				     enum hwmon_sensor_types type,
				     u32 attr, int channel,
				     const char **str)
{
	switch (type) {
	case hwmon_fan:
		if (attr == hwmon_fan_label && channel < EC_NUM_FANS) {
			*str = fan_labels[channel];
			return 0;
		}
		break;
	case hwmon_temp:
		if (attr == hwmon_temp_label) {
			*str = "ec";
			return 0;
		}
		break;
	default:
		break;
	}
	return -EOPNOTSUPP;
}

static const struct hwmon_ops asus_ec_hwmon_ops = {
	.is_visible	= asus_ec_hwmon_is_visible,
	.read		= asus_ec_hwmon_read,
	.write		= asus_ec_hwmon_write,
	.read_string	= asus_ec_hwmon_read_string,
};

static const struct hwmon_channel_info * const asus_ec_hwmon_info[] = {
	HWMON_CHANNEL_INFO(fan,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL),
	HWMON_CHANNEL_INFO(pwm,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_LABEL),
	NULL
};

static const struct hwmon_chip_info asus_ec_hwmon_chip_info = {
	.ops	= &asus_ec_hwmon_ops,
	.info	= asus_ec_hwmon_info,
};

/* ------------------------------------------------------------------ */
/* probe / remove                                                     */
/* ------------------------------------------------------------------ */

static int asus_zenbook_a14_ec_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct asus_ec *ec;
	struct device *hwmon_dev;
	u8 mode;
	int ret;

	ec = devm_kzalloc(dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	ec->dev = dev;
	ec->client = client;
	mutex_init(&ec->lock);
	mutex_init(&ec->mode_lock);
	i2c_set_clientdata(client, ec);

	ret = asus_ec_read_reg(ec, EC_REG_FAN_MODE_MAJ, EC_REG_FAN_MODE_RMIN,
			       &mode);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read fan mode\n");

	if (mode == EC_FAN_MODE_MANUAL) {
		dev_warn(dev, "EC found in MANUAL mode at probe; forcing AUTO for safety\n");
		mutex_lock(&ec->mode_lock);
		ret = asus_ec_leave_manual(ec);
		mutex_unlock(&ec->mode_lock);
		if (ret)
			return dev_err_probe(dev, ret, "failed to restore automatic fan mode\n");
	}

	hwmon_dev = devm_hwmon_device_register_with_info(dev, HWMON_NAME, ec,
						  &asus_ec_hwmon_chip_info, NULL);
	if (IS_ERR(hwmon_dev)) {
		ret = PTR_ERR(hwmon_dev);
		dev_err(dev, "hwmon registration failed: %d\n", ret);
		return ret;
	}

	dev_info(dev, "hwmon registered\n");

	return 0;
}

static void asus_zenbook_a14_ec_remove(struct i2c_client *client)
{
	struct asus_ec *ec = i2c_get_clientdata(client);

	if (!ec)
		return;

	mutex_lock(&ec->mode_lock);
	(void)asus_ec_leave_manual(ec);
	mutex_unlock(&ec->mode_lock);

	dev_info(&client->dev, "removed\n");
}

static void asus_zenbook_a14_ec_shutdown(struct i2c_client *client)
{
	struct asus_ec *ec = i2c_get_clientdata(client);

	if (!ec)
		return;

	mutex_lock(&ec->mode_lock);
	(void)asus_ec_leave_manual(ec);
	mutex_unlock(&ec->mode_lock);
}

static const struct of_device_id asus_zenbook_a14_ec_of_match[] = {
	{ .compatible = "asus,zenbook-a14-ux3407qa-ec" },
	{ }
};
MODULE_DEVICE_TABLE(of, asus_zenbook_a14_ec_of_match);

static const struct i2c_device_id asus_zenbook_a14_ec_id[] = {
	{ "asus-zenbook-a14-ec", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, asus_zenbook_a14_ec_id);

static struct i2c_driver asus_zenbook_a14_ec_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = asus_zenbook_a14_ec_of_match,
	},
	.probe = asus_zenbook_a14_ec_probe,
	.remove = asus_zenbook_a14_ec_remove,
	.shutdown = asus_zenbook_a14_ec_shutdown,
	.id_table = asus_zenbook_a14_ec_id,
};
module_i2c_driver(asus_zenbook_a14_ec_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ASUS Zenbook A14 UX3407QA EC driver");
MODULE_AUTHOR("Sombre-Osmoze");
MODULE_ALIAS("i2c:asus-zenbook-a14-ec");
