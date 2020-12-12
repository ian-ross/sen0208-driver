// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SEN0208: ultrasonic sensor for distance measuring by using GPIOs
 *
 * Copyright (c) 2020 Ian Ross <ian@skybluetrades.net>
 *           (c) 2017 Andreas Klinger <ak@it-klinger.de>
 *
 * For details about the device see:
 * https://wiki.dfrobot.com/Weather_-_proof_Ultrasonic_Sensor_with_Separate_Probe_SKU___SEN0208
 *
 * the measurement cycle as timing diagram looks like:
 *
 *          +---+
 * GPIO     |   |
 * trig:  --+   +------------------------------------------------------
 *          ^   ^
 *          |<->|
 *         udelay(trigger_pulse_us)
 *
 * ultra           +-+ +-+ +-+
 * sonic           | | | | | |
 * burst: ---------+ +-+ +-+ +-----------------------------------------
 *                           .
 * ultra                     .              +-+ +-+ +-+
 * sonic                     .              | | | | | |
 * echo:  ----------------------------------+ +-+ +-+ +----------------
 *                           .                        .
 *                           +------------------------+
 * GPIO                      |                        |
 * echo:  -------------------+                        +---------------
 *                           ^                        ^
 *                           interrupt                interrupt
 *                           (ts_rising)              (ts_falling)
 *                           |<---------------------->|
 *                              pulse time measured
 *                              --> one round trip of ultra sonic waves
 */
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

struct sen0208_cfg {
	unsigned long trigger_pulse_us;
};

struct sen0208_data {
	struct device		*dev;
	struct gpio_desc	*gpiod_trig;
	struct gpio_desc	*gpiod_echo;
	struct mutex		lock;
	int			irqnr;
	ktime_t			ts_rising;
	ktime_t			ts_falling;
	struct completion	rising;
	struct completion	falling;
	const struct sen0208_cfg	*cfg;
};

static const struct sen0208_cfg sen0208_cfg = {
	.trigger_pulse_us = 10,
};

static irqreturn_t sen0208_handle_irq(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct sen0208_data *data = iio_priv(indio_dev);
	ktime_t now = ktime_get();

	if (gpiod_get_value(data->gpiod_echo)) {
		data->ts_rising = now;
		complete(&data->rising);
	} else {
		data->ts_falling = now;
		complete(&data->falling);
	}

	return IRQ_HANDLED;
}

static int sen0208_read(struct sen0208_data *data)
{
	int ret;
	ktime_t ktime_dt;
	u64 dt_ns;

	/*
	 * just one read-echo-cycle can take place at a time
	 * ==> lock against concurrent reading calls
	 */
	mutex_lock(&data->lock);

	reinit_completion(&data->rising);
	reinit_completion(&data->falling);

	gpiod_set_value(data->gpiod_trig, 1);
	udelay(data->cfg->trigger_pulse_us);
	gpiod_set_value(data->gpiod_trig, 0);

	/* it should not take more than 20 ms until echo is rising */
	ret = wait_for_completion_killable_timeout(&data->rising, HZ/50);
	if (ret < 0) {
		mutex_unlock(&data->lock);
		return ret;
	} else if (ret == 0) {
		mutex_unlock(&data->lock);
		return -ETIMEDOUT;
	}

	/* it cannot take more than 50 ms until echo is falling */
	ret = wait_for_completion_killable_timeout(&data->falling, HZ/20);
	if (ret < 0) {
		mutex_unlock(&data->lock);
		return ret;
	} else if (ret == 0) {
		mutex_unlock(&data->lock);
		return -ETIMEDOUT;
	}

	ktime_dt = ktime_sub(data->ts_falling, data->ts_rising);

	mutex_unlock(&data->lock);

	dt_ns = ktime_to_ns(ktime_dt);
	/*
	 * measuring more than 6,45 meters is beyond the capabilities of
	 * the supported sensors
	 * ==> filter out invalid results for not measuring echos of
	 *     another us sensor
	 *
	 * formula:
	 *         distance     6,45 * 2 m
	 * time = ---------- = ------------ = 40438871 ns
	 *          speed         319 m/s
	 *
	 * using a minimum speed at -20 °C of 319 m/s
	 */
	if (dt_ns > 40438871)
		return -EIO;

  /* Return raw time of flight (divided by 2 for there-and-back). */
	return dt_ns / 2;
}

static int sen0208_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *channel, int *val,
			    int *val2, long info)
{
	struct sen0208_data *data = iio_priv(indio_dev);
	int ret;

	if (channel->type != IIO_DISTANCE)
		return -EINVAL;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = sen0208_read(data);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		/*
		 * 1 LSB is 1 ns
		 */
    /* TODO: DECIDE PROPERLY ON HOW TO DO THIS. */
		*val = 0;
		*val2 = 1;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static const struct iio_info sen0208_iio_info = {
	.read_raw		= sen0208_read_raw,
};

static const struct iio_chan_spec sen0208_chan_spec[] = {
	{
		.type = IIO_DISTANCE,
		.info_mask_separate =
				BIT(IIO_CHAN_INFO_RAW) |
				BIT(IIO_CHAN_INFO_SCALE),
	},
};

static const struct of_device_id of_sen0208_match[] = {
	{ .compatible = "dfrobot,sen0208", .data = &sen0208_cfg},
	{},
};

MODULE_DEVICE_TABLE(of, of_sen0208_match);

static int sen0208_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sen0208_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(struct sen0208_data));
	if (!indio_dev) {
		dev_err(dev, "failed to allocate IIO device\n");
		return -ENOMEM;
	}

	data = iio_priv(indio_dev);
	data->dev = dev;
	data->cfg = of_match_device(of_sen0208_match, dev)->data;

	mutex_init(&data->lock);
	init_completion(&data->rising);
	init_completion(&data->falling);

	data->gpiod_trig = devm_gpiod_get(dev, "trig", GPIOD_OUT_LOW);
	if (IS_ERR(data->gpiod_trig)) {
		dev_err(dev, "failed to get trig-gpios: err=%ld\n",
					PTR_ERR(data->gpiod_trig));
		return PTR_ERR(data->gpiod_trig);
	}

	data->gpiod_echo = devm_gpiod_get(dev, "echo", GPIOD_IN);
	if (IS_ERR(data->gpiod_echo)) {
		dev_err(dev, "failed to get echo-gpios: err=%ld\n",
					PTR_ERR(data->gpiod_echo));
		return PTR_ERR(data->gpiod_echo);
	}

	if (gpiod_cansleep(data->gpiod_echo)) {
		dev_err(data->dev, "cansleep-GPIOs not supported\n");
		return -ENODEV;
	}

	data->irqnr = gpiod_to_irq(data->gpiod_echo);
	if (data->irqnr < 0) {
		dev_err(data->dev, "gpiod_to_irq: %d\n", data->irqnr);
		return data->irqnr;
	}

	ret = devm_request_irq(dev, data->irqnr, sen0208_handle_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			pdev->name, indio_dev);
	if (ret < 0) {
		dev_err(data->dev, "request_irq: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->name = "sen0208";
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &sen0208_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = sen0208_chan_spec;
	indio_dev->num_channels = ARRAY_SIZE(sen0208_chan_spec);

	return devm_iio_device_register(dev, indio_dev);
}

static struct platform_driver sen0208_driver = {
	.probe		= sen0208_probe,
	.driver		= {
		.name		= "sen0208-gpio",
		.of_match_table	= of_sen0208_match,
	},
};

module_platform_driver(sen0208_driver);

MODULE_AUTHOR("Ian Ross <ian@skybluetrades.net>");
MODULE_DESCRIPTION("SEN0208 ultrasonic sensor for distance measuring using GPIOs");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sen0208");
