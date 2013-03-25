/*
 *  ATMEL AT91 LED on iRTK
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/err.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/gpio.h>

#include <mach/board.h>
#include <mach/cpu.h>

static struct gpio_led at91_leds[] = {
	{	/* "user" led */
		.name				= "user",
		.gpio				= AT91_PIN_PA6,
		.active_low			= 1,
		.default_trigger	= "none",
	},
};

static struct gpio_led_platform_data at91_led_data = {
	.num_leds = ARRAY_SIZE(at91_leds),
	.leds = at91_leds,
};

static void platform_dev_release(struct device * dev)
{
    return;
}

static struct platform_device at91_leds_device = {
	.name			= "leds-gpio",
	.id			= -1,
	.dev = {
		.platform_data	= &at91_led_data,
		.release = platform_dev_release,	/* nothing to do, avoid waring */
	}
};

static int __init at91_led_init(void)
{
	struct gpio_led *leds;
	int nr;
	int i;

	leds = at91_leds;
	nr = ARRAY_SIZE(at91_leds);

	for (i = 0; i < nr; i++)
		at91_set_gpio_output(leds[i].gpio, leds[i].active_low);
	
	return platform_device_register(&at91_leds_device);
}

static void __exit at91_led_exit(void)
{
	platform_device_unregister(&at91_leds_device);
}

module_init(at91_led_init);
module_exit(at91_led_exit);

MODULE_DESCRIPTION("AT91 iRTK LED Device");
MODULE_AUTHOR("Ju Jiaqi");
MODULE_LICENSE("GPL");
