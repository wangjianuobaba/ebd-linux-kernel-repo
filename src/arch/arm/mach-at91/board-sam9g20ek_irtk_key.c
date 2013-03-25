/*
 *  ATMEL AT91 key on iRTK
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

#include <linux/gpio_keys.h>
#include <linux/input.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/gpio.h>

#include <mach/board.h>
#include <mach/cpu.h>

/* debounce interval, default 10ms */
static int debounce_interval = 10;

static struct gpio_keys_button at91_keys[] = {
	{
		
		.code		= KEY_F1,
		.gpio		= AT91_PIN_PA29,
		.active_low	= 1,
		.desc		= "F1",
		.type		= EV_KEY,
		.wakeup		= 1,
		.can_disable = 1,
	},
	{
		.code		= KEY_F2,
		.gpio		= AT91_PIN_PA28,
		.active_low	= 1,
		.desc		= "F2",
		.type		= EV_KEY,
		.wakeup		= 1,
		.can_disable = 1,
	},
	{
		.code		= KEY_POWER,
		.gpio		= AT91_PIN_PA26,
		.active_low	= 1,
		.desc		= "POWER",
		.type		= EV_KEY,
		.wakeup		= 1,
		.can_disable = 1,
	}
};

static struct gpio_keys_platform_data at91_key_data = {
	.buttons	= at91_keys,
	.nbuttons	= ARRAY_SIZE(at91_keys),
};

static void platform_dev_release(struct device * dev)
{
    return;
}

static struct platform_device at91_key_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data	= &at91_key_data,
		.release = platform_dev_release,	/* nothing to do, avoid waring */
	}
};

static int __init at91_key_init(void)
{
	struct gpio_keys_button *keys;
	int nr;
	int i;

	keys = at91_keys;
	nr = ARRAY_SIZE(at91_keys);

	for (i = 0; i < nr; i++) {
		keys[i].debounce_interval = debounce_interval;
		at91_set_gpio_input(keys[i].gpio, 1);	// input, enable pullup
		at91_set_deglitch(keys[i].gpio, 1);		// enalbe deglitch
	}
	
	return platform_device_register(&at91_key_device);
}

static void __exit at91_key_exit(void)
{
	platform_device_unregister(&at91_key_device);
}

module_init(at91_key_init);
module_exit(at91_key_exit);

module_param_named(debounce, debounce_interval, int, S_IRUSR);
MODULE_PARM_DESC(debug_level, "debounce interval");

MODULE_DESCRIPTION("AT91 iRTK key Device");
MODULE_AUTHOR("Ju Jiaqi");
MODULE_LICENSE("GPL");
