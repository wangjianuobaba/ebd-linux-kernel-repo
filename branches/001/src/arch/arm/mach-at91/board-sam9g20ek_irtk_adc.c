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
#include <asm/gpio.h>

#include <mach/board.h>
#include <mach/cpu.h>
#include <linux/platform_data/at91_adc.h>

static struct at91_adc_trigger at91_adc_triggers[] = {
	[0] = {
		.name = "timer-counter-0",
		.value = AT91_ADC_TRGSEL_TC0 | AT91_ADC_TRGEN,
	},
	[1] = {
		.name = "timer-counter-1",
		.value = AT91_ADC_TRGSEL_TC1 | AT91_ADC_TRGEN,
	},
	[2] = {
		.name = "timer-counter-2",
		.value = AT91_ADC_TRGSEL_TC2 | AT91_ADC_TRGEN,
	},
	[3] = {
		.name = "external",
		.value = AT91_ADC_TRGSEL_EXTERNAL | AT91_ADC_TRGEN,
		.is_external = true,
	},
};

static struct at91_adc_reg_desc at91_adc_register_g20 = {
	.channel_base = AT91_ADC_CHR(0),
	.drdy_mask = AT91_ADC_DRDY,
	.status_register = AT91_ADC_SR,
	.trigger_register = AT91_ADC_MR,
};

static struct at91_adc_data ek_at91_adc_data = {
	.channels_used = BIT(0) | BIT(2) | BIT(3),
	.num_channels = 3,
	.registers = &at91_adc_register_g20,
	.startup_time = 10,
	.trigger_list = at91_adc_triggers,
	.trigger_number = 4,
	.use_external_triggers = false,
	.vref = 3300,
};

static struct resource adc_resources[] = {
	[0] = {
		.start	= AT91SAM9260_BASE_ADC,
		.end	= AT91SAM9260_BASE_ADC + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AT91SAM9260_ID_ADC,
		.end	= AT91SAM9260_ID_ADC,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device at91_adc_device = {
	.name		= "at91_adc",
	.id		= -1,
	.dev		= {
				.platform_data		= &ek_at91_adc_data,
	},
	.resource	= adc_resources,
	.num_resources	= ARRAY_SIZE(adc_resources),
};

static int __init at91_adc_init(void)
{
	struct at91_adc_data *data = ek_at91_adc_data;

	if (test_bit(0, &data->channels_used))
		at91_set_A_periph(AT91_PIN_PC0, 0);
	if (test_bit(1, &data->channels_used))
		at91_set_A_periph(AT91_PIN_PC1, 0);
	if (test_bit(2, &data->channels_used))
		at91_set_A_periph(AT91_PIN_PC2, 0);
	if (test_bit(3, &data->channels_used))
		at91_set_A_periph(AT91_PIN_PC3, 0);

	if (data->use_external_triggers)
		at91_set_A_periph(AT91_PIN_PA22, 0);

	platform_device_register(&at91_adc_device);
}

static void __exit at91_adc_exit(void)
{
	platform_device_unregister(&at91_adc_device);
}

module_init(at91_adc_init);
module_exit(at91_adc_exit);


MODULE_DESCRIPTION("AT91 ADC Device");
MODULE_AUTHOR("Ju Jiaqi");
MODULE_LICENSE("GPL");
