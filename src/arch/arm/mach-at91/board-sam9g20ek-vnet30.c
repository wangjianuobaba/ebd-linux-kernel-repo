/*
 *  Copyright (C) 2005 SAN People
 *  Copyright (C) 2008 Atmel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/consumer.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/at91sam9_smc.h>
#include <mach/system_rev.h>

#include "sam9_smc.h"
#include "generic.h"

#include <linux/gpio_keys.h>
#include <linux/input.h>

#include <linux/spi/spi.h>
#include <linux/spi/at73c213.h>


#include <linux/wl12xx.h>
#include <linux/mmc/host.h>
#include <linux/delay.h>

#include <linux/spi/spi.h>

#include <mach/at91_adc.h>
#include <linux/platform_data/at91_adc.h>

static void __init ek_init_early(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91_initialize(18432000);

	/* DBGU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

	/* USART0 on ttyS1. (Rx, Tx, CTS, RTS, DTR, DSR, DCD, RI) */

	at91_register_uart(AT91SAM9260_ID_US0, 1, 0 /* ATMEL_UART_CTS | ATMEL_UART_RTS
			   | ATMEL_UART_DTR | ATMEL_UART_DSR | ATMEL_UART_DCD
			   | ATMEL_UART_RI*/ );

	/* USART1 on ttyS2 ttyS3 ttyS4 ttyS5 ttyS6. (Rx, Tx) */
	at91_register_uart(AT91SAM9260_ID_US1, 2, 0/*ATMEL_UART_CTS | ATMEL_UART_RTS*/);
	at91_register_uart(AT91SAM9260_ID_US2, 3, 0/*ATMEL_UART_CTS | ATMEL_UART_RTS*/);
	at91_register_uart(AT91SAM9260_ID_US3, 4, 0/*ATMEL_UART_CTS | ATMEL_UART_RTS*/);
	at91_register_uart(AT91SAM9260_ID_US4, 5, 0/*ATMEL_UART_CTS | ATMEL_UART_RTS*/);
	at91_register_uart(AT91SAM9260_ID_US5, 6, 0/*ATMEL_UART_CTS | ATMEL_UART_RTS*/);

	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(0);
}


/*
 * USB Host port
 */
static struct at91_usbh_data __initdata ek_usbh_data = {
	.ports		= 2,
	.vbus_pin = {0, 0},
	.overcurrent_pin = {0, 0},
};


/*
 * SPI devices.
 */
static struct spi_board_info ek_spi_devices[] = {
	{	/* SCPU */
		.modalias	= "spidev",
		.chip_select	= 0,
		.max_speed_hz	= 6 * 1000 * 1000,
		.bus_num	= 1,
	},
};


/*
 * I2C devices.
 */
static struct i2c_board_info __initdata ek_i2c_devices[] = {
        {
                I2C_BOARD_INFO("fm3116", 0x52)
        },
};

/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata ek_macb_data = {
	.phy_irq_pin	= AT91_PIN_PA22,
	.is_rmii	= 1,
};

static void __init ek_add_device_macb(void)
{
	at91_add_device_eth(&ek_macb_data);
}


/*
 * NAND flash
 */
static struct mtd_partition __initdata ek_nand_partition[] = {
	{
		.name   = "system0",
		.offset = 0,
		.size   = 64 * SZ_1M,
	},
	{
		.name   = "system1",
		.offset = MTDPART_OFS_NXTBLK,
		.size   = 64 * SZ_1M,
	},
	{
		.name	= "data",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

/* det_pin is not connected */
static struct atmel_nand_data __initdata ek_nand_data = {
	.ale		= 21,
	.cle		= 22,
	.rdy_pin	= AT91_PIN_PC13,
	.enable_pin	= AT91_PIN_PC14,
	.parts		= ek_nand_partition,
	.num_parts	= ARRAY_SIZE(ek_nand_partition),
};

static struct sam9_smc_config __initdata ek_nand_smc_config = {
	.ncs_read_setup		= 0,
	.nrd_setup		= 2,
	.ncs_write_setup	= 0,
	.nwe_setup		= 2,

	.ncs_read_pulse		= 4,
	.nrd_pulse		= 4,
	.ncs_write_pulse	= 4,
	.nwe_pulse		= 4,

	.read_cycle		= 7,
	.write_cycle		= 7,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE,
	.tdf_cycles		= 3,
};

static void __init ek_add_device_nand(void)
{
	ek_nand_data.bus_width_16 = 0;
	/* setup bus-width (8 or 16) */
	if (ek_nand_data.bus_width_16)
		ek_nand_smc_config.mode |= AT91_SMC_DBW_16;
	else
		ek_nand_smc_config.mode |= AT91_SMC_DBW_8;

	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(3, &ek_nand_smc_config);

	at91_add_device_nand(&ek_nand_data);
}


/*
 * MCI (SD/MMC)
 * wp_pin and vcc_pin are not connected
 */

/* wl12xx enable pin */
#define PIN_WIFI_ENABLE		AT91_PIN_PA3

/* wl12xx irq pin and irq number */
#define PIN_WIFI_IRQ		AT91_PIN_PC12
#define WIFI_IRQ			AT91SAM9260_ID_IRQ0

struct wl12xx_platform_data ek_wl12xx_data = {
	.irq = 				WIFI_IRQ,
	.board_ref_clock = 	WL12XX_REFCLOCK_38_XTAL, /* 38.4Mhz */
	.platform_quirks = 	WL12XX_PLATFORM_QUIRK_EDGE_IRQ,
};

static void __init wl12xx_init(void)
{	
	/* set PIN_WIFI_ENABLE mode to out, value to 0, enable pull up for suspend */
	at91_set_gpio_output(PIN_WIFI_ENABLE, 1);
	gpio_request(PIN_WIFI_IRQ, "wlan_irq");
	at91_set_A_periph(PIN_WIFI_IRQ, 0);
	wl12xx_set_platform_data(&ek_wl12xx_data);
}

static struct at91_mmc_data __initdata ek_mmc_data = {
	.slot_b		= 0,	/* Only one slot so use slot B */
	.wire4		= 1,
/*
	.det_pin	= AT91_PIN_PC9,
*/
	.vcc_pin = PIN_WIFI_ENABLE,
};

static void __init ek_add_device_mmc(void)
{

	wl12xx_init();
	at91_add_device_mmc(0, &ek_mmc_data);
}


/*
 * LEDs
 */
 
static struct gpio_led ek_leds[] = {
	{	/* com_1_sel */
		.name			= "com_1_sel",
		.gpio			= AT91_PIN_PB31,
		.active_low		= 0,
		.default_trigger	= "none",
	},
	{	/* com_2_sel */
		.name			= "com_2_sel",
		.gpio			= AT91_PIN_PB16,
		.active_low		= 0,
		.default_trigger	= "none",
	},
	{	/* com_avr_sel */
		.name			= "com_avr_sel",
		.gpio			= AT91_PIN_PB17,
		.active_low		= 0,
		.default_trigger	= "none",
	},
	{	/* com_db_sel */
		.name			= "com_db_sel",
		.gpio			= AT91_PIN_PB18,
		.active_low		= 0,
		.default_trigger	= "none",
		.default_state 	= LEDS_GPIO_DEFSTATE_ON,
	},
	
	{	/* led_wifi */
		.name			= "led_wifi",
		.gpio			= AT91_PIN_PB21,
		.active_low		= 1,
		.default_trigger	= "none",
		.default_state 	= LEDS_GPIO_DEFSTATE_ON,
	},
	{	/* led_rec*/
		.name			= "led_rec",
		.gpio			= AT91_PIN_PB22,
		.active_low		= 0,
		.default_trigger	= "none",
		.default_state 	= LEDS_GPIO_DEFSTATE_ON,
	},
	{	/* led_gps_g */
		.name			= "led_gprs_g",
		.gpio			= AT91_PIN_PB23,
		.active_low		= 0,
		.default_trigger	= "none",
		.default_state 	= LEDS_GPIO_DEFSTATE_ON,
	},
	{	/* led_gps_r */
		.name			= "led_gprs_r",
		.gpio			= AT91_PIN_PB24,
		.active_low		= 0,
		.default_trigger	= "none",
		.default_state 	= LEDS_GPIO_DEFSTATE_ON,
	},
	{	/* led_pwr_g */
		.name			= "led_pwr_g",
		.gpio			= AT91_PIN_PB25,
		.active_low		= 0,
		.default_trigger	= "none",
		.default_state 	= LEDS_GPIO_DEFSTATE_ON,
	},
	{	/* led_pwr_r */
		.name			= "led_pwr_r",
		.gpio			= AT91_PIN_PB26,
		.active_low		= 0,
		.default_trigger	= "none",
		.default_state 	= LEDS_GPIO_DEFSTATE_ON,
	},
	{	/* cio */
		.name			= "cio",
		.gpio			= AT91_PIN_PB27,
		.active_low		= 0,
		.default_trigger	= "none",
		.default_state 	= LEDS_GPIO_DEFSTATE_ON,
	},
	{	/* usb_pwr */
		.name			= "usb_pwr",
		.gpio			= AT91_PIN_PB28,
		.active_low		= 0,
		.default_trigger	= "none",
	},
	{	/* gnss_pwr */
		.name			= "gnss_pwr",
		.gpio			= AT91_PIN_PB29,
		.active_low		= 0,
		.default_trigger	= "none",
		.default_state 	= LEDS_GPIO_DEFSTATE_ON,
	},
	{	/* wifi_pwr */
		.name			= "wifi_pwr",
		.gpio			= AT91_PIN_PB30,
		.active_low		= 0,
		.default_trigger	= "none",
		.default_state 	= LEDS_GPIO_DEFSTATE_OFF,
	},
	{	/* 485tr_sel */
		.name			= "485tr_sel",
		.gpio			= AT91_PIN_PC6,
		.active_low		= 0,
		.default_trigger	= "none",
	},
	{	/* 12v_pwr */
		.name			= "12v_pwr",
		.gpio			= AT91_PIN_PC8,
		.active_low		= 0,
		.default_trigger	= "none",
	},
};

static void __init ek_add_device_gpio_leds(void)
{
	at91_gpio_leds(ek_leds, ARRAY_SIZE(ek_leds));
}


/*
 * GPIO Buttons
 */

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button ek_keys[] = {
	{
		.code		= KEY_F1,
		.gpio		= AT91_PIN_PC1,
		.active_low	= 1,
		.desc		= "KEY_I",
		.type		= EV_KEY,
		.debounce_interval = 10,
		.wakeup		= 1,
		.can_disable = 1,
	},
	{
		.code		= KEY_F2,
		.gpio		= AT91_PIN_PC2,
		.active_low	= 1,
		.desc		= "KEY_F",
		.type		= EV_KEY,
		.debounce_interval = 10,
		.wakeup		= 1,
		.can_disable = 1,
	},
	{
		.code		= KEY_F3,
		.gpio		= AT91_PIN_PB19,
		.active_low	= 1,
		.desc		= "SATE_LED",
		.type		= EV_KEY,
		.debounce_interval = 0,
		.wakeup		= 1,
		.can_disable = 1,
	},
	{
		
		.code		= KEY_F4,
		.gpio		= AT91_PIN_PB20,
		.active_low	= 1,
		.desc		= "S_DCD",
		.type		= EV_KEY,
		.debounce_interval = 0,
		.wakeup		= 1,
		.can_disable = 1,
	},
	{
		.code		= KEY_F5,
		.gpio		= AT91_PIN_PC3,
		.active_low	= 1,
		.desc		= "GC_2",
		.type		= EV_KEY,
		.debounce_interval = 0,
		.wakeup		= 1,
		.can_disable = 1,
	},
	{
		.code		= KEY_F6,
		.gpio		= AT91_PIN_PC10,
		.active_low	= 1,
		.desc		= "A_INT",
		.type		= EV_KEY,
		.debounce_interval = 0,
		.wakeup		= 1,
		.can_disable = 1,
	},
};

static struct gpio_keys_platform_data ek_key_data = {
	.buttons	= ek_keys,
	.nbuttons	= ARRAY_SIZE(ek_keys),
};

static struct platform_device ek_key_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data	= &ek_key_data,
	}
};

static void __init ek_add_device_buttons(void)
{
	struct gpio_keys_button *keys = ek_keys;
	int nr = ARRAY_SIZE(ek_keys);
	int i;

	for (i = 0; i < nr; i++) {
		at91_set_gpio_input(keys[i].gpio, 1);	// input, enable pullup
		at91_set_deglitch(keys[i].gpio, 1);		// enalbe deglitch
	}

	platform_device_register(&ek_key_device);
}
#else
static void __init ek_add_device_buttons(void) {}
#endif


/*
* Regulator
*/

#if defined(CONFIG_REGULATOR_FIXED_VOLTAGE) || defined(CONFIG_REGULATOR_FIXED_VOLTAGE_MODULE)
static struct regulator_consumer_supply ek_audio_consumer_supplies[] = {
	REGULATOR_SUPPLY("AVDD", "0-001b"),
	REGULATOR_SUPPLY("HPVDD", "0-001b"),
	REGULATOR_SUPPLY("DBVDD", "0-001b"),
	REGULATOR_SUPPLY("DCVDD", "0-001b"),
};

static struct regulator_init_data ek_avdd_reg_init_data = {
	.constraints	= {
		.name	= "3V3",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = ek_audio_consumer_supplies,
	.num_consumer_supplies = ARRAY_SIZE(ek_audio_consumer_supplies),
};

static struct fixed_voltage_config ek_vdd_pdata = {
	.supply_name	= "board-3V3",
	.microvolts	= 3300000,
	.gpio		= -EINVAL,
	.enabled_at_boot = 0,
	.init_data	= &ek_avdd_reg_init_data,
};
static struct platform_device ek_voltage_regulator = {
	.name		= "reg-fixed-voltage",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &ek_vdd_pdata,
	},
};
static void __init ek_add_regulators(void)
{
	platform_device_register(&ek_voltage_regulator);
}
#else
static void __init ek_add_regulators(void) {}
#endif


/*
* ADC
*/
static struct at91_adc_trigger ek_adc_triggers[] = {
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

static struct at91_adc_reg_desc ek_adc_register_g20 = {
	.channel_base = AT91_ADC_CHR(0),
	.drdy_mask = AT91_ADC_DRDY,
	.status_register = AT91_ADC_SR,
	.trigger_register = AT91_ADC_MR,
};

static struct at91_adc_data ek_adc_data = {
	.channels_used = BIT(0),
	.num_channels = 4,
	.registers = &ek_adc_register_g20,
	.startup_time = 10,
	.trigger_list = ek_adc_triggers,
	.trigger_number = ARRAY_SIZE(ek_adc_triggers),
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

static struct platform_device ek_adc_device = {
	.name		= "at91_adc",
	.id		= -1,
	.dev		= {
				.platform_data	= &ek_adc_data,
	},
	.resource	= adc_resources,
	.num_resources	= ARRAY_SIZE(adc_resources),
};

static int __init ek_adc_init(void)
{
	if (test_bit(0, &ek_adc_data.channels_used))
		at91_set_A_periph(AT91_PIN_PC0, 0);
	if (test_bit(1, &ek_adc_data.channels_used))
		at91_set_A_periph(AT91_PIN_PC1, 0);
	if (test_bit(2, &ek_adc_data.channels_used))
		at91_set_A_periph(AT91_PIN_PC2, 0);
	if (test_bit(3, &ek_adc_data.channels_used))
		at91_set_A_periph(AT91_PIN_PC3, 0);

	if (ek_adc_data.use_external_triggers)
		at91_set_A_periph(AT91_PIN_PA22, 0);

	return platform_device_register(&ek_adc_device);
}

static void __init ek_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	
	/* Regulators */
	ek_add_regulators();
#if 1
	/* Ethernet */
	ek_add_device_macb();
	
	/* NAND */
	ek_add_device_nand();
#endif
	/* USB Host */
	at91_add_device_usbh(&ek_usbh_data);
#if 1
	/* SPI */
	at91_add_device_spi(ek_spi_devices, ARRAY_SIZE(ek_spi_devices));

	/* I2C */
	at91_add_device_i2c(ek_i2c_devices, ARRAY_SIZE(ek_i2c_devices));
	
	/* MMC */
	ek_add_device_mmc();

	/* ADC */
	ek_adc_init();

	/* LEDs */
	ek_add_device_gpio_leds();

	/* Push Buttons */
	ek_add_device_buttons();
#endif
}

MACHINE_START(AT91SAM9G20EK, "Baite AT91SAM9G20-EK")
	/* Maintainer: Baite */
	.timer		= &at91sam926x_timer,
	.map_io		= at91_map_io,
	.init_early	= ek_init_early,
	.init_irq	= at91_init_irq_default,
	.init_machine	= ek_board_init,
MACHINE_END

MACHINE_START(AT91SAM9G20EK_2MMC, "Baite AT91SAM9G20-EK 2 MMC Slot Mod")
	/* Maintainer: Baite */
	.timer		= &at91sam926x_timer,
	.map_io		= at91_map_io,
	.init_early	= ek_init_early,
	.init_irq	= at91_init_irq_default,
	.init_machine	= ek_board_init,
MACHINE_END
