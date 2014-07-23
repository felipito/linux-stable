/*  arch/arm/mach-lpc313x/board-usbpio.c
 *
 *  Author:	Luis Galdos
 *  Copyright (C) 2014 Luis Galdos
 *
 *  USB PN-IO board init routines.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dm9000.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/socket.h> /* AF_PNIO */

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/sizes.h>

#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <mach/gpio.h>
#include <mach/i2c.h>
#include <mach/board.h>
#include <mach/system.h>

#define GPIO_STATUS		(GPIO_GPIO13)

extern void __init lpc313x_timer_init(void);

static struct delayed_work usbpio_work;
static int status_value = 1;

static void usbpio_work_func(struct work_struct *work)
{
	int err;
	static struct socket  *pn_sock;

	if (!pn_sock) {
		err = sock_create(AF_PNIO, SOCK_DGRAM, 1, &pn_sock);
		if (err) {
			status_value ^= 1;
			schedule_delayed_work(&usbpio_work, msecs_to_jiffies(900));
		} else {
			sock_release(pn_sock);
			status_value = 1;
		}
	}

	gpio_set_value(GPIO_STATUS, status_value);
}

static int __init usbpio_task_init(void)
{
	int err;

	err = gpio_request(GPIO_STATUS, "usbpio-device");
	if (err) {
		printk(KERN_ERR "Couldn't get %i for USBPIO device status\n",
		       GPIO_STATUS);
		goto exit_in;
	}

	gpio_direction_output(GPIO_STATUS, status_value);
	INIT_DELAYED_WORK(&usbpio_work, usbpio_work_func);
	schedule_delayed_work(&usbpio_work, HZ);

exit_in:
	return 0;
}

static struct lpc313x_mci_irq_data irq_data = {
	.irq = IRQ_SDMMC_CD,
};

static int mci_get_cd(u32 slot_id)
{
	return gpio_get_value(GPIO_MNAND_RYBN2);
}

static irqreturn_t ea313x_mci_detect_interrupt(int irq, void *data)
{
	struct lpc313x_mci_irq_data	*pdata = data;

	/* select the opposite level senstivity */
	int level = mci_get_cd(0) ? IRQ_TYPE_LEVEL_LOW : IRQ_TYPE_LEVEL_HIGH;

	irq_set_irq_type(pdata->irq, level);

	/* change the polarity of irq trigger */
	return pdata->irq_hdlr(irq, pdata->data);
}

static int mci_init(u32 slot_id, irq_handler_t irqhdlr, void *data)
{
	int ret;
	int level;

	/* enable power to the slot */
	gpio_request(GPIO_MI2STX_DATA0, "mmc power");
	gpio_set_value(GPIO_MI2STX_DATA0, 0);
	/* set cd pins as GPIO pins */
	gpio_request(GPIO_MNAND_RYBN2, "mmc card detect");
	gpio_direction_input(GPIO_MNAND_RYBN2);

	/* select the opposite level senstivity */
	level = mci_get_cd(0) ? IRQ_TYPE_LEVEL_LOW : IRQ_TYPE_LEVEL_HIGH;
	/* set card detect irq info */
	irq_data.data = data;
	irq_data.irq_hdlr = irqhdlr;
	irq_set_irq_type(irq_data.irq, level);
	ret = request_irq(irq_data.irq,
			  ea313x_mci_detect_interrupt,
			  level,
			  "mmc-cd",
			  &irq_data);
	/****temporary for PM testing */
	enable_irq_wake(irq_data.irq);

	return irq_data.irq;
}

static int mci_get_ro(u32 slot_id)
{
	return 0;
}

static int mci_get_ocr(u32 slot_id)
{
	return MMC_VDD_32_33 | MMC_VDD_33_34;
}

static void mci_setpower(u32 slot_id, u32 volt)
{
	/* on current version of EA board the card detect
	 * pull-up in on switched power side. So can't do
	 * power management so use the always enable power
	 * jumper.
	 */
}
static int mci_get_bus_wd(u32 slot_id)
{
	return 4;
}

static void mci_exit(u32 slot_id)
{
	free_irq(irq_data.irq, &irq_data);
}

static struct resource lpc313x_mci_resources[] = {
	[0] = {
		.start  = IO_SDMMC_PHYS,
		.end	= IO_SDMMC_PHYS + IO_SDMMC_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_MCI,
		.end	= IRQ_MCI,
		.flags	= IORESOURCE_IRQ,
	},
};
static struct lpc313x_mci_board ea313x_mci_platform_data = {
	.num_slots		= 1,
	.detect_delay_ms	= 250,
	.init                   = mci_init,
	.get_ro			= mci_get_ro,
	.get_cd                 = mci_get_cd,
	.get_ocr		= mci_get_ocr,
	.get_bus_wd		= mci_get_bus_wd,
	.setpower               = mci_setpower,
	.exit			= mci_exit,
};

static u64 mci_dmamask = 0xffffffffUL;
static struct platform_device	lpc313x_mci_device = {
	.name		= "lpc313x_mmc",
	.num_resources	= ARRAY_SIZE(lpc313x_mci_resources),
	.dev		= {
		.dma_mask		= &mci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &ea313x_mci_platform_data,
	},
	.resource	= lpc313x_mci_resources,
};

static struct resource lpc313x_adc_resources[] = {
	[0] = {
		.start  = ADC_PHYS,
		.end	= ADC_PHYS + 0x30,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device lpc313x_adc_device = {
	.name		= "lpc313x_adc",
	.resource	= lpc313x_adc_resources,
	.num_resources	= ARRAY_SIZE(lpc313x_adc_resources),
};

/*
 * DM9000 ethernet device
 */
#if defined(CONFIG_DM9000)
static struct resource dm9000_resource[] = {
	[0] = {
		.start	= EXT_SRAM0_PHYS,
		.end	= EXT_SRAM0_PHYS + 0x3,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= EXT_SRAM0_PHYS + 0x8,
		.end	= EXT_SRAM0_PHYS + 0x8 + 0x3,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= IRQ_DM9000_ETH_INT,
		.end	= IRQ_DM9000_ETH_INT,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	}
};

#define GPIO_RST                (GPIO_GPIO12)
#define GPIO_PW_RST             (GPIO_GPIO11)

static void dm9000_reset(void)
{
	gpio_direction_output(GPIO_RST, 1);
	gpio_direction_output(GPIO_PW_RST, 0);

	mdelay(50);
	gpio_set_value(GPIO_PW_RST, 1);

	mdelay(50);
	gpio_set_value(GPIO_RST, 0);

	mdelay(50);
}

static struct dm9000_plat_data dm9000_platdata = {
	.flags		= DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM,
        //	.hwreset	= dm9000_reset,
};

static struct platform_device dm9000_device = {
	.name		= "dm9000",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(dm9000_resource),
	.resource	= dm9000_resource,
	.dev		= {
		.platform_data	= &dm9000_platdata,
	}
};
static void __init ea_add_device_dm9000(void)
{
	int err;

	SYS_MUX_LCD_EBI = 0x1;

	MPMC_STCONFIG0 = 0x81 | (1 << 3);

	MPMC_STWTWEN0  = 0x6;
	MPMC_STWTOEN0  = 0x5;
	MPMC_STWTRD0   = 0x9;
	MPMC_STWTWR0   = 0x11;
	MPMC_STWTTURN0 = 0xf;
	MPMC_STWTPG0   = 0xf;

	/* enable oe toggle between consec reads */
	SYS_MPMC_WTD_DEL0 = _BIT(5) | 4;

	/* Configure Interrupt pin as input, no pull-up */
	err = gpio_request(GPIO_MNAND_RYBN3, "dm9000 interrupt");
	if (err) {
		printk(KERN_ERR "Couldn't request IRQ line for DMA9000\n");
		return;
	}

	gpio_direction_input(GPIO_MNAND_RYBN3);

	/* GPIO for the power reset */
	err = gpio_request(GPIO_PW_RST, "dma9000-pwrst");
	if (err) {
		printk(KERN_ERR "Couldn't get GPIO11 for DMA9000\n");
		return;
	}

	err = gpio_request(GPIO_RST, "dma9000-rst");
	if (err) {
		printk(KERN_ERR "Couldn't request GPIO3 for DMA9000\n");
		return;
	}

	dm9000_reset();
	platform_device_register(&dm9000_device);
}
#else
static void __init ea_add_device_dm9000(void) {}
#endif /* CONFIG_DM9000 */

static struct platform_device *devices[] __initdata = {
	&lpc313x_mci_device,
	&lpc313x_adc_device,
};

static struct map_desc ea313x_io_desc[] __initdata = {
	{
		.virtual	= io_p2v(EXT_SRAM0_PHYS),
		.pfn		= __phys_to_pfn(EXT_SRAM0_PHYS),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(EXT_SRAM1_PHYS + 0x10000),
		.pfn		= __phys_to_pfn(EXT_SRAM1_PHYS + 0x10000),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_SDMMC_PHYS),
		.pfn		= __phys_to_pfn(IO_SDMMC_PHYS),
		.length		= IO_SDMMC_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_USB_PHYS),
		.pfn		= __phys_to_pfn(IO_USB_PHYS),
		.length		= IO_USB_SIZE,
		.type		= MT_DEVICE
	},
};

void lpc313x_vbus_power(int enable)
{
	//printk (KERN_INFO "enabling USB host vbus_power %d\n", enable);
	//gpio_set_value(VBUS_PWR_EN, enable);
}

static void __init usbpio_init(void)
{
	lpc313x_init();
	/* register i2cdevices */
	lpc313x_register_i2c_devices();

	platform_add_devices(devices, ARRAY_SIZE(devices));

	/* add DM9000 device */
	ea_add_device_dm9000();

	usbpio_task_init();
}

static void __init usbpio_map_io(void)
{
	lpc313x_map_io();
	iotable_init(ea313x_io_desc, ARRAY_SIZE(ea313x_io_desc));
}

MACHINE_START(EA313X, "USBPIO")
	.map_io		= usbpio_map_io,
	.init_irq	= lpc313x_init_irq,
	.init_time	= lpc313x_timer_init,
	.init_machine	= usbpio_init,
	.restart	= arch_reset,
MACHINE_END
