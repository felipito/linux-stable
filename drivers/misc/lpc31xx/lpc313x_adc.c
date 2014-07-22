/* -*- linux-c -*-
 * LPC313x ADC driver for Linux
 *
 * Copyright (C) 2014 Luis Galdos <luisgaldos@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * Lesser General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/spinlock.h>
#include <linux/rcupdate.h>
#include <linux/uaccess.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/cdev.h>		/* cdev_init() */
#include <linux/fs.h>		/* struct file_operations */
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kthread.h>	/* kthread_run() */
#include <linux/rwsem.h>

#include <mach/hardware.h>
#include <mach/registers.h>

#define LPC313X_DRV_NAME		"lpc313x_adc"
#define LPC313X_ADC_CHANNELS_MAX	(4)
#define LPC313X_ADC_CYCLE_MS		(20)
#define LPC313X_ADC_RESOLUTION_MAX	(10)
#define LPC313X_ADC_SYSFS_NAME		"adc%u"

/* Register offsets */
#define REG_CON				(0x20)
#define REG_CON_ENABLE			(1 << 1)
#define REG_CON_CSCAN			(1 << 2)
#define REG_CON_START			(1 << 3)
#define REG_CON_PROGRESS		(1 << 4)

#define REG_R0				(0x00)
#define REG_R1				(0x04)
#define REG_R2				(0x08)
#define REG_R3				(0x0c)

#define REG_CSEL			(0x24)
#define REG_CSEL_CH_SET(c, v)		(v << (c * 4))
#define REG_CSEL_CH_MASK(c)		(0xf << (c * 4))

#define REG_INT_EN			(0x28)
#define REG_INT_EN_ENABLE		(1 << 0)

#define REG_INT_STATUS			(0x2c)
#define REG_INT_STATUS_PENDING		(1 << 0)

#define REG_INT_CLEAR			(0x30)
#define REG_INT_CLEAR_STATUS		(1 << 0)

#define REG_CH(c)			(4 * c)

/* Enable debug messages */
#if 0
#define pk_dbg(fmt, args...)		do { printk(KERN_DEBUG "PN: %s() " fmt, \
						    __func__, ## args); } while (0)
#else
#define pk_dbg(fmt, args...)		do { } while (0)
#endif

/* Module infos */
MODULE_DESCRIPTION("LPC313x ADC driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Luis Galdos");

/* Set this value to zero for getting major number at runtime */
static int drv_major = 241;

struct lpc313x_adc {
	void __iomem *io;
	u32 iobase;
	u32 iosize;

	dev_t devnr;
	int minor;

	int channels;
	u16 samples[LPC313X_ADC_CHANNELS_MAX];
	u32 enabled[LPC313X_ADC_CHANNELS_MAX];
	struct semaphore lock_hw;
	struct rw_semaphore lock;
	int activated;

	struct cdev cdev;
	struct task_struct *kthread;
	unsigned long cycle;

	/* Pointer to the sysfs attributes created on run time */
	struct device_attribute attrs[LPC313X_ADC_CHANNELS_MAX];
	char *names[LPC313X_ADC_CHANNELS_MAX];
};

struct lpc313x_adc_file {
	struct lpc313x_adc *adc;
	u32 mask;
};

static int wait_int_status(struct lpc313x_adc *adc)
{
	u16 timeout = 0x1fff;
	u32 reg;

	do {
		reg = readl(adc->io + REG_INT_STATUS);
	} while ((reg & REG_INT_STATUS_PENDING) && timeout--);

	return timeout ? 0 : -ETIME;
}

static int lpc313x_init_adc(struct lpc313x_adc *adc, u32 csel)
{
	u32 reg;
	int err = 0;

	pk_dbg("Init ADC %p | csel %08x\n", adc, csel);

	SYS_ADC_PD = 0;
	cgu_clk_en_dis(CGU_SB_ADC_CLK_ID, 1);
	cgu_clk_en_dis(CGU_SB_ADC_PCLK_ID, 1);

	writel(0, adc->io + REG_CON);
	writel(0, adc->io + REG_CSEL);
	writel(0, adc->io + REG_INT_EN);
	writel(0, adc->io + REG_INT_CLEAR);

	err = wait_int_status(adc);
	if (err) {
		pr_err("Couldn't reset ADC, %i\n", err);
		goto exit_init;
	}

	writel(csel, adc->io + REG_CSEL);

	/* Enable ADC interrupt */
	reg = readl(adc->io + REG_INT_EN);
	writel(reg | REG_INT_EN_ENABLE, adc->io + REG_INT_EN);

	/* Write ADC start command and reset start bit */
	reg = readl(adc->io + REG_CON);
	writel(reg | REG_CON_ENABLE, adc->io + REG_CON);

exit_init:
	return err;
}

static int lpc313x_stop_adc(struct lpc313x_adc *adc)
{
	u32 reg;

	pk_dbg("Stopping ADC %p\n", adc);

	writel(0, adc->io + REG_CON);
	writel(0, adc->io + REG_CSEL);
	writel(0, adc->io + REG_INT_EN);
	writel(0, adc->io + REG_INT_CLEAR);

	reg = readl(adc->io + REG_INT_STATUS);
	if (reg & REG_INT_STATUS_PENDING) {
		writel(REG_INT_CLEAR_STATUS, adc->io + REG_INT_CLEAR);
		wait_int_status(adc);
	}

	cgu_clk_en_dis(CGU_SB_ADC_CLK_ID, 0);
	cgu_clk_en_dis(CGU_SB_ADC_PCLK_ID, 0);

	return 0;
}

static int channel_enable(struct lpc313x_adc *adc, ulong number, ulong resol)
{
	int err = -ENODEV;

	if (number < adc->channels) {

		if (!resol || (resol > LPC313X_ADC_RESOLUTION_MAX))
			goto exit_en;

		pk_dbg("Enabling ADC %p channel %lu\n", adc, number);
		down(&adc->lock_hw);

		/* Check if have to enable the channel */
		if (!adc->enabled[number]) {
			u32 reg;

			reg = readl(adc->io + REG_CSEL);
			reg &= ~REG_CSEL_CH_MASK(number);
			reg |= REG_CSEL_CH_SET(number, resol);

			writel(reg, adc->io + REG_CSEL);

			adc->activated++;
			if (adc->activated == 1)
				wake_up_process(adc->kthread);
		}

		adc->enabled[number]++;
		up(&adc->lock_hw);
		err = 0;
	}

exit_en:
	return err;
}

static int channel_disable(struct lpc313x_adc *adc, ulong number)
{
	u32 reg;
	int err = -ENODEV;

	if (number < adc->channels) {
		pk_dbg("Disabling ADC %p channel %lu\n", adc, number);
		down(&adc->lock_hw);

		adc->enabled[number]--;
		if (!adc->enabled[number]) {
			reg = readl(adc->io + REG_CSEL);
			reg &= ~REG_CSEL_CH_MASK(number);
			writel(reg, adc->io + REG_CSEL);

			adc->activated--;
		}

		up(&adc->lock_hw);
		err = 0;
	}

	return err;
}

static int lpc313x_adc_open(struct inode *inode, struct file *filp)
{
	struct lpc313x_adc *adc = container_of(inode->i_cdev, struct lpc313x_adc, cdev);
	struct lpc313x_adc_file *fadc;

	fadc = kmalloc(sizeof(*fadc), GFP_KERNEL);
	if (fadc) {
		fadc->mask = 0;
		fadc->adc = adc;
		filp->private_data = fadc;
	}

	return fadc ? 0 : -ENOMEM;
}

static int lpc313x_adc_release(struct inode *inode, struct file *filp)
{
	struct lpc313x_adc_file *fadc = filp->private_data;
	struct lpc313x_adc *adc = fadc->adc;
	int cnt;

	for (cnt = 0; cnt < adc->channels; cnt++) {
		if (fadc->mask & (1 << cnt))
			channel_disable(fadc->adc, cnt);
	}

	kfree(fadc);
	return 0;
}

static ssize_t lpc313x_adc_read(struct file *filp, char *user, size_t length,
				loff_t *offset)
{
	struct lpc313x_adc_file *fadc = filp->private_data;
	struct lpc313x_adc *adc = fadc->adc;
	ssize_t ret;
	u16 samples[LPC313X_ADC_CHANNELS_MAX];

	ret = 2 * adc->channels;
	if (length < ret)
		return -ENOBUFS;

	down_read(&adc->lock);
	memcpy(samples, adc->samples, ret);
	up_read(&adc->lock);

	if (copy_to_user(user, samples, ret))
		ret = -EFAULT;

	return ret;
}

static ssize_t lpc313x_adc_write(struct file *filp, const char *user, size_t length,
				 loff_t *offset)
{
	struct lpc313x_adc_file *fadc = filp->private_data;
	u8 vals[3];
	int err;
	u8 number, cmd;

	/* The first byte is the command type, and the rest depends on the command */
	if (length != 2)
		return -EINVAL;

	if (copy_from_user(vals, user, length))
		return -EFAULT;

	cmd    = vals[0];
	number = vals[1];

	switch (cmd) {
	case 0:
		/* Assure the channel is active */
		if (!(fadc->mask & (1 << number))) {
			err = -EINVAL;
			goto exit_wr;
		}

		err = channel_disable(fadc->adc, number);
		if (!err)
			fadc->mask &= ~(1 << number);
		break;

	case 1:
		/* Report error if the channel already active */
		if (fadc->mask & (1 << number)) {
			err = -EALREADY;
			goto exit_wr;
		}

		err = channel_enable(fadc->adc, number, LPC313X_ADC_RESOLUTION_MAX);
		if (!err)
			fadc->mask |= (1 << number);
		break;

	default:
		err = -EOPNOTSUPP;
		break;
	}

exit_wr:
	return err ? err : length;
}

static int lpc313x_adc_kthread(void *_adc)
{
	struct lpc313x_adc *adc = (struct lpc313x_adc *)_adc;
	int cnt;
	u32 st, con;
	ulong timeout;
	ulong cycle = adc->cycle;
	/*
	 * Use a local samples buffer as we always have to read the ADC values other
	 * wise the ADC controller starts to go crazy
	 */
	u16 samples[LPC313X_ADC_CHANNELS_MAX];

	lpc313x_init_adc(adc, 0);
	while (1) {
		if (kthread_should_stop())
			break;

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(cycle));

		down(&adc->lock_hw);

		/* When timeout different zero, a signal was sent */
		if (!adc->activated) {
			cycle = 1000; /* One second when no ADC opened */
			up(&adc->lock_hw);
			continue;
		}

		con = readl(adc->io + REG_CON);
		writel(con | REG_CON_START, adc->io + REG_CON);

		/* Check if a new sample is available */
		timeout = 0x400;
		do {
			st = readl(adc->io + REG_INT_STATUS);
		} while (--timeout && !(st & REG_INT_STATUS_PENDING));

		if (!timeout) {
			up(&adc->lock_hw);
			continue;
		}

		/* Clear status flag */
		writel(REG_INT_CLEAR_STATUS, adc->io + REG_INT_CLEAR);

		for (cnt = 0; cnt < adc->channels; cnt++) {
			if (adc->enabled[cnt])
				samples[cnt] = (u16)readl(adc->io + REG_CH(cnt));
			else
				samples[cnt] = 0xffff;
		}

		/* Disable required for next scan */
		con = readl(adc->io + REG_CON);
		writel(con & ~REG_CON_START, adc->io + REG_CON);

		cycle = adc->cycle;

		up(&adc->lock_hw);

		down_write(&adc->lock);
		memcpy(adc->samples, samples, 2 * adc->channels);
		up_write(&adc->lock);
	}

	lpc313x_stop_adc(adc);
	return 0;
}

static ssize_t show_cycle(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc313x_adc *adc = platform_get_drvdata(pdev);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", adc->cycle);
}

static ssize_t store_cycle(struct device *dev, struct device_attribute *attr,
		    const char *buf, size_t count)
{
	ulong cycle;
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc313x_adc *adc = platform_get_drvdata(pdev);

	cycle = simple_strtoul(buf, NULL, 10);
	if (!cycle || cycle < 10)
		return -EINVAL;

	adc->cycle = cycle;
	return count;
}

/* For accessing/modifying the cycle time used between ADC samples */
static DEVICE_ATTR(cycle, S_IWUSR | S_IRUGO, show_cycle, store_cycle);

static const struct file_operations lpc313x_adc_fops = {
	.owner   = THIS_MODULE,
	.open    = lpc313x_adc_open,
	.release = lpc313x_adc_release,
	.read    = lpc313x_adc_read,
	.write   = lpc313x_adc_write,
};

ssize_t show_adc_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc313x_adc *adc = platform_get_drvdata(pdev);
	uint ch;
	u16 sample;
	int err;

	sscanf(attr->attr.name, LPC313X_ADC_SYSFS_NAME, &ch);

	err = channel_enable(adc, ch, LPC313X_ADC_RESOLUTION_MAX);
	if (err)
		goto err_exit;

	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(msecs_to_jiffies(2 * adc->cycle));

	down_read(&adc->lock);
	sample = adc->samples[ch];
	up_read(&adc->lock);

	channel_disable(adc, ch);

	return scnprintf(buf, PAGE_SIZE, "%u\n", sample);

err_exit:
	return err;
}

static int create_adc_sysfs(struct device *dev, struct lpc313x_adc *adc)
{
	int cnt;
	char *name;
	int err = 0;
	struct attribute *attr;

	err = device_create_file(dev, &dev_attr_cycle);
	if (err) {
		pr_err("Couldn't create sysfs file, %i\n", err);
		return err;
	}

	for (cnt = 0; cnt < adc->channels; cnt++) {
		name = kmalloc(sizeof(LPC313X_ADC_SYSFS_NAME) + 1, GFP_KERNEL);
		if (!name) {
			err = -ENOMEM;
			goto err_free;
		}

		snprintf(name, sizeof(LPC313X_ADC_SYSFS_NAME),
			 LPC313X_ADC_SYSFS_NAME, cnt);

		adc->attrs[cnt].show = show_adc_value;
		adc->names[cnt] = name;

		attr = &adc->attrs[cnt].attr;
		sysfs_attr_init(attr);
		attr->name = name;
		attr->mode = S_IRUGO;

		err = device_create_file(dev, &adc->attrs[cnt]);
		if (err) {
			kfree(name);
			adc->names[cnt] = NULL;
			goto err_free;
		}
	}

	return 0;

err_free:
	while (--cnt >= 0) {
		device_remove_file(dev, &adc->attrs[cnt]);
		kfree(adc->names[cnt]);
	}

	return err;
}

static void release_adc_sysfs(struct device *dev, struct lpc313x_adc *adc)
{
	int cnt;

	device_remove_file(dev, &dev_attr_cycle);

	for (cnt = 0; cnt < adc->channels; cnt++) {
		device_remove_file(dev, &adc->attrs[cnt]);
		kfree(adc->names[cnt]);
	}
}

static int lpc313x_adc_probe(struct platform_device *pdev)
{
	int err;
	struct resource *res;
	struct lpc313x_adc *adc;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		err = -ENOENT;
		goto err_end;
	}

	if (!request_mem_region(res->start, resource_size(res), LPC313X_DRV_NAME)) {
		err = -EBUSY;
		goto err_end;
	}

	adc = kzalloc(sizeof(* adc), GFP_KERNEL);
	if (!adc) {
		pr_err("Allocating ADC structure\n");
		err = -ENOMEM;
		goto err_release;
	}

	adc->io = ioremap(res->start, resource_size(res));
	if (!adc->io) {
		err = -ENOMEM;
		goto err_free;
	}

	adc->iobase = res->start;
	adc->iosize = resource_size(res);

	/* XXX This info should come from platform data */
	adc->channels = LPC313X_ADC_CHANNELS_MAX;
	adc->minor    = 0;

	if (drv_major) {
		adc->devnr = MKDEV(drv_major, adc->minor);
		err = register_chrdev_region(adc->devnr, adc->channels,
					     LPC313X_DRV_NAME);
	} else {
		err = alloc_chrdev_region(&adc->devnr, adc->minor, adc->channels,
					  LPC313X_DRV_NAME);
		if (!err)
			drv_major = MAJOR(adc->devnr);
	}

	if (err)
		goto err_unmap;

	/* Create attributes for all the channels */
	err = create_adc_sysfs(&pdev->dev, adc);
	if (err)
		goto unreg;

	init_rwsem(&adc->lock);
	sema_init(&adc->lock_hw, 1);
	adc->cycle = LPC313X_ADC_CYCLE_MS;

	cdev_init(&adc->cdev, &lpc313x_adc_fops);
	cdev_add(&adc->cdev, adc->devnr, 1);

	adc->kthread = kthread_run(lpc313x_adc_kthread, adc, "adc%i", adc->minor);

	platform_set_drvdata(pdev, adc);

	pr_info("New ADC %p channels %u major %u minor %u\n",
		adc, adc->channels, MAJOR(adc->devnr), adc->minor);
	return 0;

unreg:
	unregister_chrdev_region(adc->devnr, adc->channels);

err_unmap:
	iounmap(adc->io);

err_free:
	kfree(adc);

err_release:
	release_mem_region(res->start, resource_size(res));

err_end:
	return err;
}

static int lpc313x_adc_remove(struct platform_device *pdev)
{
	struct lpc313x_adc *adc = platform_get_drvdata(pdev);

	if (adc->kthread)
		kthread_stop(adc->kthread);

	cdev_del(&adc->cdev);
	unregister_chrdev_region(adc->devnr, adc->channels);

	release_adc_sysfs(&pdev->dev, adc);

	platform_set_drvdata(pdev, NULL);

	iounmap(adc->io);
	release_mem_region(adc->iobase, adc->iosize);

	kfree(adc);
	return 0;
}

static struct platform_driver lpc313x_adc_driver = {
	.probe  = lpc313x_adc_probe,
	.remove = lpc313x_adc_remove,
	.driver = {
		.name   = LPC313X_DRV_NAME,
		.owner  = THIS_MODULE,
	},
};

module_platform_driver(lpc313x_adc_driver);
