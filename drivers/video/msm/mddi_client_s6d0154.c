/* drivers/video/msm_fb/mddi_client_eid0154.c
 *
 * Support for eid mddi client devices with Samsung S6D0154
 *
 * Copyright (C) 2007 HTC Incorporated
 * Author: Jay Tu (jay_tu@htc.com)
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/clk.h>
#include <asm/atomic.h>
#include <asm/mach-types.h>
#include "../../../arch/arm/mach-msm/proc_comm.h"
#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>
#define printk(arg,...)
#if 0
#define B(s...) printk(s)
#else
#define B(s...) do {} while (0)
#endif

#define DEBUG_VSYNC_INT 0
#define VSYNC_COUNTER 31

#define VSYNC_CLEAR (MSM_GPIO1_BASE + 0x800 + 0x9c)
#define VSYNC_STATUS (MSM_GPIO1_BASE + 0x800 + 0xac)
#define VSYNC_EN (MSM_GPIO1_BASE + 0x800 + 0x8c)

static DECLARE_WAIT_QUEUE_HEAD(samsung_vsync_wait);

struct panel_info {
	struct msm_mddi_client_data *client_data;
	struct platform_device pdev;
	struct msm_panel_data panel_data;
	struct msmfb_callback *fb_callback;
	struct wake_lock idle_lock;
	int samsung_got_int;
	atomic_t depth;
	int vsync_counter;
};

static struct clk *ebi1_clk;

static void
samsung_sendout_region(struct panel_info *panel)
{
	struct msm_mddi_client_data *client = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge =
						client->private_client_data;

	if (bridge->adjust) {
		wake_lock(&panel->idle_lock);
		bridge->adjust(client);
		wake_unlock(&panel->idle_lock);
		panel->vsync_counter = VSYNC_COUNTER;
	}
	if (atomic_read(&panel->depth) <= 0) {
		atomic_inc(&panel->depth);
		enable_irq(gpio_to_irq(97));
	}
}

static void
samsung_update_framedata(struct panel_info *panel)
{
	if (panel->fb_callback) {
		panel->fb_callback->func(panel->fb_callback);
		panel->fb_callback = NULL;
		panel->samsung_got_int = 0;
	}
}

static void samsung_dump_vsync(void)
{
	printk(KERN_INFO "STATUS %d %s EBI1 %lu\n",
			readl(VSYNC_STATUS) & 0x04,
			readl(VSYNC_EN) & 0x04 ? "ENABLED" : "DISABLED",
			clk_get_rate(ebi1_clk));
}

static inline void samsung_clear_vsync(void)
{
	unsigned val;
	int retry = 1000;

	while (retry--) {
		writel((1U << (97 - 95)), VSYNC_CLEAR);
		wmb();
		val = readl(VSYNC_STATUS);
		if (!!(val & 0x04) == 0)
			break;
	}

	if (retry == 0)
		printk(KERN_ERR "%s: clear vsync failed!\n", __func__);
}

static void
samsung_request_vsync(struct msm_panel_data *panel_data,
		      struct msmfb_callback *callback)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;

	panel->fb_callback = callback;
	client_data->activate_link(client_data); /* clears interrupt */

	if (atomic_read(&panel->depth) <= 0) {
		atomic_inc(&panel->depth);
		samsung_clear_vsync();
		enable_irq(gpio_to_irq(97));
	}
}

static int samsung_wait_vsync(void *data)
{
	struct panel_info *panel = (struct panel_info *)data;

	int rc;
	panel->vsync_counter = VSYNC_COUNTER;

	while (!kthread_should_stop()) {
		rc = wait_event_interruptible(samsung_vsync_wait,
				panel->samsung_got_int &&
				panel->fb_callback);
		if (rc != 0)
			continue;

		--(panel->vsync_counter);

		if (panel->vsync_counter <= 0)
			samsung_sendout_region(panel);
		else
			samsung_update_framedata(panel);
	}
	do_exit(0);
}

/* -------------------------------------------------------------------------- */

/* got called by msmfb_suspend */
static int samsung_suspend(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;
	int ret;

	wake_lock(&panel->idle_lock);
	ret = bridge_data->uninit(bridge_data, client_data);
	wake_unlock(&panel->idle_lock);

	if (ret) {
		B(KERN_INFO "mddi samsung client: non zero return from "
		  "uninit\n");
		return ret;
	}
	client_data->suspend(client_data);
	return 0;
}

/* got called by msmfb_resume */
static int samsung_resume(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;
	int ret;

	client_data->resume(client_data);

	wake_lock(&panel->idle_lock);
	ret = bridge_data->init(bridge_data, client_data);
	wake_unlock(&panel->idle_lock);

	if (ret)
		return ret;
	return 0;
}

static int samsung_blank(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;

	return bridge_data->blank(bridge_data, client_data);
}

static int samsung_unblank(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;

	return bridge_data->unblank(bridge_data, client_data);
}

static irqreturn_t samsung_vsync_interrupt(int irq, void *data)
{
	struct panel_info *panel = data;

	if (atomic_read(&panel->depth) > 0) {
		atomic_dec(&panel->depth);
		disable_irq(gpio_to_irq(97));
	}

	panel->samsung_got_int = 1;
	wake_up(&samsung_vsync_wait);
	return IRQ_HANDLED;
}

static int setup_vsync(struct panel_info *panel, int init)
{
	int ret;
	int gpio = 97;
	uint32_t config;
	unsigned int irq;

	if (!init) {
		ret = 0;
		goto uninit;
	}
	ret = gpio_request(gpio, "vsync");
	if (ret)
		goto err_request_gpio_failed;

	config = PCOM_GPIO_CFG(97, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
	ret = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
	if (ret)
		goto err_gpio_direction_input_failed;

	ret = irq = gpio_to_irq(gpio);
	if (ret < 0)
		goto err_get_irq_num_failed;

	samsung_clear_vsync();
	ret = request_irq(irq, samsung_vsync_interrupt, IRQF_TRIGGER_HIGH,
			"vsync", panel);
	if (ret)
		goto err_request_irq_failed;
	disable_irq(irq);

	printk(KERN_INFO "vsync on gpio %d now %d\n", gpio,
			gpio_get_value(gpio));
	return 0;

uninit:
	free_irq(gpio_to_irq(gpio), panel->client_data);
err_request_irq_failed:
err_get_irq_num_failed:
err_gpio_direction_input_failed:
	gpio_free(gpio);
err_request_gpio_failed:
	return ret;
}

static int mddi_samsung_probe(struct platform_device *pdev)
{
	int ret, err = -EINVAL;
	struct panel_info *panel;
	struct msm_mddi_client_data *client_data = pdev->dev.platform_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;

	B(KERN_DEBUG "%s: enter\n", __func__);

	panel = kzalloc(sizeof(struct panel_info), GFP_KERNEL);
	if (!panel) {
		err = -ENOMEM;
		goto err_out;
	}

	platform_set_drvdata(pdev, panel);
	ret = setup_vsync(panel, 1);
	if (ret) {
		dev_err(&pdev->dev, "mddi_samsung_setup_vsync failed\n");
		err = -EIO;
		goto err_panel;
	}

	panel->client_data = client_data;
	panel->panel_data.suspend = samsung_suspend;
	panel->panel_data.resume = samsung_resume;
	panel->panel_data.request_vsync = samsung_request_vsync;
	panel->panel_data.blank = samsung_blank;
	panel->panel_data.unblank = samsung_unblank;
	panel->panel_data.dump_vsync = samsung_dump_vsync;
	panel->panel_data.fb_data = &bridge_data->fb_data;
	panel->panel_data.caps = ~MSMFB_CAP_PARTIAL_UPDATES;
	atomic_set(&panel->depth, 0);
	panel->pdev.name = "msm_panel";
	panel->pdev.id = pdev->id;
	panel->pdev.resource = client_data->fb_resource;
	panel->pdev.num_resources = 1;
	panel->pdev.dev.platform_data = &panel->panel_data;
	platform_device_register(&panel->pdev);
	wake_lock_init(&panel->idle_lock, WAKE_LOCK_IDLE, "eid_idle_lock");
	/*for debugging vsync issue*/
	ebi1_clk = clk_get(NULL, "ebi1_clk");

	kthread_run(samsung_wait_vsync, panel, "ksamsung");
	return 0;

err_panel:
	kfree(panel);
err_out:
	return err;
}

static int mddi_samsung_remove(struct platform_device *pdev)
{
	struct panel_info *panel = platform_get_drvdata(pdev);

	setup_vsync(panel, 0);
	kfree(panel);
	return 0;
}

static struct platform_driver mddi_client_0101_0000 = {
	.probe = mddi_samsung_probe,
	.remove = mddi_samsung_remove,
	.driver = {.name = "mddi_c_0101_0154"},
};

static int __init mddi_client_samsung_init(void)
{
	return platform_driver_register(&mddi_client_0101_0000);
}

module_init(mddi_client_samsung_init);
