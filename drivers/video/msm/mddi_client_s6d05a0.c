/*
 * Copyright (C) 2008 HTC Corporation.
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
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <mach/msm_fb.h>
#include <mach/debug_display.h>

static DECLARE_WAIT_QUEUE_HEAD(samsung_vsync_wait);

struct panel_info {
	struct msm_mddi_client_data *client_data;
	struct platform_device pdev;
	struct msm_panel_data panel_data;
	struct msmfb_callback *samsung_callback;
	struct wake_lock idle_lock;
	int samsung_got_int;
	int vsync_gpio;
};

static struct platform_device mddi_samsung_cabc = {
	.name = "icong-backlight",
	.id = 0,
};

static void samsung_request_vsync(struct msm_panel_data *panel_data,
				  struct msmfb_callback *callback)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;

	panel->samsung_callback = callback;
	if (panel->samsung_got_int) {
		panel->samsung_got_int = 0;
		client_data->activate_link(client_data);
	}
}

static void samsung_clear_vsync(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;

	client_data->activate_link(client_data);
}

static void samsung_wait_vsync(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;

	if (panel->samsung_got_int) {
		panel->samsung_got_int = 0;
		client_data->activate_link(client_data); /* clears interrupt */
	}
	if (wait_event_timeout(samsung_vsync_wait, panel->samsung_got_int,
				HZ/2) == 0)
		PR_DISP_ERR("timeout waiting for VSYNC\n");
	panel->samsung_got_int = 0;
	/* interrupt clears when screen dma starts */
}

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
		PR_DISP_INFO("mddi samsung client: non zero return from "
			"uninit\n");
		return ret;
	}
	client_data->suspend(client_data);
	return 0;
}

static int samsung_resume(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;

	struct msm_mddi_bridge_platform_data *bridge_data =
		client_data->private_client_data;
	int ret;

	wake_lock(&panel->idle_lock);
	client_data->resume(client_data);
	wake_unlock(&panel->idle_lock);
	ret = bridge_data->init(bridge_data, client_data);
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

	panel->samsung_got_int = 1;
	if (panel->samsung_callback) {
		panel->samsung_callback->func(panel->samsung_callback);
		panel->samsung_callback = 0;
	}
	wake_up(&samsung_vsync_wait);
	return IRQ_HANDLED;
}

static int setup_vsync(struct panel_info *panel,
		       int init)
{
	int ret;
	int gpio = panel->vsync_gpio;
	unsigned int irq;

	if (!init) {
		ret = 0;
		goto uninit;
	}
	ret = gpio_request(gpio, "vsync");
	if (ret)
		goto err_request_gpio_failed;

	ret = gpio_direction_input(gpio);
	if (ret)
		goto err_gpio_direction_input_failed;

	ret = irq = gpio_to_irq(gpio);
	if (ret < 0)
		goto err_get_irq_num_failed;

	register_gpio_int_mask(gpio, 1);

	ret = request_irq(irq, samsung_vsync_interrupt, IRQF_TRIGGER_RISING,
			  "vsync", panel);
	if (ret)
		goto err_request_irq_failed;
	PR_DISP_INFO("vsync on gpio %d now %d\n",
	       gpio, gpio_get_value(gpio));
	return 0;

uninit:
	free_irq(gpio_to_irq(gpio), panel);
err_request_irq_failed:
err_get_irq_num_failed:
err_gpio_direction_input_failed:
	gpio_free(gpio);
err_request_gpio_failed:
	return ret;
}

static int mddi_samsung_probe(struct platform_device *pdev)
{
	int ret;
	struct msm_mddi_client_data *client_data = pdev->dev.platform_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
		client_data->private_client_data;
	struct panel_data *panel_data = &bridge_data->panel_conf;
	struct panel_info *panel =
		kzalloc(sizeof(struct panel_info), GFP_KERNEL);

	PR_DISP_DEBUG("%s\n", __func__);

	if (!panel)
		return -ENOMEM;
	platform_set_drvdata(pdev, panel);

	if (panel_data->caps & MSMFB_CAP_CABC) {
		PR_DISP_INFO("CABC enabled\n");
		mddi_samsung_cabc.dev.platform_data = client_data;
		platform_device_register(&mddi_samsung_cabc);
	}

	if (panel_data->vsync_gpio == 0)
		panel->vsync_gpio = 97;
	else
		panel->vsync_gpio = panel_data->vsync_gpio;

	ret = setup_vsync(panel, 1);
	if (ret) {
		dev_err(&pdev->dev, "mddi_bridge_setup_vsync failed\n");
		return ret;
	}
	panel->client_data = client_data;
	panel->panel_data.suspend = samsung_suspend;
	panel->panel_data.resume = samsung_resume;
	panel->panel_data.wait_vsync = samsung_wait_vsync;
	panel->panel_data.request_vsync = samsung_request_vsync;
	panel->panel_data.clear_vsync = samsung_clear_vsync;
	panel->panel_data.blank = samsung_blank;
	panel->panel_data.unblank = samsung_unblank;
	panel->panel_data.fb_data =  &bridge_data->fb_data;
	panel->panel_data.caps = MSMFB_CAP_PARTIAL_UPDATES;
	panel->pdev.name = "msm_panel";
	panel->pdev.id = pdev->id;
	panel->pdev.resource = client_data->fb_resource;
	panel->pdev.num_resources = 1;
	panel->pdev.dev.platform_data = &panel->panel_data;
	platform_device_register(&panel->pdev);
	wake_lock_init(&panel->idle_lock, WAKE_LOCK_IDLE, "samsung_idle_lock");

	return 0;
}

static int mddi_samsung_remove(struct platform_device *pdev)
{
	struct panel_info *panel = platform_get_drvdata(pdev);

	setup_vsync(panel, 0);
	kfree(panel);
	return 0;
}

static struct platform_driver mddi_client_0101_05a0 = {
	.probe = mddi_samsung_probe,
	.remove = mddi_samsung_remove,
	.driver = { .name = "mddi_c_0101_05a0" },
};

static int __init mddi_client_samsung_init(void)
{
	return platform_driver_register(&mddi_client_0101_05a0);
}

module_init(mddi_client_samsung_init);

