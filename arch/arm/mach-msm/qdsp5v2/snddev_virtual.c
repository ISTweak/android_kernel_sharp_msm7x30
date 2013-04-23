/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <asm/uaccess.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/qdsp5v2/snddev_virtual.h>
#include <mach/debug_mm.h>
#include <linux/slab.h>

/* SH_AUDIO_DRIVER -> */
#include <linux/pm_qos_params.h>
#include <mach/clk.h>

static struct pm_qos_request_list *pqos = NULL;

static int add_axi_qos(void)
{
	int rc = 0;
	MM_DBG("[%s:%s]add_axi_qos()\n", __FILE__, __func__);
	
	if(pqos){
		pm_qos_update_request(pqos, PM_QOS_DEFAULT_VALUE);
		pqos = NULL;
	}

	pqos = pm_qos_add_request(PM_QOS_SYSTEM_BUS_FREQ, PM_QOS_DEFAULT_VALUE);

	if(!pqos){
		rc = -1;
		MM_ERR("[%s:%s]request AXI bus QOS fails. Error\n", __FILE__, __func__);
	}

	return rc;
}

static void update_axi_qos(uint32_t rate)
{
	if(pqos){
		pm_qos_update_request(pqos, rate);
	}
	return;
}
/* SH_AUDIO_DRIVER <- */

static int snddev_virtual_open(struct msm_snddev_info *dev_info)
{
	int rc = 0;

	if (!dev_info)
		rc = -EINVAL;
/* SH_AUDIO_DRIVER -> */
	if( rc == 0 )
		update_axi_qos(MSM_AXI_MAX_FREQ);
/* SH_AUDIO_DRIVER <- */
	return rc;
}

static int snddev_virtual_close(struct msm_snddev_info *dev_info)
{
	int rc = 0;

/* SH_AUDIO_DRIVER -> */
	update_axi_qos(PM_QOS_DEFAULT_VALUE);
/* SH_AUDIO_DRIVER <- */
	if (!dev_info)
		rc = -EINVAL;
	return rc;
}

static int snddev_virtual_set_freq(struct msm_snddev_info *dev_info, u32 rate)
{
	int rc = 0;

	if (!dev_info)
		rc = -EINVAL;
	return rate;
}

static int snddev_virtual_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct snddev_virtual_data *pdata;
	struct msm_snddev_info *dev_info;

	if (!pdev || !pdev->dev.platform_data) {
		MM_ERR("Invalid caller\n");
		rc = -EPERM;
		goto error;
	}
	pdata = pdev->dev.platform_data;

	dev_info = kmalloc(sizeof(struct msm_snddev_info), GFP_KERNEL);
	if (!dev_info) {
		rc = -ENOMEM;
		goto error;
	}

	dev_info->name = pdata->name;
	dev_info->copp_id = pdata->copp_id;
	dev_info->acdb_id = pdata->acdb_id;
	dev_info->private_data = (void *) NULL;
	dev_info->dev_ops.open = snddev_virtual_open;
	dev_info->dev_ops.close = snddev_virtual_close;
	dev_info->dev_ops.set_freq = snddev_virtual_set_freq;
	dev_info->capability = pdata->capability;
	dev_info->sample_rate = 8000;
	dev_info->opened = 0;
	dev_info->sessions = 0;

	msm_snddev_register(dev_info);

error:
	return rc;
}

static int snddev_virtual_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver snddev_virtual_driver = {
	.probe = snddev_virtual_probe,
	.remove = snddev_virtual_remove,
	.driver = { .name = "snddev_virtual" }
};

static int __init snddev_virtual_init(void)
{
	int rc = 0;

	MM_DBG(" snddev_virtual_init \n");
	rc = platform_driver_register(&snddev_virtual_driver);
	if (IS_ERR_VALUE(rc)) {
		MM_ERR("platform driver register failure\n");
		return -ENODEV;
	}
/* SH_AUDIO_DRIVER -> */
	add_axi_qos();
/* SH_AUDIO_DRIVER <- */
	return 0;
}

static void __exit snddev_virtual_exit(void)
{
	platform_driver_unregister(&snddev_virtual_driver);

	return;
}

module_init(snddev_virtual_init);
module_exit(snddev_virtual_exit);

MODULE_DESCRIPTION("Virtual Sound Device driver");
MODULE_LICENSE("GPL v2");
