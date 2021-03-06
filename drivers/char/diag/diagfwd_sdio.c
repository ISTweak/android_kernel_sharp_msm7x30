/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/diagchar.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <asm/current.h>
#ifdef CONFIG_DIAG_OVER_USB
#include <mach/usbdiag.h>
#endif
#include "diagchar_hdlc.h"
#include "diagmem.h"
#include "diagchar.h"
#include "diagfwd.h"
#include "diagfwd_sdio.h"


void __diag_sdio_send_req(void)
{
	int r = 0;
	void *buf = driver->buf_in_sdio;

	if (driver->sdio_ch && (!driver->in_busy_sdio)) {
		r = sdio_read_avail(driver->sdio_ch);

		if (r > IN_BUF_SIZE) {
			if (r < MAX_IN_BUF_SIZE) {
				printk(KERN_ALERT "\n diag: SDIO sending"
					  " in packets more than %d bytes", r);
				buf = krealloc(buf, r, GFP_KERNEL);
			} else {
				printk(KERN_ALERT "\n diag: SDIO sending"
			  " in packets more than %d bytes", MAX_IN_BUF_SIZE);
				return;
			}
		}
		if (r > 0) {
			if (!buf)
				printk(KERN_INFO "Out of diagmem for SDIO\n");
			else {
				APPEND_DEBUG('i');
				sdio_read(driver->sdio_ch, buf, r);
				APPEND_DEBUG('j');
				driver->write_ptr_mdm->length = r;
				driver->in_busy_sdio = 1;
				diag_device_write(buf, SDIO_DATA,
						 driver->write_ptr_mdm);
			}
		}
	}
}

static void diag_read_sdio_work_fn(struct work_struct *work)
{
	__diag_sdio_send_req();
}

int diagfwd_connect_sdio(void)
{
	int err;

#ifdef CONFIG_SH_USB_CUST
	err = -1;
#else /* CONFIG_SH_USB_CUST */
	err = usb_diag_alloc_req(driver->mdm_ch, N_MDM_WRITE,
							 N_MDM_READ);
#endif /* CONFIG_SH_USB_CUST */
	if (err)
		printk(KERN_ERR "diag: unable to alloc USB req on mdm ch");
	driver->in_busy_sdio = 0;

	/* Poll USB channel to check for data*/
	queue_work(driver->diag_sdio_wq, &(driver->diag_read_mdm_work));
	/* Poll SDIO channel to check for data*/
	queue_work(driver->diag_sdio_wq, &(driver->diag_read_sdio_work));
	return 0;
}

int diagfwd_disconnect_sdio(void)
{
	driver->in_busy_sdio = 1;
#ifdef CONFIG_SH_USB_CUST
//	diag_close();
#else /* CONFIG_SH_USB_CUST */
	usb_diag_free_req(driver->mdm_ch);
#endif /* CONFIG_SH_USB_CUST */
	return 0;
}

int diagfwd_write_complete_sdio(void)
{
	driver->in_busy_sdio = 0;
	APPEND_DEBUG('q');
	queue_work(driver->diag_sdio_wq, &(driver->diag_read_sdio_work));
	return 0;
}

int diagfwd_read_complete_sdio(void)
{
	queue_work(driver->diag_sdio_wq, &(driver->diag_read_mdm_work));
	return 0;
}

void diag_read_mdm_work_fn(struct work_struct *work)
{
	if (driver->sdio_ch) {
		wait_event_interruptible(driver->wait_q, (sdio_write_avail
				(driver->sdio_ch) >= driver->read_len_mdm));
		if (driver->sdio_ch && driver->usb_buf_mdm_out &&
						 (driver->read_len_mdm > 0))
			sdio_write(driver->sdio_ch, driver->usb_buf_mdm_out,
							 driver->read_len_mdm);
		APPEND_DEBUG('x');
		driver->usb_read_mdm_ptr->buf = driver->usb_buf_mdm_out;
		driver->usb_read_mdm_ptr->length = USB_MAX_OUT_BUF;
#ifdef CONFIG_SH_USB_CUST
		diag_read(driver->usb_read_mdm_ptr);
#else /* CONFIG_SH_USB_CUST */
		usb_diag_read(driver->mdm_ch, driver->usb_read_mdm_ptr);
#endif /* CONFIG_SH_USB_CUST */
		APPEND_DEBUG('y');
	}
}

#ifdef CONFIG_SH_USB_CUST
/* diag_sdio_notify is not used */
#else /* CONFIG_SH_USB_CUST */
static void diag_sdio_notify(void *ctxt, unsigned event)
{
	if (event == SDIO_EVENT_DATA_READ_AVAIL)
		queue_work(driver->diag_sdio_wq,
				 &(driver->diag_read_sdio_work));

	if (event == SDIO_EVENT_DATA_WRITE_AVAIL)
		wake_up_interruptible(&driver->wait_q);
}
#endif /* CONFIG_SH_USB_CUST */

static int diag_sdio_probe(struct platform_device *pdev)
{
	int err;

#ifdef CONFIG_SH_USB_CUST
	err = -1;
#else /* CONFIG_SH_USB_CUST */
	err = sdio_open("SDIO_DIAG", &driver->sdio_ch, driver,
							 diag_sdio_notify);
#endif /* CONFIG_SH_USB_CUST */
	if (err)
		printk(KERN_INFO "DIAG could not open SDIO channel");
	else {
		printk(KERN_INFO "DIAG opened SDIO channel");
		queue_work(driver->diag_sdio_wq, &(driver->diag_read_mdm_work));
	}

	return err;
}

static int diagfwd_sdio_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int diagfwd_sdio_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static const struct dev_pm_ops diagfwd_sdio_dev_pm_ops = {
	.runtime_suspend = diagfwd_sdio_runtime_suspend,
	.runtime_resume = diagfwd_sdio_runtime_resume,
};

static struct platform_driver msm_sdio_ch_driver = {
	.probe = diag_sdio_probe,
	.driver = {
		   .name = "SDIO_DIAG",
		   .owner = THIS_MODULE,
		   .pm   = &diagfwd_sdio_dev_pm_ops,
		   },
};

void diagfwd_sdio_init(void)
{
	int ret;

	driver->read_len_mdm = 0;
	if (driver->buf_in_sdio == NULL)
		driver->buf_in_sdio = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_sdio == NULL)
			goto err;
	if (driver->usb_buf_mdm_out  == NULL)
		driver->usb_buf_mdm_out = kzalloc(USB_MAX_OUT_BUF, GFP_KERNEL);
		if (driver->usb_buf_mdm_out == NULL)
			goto err;
	if (driver->write_ptr_mdm == NULL)
		driver->write_ptr_mdm = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->write_ptr_mdm == NULL)
			goto err;
	if (driver->usb_read_mdm_ptr == NULL)
		driver->usb_read_mdm_ptr = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->usb_read_mdm_ptr == NULL)
			goto err;
	driver->diag_sdio_wq = create_singlethread_workqueue("diag_sdio_wq");
#ifdef CONFIG_DIAG_OVER_USB
#ifdef CONFIG_SH_USB_CUST
	driver->mdm_ch = 0;
#else /* CONFIG_SH_USB_CUST */
	driver->mdm_ch = usb_diag_open(DIAG_MDM, driver,
			diag_usb_legacy_notifier);
	if (IS_ERR(driver->mdm_ch)) {
		printk(KERN_ERR "Unable to open USB diag MDM channel\n");
		goto err;
	}
#endif /* CONFIG_SH_USB_CUST */
	INIT_WORK(&(driver->diag_read_mdm_work), diag_read_mdm_work_fn);
#endif
	INIT_WORK(&(driver->diag_read_sdio_work), diag_read_sdio_work_fn);
	ret = platform_driver_register(&msm_sdio_ch_driver);
	if (ret)
		printk(KERN_INFO "DIAG could not register SDIO device");
	else
		printk(KERN_INFO "DIAG registered SDIO device");

	return;
err:
		printk(KERN_INFO "\n Could not initialize diag buf for SDIO");
		kfree(driver->buf_in_sdio);
		kfree(driver->usb_buf_mdm_out);
		kfree(driver->write_ptr_mdm);
		kfree(driver->usb_read_mdm_ptr);
		if (driver->diag_sdio_wq)
			destroy_workqueue(driver->diag_sdio_wq);
}

void diagfwd_sdio_exit(void)
{
#ifdef CONFIG_DIAG_OVER_USB
#ifdef CONFIG_SH_USB_CUST
//	if (driver->usb_connected)
//		diag_close();
#else /* CONFIG_SH_USB_CUST */
	if (driver->usb_connected)
		usb_diag_free_req(driver->mdm_ch);
#endif /* CONFIG_SH_USB_CUST */
#endif
	platform_driver_unregister(&msm_sdio_ch_driver);
#ifdef CONFIG_DIAG_OVER_USB
#ifdef CONFIG_SH_USB_CUST
//	diag_usb_unregister();
#else /* CONFIG_SH_USB_CUST */
	usb_diag_close(driver->mdm_ch);
#endif /* CONFIG_SH_USB_CUST */
#endif
	kfree(driver->buf_in_sdio);
	kfree(driver->usb_buf_mdm_out);
	kfree(driver->write_ptr_mdm);
	kfree(driver->usb_read_mdm_ptr);
	destroy_workqueue(driver->diag_sdio_wq);
}
