/* drivers/usb/gadget/f_sh_mtp.c
 *
 * Gadget Driver for Android MTP
 *
 * Copyright (C) 2011 SHARP CORPORATION
 *
 * This code borrows from f_adb.c, which is
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>

#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include <linux/switch.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/android_composite.h>

#include <sharp/sh_android_usb_desc_str.h>


#define USB_MTP_IOC_MAGIC 0xFF

#define USB_MTP_FUNC_IOC_GET_DEVICE_STATUS_SET _IOW(USB_MTP_IOC_MAGIC, 0x26, int)
#define USB_MTP_FUNC_IOC_FORCE_DISCONNECT      _IO(USB_MTP_IOC_MAGIC, 0x28)

#define USB_DIR_MASK	USB_DIR_IN

#define BULK_RX_BUFFER_SIZE           512
#define BULK_TX_BUFFER_SIZE           4096

/* number of rx and tx requests to allocate */
#define RX_REQ_MAX 32
#define TX_REQ_MAX 4

#define D_MTP_TRANSPORT_NAME	"android_mtp"
#define D_MTP_CONTROL_NAME		"android_mtp_control"
#define D_MTP_FUNCTION_NAME		"mtp"

/* MTP setup class requests */
#define	USB_MTP_CANCEL_REQUEST				(0x64)
#define	USB_MTP_GET_EXTENDED_DATA			(0x65)
#define	USB_MTP_DEVICE_RESET_REQUEST		(0x66)
#define	USB_MTP_GET_DEVICE_STATUS			(0x67)

#define	USB_MTP_NTY_CANCEL_REQUEST				(1)
#define	USB_MTP_NTY_GET_EXTENDED_DATA			(2)
#define	USB_MTP_NTY_DEVICE_RESET_REQUEST		(3)
#define	USB_MTP_NTY_GET_DEVICE_STATUS			(4)
#define	USB_MTP_NTY_ONLINE_STATE				(5)
#define	USB_MTP_NTY_OFFLINE_STATE				(6)

#define	USB_CTRL_NTY_MAX_STR_LEN	(48)
#define	USB_CANCEL_REQUEST_STR		"Cancel_Request"
#define	USB_GET_DEVICE_STATUS_STR	"Get_Device_Status"
#define	USB_DEVICE_RESET_STR		"Device_Reset"

#define	USB_ONLINE_STATE_CHANGE_STR		"online"
#define	USB_OFFLINE_STATE_CHANGE_STR	"offline"


struct mtp_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;
	struct usb_ep *ep_intr;

	atomic_t online;
	atomic_t error;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;

	struct list_head tx_idle;
	struct list_head rx_idle;
	struct list_head rx_done;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;

	/* the request we're currently reading from */
	struct usb_request *read_req;
	unsigned char *read_buf;
	unsigned read_count;

	/* for mtp_control */
	atomic_t open_ctrl_excl;
	atomic_t ctrl_read_excl;
	wait_queue_head_t ctrl_read_wq;
	unsigned char ctrl_set_size;
	unsigned char *ctrl_read_buf;

	int cancel_request;
	struct switch_dev sdev;
};

static struct usb_interface_descriptor mtp_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 3,
	.bInterfaceClass        = USB_CLASS_STILL_IMAGE,
	.bInterfaceSubClass     = 0x01,
	.bInterfaceProtocol     = 0x01,
	/* .iInterface = DYNAMIC */
};

static struct usb_endpoint_descriptor mtp_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor mtp_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor mtp_highspeed_notify_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize         = __constant_cpu_to_le16(64),
	.bInterval              = 16,
};

static struct usb_endpoint_descriptor mtp_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor mtp_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor mtp_fullspeed_notify_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize         = __constant_cpu_to_le16(64),
	.bInterval              = 32,
};

static struct usb_descriptor_header *fs_mtp_descs[] = {
	(struct usb_descriptor_header *) &mtp_interface_desc,
	(struct usb_descriptor_header *) &mtp_fullspeed_in_desc,
	(struct usb_descriptor_header *) &mtp_fullspeed_out_desc,
	(struct usb_descriptor_header *) &mtp_fullspeed_notify_in_desc,
	NULL,
};

static struct usb_descriptor_header *hs_mtp_descs[] = {
	(struct usb_descriptor_header *) &mtp_interface_desc,
	(struct usb_descriptor_header *) &mtp_highspeed_in_desc,
	(struct usb_descriptor_header *) &mtp_highspeed_out_desc,
	(struct usb_descriptor_header *) &mtp_highspeed_notify_in_desc,
	NULL,
};

void ep0_setup_send_delay_response(u8 *buf ,unsigned length);

/* string descriptors: */
static struct usb_string mtp_string_defs[] = {
	[0].s = USB_MTP_STRING_DESC_WORD,
	{  } /* end of list */
};

static struct usb_gadget_strings mtp_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		mtp_string_defs,
};

static struct usb_gadget_strings *mtp_strings[] = {
	&mtp_string_table,
	NULL,
};

/* Module */
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_LICENSE("GPL");


/* temporary variable used between mtp_open() and mtp_gadget_bind() */
static struct mtp_dev *_mtp_dev;

static inline struct mtp_dev *func_to_dev(struct usb_function *f)
{
	return container_of(f, struct mtp_dev, function);
}

static inline int usb_ept_get_max_packet(struct usb_ep *ep) 
{
	return ep->maxpacket;
}

static struct usb_request *mtp_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	/* now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void mtp_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int _lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void _unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

/* add a request to the tail of a list */
void mtp_req_put(struct mtp_dev *dev, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

/* remove a request from the head of a list */
struct usb_request *mtp_req_get(struct mtp_dev *dev, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&dev->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return req;
}

static void mtp_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct mtp_dev *dev = _mtp_dev;

	if (req->status != 0)
		atomic_set(&dev->error, 1);

	mtp_req_put(dev, &dev->tx_idle, req);

	wake_up(&dev->write_wq);
}

static void mtp_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct mtp_dev *dev = _mtp_dev;
	if (req->status != 0) {
		atomic_set(&dev->error, 1);
		mtp_req_put(dev, &dev->rx_idle, req);
	} else {
		dev->cancel_request = 0;
		mtp_req_put(dev, &dev->rx_done, req);
	}

	wake_up(&dev->read_wq);
}

static int create_bulk_endpoints(struct mtp_dev *dev,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc,
				struct usb_endpoint_descriptor *intr_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

	DBG(cdev, "create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for ep_in got %s\n", ep->name);
	dev->ep_in = ep;
	dev->ep_in->driver_data = cdev;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for mtp ep_out got %s\n", ep->name);
	dev->ep_out = ep;
	dev->ep_out->driver_data = cdev;

	ep = usb_ep_autoconfig(cdev->gadget, intr_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_intr failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for mtp ep_intr got %s\n", ep->name);
	dev->ep_intr = ep;
	dev->ep_intr->driver_data = cdev;

	/* now allocate requests for our endpoints */
	for (i = 0; i < RX_REQ_MAX; i++) {
		req = mtp_request_new(dev->ep_out, BULK_RX_BUFFER_SIZE);
		if (!req)
			goto fail;
		req->complete = mtp_complete_out;
		mtp_req_put(dev, &dev->rx_idle, req);
	}

	for (i = 0; i < TX_REQ_MAX; i++) {
		req = mtp_request_new(dev->ep_in, BULK_TX_BUFFER_SIZE);
		if (!req)
			goto fail;
		req->complete = mtp_complete_in;
		mtp_req_put(dev, &dev->tx_idle, req);
	}

	return 0;

fail:
	printk(KERN_ERR "mtp_bind() could not allocate requests\n");
	return -1;
}

static ssize_t mtp_read(struct file *fp, char __user *buf,
				size_t count, loff_t *pos)
{
	struct mtp_dev *dev = fp->private_data;
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	int r = count, xfer;
	int ret;
	int	timeout_onflg = 0;	/* timeout-flg */

	DBG(cdev, "mtp_read(%d)\n", count);

	if (_lock(&dev->read_excl))
		return -EBUSY;

	/* we will block until we're online */
	while (!(atomic_read(&dev->online) || atomic_read(&dev->error))) {
		DBG(cdev, "mtp_read: waiting for online state\n");
		ret = wait_event_interruptible(dev->read_wq,
			(atomic_read(&dev->online) ||
			atomic_read(&dev->error)));
		if (ret < 0) {
			_unlock(&dev->read_excl);
			return ret;
		}
		else if (!atomic_read(&dev->online)) {
			_unlock(&dev->read_excl);
			return -EIO;
		}
	}

	while (count > 0) {
		if (atomic_read(&dev->error)) {
			DBG(cdev, "mtp_read dev->error\n");
			r = -EIO;
			break;
		}

		/* if we have idle read requests, get them queued */
		while ((req = mtp_req_get(dev, &dev->rx_idle))) {
requeue_req:
			req->length = usb_ept_get_max_packet(dev->ep_out);
			if (req->length > BULK_RX_BUFFER_SIZE)
				req->length = BULK_RX_BUFFER_SIZE;

			ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);

			if (ret < 0) {
				r = -EIO;
				atomic_set(&dev->error, 1);
				mtp_req_put(dev, &dev->rx_idle, req);
				goto fail;
			} else {
				DBG(cdev, "rx %p queue\n", req);
			}
		}

		/* if we have data pending, give it to userspace */
		if (dev->read_count > 0) {
			if (dev->read_count < count)
				xfer = dev->read_count;
			else
				xfer = count;

			if (copy_to_user(buf, dev->read_buf, xfer)) {
				r = -EFAULT;
				break;
			}
			dev->read_buf += xfer;
			dev->read_count -= xfer;
			buf += xfer;
			count -= xfer;

			/* if we've emptied the buffer, release the request */
			if (dev->read_count == 0) {
				mtp_req_put(dev, &dev->rx_idle, dev->read_req);
				dev->read_req = 0;
				timeout_onflg = 1;
				/* for short packet  */
				if( (r - count) % usb_ept_get_max_packet(dev->ep_out) != 0) {
					r = r - count;
					break;
				}
			}
			continue;
		}

		/* wait for a request to complete */
		req = 0;
		if( timeout_onflg == 0 )
			ret = wait_event_interruptible(dev->read_wq,
				((req = mtp_req_get(dev, &dev->rx_done)) ||
				 atomic_read(&dev->error)));
		else
			ret = wait_event_interruptible_timeout(dev->read_wq,
				((req = mtp_req_get(dev, &dev->rx_done)) ||
				 atomic_read(&dev->error)) , HZ / 10);

		if (req != 0) {
			/* if we got a 0-len one we need to put it back into
			** service.  if we made it the current read req we'd
			** be stuck forever
			*/
			if (req->actual == 0)
				goto requeue_req;

			dev->read_req = req;
			dev->read_count = req->actual;
			dev->read_buf = req->buf;
			DBG(cdev, "rx %p %d\n", req, req->actual);
		}
		else if((ret == 0) && (!atomic_read(&dev->error))) { /* for timeout */
			r = r - count;
			break;
		}

		if (ret < 0) {
			r = ret;
			break;
		}
	}

fail:
	if (dev->cancel_request == 1) {
		dev->read_buf = 0;
		dev->read_count = 0;
		if (dev->read_req) {
			mtp_req_put(dev, &dev->rx_idle, dev->read_req);
			dev->read_req = 0;
		}
	}

	_unlock(&dev->read_excl);
	/* interrupts are handled as errs */
	if (r == -ERESTARTSYS)
		r = -EIO;
	DBG(cdev, "mtp_read returning %d\n", r);
	return r;
}

static ssize_t mtp_write(struct file *fp, const char __user *buf,
				 size_t count, loff_t *pos)
{
	struct mtp_dev *dev = fp->private_data;
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;

	DBG(cdev, "mtp_write(%d)\n", count);

	if (_lock(&dev->write_excl))
		return -EBUSY;

	if (!atomic_read(&dev->online)) {
		_unlock(&dev->write_excl);
		return -EIO;
	}

	while (count > 0) {
		if (dev->cancel_request) {
			r = -EAGAIN;
			break;
		}
		if (atomic_read(&dev->error)) {
			DBG(cdev, "mtp_write dev->error\n");
			r = -EIO;
			break;
		}

		/* get an idle tx request to use */
		req = 0;
		ret = wait_event_interruptible(dev->write_wq,
			(atomic_read(&dev->error) ||
			 (req = mtp_req_get(dev, &dev->tx_idle))));

		if (ret < 0) {
			r = ret;
			break;
		}

		if (req != 0) {
			if (count > BULK_TX_BUFFER_SIZE)
				xfer = BULK_TX_BUFFER_SIZE;
			else
				xfer = count;
			if (copy_from_user(req->buf, buf, xfer)) {
				r = -EFAULT;
				break;
			}

			req->length = xfer;
			ret = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
			if (ret < 0) {
				DBG(cdev, "mtp_write: xfer error %d\n", ret);
				atomic_set(&dev->error, 1);
				r = -EIO;
				break;
			}

			buf += xfer;
			count -= xfer;

			/* zero this so we don't try to free it on error exit */
			req = 0;
		}
	}

	if (req)
		mtp_req_put(dev, &dev->tx_idle, req);

	_unlock(&dev->write_excl);
	/* interrupts are handled as errs */
	if (r == -ERESTARTSYS)
		r = -EIO;
	DBG(cdev, "mtp_write returning %d\n", r);
	return r;
}

static int mtp_open(struct inode *ip, struct file *fp)
{
	if (!_mtp_dev)
		return -EIO;

	if (_lock(&_mtp_dev->open_excl))
		return -EBUSY;

	fp->private_data = _mtp_dev;

	/* clear the error latch */
	atomic_set(&_mtp_dev->error, 0);

	return 0;
}

static int mtp_release(struct inode *ip, struct file *fp)
{
	struct mtp_dev *dev = fp->private_data;
	struct usb_request *req_out;

	/* retire any completed rx requests from previous session */
	while ((req_out = mtp_req_get(dev, &dev->rx_done)))
			mtp_req_put(dev, &dev->rx_idle, req_out);
	if (_mtp_dev)
		_unlock(&_mtp_dev->open_excl);
	return 0;
}

/* file operations for ADB device /dev/android_mtp */
static struct file_operations mtp_fops = {
	.owner = THIS_MODULE,
	.read = mtp_read,
	.write = mtp_write,
	.open = mtp_open,
	.release = mtp_release,
};

static struct miscdevice mtp_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = D_MTP_TRANSPORT_NAME,
	.fops = &mtp_fops,
};

static void mtp_control_notify( unsigned char type,void* opt )
{
	unsigned long flags;

	if (!_mtp_dev)
		return;

	spin_lock_irqsave(&_mtp_dev->lock, flags);
	if(!_mtp_dev->ctrl_read_buf) {
		spin_unlock_irqrestore(&_mtp_dev->lock, flags);
		return;
	}

	switch(type) {
	case USB_MTP_NTY_CANCEL_REQUEST:
		_mtp_dev->ctrl_set_size = strlen(USB_CANCEL_REQUEST_STR);
		memcpy(_mtp_dev->ctrl_read_buf, USB_CANCEL_REQUEST_STR , _mtp_dev->ctrl_set_size );
		break;
	case USB_MTP_NTY_GET_DEVICE_STATUS:
		_mtp_dev->ctrl_set_size = strlen(USB_GET_DEVICE_STATUS_STR);
		memcpy(_mtp_dev->ctrl_read_buf, USB_GET_DEVICE_STATUS_STR , _mtp_dev->ctrl_set_size );
		break;
	case USB_MTP_NTY_DEVICE_RESET_REQUEST:
		_mtp_dev->ctrl_set_size = strlen(USB_DEVICE_RESET_STR);
		memcpy(_mtp_dev->ctrl_read_buf, USB_DEVICE_RESET_STR , _mtp_dev->ctrl_set_size );
		break;
	case USB_MTP_NTY_ONLINE_STATE:
		_mtp_dev->ctrl_set_size = strlen(USB_ONLINE_STATE_CHANGE_STR);
		memcpy(_mtp_dev->ctrl_read_buf, USB_ONLINE_STATE_CHANGE_STR , _mtp_dev->ctrl_set_size );
		break;
	case USB_MTP_NTY_OFFLINE_STATE:
		_mtp_dev->ctrl_set_size = strlen(USB_OFFLINE_STATE_CHANGE_STR);
		memcpy(_mtp_dev->ctrl_read_buf, USB_OFFLINE_STATE_CHANGE_STR , _mtp_dev->ctrl_set_size );
		break;
	default:
		spin_unlock_irqrestore(&_mtp_dev->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&_mtp_dev->lock, flags);

	wake_up(&_mtp_dev->ctrl_read_wq);

	return;
}


static ssize_t mtp_control_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	int size;
	int ret = 0;
	unsigned long flags;

	if (!_mtp_dev)
		return -EIO;

	if (_lock(&_mtp_dev->ctrl_read_excl))
		return -EBUSY;

	ret = wait_event_interruptible(_mtp_dev->ctrl_read_wq, _mtp_dev->ctrl_set_size );
	if (ret < 0) {
		_unlock(&_mtp_dev->ctrl_read_excl);
		return ret;
	}

	if( count < _mtp_dev->ctrl_set_size ) {
		_unlock(&_mtp_dev->ctrl_read_excl);
		return -EFAULT;
	}

	spin_lock_irqsave(&_mtp_dev->lock, flags);
	if (copy_to_user(buf, _mtp_dev->ctrl_read_buf, _mtp_dev->ctrl_set_size)) {
		size =  -EFAULT;
	}
	else {
		size = _mtp_dev->ctrl_set_size;
		_mtp_dev->ctrl_set_size = 0;
	}
	spin_unlock_irqrestore(&_mtp_dev->lock, flags);

	_unlock(&_mtp_dev->ctrl_read_excl);

	return size;
}

static int mtp_control_open(struct inode *ip, struct file *fp)
{
	if (!_mtp_dev)
		return -EIO;

	if (_lock(&_mtp_dev->open_ctrl_excl))
		return -EBUSY;

	fp->private_data = _mtp_dev;

	_mtp_dev->ctrl_set_size = 0;
	if( _mtp_dev->ctrl_read_buf )
		kfree(_mtp_dev->ctrl_read_buf);
	_mtp_dev->ctrl_read_buf = kzalloc( USB_CTRL_NTY_MAX_STR_LEN, GFP_KERNEL);

	if (!_mtp_dev->ctrl_read_buf) {
		_unlock(&_mtp_dev->open_ctrl_excl);
		return -ENOMEM;
	}

	if( atomic_read( &_mtp_dev->online ) )
		mtp_control_notify(USB_MTP_NTY_ONLINE_STATE , NULL);
	else
		mtp_control_notify(USB_MTP_NTY_OFFLINE_STATE , NULL);

	return 0;
}

static int mtp_control_release(struct inode *ip, struct file *fp)
{
	unsigned long flags;

	if (!_mtp_dev)
		return 0;

	_unlock(&_mtp_dev->open_ctrl_excl);

	spin_lock_irqsave(&_mtp_dev->lock, flags);
	_mtp_dev->ctrl_set_size = 0;
	if( _mtp_dev->ctrl_read_buf ) {
		kfree(_mtp_dev->ctrl_read_buf);
		_mtp_dev->ctrl_read_buf = 0;
	}
	spin_unlock_irqrestore(&_mtp_dev->lock, flags);

	return 0;
}

static int mtp_control_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	struct mtp_dev *dev = file->private_data;
	struct usb_composite_dev *cdev = dev->cdev;
	int ret = 0;
	unsigned short sts[2];

	if (!_mtp_dev)
		return -EIO;

	switch (cmd) {
	case USB_MTP_FUNC_IOC_GET_DEVICE_STATUS_SET:
		DBG(cdev, "mtp_control_ioctl USB_MTP_FUNC_IOC_GET_DEVICE_STATUS_SET\n");
		sts[0] = 0x04;					/* length */
		sts[1] = (unsigned short)arg;	/* code */

		/* send setup */
		ep0_setup_send_delay_response((u8 *)sts,sizeof(sts));
		
		if ( sts[1] == 0x2001 ) {
			DBG(cdev, "mtp_control_ioctl:reset cancel_request \n");
			if (atomic_read(&_mtp_dev->read_excl)) {
				int ret_xfer;
				struct usb_request *req = 0;

				req = mtp_req_get(_mtp_dev, &_mtp_dev->rx_idle);
				if (req) {
					req->length = usb_ept_get_max_packet(dev->ep_out);
					if (req->length > BULK_RX_BUFFER_SIZE)
						req->length = BULK_RX_BUFFER_SIZE;

					ret_xfer = usb_ep_queue(_mtp_dev->ep_out, req, GFP_ATOMIC);
					if (ret_xfer < 0) {
						atomic_set(&_mtp_dev->error, 1);
						mtp_req_put(_mtp_dev, &_mtp_dev->rx_idle, req);
					}
				}
			}
		}
		break;
	case USB_MTP_FUNC_IOC_FORCE_DISCONNECT:
		if (atomic_read(&_mtp_dev->read_excl))
			_mtp_dev->function.disable(&_mtp_dev->function);
		if (atomic_read(&_mtp_dev->ctrl_read_excl))
			mtp_control_notify(USB_MTP_NTY_OFFLINE_STATE , NULL);
		break;
	default:
		return -ENOTTY;
	}

	return ret;
}

static struct file_operations mtp_control_fops = {
	.owner =   THIS_MODULE,
	.read =    mtp_control_read,
	.open =    mtp_control_open,
	.release = mtp_control_release,
	.ioctl =   mtp_control_ioctl,
};

static struct miscdevice mtp_control_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = D_MTP_CONTROL_NAME,
	.fops = &mtp_control_fops,
};

static void
mtp_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct mtp_dev	*dev = func_to_dev(f);
	struct usb_request *req;

	while ((req = mtp_req_get(dev, &dev->rx_idle)))
		mtp_request_free(req, dev->ep_out);
	while ((req = mtp_req_get(dev, &dev->tx_idle)))
		mtp_request_free(req, dev->ep_in);
}

static int
mtp_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct mtp_dev	*dev = func_to_dev(f);
	int			id;
	int			ret;

	dev->cdev = cdev;
	DBG(cdev, "mtp_function_bind dev: %p\n", dev);

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	mtp_interface_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = create_bulk_endpoints(dev, &mtp_fullspeed_in_desc,
			&mtp_fullspeed_out_desc,
			&mtp_fullspeed_notify_in_desc);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		mtp_highspeed_in_desc.bEndpointAddress =
			mtp_fullspeed_in_desc.bEndpointAddress;
		mtp_highspeed_out_desc.bEndpointAddress =
			mtp_fullspeed_out_desc.bEndpointAddress;
		mtp_highspeed_notify_in_desc.bEndpointAddress =
			mtp_fullspeed_notify_in_desc.bEndpointAddress;
	}

	DBG(cdev, "%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name, dev->ep_out->name);
	return 0;
}

void mtp_function_exit(void)
{
	misc_deregister(&mtp_control_device);
	misc_deregister(&mtp_device);
	if (_mtp_dev) {
		kfree(_mtp_dev);
		_mtp_dev = NULL;
	}
}


static int mtp_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct mtp_dev	*dev = func_to_dev(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;
	struct usb_request *req;

	DBG(cdev, "mtp_function_set_alt intf: %d alt: %d\n", intf, alt);
	
	if( atomic_read(&dev->online) )
		return 0;
	
	ret = usb_ep_enable(dev->ep_in,
			ep_choose(cdev->gadget,
				&mtp_highspeed_in_desc,
				&mtp_fullspeed_in_desc));
	if (ret)
		return ret;
	ret = usb_ep_enable(dev->ep_out,
			ep_choose(cdev->gadget,
				&mtp_highspeed_out_desc,
				&mtp_fullspeed_out_desc));
	if (ret) {
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	ret = usb_ep_enable(dev->ep_intr,
			ep_choose(cdev->gadget,
				&mtp_highspeed_notify_in_desc,
				&mtp_fullspeed_notify_in_desc));
	if (ret) {
		usb_ep_disable(dev->ep_in);
		usb_ep_disable(dev->ep_out);
		return ret;
	}

	atomic_set(&dev->online, 1);
	dev->cancel_request = 0;

	atomic_set(&dev->error, 0);
	if (atomic_read(&_mtp_dev->read_excl)) {
		req = mtp_req_get(dev, &dev->rx_idle);
		if (req) {
			req->length = usb_ept_get_max_packet(dev->ep_out);
			if (req->length > BULK_RX_BUFFER_SIZE)
				req->length = BULK_RX_BUFFER_SIZE;
			ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
			if (ret < 0) {
				printk(KERN_ERR "mtp_function_set_alt() could not get requests\n");
				atomic_set(&dev->error, 1);
				mtp_req_put(dev, &dev->rx_idle, req);
			}
		}
	}
	mtp_control_notify(USB_MTP_NTY_ONLINE_STATE , NULL);

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);
	return 0;
}

static void mtp_function_disable(struct usb_function *f)
{
	struct mtp_dev	*dev = func_to_dev(f);
	struct usb_composite_dev	*cdev = dev->cdev;

	DBG(cdev, "mtp_function_disable\n");

	if( !cdev )
		return;

	atomic_set(&dev->online, 0);
	atomic_set(&dev->error, 1);
	wake_up(&dev->read_wq);

	usb_ep_fifo_flush(dev->ep_in);
	usb_ep_fifo_flush(dev->ep_out);
	usb_ep_fifo_flush(dev->ep_intr);
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);
	usb_ep_disable(dev->ep_intr);

	if((usb_gadget_is_vbus_connected(cdev->gadget) == 1)
		&& (dev->function.disabled == 0) ){
		return;
	}
	
	mtp_control_notify(USB_MTP_NTY_OFFLINE_STATE , NULL);
	
	VDBG(cdev, "%s disabled\n", dev->function.name);
}

int mtp_function_setup(struct usb_function *f,
	const struct usb_ctrlrequest *ctrl)
{
	struct mtp_dev	*dev = func_to_dev(f);
	struct usb_composite_dev	*cdev = dev->cdev;
	struct usb_request	*req = cdev->req;
	int			value = -EOPNOTSUPP;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);
	struct usb_request *req_out;

	DBG(cdev, "mtp_function_setup\n");
	if (w_index != mtp_interface_desc.bInterfaceNumber)
		return value;

	/* Handle Bulk-only class-specific requests */
	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_CLASS) {
		DBG(cdev, "USB_TYPE_CLASS\n");
		switch (ctrl->bRequest) {
		case USB_MTP_CANCEL_REQUEST:		/* Cancel Request */
			if ((ctrl->bRequestType & (USB_DIR_MASK)) != (USB_DIR_OUT))
				break;
			mtp_control_notify(USB_MTP_NTY_CANCEL_REQUEST,NULL);

			dev->cancel_request = 1;

			/* flush out-transaction */
			usb_ep_fifo_flush(dev->ep_out);

			/* if we have a stale request being read, recycle it */
			if (!atomic_read(&_mtp_dev->read_excl)) {
				dev->read_buf = 0;
				dev->read_count = 0;
				if (dev->read_req) {
					mtp_req_put(dev, &dev->rx_idle, dev->read_req);
					dev->read_req = 0;
				}
			}

			/* retire any completed rx requests from previous session */
			while ((req_out = mtp_req_get(dev, &dev->rx_done)))
				mtp_req_put(dev, &dev->rx_idle, req_out);

			usb_ep_fifo_flush(dev->ep_in);
			wake_up(&dev->write_wq);

			atomic_set(&dev->error, 0);

			/* We need to queue a request to read the remaining
			 *  bytes, but we don't actually need to look at
			 * the contents.
			 */
			value = w_length;

			break;
		case USB_MTP_GET_EXTENDED_DATA:		/* Get Extended Data */
			DBG(cdev, "mtp_setup USB_MTP_GET_EXTENDED_DATA\n");
			if ((ctrl->bRequestType & (USB_DIR_MASK)) != (USB_DIR_IN))
				break;
			break;
		case USB_MTP_DEVICE_RESET_REQUEST:	/* Device Reset Request */
			DBG(cdev,"mtp_setup USB_MTP_DEVICE_RESET_REQUEST\n");
			mtp_control_notify(USB_MTP_NTY_DEVICE_RESET_REQUEST,NULL);
			value = 0;
			break;
		case USB_MTP_GET_DEVICE_STATUS:		/* Get Device Status */
			DBG(cdev,"mtp_setup USB_MTP_GET_DEVICE_STATUS\n");
			if ((ctrl->bRequestType & (USB_DIR_MASK)) != (USB_DIR_IN))
				break;
			DBG(cdev,"mtp_setup USB_MTP_GET_DEVICE_STATUS OK\n");
			mtp_control_notify(USB_MTP_NTY_GET_DEVICE_STATUS,NULL);
			value = D_USB_GADGET_SETUP_PENDING;
			break;
		default:
			DBG(cdev,"mtp_setup other\n");
			break;
		}
	}

	if (value == -EOPNOTSUPP)
		VDBG(cdev,
			"unknown class-specific control req "
			"%02x.%02x v%04x i%04x l%u\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	if (value >= 0) {
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			req->status = 0;
	}
	return value;
}

static void mtp_reset_descriptor(struct usb_configuration *c, 
                      struct usb_function *f,  
                      struct usb_descriptor_header **descriptors ,
                      u8 bInterfaceNumber)
{
	mtp_interface_desc.bInterfaceNumber = bInterfaceNumber;
	return;
}


static int mtp_bind_config(struct usb_configuration *c)
{
	struct mtp_dev *dev;
	int ret;

	/* maybe allocate device-global string ID */
	if (mtp_string_defs[0].id == 0) {
		ret = usb_string_id(c->cdev);
		if (ret < 0)
			return ret;
		mtp_string_defs[0].id = ret;
		mtp_interface_desc.iInterface = ret;
	}

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	spin_lock_init(&dev->lock);

	init_waitqueue_head(&dev->read_wq);
	init_waitqueue_head(&dev->write_wq);

	atomic_set(&dev->open_excl, 0);
	atomic_set(&dev->read_excl, 0);
	atomic_set(&dev->write_excl, 0);

	INIT_LIST_HEAD(&dev->rx_idle);
	INIT_LIST_HEAD(&dev->rx_done);
	INIT_LIST_HEAD(&dev->tx_idle);

	/* reset any added variaties */
	atomic_set(&dev->open_ctrl_excl, 0);
	atomic_set(&dev->ctrl_read_excl, 0);

	init_waitqueue_head(&dev->ctrl_read_wq);

	dev->ctrl_set_size = 0;
	dev->ctrl_read_buf = NULL;

	dev->cdev = c->cdev;
	dev->function.name = D_MTP_FUNCTION_NAME;
	dev->function.descriptors = fs_mtp_descs;
	dev->function.hs_descriptors = hs_mtp_descs;
	dev->function.bind = mtp_function_bind;
	dev->function.unbind = mtp_function_unbind;
	dev->function.set_alt = mtp_function_set_alt;
	dev->function.disable = mtp_function_disable;

	/* ADD */
	dev->function.strings = mtp_strings;
	dev->function.setup = mtp_function_setup;
	dev->function.reset_descriptor = mtp_reset_descriptor;

	/* _mtp_dev must be set before calling usb_gadget_register_driver */
	_mtp_dev = dev;

	ret = misc_register(&mtp_device);
	if (ret)
		goto err1;

	//register for mtp_control
	ret = misc_register(&mtp_control_device);
	if (ret)
		goto err2;

	ret = usb_add_function(c, &dev->function);
	if (ret)
		goto err3;

	return ret;
err3:
	misc_deregister(&mtp_control_device);
err2:
	misc_deregister(&mtp_device);
err1:
	kfree(dev);
	printk(KERN_ERR "mtp gadget driver failed to initialize\n");
	return ret;
}

static struct android_usb_function mtp_function = {
	.name = D_MTP_FUNCTION_NAME,
	.bind_config = mtp_bind_config,
};

static int __init init(void)
{
	android_register_function(&mtp_function);
	return 0;
}
module_init(init);
