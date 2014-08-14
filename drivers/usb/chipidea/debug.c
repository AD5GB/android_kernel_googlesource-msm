#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include "ci.h"
#include "udc.h"
#include "bits.h"
#include "debug.h"

/**
 * ci_device_show: prints information about device capabilities and status
 */
static int ci_device_show(struct seq_file *s, void *data)
{
	struct ci13xxx *ci = s->private;
	struct usb_gadget *gadget = &ci->gadget;

	seq_printf(s, "speed             = %d\n", gadget->speed);
	seq_printf(s, "max_speed         = %d\n", gadget->max_speed);
	seq_printf(s, "is_otg            = %d\n", gadget->is_otg);
	seq_printf(s, "is_a_peripheral   = %d\n", gadget->is_a_peripheral);
	seq_printf(s, "b_hnp_enable      = %d\n", gadget->b_hnp_enable);
	seq_printf(s, "a_hnp_support     = %d\n", gadget->a_hnp_support);
	seq_printf(s, "a_alt_hnp_support = %d\n", gadget->a_alt_hnp_support);
	seq_printf(s, "name              = %s\n",
		   (gadget->name ? gadget->name : ""));

	if (!ci->driver)
		return 0;

	seq_printf(s, "gadget function   = %s\n",
		       (ci->driver->function ? ci->driver->function : ""));
	seq_printf(s, "gadget max speed  = %d\n", ci->driver->max_speed);

	return 0;
}

static int ci_device_open(struct inode *inode, struct file *file)
{
	return single_open(file, ci_device_show, inode->i_private);
}

static const struct file_operations ci_device_fops = {
	.open		= ci_device_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/**
 * dbg_usb_op_fail: prints USB Operation FAIL event
 * @addr: endpoint address
 * @mEp:  endpoint structure
 */
void dbg_usb_op_fail(u8 addr, const char *name,
				const struct ci13xxx_ep *mEp)
{
	char msg[DBG_DATA_MSG];
	struct ci13xxx_req *req;
	struct list_head *ptr = NULL;

	if (mEp != NULL) {
		scnprintf(msg, sizeof(msg),
			"%s Fail EP%d%s QH:%08X",
			name, mEp->num,
			mEp->dir ? "IN" : "OUT", mEp->qh.ptr->cap);
		dbg_print(addr, name, 0, msg);
		scnprintf(msg, sizeof(msg),
				"cap:%08X %08X %08X\n",
				mEp->qh.ptr->curr, mEp->qh.ptr->td.next,
				mEp->qh.ptr->td.token);
		dbg_print(addr, "QHEAD", 0, msg);

		list_for_each(ptr, &mEp->qh.queue) {
			req = list_entry(ptr, struct ci13xxx_req, queue);
			scnprintf(msg, sizeof(msg),
					"%08X:%08X:%08X\n",
					req->dma, req->ptr->next,
					req->ptr->token);
			dbg_print(addr, "REQ", 0, msg);
			scnprintf(msg, sizeof(msg), "%08X:%d\n",
					req->ptr->page[0],
					req->req.status);
			dbg_print(addr, "REQPAGE", 0, msg);
		}
	}
}

/**
 * ci_port_test_show: reads port test mode
 */
static int ci_port_test_show(struct seq_file *s, void *data)
{
	*idx = (*idx + 1) & (DBG_DATA_MAX-1);
}

static unsigned int ep_addr_txdbg_mask;
module_param(ep_addr_txdbg_mask, uint, S_IRUGO | S_IWUSR);
static unsigned int ep_addr_rxdbg_mask;
module_param(ep_addr_rxdbg_mask, uint, S_IRUGO | S_IWUSR);

static int allow_dbg_print(u8 addr)
{
	int dir, num;

	/* allow bus wide events */
	if (addr == 0xff)
		return 1;

	dir = addr & USB_ENDPOINT_DIR_MASK ? TX : RX;
	num = addr & ~USB_ENDPOINT_DIR_MASK;
	num = 1 << num;

	if ((dir == TX) && (num & ep_addr_txdbg_mask))
		return 1;
	if ((dir == RX) && (num & ep_addr_rxdbg_mask))
		return 1;

	return 0;
}

/**
 * dbg_print:  prints the common part of the event
 * @addr:   endpoint address
 * @name:   event name
 * @status: status
 * @extra:  extra information
 */
static void dbg_print(u8 addr, const char *name, int status, const char *extra)
{
	struct timeval tval;
	unsigned int stamp;
	unsigned long flags;
	unsigned mode;

	if (!allow_dbg_print(addr))
		return;

	write_lock_irqsave(&dbg_data.lck, flags);

	do_gettimeofday(&tval);
	stamp = tval.tv_sec & 0xFFFF;	/* 2^32 = 4294967296. Limit to 4096s */
	stamp = stamp * 1000000 + tval.tv_usec;

	scnprintf(dbg_data.buf[dbg_data.idx], DBG_DATA_MSG,
		  "%04X\t? %02X %-7.7s %4i ?\t%s\n",
		  stamp, addr, name, status, extra);

	dbg_inc(&dbg_data.idx);

	seq_printf(s, "mode = %u\n", mode);

	return 0;
}

/**
 * ci_port_test_write: writes port test mode
 */
static ssize_t ci_port_test_write(struct file *file, const char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct ci13xxx *ci = s->private;
	unsigned long flags;
	unsigned mode;
	char buf[32];
	int ret;

	if (copy_from_user(buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (sscanf(buf, "%u", &mode) != 1)
		return -EINVAL;

	spin_lock_irqsave(&ci->lock, flags);
	ret = hw_port_test_set(ci, mode);
	spin_unlock_irqrestore(&ci->lock, flags);

	return ret ? ret : count;
}

static int ci_port_test_open(struct inode *inode, struct file *file)
{
	return single_open(file, ci_port_test_show, inode->i_private);
}

static const struct file_operations ci_port_test_fops = {
	.open		= ci_port_test_open,
	.write		= ci_port_test_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/**
 * dbg_usb_op_fail: prints USB Operation FAIL event
 * @addr: endpoint address
 * @mEp:  endpoint structure
 */
void dbg_usb_op_fail(u8 addr, const char *name,
				const struct ci13xxx_ep *mEp)
{
	char msg[DBG_DATA_MSG];
	struct ci13xxx_req *req;
	struct list_head *ptr = NULL;

	if (mEp != NULL) {
		scnprintf(msg, sizeof(msg),
			"%s Fail EP%d%s QH:%08X",
			name, mEp->num,
			mEp->dir ? "IN" : "OUT", mEp->qh.ptr->cap);
		dbg_print(addr, name, 0, msg);
		scnprintf(msg, sizeof(msg),
				"cap:%08X %08X %08X\n",
				mEp->qh.ptr->curr, mEp->qh.ptr->td.next,
				mEp->qh.ptr->td.token);
		dbg_print(addr, "QHEAD", 0, msg);

		list_for_each(ptr, &mEp->qh.queue) {
			req = list_entry(ptr, struct ci13xxx_req, queue);
			scnprintf(msg, sizeof(msg),
					"%08X:%08X:%08X\n",
					req->dma, req->ptr->next,
					req->ptr->token);
			dbg_print(addr, "REQ", 0, msg);
			scnprintf(msg, sizeof(msg), "%08X:%d\n",
					req->ptr->page[0],
					req->req.status);
			dbg_print(addr, "REQPAGE", 0, msg);
		}
	}
}

/**
 * show_events: displays the event buffer
 *
 * Check "device.h" for details
 */
static int ci_qheads_show(struct seq_file *s, void *data)
{
	struct ci13xxx *ci = s->private;
	unsigned long flags;
	unsigned i, j;

	if (ci->role != CI_ROLE_GADGET) {
		seq_printf(s, "not in gadget mode\n");
		return 0;
	}

	spin_lock_irqsave(&ci->lock, flags);
	for (i = 0; i < ci->hw_ep_max/2; i++) {
		struct ci13xxx_ep *mEpRx = &ci->ci13xxx_ep[i];
		struct ci13xxx_ep *mEpTx =
			&ci->ci13xxx_ep[i + ci->hw_ep_max/2];
		seq_printf(s, "EP=%02i: RX=%08X TX=%08X\n",
			   i, (u32)mEpRx->qh.dma, (u32)mEpTx->qh.dma);
		for (j = 0; j < (sizeof(struct ci13xxx_qh)/sizeof(u32)); j++)
			seq_printf(s, " %04X:    %08X    %08X\n", j,
				   *((u32 *)mEpRx->qh.ptr + j),
				   *((u32 *)mEpTx->qh.ptr + j));
	}
	spin_unlock_irqrestore(&ci->lock, flags);

	return 0;
}

static int ci_qheads_open(struct inode *inode, struct file *file)
{
	return single_open(file, ci_qheads_show, inode->i_private);
}

static const struct file_operations ci_qheads_fops = {
	.open		= ci_qheads_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/**
 * ci_requests_show: DMA contents of all requests currently queued (all endpts)
 */
static int ci_requests_show(struct seq_file *s, void *data)
{
	struct ci13xxx *ci = s->private;
	unsigned long flags;
	struct list_head   *ptr = NULL;
	struct ci13xxx_req *req = NULL;
	unsigned i, j, qsize = sizeof(struct ci13xxx_td)/sizeof(u32);

	if (ci->role != CI_ROLE_GADGET) {
		seq_printf(s, "not in gadget mode\n");
		return 0;
	}

	spin_lock_irqsave(&ci->lock, flags);
	for (i = 0; i < ci->hw_ep_max; i++)
		list_for_each(ptr, &ci->ci13xxx_ep[i].qh.queue) {
			req = list_entry(ptr, struct ci13xxx_req, queue);

			seq_printf(s, "EP=%02i: TD=%08X %s\n",
				   i % (ci->hw_ep_max / 2), (u32)req->dma,
				   ((i < ci->hw_ep_max/2) ? "RX" : "TX"));

			for (j = 0; j < qsize; j++)
				seq_printf(s, " %04X:    %08X\n", j,
					   *((u32 *)req->ptr + j));
		}
	spin_unlock_irqrestore(&ci->lock, flags);

	return 0;
}

static int ci_requests_open(struct inode *inode, struct file *file)
{
	return single_open(file, ci_requests_show, inode->i_private);
}

static const struct file_operations ci_requests_fops = {
	.open		= ci_requests_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int ci_role_show(struct seq_file *s, void *data)
{
	struct ci13xxx *ci = s->private;

	seq_printf(s, "%s\n", ci_role(ci)->name);

	return 0;
}

static ssize_t ci_role_write(struct file *file, const char __user *ubuf,
			     size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct ci13xxx *ci = s->private;
	enum ci_role role;
	char buf[8];
	int ret;

	if (copy_from_user(buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	for (role = CI_ROLE_HOST; role < CI_ROLE_END; role++)
		if (ci->roles[role] &&
		    !strncmp(buf, ci->roles[role]->name,
			     strlen(ci->roles[role]->name)))
			break;

	if (role == CI_ROLE_END || role == ci->role)
		return -EINVAL;

	ci_role_stop(ci);
	ret = ci_role_start(ci, role);

	return ret ? ret : count;
}

static int ci_role_open(struct inode *inode, struct file *file)
{
	return single_open(file, ci_role_show, inode->i_private);
}

static const struct file_operations ci_role_fops = {
	.open		= ci_role_open,
	.write		= ci_role_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/* EP# and Direction */
static ssize_t ci_prime_write(struct file *file, const char __user *ubuf,
			     size_t count, loff_t *ppos)
{
	struct ci13xxx *ci = file->private;
	struct ci13xxx_ep *mEp;
	unsigned int ep_num, dir;
	int n;
	struct ci13xxx_req *mReq = NULL;

	if (sscanf(buf, "%u %u", &ep_num, &dir) != 2) {
		dev_err(dev, "<ep_num> <dir>: prime the ep");
		goto done;
	}

	if (dir)
		mEp = &ci->ci13xxx_ep[ep_num + hw_ep_max/2];
	else
		mEp = &ci->ci13xxx_ep[ep_num];

	n = hw_ep_bit(mEp->num, mEp->dir);
	mReq =  list_entry(mEp->qh.queue.next, struct ci13xxx_req, queue);
	mEp->qh.ptr->td.next   = mReq->dma;
	mEp->qh.ptr->td.token &= ~TD_STATUS;

	wmb();

	hw_write(ci, OP_ENDPTPRIME, BIT(n), BIT(n));
	while (hw_read(ci, OP_ENDPTPRIME, BIT(n)))
		cpu_relax();

	pr_info("%s: prime:%08x stat:%08x ep#%d dir:%s\n", __func__,
			hw_read(ci, OP_ENDPTPRIME, ~0),
			hw_read(ci, OP_ENDPTSTAT, ~0),
			mEp->num, mEp->dir ? "IN" : "OUT");
done:
	return count;

}

static const struct file_operations ci_prime_fops = {
	.open		= simple_open,
	.write		= ci_prime_write,
};


/* EP# and Direction */
static ssize_t ci_dtds_write(struct file *file, const char __user *ubuf,
			     size_t count, loff_t *ppos)
{
	struct ci13xxx *ci = file->private;
	struct ci13xxx_ep *mEp;
	unsigned int ep_num, dir;
	int n;
	struct list_head   *ptr = NULL;
	struct ci13xxx_req *req = NULL;

	if (sscanf(buf, "%u %u", &ep_num, &dir) != 2) {
		dev_err(dev, "<ep_num> <dir>: to print dtds");
		goto done;
	}

	if (dir)
		mEp = &ci->ci13xxx_ep[ep_num + hw_ep_max/2];
	else
		mEp = &ci->ci13xxx_ep[ep_num];

	n = hw_ep_bit(mEp->num, mEp->dir);
	pr_info("%s: prime:%08x stat:%08x ep#%d dir:%s"
			"dTD_update_fail_count: %lu "
			"mEp->dTD_update_fail_count: %lu"
			"mEp->prime_fail_count: %lu\n", __func__,
			hw_read(ci, OP_ENDPTPRIME, ~0),
			hw_read(ci, OP_ENDPTSTAT, ~0),
			mEp->num, mEp->dir ? "IN" : "OUT",
			udc->dTD_update_fail_count,
			mEp->dTD_update_fail_count,
			mEp->prime_fail_count);

	pr_info("QH: cap:%08x cur:%08x next:%08x token:%08x\n",
			mEp->qh.ptr->cap, mEp->qh.ptr->curr,
			mEp->qh.ptr->td.next, mEp->qh.ptr->td.token);

	list_for_each(ptr, &mEp->qh.queue) {
		req = list_entry(ptr, struct ci13xxx_req, queue);

		pr_info("\treq:%08x next:%08x token:%08x page0:%08x status:%d\n",
				req->dma, req->ptr->next, req->ptr->token,
				req->ptr->page[0], req->req.status);
	}
done:
	return count;

}

static const struct file_operations ci_dtds_fops = {
	.open		= simple_open,
	.write		= ci_dtds_write,
};

static ssize_t ci_wakeup_write(struct file *file, const char __user *ubuf,
			     size_t count, loff_t *ppos)
{
	struct ci13xxx *ci = file->private;

	ci13xxx_wakeup(&ci->gadget);

	return count;
}

static const struct file_operations ci_wakeup_fops = {
	.open		= simple_open,
	.write		= ci_role_write,
};

/* EP# and Direction */
static ssize_t prime_ept(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct ci13xxx *ci = container_of(dev, struct ci13xxx, gadget.dev);
	struct ci13xxx_ep *mEp;
	unsigned int ep_num, dir;
	int n;
	struct ci13xxx_req *mReq = NULL;

	if (sscanf(buf, "%u %u", &ep_num, &dir) != 2) {
		dev_err(dev, "<ep_num> <dir>: prime the ep");
		goto done;
	}

	if (dir)
		mEp = &ci->ci13xxx_ep[ep_num + hw_ep_max/2];
	else
		mEp = &ci->ci13xxx_ep[ep_num];

	n = hw_ep_bit(mEp->num, mEp->dir);
	mReq =  list_entry(mEp->qh.queue.next, struct ci13xxx_req, queue);
	mEp->qh.ptr->td.next   = mReq->dma;
	mEp->qh.ptr->td.token &= ~TD_STATUS;

	wmb();

	hw_cwrite(CAP_ENDPTPRIME, BIT(n), BIT(n));
	while (hw_cread(CAP_ENDPTPRIME, BIT(n)))
		cpu_relax();

	pr_info("%s: prime:%08x stat:%08x ep#%d dir:%s\n", __func__,
			hw_cread(CAP_ENDPTPRIME, ~0),
			hw_cread(CAP_ENDPTSTAT, ~0),
			mEp->num, mEp->dir ? "IN" : "OUT");
done:
	return count;

}
static DEVICE_ATTR(prime, S_IWUSR, NULL, prime_ept);

/* EP# and Direction */
static ssize_t print_dtds(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct ci13xxx *ci = container_of(dev, struct ci13xxx, gadget.dev);
	struct ci13xxx_ep *mEp;
	unsigned int ep_num, dir;
	int n;
	struct list_head   *ptr = NULL;
	struct ci13xxx_req *req = NULL;

	if (sscanf(buf, "%u %u", &ep_num, &dir) != 2) {
		dev_err(dev, "<ep_num> <dir>: to print dtds");
		goto done;
	}

	if (dir)
		mEp = &ci->ci13xxx_ep[ep_num + hw_ep_max/2];
	else
		mEp = &ci->ci13xxx_ep[ep_num];

	n = hw_ep_bit(mEp->num, mEp->dir);
	pr_info("%s: prime:%08x stat:%08x ep#%d dir:%s"
			"dTD_update_fail_count: %lu "
			"mEp->dTD_update_fail_count: %lu"
			"mEp->prime_fail_count: %lu\n", __func__,
			hw_read(ci, OP_ENDPTPRIME, ~0),
			hw_read(ci, OP_ENDPTSTAT, ~0),
			mEp->num, mEp->dir ? "IN" : "OUT",
			udc->dTD_update_fail_count,
			mEp->dTD_update_fail_count,
			mEp->prime_fail_count);

	pr_info("QH: cap:%08x cur:%08x next:%08x token:%08x\n",
			mEp->qh.ptr->cap, mEp->qh.ptr->curr,
			mEp->qh.ptr->td.next, mEp->qh.ptr->td.token);

	list_for_each(ptr, &mEp->qh.queue) {
		req = list_entry(ptr, struct ci13xxx_req, queue);

		pr_info("\treq:%08x next:%08x token:%08x page0:%08x status:%d\n",
				req->dma, req->ptr->next, req->ptr->token,
				req->ptr->page[0], req->req.status);
	}
done:
	return count;

}
static DEVICE_ATTR(dtds, S_IWUSR, NULL, print_dtds);

static ssize_t usb_remote_wakeup(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ci13xxx *ci = container_of(dev, struct ci13xxx, gadget.dev);

	ci13xxx_wakeup(&ci->gadget);

	return count;
}
static DEVICE_ATTR(wakeup, S_IWUSR, 0, usb_remote_wakeup);

/**
 * dbg_create_files: initializes the attribute interface
 * @ci: device
 *
 * This function returns an error code
 */
int dbg_create_files(struct ci13xxx *ci)
{
	struct dentry *dent;

	ci->debugfs = debugfs_create_dir(dev_name(ci->dev), NULL);
	if (!ci->debugfs)
		return -ENOMEM;

	dent = debugfs_create_file("device", S_IRUGO, ci->debugfs, ci,
				   &ci_device_fops);
	if (!dent)
		goto err;

	dent = debugfs_create_file("port_test", S_IRUGO | S_IWUSR, ci->debugfs,
				   ci, &ci_port_test_fops);
	if (!dent)
		goto err;

	dent = debugfs_create_file("qheads", S_IRUGO, ci->debugfs, ci,
				   &ci_qheads_fops);
	if (!dent)
		goto err;

	dent = debugfs_create_file("requests", S_IRUGO, ci->debugfs, ci,
				   &ci_requests_fops);
	if (!dent)
		goto err;

	retval = debugfs_create_file("wakeup", S_IWUSR, ci->debugfs, ci,
				   &ci_wakeup_fops);
	if (retval)
		goto err;

	retval = debugfs_create_file("prime", S_IWUSR, ci->debugfs, ci,
				   &ci_prime_fops);
	if (retval)
		goto err;

	retval = debugfs_create_file("dtds", S_IWUSR, ci->debugfs, ci,
				   &ci_dtds_fops);
	if (retval)
		goto rm_registers;
	retval = device_create_file(dev, &dev_attr_wakeup);
	if (retval)
		goto rm_requests;
	retval = device_create_file(dev, &dev_attr_prime);
	if (retval)
		goto rm_wakeup;
	retval = device_create_file(dev, &dev_attr_dtds);
	if (retval)
		goto rm_prime;
	return 0;

 rm_prime:
	device_remove_file(dev, &dev_attr_prime);
 rm_wakeup:
	device_remove_file(dev, &dev_attr_wakeup);
 rm_requests:
	device_remove_file(dev, &dev_attr_requests);
 rm_registers:
	device_remove_file(dev, &dev_attr_registers);
 rm_qheads:
	device_remove_file(dev, &dev_attr_qheads);
 rm_port_test:
	device_remove_file(dev, &dev_attr_port_test);
 rm_inters:
	device_remove_file(dev, &dev_attr_inters);
 rm_events:
	device_remove_file(dev, &dev_attr_events);
 rm_driver:
	device_remove_file(dev, &dev_attr_driver);
 rm_device:
	device_remove_file(dev, &dev_attr_device);
 done:
	return retval;
}

/**
 * dbg_remove_files: destroys the attribute interface
 * @ci: device
 */
void dbg_remove_files(struct ci13xxx *ci)
{
	if (dev == NULL)
		return -EINVAL;
	device_remove_file(dev, &dev_attr_wakeup);
	device_remove_file(dev, &dev_attr_requests);
	device_remove_file(dev, &dev_attr_registers);
	device_remove_file(dev, &dev_attr_qheads);
	device_remove_file(dev, &dev_attr_port_test);
	device_remove_file(dev, &dev_attr_inters);
	device_remove_file(dev, &dev_attr_events);
	device_remove_file(dev, &dev_attr_driver);
	device_remove_file(dev, &dev_attr_device);
	return 0;
}
