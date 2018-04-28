/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Spiros Dontas
 * Lefteris Kalafatis
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	
	WARN_ON ( !(sensor = state->sensor));
	/* ? */
    if ( sensor->msr_data[state->type]->last_update != state->buf_timestamp)
        return 1;

	/* The following return is bogus, just for the stub to compile */
    debug("leaving with ret = 0");
	return 0; /* ? */
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
    uint16_t data;
    uint32_t timestamp;
    long fixed;
    int integer, decadic;
	
	debug("entering\n");

	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	/* ? */
    sensor = state->sensor;
    spin_lock(&sensor->lock);
    data = (uint16_t) sensor->msr_data[state->type]->values[0];
    timestamp = sensor->msr_data[state->type]->last_update;
    spin_unlock(&sensor->lock);
	/* Why use spinlocks? See LDD3, p. 119 */

	/*
	 * Any new data available?
	 */
	/* ? */
    if ( !lunix_chrdev_state_needs_refresh(state) )
        return -EAGAIN;

	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */

	/* ? */
    if ( state->type == BATT ) {
        fixed = lookup_voltage[data];
    } else if ( state->type == TEMP ) {
        fixed = lookup_temperature[data];
    } else {
        fixed = lookup_light[data];
    }
    integer = fixed / 1000;
    decadic = fixed % 1000;
    state->written = snprintf(state->buf_data, LUNIX_CHRDEV_BUFSZ, "%d.%03d", integer, decadic);
    state->buf_timestamp = timestamp;

	debug("leaving with ret = 0\n");
	return 0;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	/* ? */
	int ret;
    /* Our Declarations */
    struct lunix_chrdev_state_struct *dev; /* Device Information */
    unsigned int minor;
    unsigned int NO;   
	unsigned int TYPE;
    
    debug("entering\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

    /* dev = container_of(inode->i_cdev, struct lunix_chrdev_state_struct, cdev); */
    dev = (struct lunix_chrdev_state_struct *)kzalloc(sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL);
    if ( dev != NULL )
        ret = 0;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
    minor = iminor(inode);
    NO    = minor / 8;
    TYPE  = minor % 8;

    dev->type   = TYPE;
    dev->sensor = &lunix_sensors[NO];
    sema_init(&dev->lock, 1);
	
	/* Allocate a new Lunix character device private state structure */
	/* ? */
    filp->private_data = dev;
out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	/* ? */
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Why? */
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret;
    size_t to_write;

	struct lunix_sensor_struct       *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

    debug("entering with cnt = %d", cnt);

	/* Lock? */
    debug("trying to lock the semaphore");
    if ( down_interruptible(&state->lock) )
        return -ERESTARTSYS;
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
    debug("checking the f_pos = %ld", *f_pos);
	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			/* ? */
            debug("no update");
            up(&state->lock);
            debug("sleeping");
            if ( wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state) == 1) )
                return -ERESTARTSYS;
            debug("woken up");
            if ( down_interruptible(&state->lock) )
                return -ERESTARTSYS;
			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */
		}
	}

	/* End of file */
	/* ? */
	
	/* Determine the number of cached bytes to copy to userspace */
	/* ? */
    to_write = state->written - (size_t)*f_pos;

    ret = min(cnt, to_write);

    if ( copy_to_user(usrbuf, state->buf_data + (ssize_t)*f_pos, ret) ) {
        up(&state->lock);
        return -EFAULT;
    }

	/* Auto-rewind on EOF mode? */
	/* ? */
out:
	/* Unlock? */
    up(&state->lock);
    debug("leaving with ret = %d", ret);
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = 
{
    .owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int          ret;
	dev_t        dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
	
	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;
	
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	/* ? */
	/* register_chrdev_region? */
    ret = 0;
	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}	
	/* ? */
	/* cdev_add? */
    ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t        dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
		
	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
