/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * Project:     PCIe pilot with Raggedstone 4 (refs #3992)
 * Purpose:     PCIe communication between the Raggedstone 4 and a PCI root port
 * Author:      György Kövesdi <gyorgy.kovesdi@eutecus.com>
 * License:     Eutecus Proprietary
 * Comments:    
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "eutecus-altera-pci.h"

static int eutecus_pci_open(struct inode * inode, struct file * fp);
static int eutecus_pci_mmap(struct file * fp, struct vm_area_struct * vma);
static int eutecus_pci_release(struct inode * inode, struct file * fp);
static ssize_t eutecus_pci_read(struct file * fp, char __user * buf, size_t count, loff_t * ppos);
static ssize_t eutecus_pci_write(struct file * fp, const char __user * buf, size_t count, loff_t * ppos);
static loff_t eutecus_pci_llseek(struct file *fp, loff_t offset, int orig);

static const struct file_operations eutecus_pci_fops = {
    .owner = THIS_MODULE,
    .read = eutecus_pci_read,
    .write = eutecus_pci_write,
    .open = eutecus_pci_open,
    .mmap = eutecus_pci_mmap,
    .release = eutecus_pci_release,
    .llseek = eutecus_pci_llseek,
};

static struct eutecus_pci_data * the_pci;

/// Creates a device file for a PCI resource on demand
static int create_device_file(struct eutecus_pci_data * data, int index)
{
    struct eutecus_pci_resources * res = data->resources + index;

    res->minor = MINOR(data->number) + index;

    res->devicefile = device_create(data->cl, NULL, MKDEV(MAJOR(data->number), res->minor), res, MY_MODULE_NAME "-%d", index);
    if (IS_ERR_OR_NULL(res->devicefile)) {
        int err = PTR_ERR(res->devicefile);
        res->devicefile = NULL;
        return err;
    }

    DEBUG(files, "device file #%d (%d:%d) created\n", index, MAJOR(data->number), res->minor);

    return 0;
}

/// Deletes the device file of a PCI resource
static void destroy_device_file(struct eutecus_pci_data * data, int index)
{
    struct eutecus_pci_resources * res = data->resources + index;
    if (res->devicefile) {
        device_destroy(data->cl, MKDEV(MAJOR(data->number), res->minor));
        DEBUG(files, "device file #%d (%d:%d) removed\n", index, MAJOR(data->number), res->minor);
        res->devicefile = NULL;
    } else {
        DEBUG(files, "device file #%d (%d:%d) has already been removed or not created\n", index, MAJOR(data->number), res->minor);
    }
}

static int fill_pci_info(struct eutecus_pci_data * data)
{
    ENTER();

    data->frame_buffers = data->resources[EUTECUS_PCI_RESOURCE_SHARED_MEMORY].memory;
    data->end_buffers = (const char *)data->resources[EUTECUS_PCI_RESOURCE_SHARED_MEMORY].memory + data->resources[EUTECUS_PCI_RESOURCE_SHARED_MEMORY].size;

    eutecus_init_v4l2_buffers(data->frame_buffers, data->resources[EUTECUS_PCI_RESOURCE_SHARED_MEMORY].start);
    data->frame_buffers->tegra.pci = data;  // Pointer back to the PCI info. Must be after eutecus_init_v5l2_buffers() call.

    LEAVE_V("%d", 0);
    return 0;
}

static int init_resource(struct eutecus_pci_data * data, int index, int pci_resource)
{
    struct eutecus_pci_resources * res = data->resources + index;

    ENTER();

    res->start  = pci_resource_start(data->dev, pci_resource);
    res->dma    = phys_to_dma(&data->dev->dev, res->start);
    res->size   = pci_resource_len(data->dev, pci_resource);
    res->parent = data;

    res->memory = ioremap_nocache(res->start, res->size);

    if (!res->memory) {
        ERROR("could not map PCI resource #%d to index %d (start=%p, size=%p)\n", pci_resource, index, (void*)res->start, (void*)res->size);
        LEAVE_V("%d", -ENOMEM);
        return -ENOMEM;
    }

    DEBUG(resources, "PCI resource #%d mapped to index %d at %p, size=%p\n", pci_resource, index, res->memory, (void*)res->size);

    LEAVE_V("%d", 0);
    return 0;
}

static void uninit_resource(struct eutecus_pci_resources * res)
{
    if (res->memory) {
        iounmap(res->memory);
        res->memory = NULL;
    }
}

/// The PCI resource assignments
/*! The following PCI resources are used:
    - 0:    This is the memory window for communication. Its size is now 16 MiB.
    - 2:    Interrupt to the RS4 board.
    - 3:    Interrupt to the Jetson board. */
static const struct {
    int index;
    int pci_resource;
} resource_table[] = {
    { EUTECUS_PCI_RESOURCE_SHARED_MEMORY,   0 },
    { EUTECUS_PCI_RESOURCE_INTERRUPT_ACK,   2 },
    { EUTECUS_PCI_RESOURCE_INTERRUPT_2_RS4, 3 }
};

/// Constructor for \ref eutecus_pci_data
int init_cfile(struct eutecus_pci_data * data, struct pci_dev * dev)
{
    int st = 0;
    int i;

    ENTER();

    data->dev = dev;

    do {
        for (i = 0; i < sizeof(resource_table)/sizeof(resource_table[0]); ++i) {
            st = init_resource(data, resource_table[i].index, resource_table[i].pci_resource);
            if (st) {
                break;
            }
        }
        if (st) {
            break;
        }

        data->cfile = cdev_alloc();
        if (!data->cfile) {
            ERROR("cdev_alloc() failed\n");
            st = -ENOMEM;
            break;
        }
        cdev_init(data->cfile, &eutecus_pci_fops);
        data->cfile->owner = THIS_MODULE;

        do {
            if (dbg.devicefile) {
                st = alloc_chrdev_region(&data->number, 1, 3, MY_MODULE_NAME);
                if (st) {
                    ERROR("could not allocate pci device\n");
                    break;
                }
                DEBUG(files, "alloc_chrdev_region(%d, 1, 3, '%s') OK\n", (int)data->number, MY_MODULE_NAME);
            }

            do {
                if (dbg.devicefile) {
                    data->cl = class_create(THIS_MODULE, "eutecus-pci-driver-class");
                    if (IS_ERR_OR_NULL(data->cl)) {
                        ERROR("could not create driver class\n");
                        st = PTR_ERR(data->cl);
                        data->cl = NULL;
                        break;
                    }
                    DEBUG(files, "class '%s' created\n", "eutecus-pci-driver-class");
                }

                do {
                    if (dbg.devicefile) {
                        int i;
                        for (i = 0; i < 3; ++i) {
                            st = create_device_file(data, i);
                            if (st) {
                                ERROR("could not create file #%d\n", i);
                                break;
                            }
                        }
                        if (st) {
                            while (--i >= 0) {
                                destroy_device_file(data, i);   // The previously created ones are removed here
                            }
                            break;
                        }
                    }

                    do {
                        st = cdev_add(data->cfile, data->number, 3);
                        if (st) {
                            ERROR("could not register pci device\n");
                            break;
                        }

                        the_pci = data;

                        st = fill_pci_info(data);
                        if (st) {
                            ERROR("fill_pci_info() failed\n");
                            break;
                        }

                        DEBUG(level, "added device %d/%d\n", MAJOR(data->number), MINOR(data->number));

                        LEAVE();
/* ------------- */     return 0;       /* Success  --------------------------------------------------- */

                    } while (0);

                    if (dbg.devicefile) {
                        int i;
                        for (i = 2; i >= 0; --i) {
                            destroy_device_file(data, i);
                        }
                    }

                } while (0);

                if (dbg.devicefile) {
                    class_destroy(data->cl);
                    data->cl = NULL;
                }

                DEBUG(files, "class '%s' destroyed\n", "eutecus-pci-driver-class");

            } while (0);

            if (dbg.devicefile) {
                unregister_chrdev_region(data->number, 3);
                DEBUG(files, "unregister_chrdev_region(%d, 3)\n", (int)data->number);
            }

        } while (0);

        cdev_del(data->cfile);
        data->cfile = NULL;

    } while (0);

    LEAVE_V("%d", st);
    return st;
}

/// Destructor for \ref eutecus_pci_data
void destroy_cfile(struct eutecus_pci_data * data)
{
    int i;

    ENTER();

    if (data->cl) {
        for (i = 2; i >= 0; --i) {
            destroy_device_file(data, i);
        }
        class_destroy(data->cl);
        DEBUG(files, "class '%s' destroyed\n", "eutecus-pci-driver-class");
    }

    unregister_chrdev_region(data->number, 3);
    DEBUG(files, "unregister_chrdev_region(%d, 3)\n", (int)data->number);

    if (data->cfile) {
        cdev_del(data->cfile);
        data->cfile = NULL;
    }

    for (i = 2; i >= 0; --i) {
        uninit_resource(data->resources+i);
    }

    kfree(data);

    LEAVE();
}

/// Assigns the local resource to a given minor number
static struct eutecus_pci_resources * get_eutecus_file(int dev)
{
    if (!the_pci) {
        return NULL;
    }

    if (dev >= 1 && dev <= 3) {
        return &the_pci->resources[dev-1];
    }

    return NULL;
}

/// File open() function
static int eutecus_pci_open(struct inode * inode, struct file * fp)
{
    struct eutecus_pci_resources * res;
    int dev;

    DEBUG(files, "eutecus_pci_open()\n");

    dev = iminor(inode);
    res = get_eutecus_file(dev);
    if (!res) {
        return -ENODEV;
    }

    DEBUG(files, "opened minor %d: at %p, mem %p, len %p\n", dev, (void*)res->start, res->memory, (void*)res->size);

    DEBUG(files, "private data is %p/%p\n", fp->private_data, res);

    fp->private_data = res;

    return 0;
}

/// File close() function
static int eutecus_pci_release(struct inode * inode, struct file * fp)
{
    const struct eutecus_pci_resources * res = fp->private_data;

    DEBUG(files, "released dev at %p, mem %p, len %p\n", (void*)res->start, res->memory, (void*)res->size);

    return 0;
}

static void eutecus_pci_vma_open(struct vm_area_struct * vma)
{
    ENTER();
    LEAVE();
}

static void eutecus_pci_vma_close(struct vm_area_struct * vma)
{
    ENTER();
    LEAVE();
}

static struct vm_operations_struct eutecus_pci_vm_ops = {
    .open =  eutecus_pci_vma_open,
    .close = eutecus_pci_vma_close,
};

/// File mmap() function
static int eutecus_pci_mmap(struct file * fp, struct vm_area_struct * vma)
{
    int rs;
    const struct eutecus_pci_resources * res = fp->private_data;

    ENTER();
    DEBUG(memory, "phys=%p, start=%p, end=%p, off=%lu, prot=%lu \n", (void*)res->start, (void*)vma->vm_start, (void*)vma->vm_end, vma->vm_pgoff, pgprot_val(vma->vm_page_prot));

    rs = eutecus_remap(vma, res->start);

    if (!rs) {
        vma->vm_ops = &eutecus_pci_vm_ops;
        eutecus_pci_vma_open(vma);
    }

    LEAVE_V("%d", rs);
    return rs;
}

/// File read() function
static ssize_t eutecus_pci_read(struct file * fp, char __user * buf, size_t count, loff_t * ppos)
{
    const struct eutecus_pci_resources * res = fp->private_data;

    DEBUG(files, "read off=%ld, size=%lu, at %p, mem %p, len %p\n", (long)*ppos, count, (void*)res->start, res->memory, (void*)res->size);

    if (*ppos >= res->size) {
        return 0;
    }

    if (count + *ppos > res->size) {
        count = res->size - *ppos;
    }

    DEBUG(files, "reading %p bytes from %p to %p\n", (void*)count, res->memory + *ppos, buf);

    if (copy_to_user(buf, res->memory + *ppos, count)) {
        return -EFAULT;
    }

    *ppos += count;

    DEBUG(files, "read finished.\n");

    return count;
}

/// File write() function
static ssize_t eutecus_pci_write(struct file * fp, const char __user * buf, size_t count, loff_t * ppos)
{
    const struct eutecus_pci_resources * res = fp->private_data;

    DEBUG(files, "write off=%ld, size=%lu, at %p, mem %p, len %p\n", (long)*ppos, count, (void*)res->start, res->memory, (void*)res->size);

    if (res->size == 0) {
        return -ENODEV;
    }

    if (*ppos >= res->size) {
        return 0;
    }

    if (count + *ppos > res->size) {
        count = res->size - *ppos;
    }

    DEBUG(files, "writing %p bytes from %p to %p\n", (void*)count, buf, res->memory + *ppos);

    if (copy_from_user(res->memory + *ppos, buf, count)) {
        return -EFAULT;
    }

    *ppos += count;

    DEBUG(files, "write finished.\n");

    return count;
}

static loff_t eutecus_pci_llseek(struct file * fp, loff_t offset, int orig)
{
    const struct eutecus_pci_resources * res = fp->private_data;

    ENTER();

    switch (orig) {
        case SEEK_CUR:
            offset += fp->f_pos;
        break;

        case SEEK_SET:
            // Nothing to do here
        break;

        case SEEK_END:
            offset += res->size;
        break;

        default:
            // Unknown parameter:
            LEAVE();
            return -EINVAL;
        break;
    }

    if (offset < 0 || offset >= res->size) {
        LEAVE();
        return -EINVAL;
    }

    fp->f_pos = offset;

    LEAVE();
    return offset;
}

/* * * * * * * * * * * * * End - of - File * * * * * * * * * * * * * * */
