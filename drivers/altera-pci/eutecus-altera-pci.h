/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * Project:     PCIe pilot with Raggedstone 4 (refs #3992)
 * Purpose:     PCIe communication between the Raggedstone 4 and a PCI root port
 * Author:      György Kövesdi <gyorgy.kovesdi@eutecus.com>
 * License:     Eutecus Proprietary
 * Comments:    
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef __DRIVERS_ALTERA_PCI_EUTECUS_ALTERA_PCI_H_INCLUDED__
#define __DRIVERS_ALTERA_PCI_EUTECUS_ALTERA_PCI_H_INCLUDED__

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <asm/ptrace.h>
#include <asm/irq.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/dma-mapping.h>
#include <asm/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/compiler.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/types.h>   // for dev_t typedef
#include <linux/kdev_t.h>  // for format_dev_t
#include <linux/fs.h>      // for alloc_chrdev_region()
#include <linux/mm.h>

#include "eutecus-v4l2-info.h"
#include "eutecus-v4l2-frame.h"

#define MY_MODULE_NAME          "rs4-pci"
#define DRV_VERSION             "0.1"

#define PCI_VENDOR_ID_ALTERA    0x1172
#define PCI_DEVICE_ID_RPDE      0xE000
#define PCI_DEVICE_ID_EPDE      0xE001

#define MSG(msg...)             printk(MY_MODULE_NAME ": " msg)
#define ERROR(msg...)           printk(MY_MODULE_NAME " Error: " msg)

#if 1   /* debug is on: */
#define DEBUG(mode, msg...)     do { if (dbg.mode) printk(MY_MODULE_NAME " debug: " msg); } while(0)
#define NODEBUG(mode, msg...)   do { } while(0)
#else
#define DEBUG(mode, msg...)     do { } while(0)
#define NODEBUG(msg...)         MSG(msg)
#endif

#define ENTER()                 DEBUG(calltrace, "%s() enter\n", __func__)
#define ENTER_V(V, N...)        DEBUG(calltrace, "%s() enter: " V "\n", __func__, N)
#define LEAVE()                 DEBUG(calltrace, "%s() leave\n", __func__)
#define LEAVE_V(V, N...)        DEBUG(calltrace, "%s() leave: " V "\n", __func__, N);

#define EUTECUS_PCI_RESOURCE_SHARED_MEMORY      0   ///< Shared memory window for media stream
#define EUTECUS_PCI_RESOURCE_INTERRUPT_ACK      1   ///< Interrupt from the RS4 board
#define EUTECUS_PCI_RESOURCE_INTERRUPT_2_RS4    2   ///< Interrupt to the RS4 board

union dbg_info {
    int level;

    struct {
        /// Generic debug info
        int generic:1;

        /// Display file oprations
        int files:1;

        /// Display PCI resource information
        int resources:1;

        /// Display detailed PCI configuration information
        int config:1;

        /// Display function calltrace
        int calltrace:1;

        /// Create device files for resources
        int devicefile:1;

        /// Display memory-related operations
        int memory:1;

        /// Display cideo-related information
        int video:1;
    };
};

typedef union dbg_info dbg_info;

extern dbg_info dbg;

struct eutecus_pci_data;

struct eutecus_pci_resources {

    /// Physical address of this resource
    resource_size_t start;

    /// DMA address of this resource
    /*! \note   On several platforms it is the same as physical address.
        \note   It is not used in this driver. */
    dma_addr_t dma;

    /// Size of this resource (in bytes)
    resource_size_t size;

    /// Mapped address of this resource
    void * memory;

    struct device * devicefile;

    int minor;

    struct eutecus_pci_data * parent;
};

/// Holds all information about our PCI device
struct eutecus_pci_data {

    struct pci_dev * dev;

    struct cdev * cfile;

    struct class * cl;

    dev_t number;

    int irq;

    int v4l2_init_state;

    struct videoout_dev vidout;

    struct eutecus_v4l2_buffers * frame_buffers;

    const void * end_buffers;

    struct eutecus_pci_resources resources[3];
};

/// The resource of the shared memory
static inline struct eutecus_pci_resources * get_media_memory(struct eutecus_pci_data * pci)
{
    return &pci->resources[EUTECUS_PCI_RESOURCE_SHARED_MEMORY];
}

/// Clears the interrupt request from the RS4 board
static inline void interrupt_acknowledge_2_RS4(struct eutecus_pci_data * pci)
{
    volatile uint32_t * flag = (volatile uint32_t *)pci->resources[EUTECUS_PCI_RESOURCE_INTERRUPT_ACK].memory;
    if (!flag) {
        /* Resource allocation problem: cannot acknowledge */
        ERROR("could not acknowledge FPGA IRQ: resource is unavailable\n");
        return;
    }

    *flag = 0;
}

/// Triggers an interrupt to the RS4 board
static inline void interrupt_request_2_RS4(struct eutecus_pci_data * pci)
{
    volatile uint32_t * flag = (volatile uint32_t *)pci->resources[EUTECUS_PCI_RESOURCE_INTERRUPT_2_RS4].memory;
    if (!flag) {
        /* Resource allocation problem: cannot request interrupt */
        ERROR("could not request FPGA IRQ: resource is unavailable\n");
        return;
    }

    *flag = 1;
}

static inline struct eutecus_pci_data * vb2_get_eutecus_pci_data(struct vb2_queue * vq)
{
    struct pci_dev * pci = vb2_get_drv_priv(vq);
    return pci_get_drvdata(pci);
}

static inline struct videoout_dev * vb2_get_videoout_dev(struct vb2_queue * vq)
{
    return &vb2_get_eutecus_pci_data(vq)->vidout;
}

/// Maps the given physical address to userspace
/*! \param  vma     Pointer to the map parameters
    \param  start   Physical address to be mapped
    \retval int     Error code, zero on success. */
inline static int eutecus_remap(struct vm_area_struct * vma, resource_size_t start)
{
    if (start & (PAGE_SIZE-1)) {
        return -EINVAL; // not page-aligned
    }

    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
    return io_remap_pfn_range(vma, vma->vm_start, start >> PAGE_SHIFT, vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

static const char * vb_state_names[] = {
	"VB2_BUF_STATE_DEQUEUED",
	"VB2_BUF_STATE_PREPARING",
	"VB2_BUF_STATE_PREPARED",
	"VB2_BUF_STATE_QUEUED",
	"VB2_BUF_STATE_REQUEUEING",
	"VB2_BUF_STATE_ACTIVE",
	"VB2_BUF_STATE_DONE",
	"VB2_BUF_STATE_ERROR",
};

static inline const char * get_vb_state_name(struct vb2_buffer * vb)
{
    int state;

    if(!vb){
        return "null";
    }

    state = vb->state;

    if (state < 0 || state > VB2_BUF_STATE_ERROR) {
        return "unknown";
    }

    return vb_state_names[state];
}

int init_cfile(struct eutecus_pci_data * data, struct pci_dev * dev);
void destroy_cfile(struct eutecus_pci_data * data);
int altera_v4l2_initialize(struct pci_dev * dev);
void altera_v4l2_destroy(struct pci_dev * dev);
irqreturn_t eutecus_pci_isr(int this_irq, void * param);
int eutecus_videoout_thread(void * data);

#endif /* __DRIVERS_ALTERA_PCI_EUTECUS_ALTERA_PCI_H_INCLUDED__ */

/* * * * * * * * * * * * * End - of - File * * * * * * * * * * * * * * */
