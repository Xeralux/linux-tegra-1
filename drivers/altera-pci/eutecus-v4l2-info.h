/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * Project:     PCIe pilot with Raggedstone 4 (refs #3992)
 * Purpose:     PCIe communication between the Raggedstone 4 and a PCI root port
 * Author:      György Kövesdi <gyorgy.kovesdi@eutecus.com>
 * License:     Eutecus Proprietary
 * Comments:    
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef __DRIVERS_ALTERA_PCI_EUTECUS_V4L2_INFO_H_INCLUDED__
#define __DRIVERS_ALTERA_PCI_EUTECUS_V4L2_INFO_H_INCLUDED__

#include <media/v4l2-device.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-memops.h>
#include <linux/spinlock_types.h>
#include <linux/mutex.h>

struct videoout_dmaqueue
{
    struct list_head       active;

    int IsOutChainStarted;
};

struct video_data_format;

struct videoout_dev
{
    struct v4l2_device          v4l2_dev;
    struct video_device         vdev;
    struct list_head            videoout_devlist;

    struct videoout_dmaqueue    vidq;
    struct vb2_queue            vb_vidq;

    spinlock_t                  slock;
    struct mutex                mutex;

    int                         width;
    int                         height;

    const struct video_data_format * fmt;

}; // struct videoout_dev

struct videoout_buffer
{
    struct vb2_v4l2_buffer      vb;
    struct list_head            list;

    const struct video_data_format * fmt;

    int queued;
};

struct videoout_dc_conf {
    struct device * dev;
};

struct eutecus_v4l2_frame;

struct videoout_dc_buf {
/* ---------------- copied from videobuf2-dma-contig.c -------------- */
    struct device * dev;                                           /* */
    void * vaddr;                                                  /* */
    unsigned long size;                                            /* */
    dma_addr_t dma_addr;                                           /* */
    enum dma_data_direction dma_dir;                               /* */
    struct sg_table * dma_sgt;                                     /* */
    struct vb2_vmarea_handler handler;                             /* */
    atomic_t refcount;                                             /* */
    struct sg_table * sgt_base;                                    /* */
    struct vm_area_struct * vma;                                   /* */
    struct dma_buf_attachment * db_attach;                         /* */
/* --------------------------- end of copy -------------------------- */

    struct eutecus_v4l2_frame * frame;

    struct eutecus_v4l2_buffers * parent;
};

void videoout_dc_init(struct vb2_mem_ops * memops);

static inline void videoout_buffer_done(struct videoout_buffer * buf, enum vb2_buffer_state state)
{
    buf->queued = 1;
    vb2_buffer_done(&buf->vb.vb2_buf, state);
}

#endif /* __DRIVERS_ALTERA_PCI_EUTECUS_V4L2_INFO_H_INCLUDED__ */

/* * * * * * * * * * * * * End - of - File * * * * * * * * * * * * * * */
