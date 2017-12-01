/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * Project:     PCIe pilot with Raggedstone 4 (refs #3992)
 * Purpose:     Video buffer handling functions
 * Author:      György Kövesdi <gyorgy.kovesdi@eutecus.com>
 * License:     GPL (see file 'COPYING' in the project root for more details)
 * Comments:    
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "eutecus-altera-pci.h"
#include "eutecus-v4l2-info.h"

extern const struct vb2_mem_ops vb2_dma_contig_memops;

static void videoout_dc_put(void * buf_priv);   // see below

int eutecus_init_v4l2_buffers(struct eutecus_v4l2_buffers * buf, resource_size_t phys_start)
{
    ENTER();
    DEBUG(memory, "v4l2 buffer at %p: phys=%p \n", buf, (void*)phys_start);

    // Initialize variables (excluding the CycloneV related ones):
    buf->indices_used = 0;

    // Check the FPGA shared memory availability:
    // We found that the FPGA memory can be problematic: sometimes it is not initialized correctly
    // and no memory accessible at this address. It can be detected here:
    if (buf->indices_used) {
        ERROR("the FPGA memory is not available! It means that the FPGA has not been initialized yet, probably there was problem with SocFPGA booting.\n");
        LEAVE_V("%d", -ENODEV);
        return -ENODEV;
    }

    buf->next_offset = 0;
    buf->next_serial = 0;
    buf->frames_dropped_by_tegra = 0;
    buf->number_of_input_frames = 0;
    buf->input_fps = 0;
    memset_io(buf->offset, 0, sizeof(buf->offset));
    memset_io(&buf->stream, 0, sizeof(buf->stream));
    buf->tegra.pci = NULL;
    // Set up the physical address:
    buf->tegra.kernel_address = phys_start;

    LEAVE_V("%d", 0);
    return 0;
}

static struct eutecus_v4l2_frame * eutecus_init_v4l2_frame_by_index(struct eutecus_v4l2_buffers * buf, unsigned int index, unsigned long size)
{
    struct eutecus_v4l2_frame * frame;

    ENTER();

    /* Set the position of the next buffer: */
    eutecus_set_v4l2_frame_offset(buf, buf->indices_used, buf->next_offset);
    /* Get the next buffer: */
    frame = eutecus_get_v4l2_frame_by_index(buf, buf->indices_used);

    memset_io(frame, 0, sizeof(*frame));   // Clears only the header
    frame->header.full_size = eutecus_v4l2_buffer_size(size);
    frame->header.frame_size = size;
    frame->header.tegra.kernel_address = eutecus_get_v4l2_physical_by_index(buf, buf->indices_used);
    /* The following some operations are implied into memset(): */
    /* frame->header.serial = 0;                */
    /* frame->header.state = FRAME_INITIAL;     */
    /* frame->header.tegra.vob = 0;             */
    /* Note: the kernel address of the CycloneV side must be recalculated because its offset is changed: */
    /* frame->header.cycv.kernel_address = 0;   */

    /* Step forward one buffer: */
    ++buf->indices_used;

    LEAVE_V("new frame at %p", frame);
    return frame;
}

/// Allocate a new V4l2 buffer
static void * videoout_dc_alloc(void * alloc_ctx, unsigned long size, enum dma_data_direction dma_dir, gfp_t gfp_flags)
{
    struct videoout_dc_conf * conf = alloc_ctx;
    struct device * dev = conf->dev;
    struct pci_dev * pci = container_of(dev, struct pci_dev, dev);
    struct eutecus_pci_data * data = pci_get_drvdata(pci);
    struct eutecus_v4l2_buffers * buf = data->frame_buffers;
    struct videoout_dc_buf * dc;
    u32 next_offset = buf->next_offset + eutecus_v4l2_buffer_size(size);

    ENTER();

    DEBUG(memory, "driver_data at index %u, offset %u (next offset: %u)\n", buf->indices_used, buf->next_offset, next_offset);

    if (buf->indices_used >= EUTECUS_MAX_NUMBER_OF_FRAMES || (const void*)(buf->frames+next_offset) > data->end_buffers) {
        printk("ERROR: not enough space in the PCI structure for %lu bytes!\n", size);
        LEAVE();
        return ERR_PTR(-ENOMEM);
    }

    dc = kzalloc(sizeof *dc, GFP_KERNEL);
    if (!dc) {
        LEAVE();
        return ERR_PTR(-ENOMEM);
    }

    dc->frame = eutecus_init_v4l2_frame_by_index(buf, buf->indices_used, size);

    buf->next_offset = next_offset;

    DEBUG(memory, "allocated buffer at %p, frame size=%lu\n", dc->frame, size);

    dc->parent = buf;

    /* v4l2-related stuff: */

    dc->vaddr = dc->frame->payload;   // The plane_vaddr of the frame

    /* Prevent the device from being released while the buffer is in use */
    dc->dev = get_device(dev);

    dc->handler.refcount = &dc->refcount;
    dc->handler.put = videoout_dc_put;
    dc->handler.arg = dc;

    atomic_inc(&dc->refcount);

    LEAVE();
    return dc;
}

/// Release a V4l2 buffer
static void videoout_dc_put(void * buf_priv)
{
    struct videoout_dc_buf * buf = buf_priv;
    struct eutecus_v4l2_buffers * parent = buf->parent;
    struct eutecus_v4l2_frame * frame = buf->frame;

    ENTER();

    if (!atomic_dec_and_test(&buf->refcount)) {
        LEAVE();
        return;
    }

    DEBUG(memory, "buffer free: %p \n", frame);

    put_device(buf->dev);

    if (parent->indices_used) {
        --parent->indices_used;
        DEBUG(memory, "buffers remaining: %d \n", parent->indices_used);
    } else {
        ERROR("no more video DC entries to free\n");
    }

    if (parent->next_offset >= frame->header.full_size) {
        parent->next_offset -= frame->header.full_size;
        DEBUG(memory, "last offset: %#x \n", parent->next_offset);
    } else {
        ERROR("video DC offset is negative (offset=%u, full size=%u, frame size: %u)\n", parent->next_offset, frame->header.full_size, frame->header.frame_size);
    }

    kfree(buf);

    LEAVE();
}

static void videoout_vma_open(struct vm_area_struct * vma)
{
    ENTER();
    LEAVE();
}

static void videoout_vma_close(struct vm_area_struct * vma)
{
    ENTER();
    LEAVE();
}

static struct vm_operations_struct videoout_vm_ops = {
    .open =  videoout_vma_open,
    .close = videoout_vma_close,
};

/// Maps a V4l2 buffer into virtual memory
static int videoout_dc_mmap(void * buf_priv, struct vm_area_struct * vma)
{
    int rs;
    struct videoout_dc_buf * buf = buf_priv;

    ENTER();

    DEBUG(memory, "phys=%p, start=%p, end=%p, off=%lu, prot=0x%llx \n", (void*)buf->frame->header.tegra.kernel_address, (void*)vma->vm_start, (void*)vma->vm_end, vma->vm_pgoff, pgprot_val(vma->vm_page_prot));

    rs = eutecus_remap(vma, buf->frame->header.tegra.kernel_address);

    if (!rs) {
        vma->vm_ops = &videoout_vm_ops;
        videoout_vma_open(vma);
    }

    LEAVE_V("%d", rs);
    return rs;
}

void videoout_dc_init(struct vb2_mem_ops * memopts)
{
    *memopts = vb2_dma_contig_memops;
    memopts->alloc = videoout_dc_alloc;
    memopts->put = videoout_dc_put;
    memopts->mmap = videoout_dc_mmap;
}

/* * * * * * * * * * * * * End - of - File * * * * * * * * * * * * * * */
