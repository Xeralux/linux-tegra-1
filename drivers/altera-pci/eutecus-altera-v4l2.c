/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * Project:     PCIe pilot with Raggedstone 4 (refs #3992)
 * Purpose:     V4l2-related functions
 * Author:      György Kövesdi <gyorgy.kovesdi@eutecus.com>
 * License:     Eutecus Proprietary
 * Comments:    
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "eutecus-altera-pci.h"
#include "eutecus-altera-v4l2-ioctl.h"

static void videoout_got_new_frame(struct eutecus_v4l2_buffers * buf, struct videoout_buffer * vob)
{
    struct vb2_buffer * vb = &vob->vb;
    struct eutecus_v4l2_frame * frame = container_of(vb2_plane_vaddr(vb, 0), struct eutecus_v4l2_frame, payload);
    u32 serial = ++buf->next_serial;

    ENTER();

    frame->header.serial = serial;

    switch (frame->header.state) {
        case FRAME_READY:
            frame->header.tegra.vob = (u64)vob;
            frame->header.state = FRAME_BUSY;
            interrupt_request_2_RS4(buf->tegra.pci);    /* Send an interrupt to the analytics: the frame is to be processed */
            DEBUG(video, "Got frame #%u (state: ready -> busy) at %p (IRQ)\n", serial, frame);
        break;

        default:
            frame->header.tegra.vob = 0UL;
            videoout_buffer_done(vob, VB2_BUF_STATE_DONE);
            DEBUG(video, "frame #%u dropped (state: %s) at %p\n", serial, get_frame_state_name(frame), frame);
        break;
    }

    LEAVE();
}

static int queue_setup(struct vb2_queue * vq, const void * parg, unsigned int * nbuffers, unsigned int * nplanes, unsigned int sizes[], void * alloc_ctxs[])
{
    const struct v4l2_format * fmt = parg;
    struct videoout_dev * vid = vb2_get_videoout_dev(vq);
    struct eutecus_pci_data * pci = container_of(vid, struct eutecus_pci_data, vidout);
    struct eutecus_v4l2_buffers * buf = pci->frame_buffers;
    struct pci_dev * dev = vq->drv_priv;

    ENTER();

    if (fmt) {
        switch (fmt->type) {
            case V4L2_BUF_TYPE_VIDEO_OUTPUT:
            {
                const struct v4l2_pix_format * pf = &fmt->fmt.pix;

                DEBUG(generic, "(single plane) frame size: %dx%d\n", (int)pf->width, (int)pf->height);

                buf->stream.width = pf->width;
                buf->stream.height = pf->height;
                buf->stream.fourcc = pf->pixelformat;

                sizes[0] = pf->sizeimage;
                *nplanes = 1;
                alloc_ctxs[0] = vb2_dma_contig_init_ctx(&dev->dev);
            }
            break;

            case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
            {
                unsigned int i;
                const struct v4l2_pix_format_mplane * pf = &fmt->fmt.pix_mp;

                DEBUG(generic, "(multiplane=%u) frame size: %dx%d\n", (unsigned)pf->num_planes, (int)pf->width, (int)pf->height);

                buf->stream.width = pf->width;
                buf->stream.height = pf->height;
                buf->stream.fourcc = pf->pixelformat;

                *nplanes = pf->num_planes;

                for (i = 0U; i < pf->num_planes; ++i) {
                    const struct v4l2_plane_pix_format * pp = &pf->plane_fmt[i];
                    sizes[i] = pp->sizeimage;
                    alloc_ctxs[i] = vb2_dma_contig_init_ctx(&dev->dev);
                }
            }
            break;

            default:
                ERROR("invalid buf type (%d) in format", fmt->type);
                LEAVE_V("%d", -EINVAL);
                return -EINVAL;
            break;
        }
    } else {
        if (vid->fmt) {
            buf->stream.width = vid->width;
            buf->stream.height = vid->height;
            buf->stream.fourcc = vid->fmt->fourcc;

            switch (vid->fmt->type) {
                case V4L2_BUF_TYPE_VIDEO_OUTPUT:
                    DEBUG(generic, "(dev single plane) size: %dx%d, %d bits per pixel\n", (int)vid->width, (int)vid->height, (int)vid->fmt->bpp);
                    *nplanes = 1;
                    sizes[0] = (vid->width*vid->height * vid->fmt->bpp) / 8;
                    alloc_ctxs[0] = vb2_dma_contig_init_ctx(&dev->dev);
                break;

                case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
                    DEBUG(generic, "(dev multiplane=%u) size: %dx%d, %d bits per pixel\n", (unsigned)vid->fmt->n_planes, (int)vid->width, (int)vid->height, (int)vid->fmt->bpp);
                    /* We allocate one physical plane, because it is contiguous: */
                    *nplanes = 1;
                    sizes[0] = (vid->width*vid->height * vid->fmt->bpp) / 8;
                    alloc_ctxs[0] = vb2_dma_contig_init_ctx(&dev->dev);
                break;

                default:
                    ERROR("invalid buf type (%d) in queue", fmt->type);
                    LEAVE_V("%d", -EINVAL);
                    return -EINVAL;
                break;
            }
        } else {
            ERROR("no format set.\n");
            return -EINVAL;
        }
    }

    buf->stream.numerator = 30;  /* FIXME: I could not find this info in the video out device structures, so it is hardwired. */
    buf->stream.denominator = 1;

    if (*nbuffers < MIN_BUFFERS) {
        DEBUG(generic, "nbuffers is increased from %u to %u\n", *nbuffers, MIN_BUFFERS);
        *nbuffers = MIN_BUFFERS;
    }

    DEBUG(generic, "buffers=%u on dev %p\n", *nbuffers, vid);

    LEAVE();
    return 0;
}

static int buffer_prepare(struct vb2_buffer * vb)
{
    struct videoout_dev * dev = vb2_get_videoout_dev(vb->vb2_queue);
    struct videoout_buffer * buf = container_of(vb, struct videoout_buffer, vb);
    unsigned long size;

    ENTER();

    BUG_ON(!dev->fmt);

    size = (dev->width * dev->height * dev->fmt->bpp) / 8;
    DEBUG(generic, "dev=%p, vb=%p, size=%lu \n", dev, vb, size);

    if (vb2_plane_size(vb, 0) < size) {
        DEBUG(generic, "data will not fit into plane (%lu < %lu)\n", vb2_plane_size(vb, 0), size);
        LEAVE_V("%d", -EINVAL);
        return -EINVAL;
    }

    vb2_set_plane_payload(&buf->vb, 0, size);

    buf->fmt = dev->fmt;

    LEAVE();

    return 0;
}

static void buffer_queue(struct vb2_buffer * vb)
{
    struct videoout_dev * dev = vb2_get_videoout_dev(vb->vb2_queue);
    struct videoout_buffer * vob = container_of(vb, struct videoout_buffer, vb);

    ENTER();
    DEBUG(generic, "dev=%p, vb=%p\n", dev, vb);

    vob->queued = 0;

    if (0) {    /* Threaded mode: */
        struct videoout_dmaqueue * dma_q = &dev->vidq; // ok
        unsigned long flags = 0;

        spin_lock_irqsave(&dev->slock, flags);
        list_add_tail(&vob->list, &dma_q->active);
        spin_unlock_irqrestore(&dev->slock, flags);

    } else {    /* Direct mode: */
        struct eutecus_pci_data * pci = container_of(dev, struct eutecus_pci_data, vidout);

        videoout_got_new_frame(pci->frame_buffers, vob);
    }

    LEAVE();
}

static int start_streaming(struct vb2_queue * vq, unsigned int count)
{
    int err = 0;
    struct videoout_dev * dev = vb2_get_videoout_dev(vq);
    //struct videoout_dmaqueue * dma_q = &dev->vidq;
    struct eutecus_pci_data * pci = container_of(dev, struct eutecus_pci_data, vidout);
    struct eutecus_v4l2_buffers * buf = pci->frame_buffers;

    ENTER();

    buf->stream.active = 1;

    LEAVE_V("%d", err);
    return err;
}

static void stop_streaming(struct vb2_queue * vq)
{
    struct videoout_dev * dev = vb2_get_videoout_dev(vq);
    //struct videoout_dmaqueue * dma_q = &dev->vidq;
    struct eutecus_pci_data * pci = container_of(dev, struct eutecus_pci_data, vidout);
    struct eutecus_v4l2_buffers * buf = pci->frame_buffers;
    u32 i;

    ENTER();

    buf->stream.active = 0;

    /* Release all active buffers */
    for (i = 0; i < buf->indices_used; ++i) {
        struct eutecus_v4l2_frame * frame = eutecus_get_v4l2_frame_by_index(buf, i);
        if (frame->header.tegra.vob) {
            struct videoout_buffer * vob = (struct videoout_buffer *)frame->header.tegra.vob;
            if (!vob->queued) {
                DEBUG(video, "frame #%u is DONE at %p\n", frame->header.serial, frame);
                videoout_buffer_done(vob, VB2_BUF_STATE_QUEUED);
            }
        }
    }

    LEAVE();
}

static void videoout_lock(struct vb2_queue * vq)
{
    struct videoout_dev * dev = vb2_get_videoout_dev(vq);

    ENTER();

    mutex_lock(&dev->mutex);

    LEAVE();
}

static void videoout_unlock(struct vb2_queue * vq)
{
    struct videoout_dev * dev = vb2_get_videoout_dev(vq);

    ENTER();

    mutex_unlock(&dev->mutex);

    LEAVE();
}

static const struct vb2_ops videoout_video_qops = {
    .queue_setup        =   queue_setup,
    .buf_prepare        =   buffer_prepare,
    .buf_queue          =   buffer_queue,
    .start_streaming    =   start_streaming,
    .stop_streaming     =   stop_streaming,
    .wait_prepare       =   videoout_unlock,
    .wait_finish        =   videoout_lock,
};

static int eutecus_vb2_fop_mmap(struct file * file, struct vm_area_struct * vma)
{
    int res;

    ENTER();

    res = vb2_fop_mmap(file, vma);

    LEAVE_V("%d", res);
    return res;
}

static const struct v4l2_file_operations videoout_fops = {
    .owner              =   THIS_MODULE,
    .open               =   v4l2_fh_open,
    .release            =   vb2_fop_release,
    .read               =   vb2_fop_read,
    .poll               =   vb2_fop_poll,
    .unlocked_ioctl     =   video_ioctl2,
    .mmap               =   eutecus_vb2_fop_mmap,
};

static const struct video_device videoout_template = {
    .name               =   "PCI-videoout",
    .vfl_type           =   V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING,
    .fops               =   &videoout_fops,
    .release            =   video_device_release_empty,
    .minor              =   -1,
    .ioctl_ops          =   &videoout_ioctl_ops,
};

struct vb2_mem_ops videoout_memops;

static int altera_v4l2_queue_init(struct vb2_queue * q, struct pci_dev * dev)
{
    int status = 0;
    ENTER();

    videoout_dc_init(&videoout_memops);

    q->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE; // will be updated later
    q->io_modes = VB2_MMAP; // We must use our HW address for buffers
    q->drv_priv = dev;
    q->buf_struct_size = sizeof(struct videoout_buffer);
    q->ops = &videoout_video_qops;
    q->mem_ops = &videoout_memops;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
    q->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
#else
    q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
#endif

    status = vb2_queue_init(q);

    LEAVE_V("%d", status);

    return status;
}

static int altera_v4l2_video_device_register(struct videoout_dev * vo)
{
    int status = 0;
    struct video_device * vfd = &vo->vdev;

    ENTER();

    *vfd = videoout_template;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
    vfd->debug = 0;
#else
    vfd->dev_debug = 0;
#endif
    vfd->v4l2_dev = &vo->v4l2_dev;
    vfd->queue = &vo->vb_vidq;
    set_bit(V4L2_FL_USES_V4L2_FH, &vfd->flags);
    vfd->vfl_dir = VFL_DIR_M2M;
    vfd->lock = &vo->mutex;
    video_set_drvdata(vfd, vo);

    status = video_register_device(vfd, VFL_TYPE_GRABBER, -1);

    LEAVE_V("%d", status);

    return status;
}

int altera_v4l2_initialize(struct pci_dev * dev)
{
    int rc = 0;
    struct eutecus_pci_data * data = pci_get_drvdata(dev);
    //const struct eutecus_pci_resources * res = data->resources + 1;
    struct videoout_dev * vo = &data->vidout;

    ENTER();

    snprintf(vo->v4l2_dev.name, sizeof(vo->v4l2_dev.name), "%s-%03d", MY_MODULE_NAME, 0);

    spin_lock_init(&vo->slock);
    mutex_init(&vo->mutex);

    do {
        DEBUG(generic, "KGY: dev=%p, dev->dev=%p, v4l2dev=%p\n", dev, &dev->dev, &vo->v4l2_dev);

        rc = v4l2_device_register(&dev->dev, &vo->v4l2_dev);
        if (rc) {
            ERROR("could not register V4l2 device '%s'\n", vo->v4l2_dev.name);
            break;
        }

        data->v4l2_init_state = 1;

        INIT_LIST_HEAD(&vo->vidq.active);

        do {
            rc = altera_v4l2_queue_init(&vo->vb_vidq, dev);
            if (rc) {
                ERROR("could not initialize vb queue\n");
                break;
            }

            data->v4l2_init_state = 2;

            do {
                rc = altera_v4l2_video_device_register(vo);
                if (rc < 0) {
                    ERROR("could not register video device\n");
                    break;
                }

                data->v4l2_init_state = 3;

                v4l2_info(&vo->v4l2_dev, "V4L2 device registered as %s\n", video_device_node_name(&vo->vdev));

                LEAVE();
                return 0;   // Normal return point

                // ---------------- Error handling below: -------------------------------------------------------------

            } while(0);

            vb2_queue_release(&vo->vb_vidq);

        } while (0);

        v4l2_device_unregister(&vo->v4l2_dev);

    } while (0);

    LEAVE_V("%d", rc);
    return rc;
}

void altera_v4l2_destroy(struct pci_dev * dev)
{
    struct eutecus_pci_data * data = pci_get_drvdata(dev);
    struct videoout_dev * vo = &data->vidout;

    ENTER_V("state=%d", data->v4l2_init_state);

    switch (data->v4l2_init_state) {
        default:
            // Never get here, just for safety.
            ERROR("v4l2 data corruption\n");
        break;

        case 3:
            // Note: video_device_release() must not be called at all because videoout_dev is a member of our structure.
            video_unregister_device(&vo->vdev);
            // No break!
        case 2:
            vb2_queue_release(&vo->vb_vidq);
            // No break!
        case 1:
            v4l2_info(&vo->v4l2_dev, "V4L2 device %s unregistered\n", video_device_node_name(&vo->vdev));
            v4l2_device_unregister(&vo->v4l2_dev);
            data->v4l2_init_state = 0;
        break;

        case 0:
            // Nothing to do here
        break;
    }

    mutex_destroy(&vo->mutex);

    LEAVE();
}

/* * * * * * * * * * * * * End - of - File * * * * * * * * * * * * * * */
