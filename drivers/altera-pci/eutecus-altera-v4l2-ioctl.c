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
#include "eutecus-altera-v4l2-ioctl.h"

static struct video_data_format formats[] = {
    {
        .name = "4:2:2, packed, UYVY",
        .bpp = 16,
        .fourcc = V4L2_PIX_FMT_UYVY,
        .colorspace = V4L2_COLORSPACE_RAW,
        .n_planes = 1,
        .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
        .frame_intervals = {
            {
                .numerator = 1,
                .denominator = 30,
            },
        },
    },
    {
        .name = "4:2:2, packed, YUYV",
        .bpp = 16,
        .fourcc = V4L2_PIX_FMT_YUYV,
        .colorspace = V4L2_COLORSPACE_RAW,
        .n_planes = 1,
        .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
        .frame_intervals = {
            {
                .numerator = 1,
                .denominator = 30,
            },
        },
    },
    {
        .name = "4:2:0, planar, I420",
        .bpp = 12,
        .fourcc = v4l2_fourcc('I', '4', '2', '0'),
		.colorspace = V4L2_COLORSPACE_RAW,
        .n_planes = 1,  /* one plane on v4l2 layer, multiplane otherwise*/
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
		.frame_intervals = {
            {
                .numerator = 1,
                .denominator = 30,
            },
        },
        .plane = {
            {
                .horizontal = 8,
                .vertical = 8,
            },
            {
                .horizontal = 4,
                .vertical = 4,
            },
            {
                .horizontal = 4,
                .vertical = 4,
            },
        },
    }
};

static int videoout_querycap(struct file * file, void * fh, struct v4l2_capability * cap)
{
    struct videoout_dev * dev = video_get_drvdata(video_devdata(file));
    (void)dev;

    ENTER();

    memset(cap, 0, sizeof(*cap));

    strcpy(cap->driver, "PCI-vidout");
    strcpy(cap->card, "PCI-vidout");
    cap->bus_info[0] = '\0';
    cap->device_caps = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING;
    cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

    LEAVE_V("%d", 0);
    return 0;
}

static int vid_create_bufs(struct file * file, void * priv, struct v4l2_create_buffers * create)
{
    int rs;

    ENTER();
    DEBUG(generic, "file=%p, priv=%p, buf=%p, requested %d buffers...\n", file, priv, create, create->count);

    rs = vb2_ioctl_create_bufs(file, priv, create);

    LEAVE_V("%d (count=%d)", rs, create->count);
    return rs;
}

static int vid_prepare_buf(struct file * file, void * fh, struct v4l2_buffer * b)
{
    int rs;

    ENTER();
    DEBUG(generic, "file=%p, priv=%p, buf=%p \n", file, fh, b);

    rs = vb2_ioctl_prepare_buf(file, fh, b);

    LEAVE_V("%d", rs);
    return rs;
}

static int vid_reqbufs(struct file * file, void * fh, struct v4l2_requestbuffers * b)
{
    int rs;
    struct video_device * vdev = video_devdata(file);
    (void)vdev;

    ENTER();
    DEBUG(generic, "file=%p, priv=%p, buf=%p \n", file, fh, b);

    rs = vb2_ioctl_reqbufs(file, fh, b);

    LEAVE_V("%d", rs);
    return rs;
}

static int vid_querybuf(struct file * file, void * fh, struct v4l2_buffer * b)
{
    int rs;

    ENTER();
    DEBUG(generic, "file=%p, priv=%p, buf=%p \n", file, fh, b);

    rs = vb2_ioctl_querybuf(file, fh, b);

    LEAVE_V("%d", rs);
    return rs;
}

static int vid_qbuf(struct file * file, void * fh, struct v4l2_buffer * b)
{
    int rs;

    ENTER();
    DEBUG(generic, "file=%p, priv=%p, buf=%p \n", file, fh, b);

    rs = vb2_ioctl_qbuf(file, fh, b);

    LEAVE_V("%d", rs);
    return rs;
}

static int vid_dqbuf(struct file * file, void * fh, struct v4l2_buffer * b)
{
    int rs;

    ENTER();
    DEBUG(generic, "file=%p, priv=%p, buf=%p \n", file, fh, b);

    rs = vb2_ioctl_dqbuf(file, fh, b);

    LEAVE_V("%d", rs);
    return rs;
}

static int videoout_streamon(struct file * file, void * priv, enum v4l2_buf_type t)
{
    int rs;

    ENTER();

    switch (t) {
        case V4L2_BUF_TYPE_VIDEO_OUTPUT:
        case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
            /* accepted */
        break;

        default:
            return -EINVAL;
        break;
    }

    rs = vb2_ioctl_streamon(file, priv, t);

    LEAVE_V("%d", rs);
    return rs;
}

static int videoout_streamoff(struct file * file, void * priv, enum v4l2_buf_type t)
{
    int rs;

    ENTER();

    switch (t) {
        case V4L2_BUF_TYPE_VIDEO_OUTPUT:
        case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
            /* accepted */
        break;

        default:
            return -EINVAL;
        break;
    }

    rs = vb2_ioctl_streamoff(file, priv, t);

    LEAVE_V("%d", rs);
    return rs;
}

static int videoout_enum_input(struct file * file, void * fh, struct v4l2_input * inp)
{
    ENTER();

    LEAVE_V("%d", -EINVAL);
    return -EINVAL;
}

static int videoout_enum_framesizes(struct file * file, void * fh, struct v4l2_frmsizeenum * fsize)
{
    ENTER();

    LEAVE_V("%d", -EINVAL);
    return -EINVAL;
}

static int videoout_g_std(struct file * file, void * priv, v4l2_std_id * std)
{
    ENTER();

    *std = V4L2_STD_UNKNOWN;

    LEAVE_V("%d", 0);
    return 0;
}

static int videoout_enum_output(struct file * file, void * fh, struct v4l2_output * out)
{
    struct videoout_dev * dev = video_drvdata(file);
    (void)dev;

    ENTER();

    if (out->index > 0) {
        LEAVE_V("%d", -EINVAL);
        return -EINVAL;
    }

    out->type = V4L2_OUTPUT_TYPE_MODULATOR;
    sprintf(out->name, "FPGA out %u", out->index);

    LEAVE_V("%d", 0);
    return 0;
}

static int videoout_g_output(struct file * file, void * fh, unsigned int * i)
{
    struct videoout_dev * dev = video_drvdata(file);
    (void)dev;

    ENTER();

    *i = 0; // output index

    LEAVE_V("%d", 0);
    return 0;
}

static int videoout_s_output(struct file * file, void * fh, unsigned int i)
{
    struct videoout_dev * dev = video_drvdata(file);
    (void)dev;

    ENTER();

    if (i) {
        LEAVE_V("%d", -EINVAL);
        return -EINVAL;
    }

    LEAVE_V("%d", 0);
    return 0;
}

static int videoout_enum_frameintervals(struct file * file, void * priv, struct v4l2_frmivalenum * fval)
{
    struct videoout_dev * dev = video_drvdata(file);
    struct eutecus_pci_data * pci = container_of(dev, struct eutecus_pci_data, vidout);
    struct eutecus_v4l2_buffers * buf = pci->frame_buffers;
    const struct frame_interval * fi;
    (void)buf;

    ENTER();

    /* We have only one choice: */
    if (fval->index >= MAX_FRAME_INTERVALS) {
        LEAVE_V("%d", -EINVAL);
        return -EINVAL;
    }

    if (!dev->fmt) {
        ERROR("format is not detected properly.\n");
        LEAVE_V("%d", -EINVAL);
        return -EINVAL;
    }

    fi = &dev->fmt->frame_intervals[fval->index];

    if (!fi->numerator || !fi->denominator) {
        LEAVE_V("%d", -EINVAL);
        return -EINVAL;
    }

    fval->type = V4L2_FRMIVAL_TYPE_DISCRETE;

    fval->discrete.numerator = fi->numerator;
    fval->discrete.denominator = fi->denominator;

    LEAVE_V("%d %u/%u FPS", 0, fval->discrete.numerator, fval->discrete.denominator);
    return 0;
};

static int videoout_s_param(struct file * file, void * fh, struct v4l2_streamparm * par)
{
    struct videoout_dev * dev = video_drvdata(file);
    struct eutecus_pci_data * pci = container_of(dev, struct eutecus_pci_data, vidout);
    struct eutecus_v4l2_buffers * buf = pci->frame_buffers;

    ENTER();

    if (!dev->fmt) {
        ERROR("format is not detected properly.\n");
        LEAVE_V("%d", -EINVAL);
        return -EINVAL;
    }

    switch (par->type) {
        case V4L2_BUF_TYPE_VIDEO_OUTPUT:
        case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
            /* accepted */
        break;

        default:
            return -EINVAL;
        break;
    }

    /* The frame interval time: */
    buf->stream.numerator = par->parm.output.timeperframe.numerator;
    buf->stream.denominator = par->parm.output.timeperframe.denominator;

    LEAVE_V("%d %u/%u FPS", 0, buf->stream.numerator, buf->stream.denominator);
    return 0;
}

static int videoout_enum_fmt_video_output(struct file * file, void * fh, struct v4l2_fmtdesc * fmt)
{
    const struct video_data_format * f = formats + fmt->index;
    unsigned int index = fmt->index;

    ENTER_V("idx=%d", fmt->index);

    memset(fmt, 0, sizeof(*fmt));

    if (index >= ARRAY_SIZE(formats)) {
        DEBUG(generic, "fmt index %d is out of range (end of iteration).\n", index);
        LEAVE_V("%d", -EINVAL);
        return -EINVAL;
    }

    DEBUG(generic, "format: '%s'\n", f->name);

    fmt->index = index; // Just write it back
    fmt->type = f->type;
    fmt->pixelformat = f->fourcc;
    strncpy(fmt->description, f->name, sizeof(fmt->description));

    LEAVE_V("%d", 0);
    return 0;
}

/// Check if the format is known
static struct video_data_format * videoout_get_format(struct v4l2_format * fmt)
{
    int i;
    u32 pixelformat = 0U;

    ENTER();

    switch (fmt->type) {
        case V4L2_BUF_TYPE_VIDEO_OUTPUT:
            pixelformat = fmt->fmt.pix.pixelformat;
        break;

        case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
            pixelformat = fmt->fmt.pix_mp.pixelformat;
        break;

        default:
            ERROR("unknown v4l2_format type: %d\n", fmt->type);
            LEAVE_V("%p", NULL);
            return NULL;
        break;
    }

    for (i = 0; i < ARRAY_SIZE(formats); ++i) {
        if (formats[i].fourcc == pixelformat) {
            DEBUG(generic, "found format '%s'\n", formats[i].name);
            LEAVE_V("format at %p", formats + i);
            return formats + i;
        }
    }

    DEBUG(generic, "pixel format '" FOURCC_FORMAT "' not found\n", FOURCC_CHARS(pixelformat));

    LEAVE_V("%p", NULL);
    return NULL;
}

static int videoout_try_fmt_video_output(struct file * file, void * fh, struct v4l2_format * fmt)
{
    struct videoout_dev * dev = video_get_drvdata(video_devdata(file));

    ENTER();

    dev->fmt = videoout_get_format(fmt);
    if (!dev->fmt) {
        LEAVE_V("%d", -EINVAL);
        return -EINVAL;
    }

    switch (fmt->type) {
        case V4L2_BUF_TYPE_VIDEO_OUTPUT:
        {
            int w, h;
            struct v4l2_pix_format * pf = &fmt->fmt.pix;

            w = pf->width;
            h = pf->height;

            w = min(w, MAX_WINDOW_WIDTH);
            w = max(w, 8);
            h = min(h, MAX_WINDOW_HEIGHT);
            h = max(h, 8);

            dev->width = w;
            dev->height = h;

            pf->width = w;
            pf->height = h;
            pf->pixelformat = dev->fmt->fourcc;
            pf->field = V4L2_FIELD_NONE;

            switch (dev->fmt->type) {
                case V4L2_BUF_TYPE_VIDEO_OUTPUT:
                    pf->bytesperline = (pf->width * dev->fmt->bpp) / 8;
                    pf->sizeimage = pf->height * pf->bytesperline;
                break;

                case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
                {
                    /* Use the stride from the first plane (Y), but the imagesize is the same: */
                    const struct plane_info * pi = &dev->fmt->plane[0];
                    pf->bytesperline = (pf->width * pi->horizontal) / 8;
                    pf->sizeimage = pf->height * (pf->width * dev->fmt->bpp) / 8;
                }
                break;

                default:
                    ERROR("Internal error: invalid frame format!\n");
                    LEAVE_V("%d", -EINVAL);
                    return -EINVAL;
                break;
            }

            DEBUG(generic, "using format '%s' and size %dx%d on dev %p (stride=%u, size=%u)\n", dev->fmt->name, w, h, dev, pf->bytesperline, pf->sizeimage);
        }
        break;

        case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
        {
            int w, h;
            struct v4l2_pix_format_mplane * pf = &fmt->fmt.pix_mp;

            w = pf->width;
            h = pf->height;

            w = min(w, MAX_WINDOW_WIDTH);
            w = max(w, 8);
            h = min(h, MAX_WINDOW_HEIGHT);
            h = max(h, 8);

            dev->width = w;
            dev->height = h;

            pf->width = w;
            pf->height = h;
            pf->pixelformat = dev->fmt->fourcc;
            pf->colorspace = dev->fmt->colorspace;
            pf->field = V4L2_FIELD_NONE;
            pf->num_planes = dev->fmt->n_planes;

            switch (dev->fmt->type) {
                case V4L2_BUF_TYPE_VIDEO_OUTPUT:
                {
                    struct v4l2_plane_pix_format * pp = &pf->plane_fmt[0];
                    pp->bytesperline = (pf->width * dev->fmt->bpp) / 8;
                    pp->sizeimage = pf->height * pp->bytesperline;
                }
                break;

                case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
                {
                    unsigned int i;
                    for (i = 0U; i < dev->fmt->n_planes; ++i) {
                        struct v4l2_plane_pix_format * pp = &pf->plane_fmt[i];
                        const struct plane_info * pi = &dev->fmt->plane[i];
                        pp->bytesperline = (pf->width * pi->horizontal) / 8;
                        pp->sizeimage = (pf->height * pp->bytesperline * pi->vertical) / 8;
                    }
                }
                break;

                default:
                    ERROR("Internal error: invalid frame format!\n");
                    LEAVE_V("%d", -EINVAL);
                    return -EINVAL;
                break;
            }

            DEBUG(generic, "using format '%s' and size %dx%d on dev %p (stride={%u,%u,%u,%u} size={%u,%u,%u,%u})\n", dev->fmt->name, w, h, dev, pf->plane_fmt[0].bytesperline, pf->plane_fmt[1].bytesperline, pf->plane_fmt[2].bytesperline, pf->plane_fmt[3].bytesperline, pf->plane_fmt[0].sizeimage, pf->plane_fmt[1].sizeimage, pf->plane_fmt[2].sizeimage, pf->plane_fmt[3].sizeimage);
        }
        break;

        default:
            LEAVE_V("%d", -ENOENT);
            return -ENOENT; /* sanity check only */
        break;
    }

    LEAVE_V("%d", 0);
    return 0;
}

static int videoout_g_fmt_video_output(struct file * file, void * fh, struct v4l2_format * f)
{
    struct v4l2_pix_format_mplane * pf = &f->fmt.pix_mp;
    struct videoout_dev * dev = video_get_drvdata(video_devdata(file));
    const struct video_data_format * fmt = dev->fmt;
    (void)pf;

    ENTER();

    if (!fmt) {
        ERROR("format is not detected properly.\n");
        return -EINVAL;
    }

    if (f->type != fmt->type) {
        ERROR("incompatible v4l2_format type: %d\n", f->type);
        LEAVE_V("%d", -EINVAL);
        return -EINVAL;
    }

    switch (fmt->type) {
        case V4L2_BUF_TYPE_VIDEO_OUTPUT:
        {
            struct v4l2_pix_format * pf = &f->fmt.pix;

            pf->width = dev->width;
            pf->height = dev->height;
            pf->pixelformat = fmt->fourcc;
            pf->colorspace = fmt->colorspace;
            pf->field = V4L2_FIELD_NONE;

            pf->bytesperline = (pf->width * dev->fmt->bpp) / 8;
            pf->sizeimage = pf->height * pf->bytesperline;

            DEBUG(generic, "using format '%s' and size %dx%d on dev %p (stride=%u, size=%u)\n", fmt->name, pf->width, pf->height, dev, pf->bytesperline, pf->sizeimage);
        }
        break;

        case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
        {
            unsigned int i;
            struct v4l2_pix_format_mplane * pf = &f->fmt.pix_mp;

            pf->width = dev->width;
            pf->height = dev->height;
            pf->pixelformat = fmt->fourcc;
            pf->colorspace = fmt->colorspace;
            pf->field = V4L2_FIELD_NONE;
            pf->num_planes = fmt->n_planes;

            for (i = 0U; i < pf->num_planes; ++i) {
                struct v4l2_plane_pix_format * pp = &pf->plane_fmt[i];
                const struct plane_info * pi = &fmt->plane[i];
                pp->bytesperline = (pf->width * pi->horizontal) / 8;
                pp->sizeimage = (pf->height * pp->bytesperline * pi->vertical) / 8;
            }

            DEBUG(generic, "using format '%s' and size %dx%d on dev %p (stride=%u,%u,%u,%u size=%u,%u,%u,%u)\n", fmt->name, pf->width, pf->height, dev, pf->plane_fmt[0].bytesperline, pf->plane_fmt[1].bytesperline, pf->plane_fmt[2].bytesperline, pf->plane_fmt[3].bytesperline, pf->plane_fmt[0].sizeimage, pf->plane_fmt[1].sizeimage, pf->plane_fmt[2].sizeimage, pf->plane_fmt[3].sizeimage);
        }
        break;

        default:
            LEAVE_V("%d", -ENOENT);
            return -ENOENT; /* sanity check only */
        break;
    }

    LEAVE_V("%d", 0);
    return 0;
}

static int videoout_s_fmt_video_output(struct file * file, void * fh, struct v4l2_format * f)
{
    struct videoout_dev * dev = video_get_drvdata(video_devdata(file));
    int rs;

    ENTER();

    rs = videoout_try_fmt_video_output(file, fh, f);
    if (rs) {
        LEAVE_V("%d", rs);
        return rs;
    }

    if (!dev->fmt) {
        ERROR("format is not detected properly.\n");
        LEAVE_V("%d", -EINVAL);
        return -EINVAL;
    }

    /* Overwrite the queue type: */
    dev->vb_vidq.type = f->type;

    DEBUG(generic, "format: '%s', size: %dx%d\n", dev->fmt->name, dev->width, dev->height);

    LEAVE_V("%d", rs);
    return rs;
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

const struct v4l2_ioctl_ops videoout_ioctl_ops = {
    .vidioc_querycap                =   videoout_querycap,

    .vidioc_create_bufs             =   vid_create_bufs,
    .vidioc_prepare_buf             =   vid_prepare_buf,
    .vidioc_reqbufs                 =   vid_reqbufs,
    .vidioc_querybuf                =   vid_querybuf,
    .vidioc_qbuf                    =   vid_qbuf,
    .vidioc_dqbuf                   =   vid_dqbuf,

    .vidioc_streamon                =   videoout_streamon,
    .vidioc_streamoff               =   videoout_streamoff,

    .vidioc_enum_input              =   videoout_enum_input,
    .vidioc_enum_framesizes         =   videoout_enum_framesizes,
    .vidioc_g_std                   =   videoout_g_std,
    .vidioc_s_parm                  =   videoout_s_param,

    .vidioc_enum_output             =   videoout_enum_output,
    .vidioc_g_output                =   videoout_g_output,
    .vidioc_s_output                =   videoout_s_output,

    .vidioc_enum_frameintervals     =   videoout_enum_frameintervals,
    .vidioc_enum_fmt_vid_out        =   videoout_enum_fmt_video_output,
    .vidioc_try_fmt_vid_out         =   videoout_try_fmt_video_output,
    .vidioc_try_fmt_vid_out_mplane  =   videoout_try_fmt_video_output,
    .vidioc_g_fmt_vid_out           =   videoout_g_fmt_video_output,
    .vidioc_g_fmt_vid_out_mplane    =   videoout_g_fmt_video_output,
    .vidioc_s_fmt_vid_out           =   videoout_s_fmt_video_output,
    .vidioc_s_fmt_vid_out_mplane    =   videoout_s_fmt_video_output,
};

/* * * * * * * * * * * * * End - of - File * * * * * * * * * * * * * * */
