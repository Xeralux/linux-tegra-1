/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * Project:     PCIe pilot with Raggedstone 4 (refs #3992)
 * Purpose:     PCIe communication between the Raggedstone 4 and a PCI root port
 * Author:      György Kövesdi <gyorgy.kovesdi@eutecus.com>
 * License:     Eutecus Proprietary
 * Comments:    
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef __DRIVERS_ALTERA_PCI_EUTECUS_ALTERA_V4L2_IOCTL_H_INCLUDED__
#define __DRIVERS_ALTERA_PCI_EUTECUS_ALTERA_V4L2_IOCTL_H_INCLUDED__

#include <media/v4l2-ioctl.h>
#include <uapi/linux/videodev2.h>

struct plane_info {
    /* Horizontal bpp */
    unsigned int horizontal;

    /* Vertical bpp */
    unsigned int vertical;
};

struct video_data_format {
    /* Descriptive name */
    char name[32];

    /* Average bpp for the whole frame */
    unsigned int bpp;

    u32 fourcc;

    enum v4l2_colorspace colorspace;

    /* Number of planes.
     * If this is >1, then the member 'plane' is in use (see below). */
    unsigned int n_planes;

    enum v4l2_buf_type type;

    /* Info for each plane
     * Used if type==V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE */
    struct plane_info plane[4];
};

extern const struct v4l2_ioctl_ops videoout_ioctl_ops;

#endif /* __DRIVERS_ALTERA_PCI_EUTECUS_ALTERA_V4L2_IOCTL_H_INCLUDED__ */

/* * * * * * * * * * * * * End - of - File * * * * * * * * * * * * * * */
