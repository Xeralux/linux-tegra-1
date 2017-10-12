/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * Project:     PCIe pilot with Raggedstone 4 (refs #3992)
 * Purpose:     
 * Author:      György Kövesdi <gyorgy.kovesdi@eutecus.com>
 * License:     Eutecus Proprietary
 * Comments:    
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef __DRIVERS_ALTERA_PCI_EUTECUS_V4L2_FRAME_H_INCLUDED__
#define __DRIVERS_ALTERA_PCI_EUTECUS_V4L2_FRAME_H_INCLUDED__

#include <linux/stddef.h>
#include <asm/page.h>

#include "eutecus-v4l2-shared.h"

#define MAX_WINDOW_WIDTH    3840
#define MAX_WINDOW_HEIGHT   1080

#define MIN_BUFFERS 8

/// Calculates the whole buffer size, including header
/*! \param  frame_size  Size of the frame, in bytes
    \retval unsigned    The whole structure size rounded up to the page size */
inline static unsigned long eutecus_v4l2_buffer_size(unsigned long frame_size)
{
    // This is the real size:
    unsigned long size = offsetof(struct eutecus_v4l2_frame, payload[frame_size]);

    // The size must be rounded up to the next page:
    return (size + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);
}

inline static void eutecus_set_v4l2_frame_offset(struct eutecus_v4l2_buffers * buf, unsigned int index, u32 offset)
{
    buf->offset[index] = offset;
}

/// Returns the physical address of the payload by buffer index
/*! \param  buf     The frame buffer structure
    \param  index   Index of the buffer
    \retval u64     Physical address of the payload. It can be used for mapping the buffer for V4L2 operations. */
inline static u64 eutecus_get_v4l2_physical_by_index(struct eutecus_v4l2_buffers * buf, unsigned int index)
{
    return (u64)((struct eutecus_v4l2_buffers *)buf->tegra.kernel_address)->frames[buf->offset[index]].frame->payload;
}

int eutecus_init_v4l2_buffers(struct eutecus_v4l2_buffers * buf, resource_size_t phys_start);

#define FOURCC_CHARS(p) (((p) >> 0) & 0xff), (((p) >> 8) & 0xff), (((p) >> 16) & 0xff), (((p) >> 24) & 0xff)
#define FOURCC_FORMAT   "%c%c%c%c"

#endif /* __DRIVERS_ALTERA_PCI_EUTECUS_V4L2_FRAME_H_INCLUDED__ */

/* * * * * * * * * * * * * End - of - File * * * * * * * * * * * * * * */
