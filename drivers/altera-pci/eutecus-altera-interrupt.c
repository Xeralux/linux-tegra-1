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

#include <asm/delay.h>

static int videoout_interrupt_from_analitics(struct eutecus_pci_data * data)
{
    struct eutecus_v4l2_buffers * buf = data->frame_buffers;
    u32 i;

    if (buf->indices_used > EUTECUS_MAX_NUMBER_OF_FRAMES) {
        ERROR("The PCI memory map is probably lost!\n");
        return -ENODEV;
    }

    for (i = 0; i < buf->indices_used; ++i) {
        struct eutecus_v4l2_frame * frame = eutecus_get_v4l2_frame_by_index(buf, i);

        if (((u64)frame) & (PAGE_SIZE-1)) {
            ERROR("unaligned frame: index=%u of %u, at %p, offset=%u (internal driver error)\n", i, buf->indices_used, frame, buf->offset[i]);
            return 0;
        }

        // DEBUG(interrupt, "frame #%u is %s at %p\n", frame->header.serial, get_shared_frame_state_name(frame), frame);
        switch (frame->header.state) {
            case FRAME_READY:
                if (frame->header.tegra.vob) {
                    struct videoout_buffer * vob = (struct videoout_buffer *)frame->header.tegra.vob;
                    if (!vob->queued) {
                        /* According to my measurements, such a call needs 18 microsecs time. If it is too
                         * much, it can be moved into a thread instead of calling it directly. */
                        videoout_buffer_done(vob, VB2_BUF_STATE_DONE);
                        DEBUG(video, "frame #%u is DONE (%s -> free) at %p\n", frame->header.serial, get_shared_frame_state_name(frame), frame);
                    } else {
                        DEBUG(video, "frame #%u has already been queued (%s -> free) at %p\n", frame->header.serial, get_shared_frame_state_name(frame), frame);
                    }
                } else {
                    DEBUG(video, "frame #%u first time (%s -> free) at %p\n", frame->header.serial, get_shared_frame_state_name(frame), frame);
                }

                frame->header.state = FRAME_FREE;
            break;

            default:
                /* Nothing to do here */
            break;
        }
    }

    return 0;
}

irqreturn_t eutecus_pci_isr(int this_irq, void * param)
{
    struct pci_dev * dev = param;
    struct eutecus_pci_data * data = pci_get_drvdata(dev);

    if (data->frame_buffers) {
        if (videoout_interrupt_from_analitics(data)) {
            ERROR("To prevent further problems, the device interrupt (#%d) is disabled. The V4l2 connection will not work. This module must be reloaded.\n", (int)data->irq);
            free_irq(data->irq, dev);
            data->irq = 0;
        }
    }

    interrupt_acknowledge_2_RS4(data);

    return IRQ_HANDLED;
}

/* * * * * * * * * * * * * End - of - File * * * * * * * * * * * * * * */
