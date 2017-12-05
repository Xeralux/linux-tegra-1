/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * Project:     Merlin: Communication between Tegra and Cyclone-5
 * Purpose:     Some structures shared between two platforms
 * Author:      György Kövesdi <gyorgy.kovesdi@eutecus.com>
 * License:     Eutecus Proprietary
 * Comments:    The structures of this include file are compiled for two different platforms
 *              (64-bit Tegra and 32-bit CycloneV) and the result must be exactly the same.
 *              See the comments about alignment of the corresponding structures.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef __DRIVERS_ALTERA_PCI_EUTECUS_V4L2_SHARED_H_INCLUDED__
#define __DRIVERS_ALTERA_PCI_EUTECUS_V4L2_SHARED_H_INCLUDED__

#include <linux/types.h>
#include <linux/time.h>
#include <linux/videodev2.h>

#define EUTECUS_MAX_NUMBER_OF_FRAMES    16

#ifndef PACKED
#define PACKED __attribute__ ((__packed__))
#endif

struct videoout_buffer;

enum eutecus_v4l2_frame_state {
    /// Frame is in initial state
    /*! The frame is just created. It is set when the Tegra side is started. Such a frame
        cannot be used while the Cyclone side sets it to \ref FRAME_FREE */
    FRAME_INITIAL = 0,

    /// Frame is ready to be processed
    /*! The CycloneV has accepted a \ref FRAME_INITIAL frame */
    FRAME_FREE,

    /// Frame is available for V4L2
    /*! internal state change by Tegra: the buffer is made available for the V4L2 system, at startup or the buffer back from SoCFPGA */
    FRAME_USER,

    /// There is a new frame in the buffer
    /*! It means that there is an image in this buffer (put by Tegra) and can be processed by the socfpga */
    FRAME_TO_CONVERT,

    /// Frame is being processed by color converter
    /*! The SocFPGA has accepted the buffer and the color converter started to work on the frame */
    FRAME_CONVERTING,

    /// Frame is processed by color converter
    /*! The SocFPGA has accepted the buffer and the color converter started to work on the frame */
    FRAME_CONVERTED,

    /// Last entry, just for size check
    _FRAME_STATE_SIZE
};

/// Frame Info and Flags
struct eutecus_v4l2_header {
    /// The full size of the frame structure, including this header
    u32 full_size;

    /// The payload size of the frame
    u32 frame_size;

    /// Serial number (timestamp) of the frame
    /*! \see \ref next_serial for more details */
    u32 serial;

    /// State of this frame
    /*! \see    The possible values are in \ref eutecus_v4l2_frame_state */
    u32 state;

    /// Parameters for CycloneV side
    /*! This information is relevant only for CycloneV side and pointless on Tegra side. */
    struct {
        /// Kernel physical address of the payload
        /*! \note   It is necessary for memory mapping. */
        u32 dma_address;

        /// Pointer to corresponding vb2_buffer
        u32 vob;

    } PACKED cycv;

    /// Parameters for Tegra side
    /*! This information is relevant only for Tegra side and pointless on CycloneV side. */
    struct {
        /// Kernel physical address of the payload
        /*! \note   It is necessary for memory mapping. */
        u64 kernel_address;

        /// The actual buffer of this frame
        u64 vob;

    } PACKED tegra;

    /// Timestamp passthrough
    /*! This information comes from the v4l2 frame header on the Tegra side, and passed to the CycloneV side as is. */
    struct {
        /// Seconds of v4l2 timestamp
        /*! Note: the original struct timeval cannot be used here because it is treated differently on 32/64 bit platforms. */
        s64 seconds;

        /// Microseconds of v4l2 timestamp
        s64 microseconds;

        /// v4l2 timecode
        struct v4l2_timecode timecode;

        /// v4l2 frame sequence number
        u32 sequence;

        /// v4l2 frame index
        u32 index;

        /// v4l2 frame flags
        u32 flags;

        /// v4l2 frame field
        u32 field;

    } PACKED;

} PACKED;

/// Buffer and flags for one frame
struct eutecus_v4l2_frame {
    union {
        /// Administrative information about this buffer
        struct eutecus_v4l2_header header;

        char _dummy[PAGE_SIZE]; // The payload is put one page below: that is DMA-able this way

    } PACKED;

    /// The raw frame data
    char payload[0];

} PACKED;

/// Readable names for \ref eutecus_v4l2_frame_state values
#define FRAME_NAMES { "initial", "free", "user", "to_convert", "converting", "converted" }

/// Return readable name of the frame state
static inline const char * get_shared_frame_state_name(const struct eutecus_v4l2_frame * f)
{
    u32 state = f->header.state;
    static char * names[] = FRAME_NAMES;
    if (state >= _FRAME_STATE_SIZE) {
        return "unknown";
    }
    return names[state];
}

/// All buffers
/*! This structure is stored in the shared (FPGA) memory. It manages the frame buffers.
    \note   The whole structure is packed (aligned to 1) because the same structure is compiled to two different
            platforms (which may have different alignments). Some padding data must be added to set up the alignment
            correctly.
    \note   Because this memory can be accessed by two ports, the proper initialization is problematic.<br>
            A simple way to do it well is clearing this area by zeroes in the bootloader (u-boot). */
struct eutecus_v4l2_buffers {
    union {
        struct {
            /// Number of entries used in the \ref offset[] array
            u32 volatile indices_used;

            /// Byte offset of the first free entry in the \ref frames[] array
            u32 next_offset;

            /// Current serial number (timestamp) of the frame
            /*! \note   It is calculated by the sink-side driver, mainly for debug purposes. The driver prints this serial number in the
                        debug messages, but the V4l2 system has its own serial number. */
            u32 next_serial;

            /// Frames dropped by the tegra because the color converter is too slow
            u32 frames_dropped_by_tegra;

            /// Frames received on the Tegra side
            u32 number_of_input_frames;

            /// Input framerate
            s32 input_fps;

            /// Offsets of frames within this structure
            /*! The array \ref frames[] can be accessed by these values to get frame structures (see \ref eutecus_v4l2_frame). */
            u32 offset[(EUTECUS_MAX_NUMBER_OF_FRAMES+1)&~1];

            /// Actual stream properties
            struct {
                u32 width;              ///< Frame Width of the current stream
                u32 height;             ///< Frame Height of the current stream
                u32 fourcc;             ///< Pixel Format of the current stream
                u32 numerator;          ///< FPS numerator
                u32 denominator;        ///< FPS denominator
                u32 active;             ///< Nonzero if the stream is running

            } PACKED stream;

            /// Parameters for CycloneV side
            /*! This information is relevant only for CycloneV side and pointless on Tegra side. */
            struct {
                /// Kernel physical address of the whole framebuffer structure on the Cyclone-5 side
                /*! \note   It is necessary for memory mapping. */
                u32 kernel_address;

                /// It is used for dc buffer initialization
                u32 frame_index;

            } PACKED cycv;

            /// Parameters for Tegra side
            /*! This information is relevant only for Tegra side and pointless on CycloneV side. */
            struct {
                /// Kernel physical address of the whole framebuffer structure on the Tegra side
                /*! \note   It is necessary for memory mapping. */
                u64 kernel_address;

                /// Pointer back to the parent structure
                struct eutecus_pci_data * pci;

            } PACKED tegra;

        } PACKED;

        char _dummy[PAGE_SIZE]; // The frames are put one page below: keep DMA-able this way

    } PACKED;

    struct {
        struct eutecus_v4l2_frame frame[0];

        char _dummy; // just to take one byte space

    } PACKED frames[0];

} PACKED;

inline static struct eutecus_v4l2_frame * eutecus_get_v4l2_frame_by_index(struct eutecus_v4l2_buffers * buf, unsigned int index)
{
    return buf->frames[buf->offset[index]].frame;
}

#endif /* __DRIVERS_ALTERA_PCI_EUTECUS_V4L2_SHARED_H_INCLUDED__ */

/* * * * * * * * * * * * * End - of - File * * * * * * * * * * * * * * */
