#include <thread>
#include <iostream>
#include <fcntl.h>

#include <giomm.h>
#include <glib-unix.h>

#include "tflow-capture.h"
#include "v4l2Device.h"

#define CAM_DESCR 1

gboolean cam_io_in_dispatch(GSource* g_source, GSourceFunc callback, gpointer user_data)
{
    V4L2Device::GSourceCam* source = (V4L2Device::GSourceCam*)g_source;
    V4L2Device* cam = source->cam;


//    g_info("IO IN DISPATCH ");

    int rc = cam->onBuff();

    //int index;
    //timeval frame_ts;
    //cam->ioctlDequeueBuffer(&index, &frame_ts);

    return G_SOURCE_CONTINUE;
}


gboolean g_cam_io_in_cb(gpointer user_data)
{
    g_info("IO IN CB");
    return 0;
}

//V4L2Device::V4L2Device(TFlowCapture* in_parent)
V4L2Device::V4L2Device(GMainContext* in_context)
{
    context = in_context;
    buf_srv = NULL;

    io_in_tag = NULL;
    io_in_src = NULL;
    CLEAR(gsource_funcs);

    gsource_funcs.dispatch = cam_io_in_dispatch;
    io_in_src = (GSourceCam*)g_source_new(&gsource_funcs, sizeof(GSourceCam));
    io_in_src->cam = this;
    g_source_attach((GSource*)io_in_src, in_context);

}

V4L2Device::~V4L2Device()
{
    if (io_in_src) {
        if (io_in_tag) {
            g_source_remove_unix_fd((GSource*)io_in_src, io_in_tag);
            io_in_tag = nullptr;
        }
        g_source_destroy((GSource*)io_in_src);
        g_source_unref((GSource*)io_in_src);
        io_in_src = nullptr;
    }

    Deinit();
/*
    {
        // Release all previously requested DMA buffers
        // implicit VIDIOC_STREAMOFF.
        v4l2_requestbuffers req;
        int rc;
        CLEAR(req);

        req.count = 0;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;  // Is it necessary then count == 0?
        req.memory = V4L2_MEMORY_MMAP;                  // Is it necessary then count == 0?
        rc = ioctl(dev_fd, VIDIOC_REQBUFS, &req);
        if (-1 == rc) {
            g_warning("clear_me");
        }
        // TODO: Check STREAMOFF
    }
*/
    // TODO: Check STREAMOFF after DMA buffers release
    if (m_isStreamOn) {
        ioctlSetStreamOff();
    }
}

void V4L2Device::redeem(int indx)
{
    // Sanity check ???

    ioctlQueueBuffer(indx);
}
/*
 * The function is a calback called on async VIDIOC_DQBUF completion
 */
int V4L2Device::onBuff()
{
    v4l2_buffer v4l2_buf;
    v4l2_plane mplanes[PLANE_NUM];
    int rc;

    CLEAR(v4l2_buf);
    v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    v4l2_buf.memory = V4L2_MEMORY_MMAP;
    v4l2_buf.m.planes = mplanes;
    v4l2_buf.length = PLANE_NUM;

    //Dequeue buffers
    while (1) {


        rc = ioctl(dev_fd, VIDIOC_DQBUF, &v4l2_buf);
        if (rc == 0) {
            // new buffer received normally
            {
                static int cnt = 0;
                cnt++;
                if ((cnt & 0x3F) == 0) {
                    g_info("Consumed %d frames", cnt);
                }
            }
            buf_srv->consume(v4l2_buf);
        }
        else if (rc == -1) {
            if (errno == EAGAIN) {
                // std::cout << "EAGAIN..." << std::endl;
                break;
            }
            else {
                g_warning("VIDIOC_DQBUF: unexpected error (%d) - %s",
                    errno, strerror(errno));

                // TODO: Close the device?
                // ...
            }
        }
    }

    return 0;

}
// Open video device
int V4L2Device::Open(const char* dev_fname)
{
    int rc = 0;

    dev_fd = open(dev_fname, O_RDWR | O_NONBLOCK);
    if (dev_fd == -1) {
        g_warning("Can't open the video device %s (%d) - %s", 
            dev_fname, errno, strerror(errno));
        return -1;
    }
    m_fname = strdup(dev_fname);

#if CAM_DESCR
    rc |= ioctlQueryCapability();
    rc |= ioctlGetStreamParm();
    rc |= ioctlEnumFmt();
    //    rc |= ioctlGetStreamFmt();
#endif

    return rc;
}

// Close video device
void V4L2Device::Close()
{
    if (dev_fd != -1) {
        if (close(dev_fd) == -1) {
            std::cerr << m_fname << " close device failed" << std::endl;
        }
        dev_fd = -1;
    }
}

// Get device's capabilities
int V4L2Device::ioctlQueryCapability()
{
    v4l2_capability capa;
    CLEAR(capa);

    if (-1 == ioctl(dev_fd, VIDIOC_QUERYCAP, &capa)) {
        perror("VIDIOC_QUERYCAP");
        return -1;
    }
    std::cout << "\nBasic Information:" << "\ndriver=" << capa.driver << "\ncard=" << capa.card
        << "\nbus_info=" << capa.bus_info << "\nversion=" << capa.version << std::hex
        << "\tcapabilities=" << capa.capabilities << std::dec << "\n";

    if (capa.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE == 0) {
        std::cerr << "Device must support capture multiplane feature." << std::endl;
        return -1;
    } else {
        std::cout << "Device supports capture multiplane.\n";
    }
    if (capa.capabilities & V4L2_CAP_STREAMING) {
        std::cout << "Device supports stream.\n";
    }
    return 0;
}

// Query and list the formats supported
int V4L2Device::ioctlEnumFmt()
{
    v4l2_fmtdesc fmtdesc;

    CLEAR(fmtdesc);
    
    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    std::cout << "\nSupport Format:\n";
    while (-1 != ioctl(dev_fd, VIDIOC_ENUM_FMT, &fmtdesc)) {
        v4l2_frmsizeenum frmsize;
        CLEAR(frmsize);
        frmsize.pixel_format = fmtdesc.pixelformat;
        frmsize.index = 0;
        while (-1 != ioctl(dev_fd, VIDIOC_ENUM_FRAMESIZES, &frmsize)) {
            std::cout << "description=" << fmtdesc.description << "\tpixelformat=" << std::oct
                << char(fmtdesc.pixelformat&0xFF) << char((fmtdesc.pixelformat>>8)&0xFF)
                << char((fmtdesc.pixelformat>>16)&0xFF) << char((fmtdesc.pixelformat>>24)&0xFF)
                << std::dec << "\tWxH=" << frmsize.discrete.width << "x" << frmsize.discrete.height
                // << "\tflags=" << fmtdesc.flags << "\ttype=" << frmsize.type
                << "\n";

            frmsize.index++;
        }
        fmtdesc.index++;
    }

    return 0;
}

// Get current format and frame rate
int V4L2Device::ioctlGetStreamParm()
{
    v4l2_streamparm streamparm;
    CLEAR(streamparm);

    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (-1 == ioctl(dev_fd, VIDIOC_G_PARM, &streamparm)) {
        g_warning("Can't VIDIOC_G_PARM (%d)", errno);
        return -1;
    }
    
    std::cout << "\nCapture Streamparm:\n"
        << "Capture capability=" << streamparm.parm.capture.capability
        << "\tCapture mode=" << streamparm.parm.capture.capturemode
        << "\tFrame Rate=" << streamparm.parm.capture.timeperframe.numerator << "/"
        << streamparm.parm.capture.timeperframe.denominator << "\n";

    return 0;
}

int V4L2Device::ioctlSetStreamParm(bool highQuality, u_int timeperframe)
{
    v4l2_streamparm streamparm;
    CLEAR(streamparm);
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    streamparm.parm.capture.capturemode = V4L2_CAP_TIMEPERFRAME | (u_char)highQuality;
    streamparm.parm.capture.timeperframe.numerator = 1;
    streamparm.parm.capture.timeperframe.denominator = timeperframe;
    if (-1 == ioctl(dev_fd, VIDIOC_S_PARM, &streamparm)) {
        perror("VIDIOC_S_PARM");
        return -1;
    }
    return 0;
}

// Get format
int V4L2Device::ioctlGetStreamFmt()
{
    v4l2_format fmt;
    CLEAR(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (-1 == ioctl(dev_fd, VIDIOC_G_FMT, &fmt)) {
        perror("VIDIOC_G_FMT");
        return -1;
    }
    std::cout << "\nCurrent pixformat:\n"
        << "WxH=" << fmt.fmt.pix_mp.width << "x" << fmt.fmt.pix_mp.height
        << "\tpixelformat=" << std::oct
        << char(fmt.fmt.pix_mp.pixelformat&0xFF) << char((fmt.fmt.pix_mp.pixelformat>>8)&0xFF)
        << char((fmt.fmt.pix_mp.pixelformat>>16)&0xFF) << char((fmt.fmt.pix_mp.pixelformat>>24)&0xFF)
        << std::dec
        << "\tfield=" << fmt.fmt.pix_mp.field
        << "\tbytesperline=" << fmt.fmt.pix.bytesperline
        << "\tsizeimage=" << fmt.fmt.pix.sizeimage
        << "\tcolorspace=" << fmt.fmt.pix_mp.colorspace << "\n";

    return 0;
}

int V4L2Device::ioctlSetStreamFmt(u_int pixelformat, u_int width, u_int height)
{
    v4l2_format fmt;
    CLEAR(fmt);

#if 1    
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (-1 == ioctl(dev_fd, VIDIOC_G_FMT, &fmt)) {
        g_warning("Can't VIDIOC_G_FMT (%d)", errno);
        return -1;
    }

    if ((width > fmt.fmt.pix_mp.width) || (height > fmt.fmt.pix_mp.height)) {
        std::cerr << m_fname << ": out of support range." << std::endl;
        return -1;
    }
#endif

    fmt.fmt.pix_mp.width = width;
    fmt.fmt.pix_mp.height = height;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.num_planes = 1;
    fmt.fmt.pix_mp.pixelformat = pixelformat;
    fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
    if (-1 == ioctl(dev_fd, VIDIOC_S_FMT, &fmt)) {
        g_warning("Can't VIDIOC_S_FMT (%d)", errno);
        return -1;
    }
    return 0;
}

void V4L2Device::DeinitBuffers()
{
    v4l2_requestbuffers req;
    int rc;
    
    /*
     * Unmap DMA memory
     */
    for (auto &buf : m_buffers) {
        if (buf.start) {
            munmap(buf.start, buf.length);
            buf.start = NULL;
        }
    }

    /*
     * Returns all memory buffers to the Kernel
     */

    // Release all previously requested DMA buffers
    // implicit VIDIOC_STREAMOFF.
    CLEAR(req);
    req.count = 0;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;  // Is it necessary then count == 0?
    req.memory = V4L2_MEMORY_MMAP;                  // Is it necessary then count == 0?
    rc = ioctl(dev_fd, VIDIOC_REQBUFS, &req);
    if (-1 == rc) {
        g_warning("clear me as we don't care");
    }
    // TODO: Check STREAMOFF
    // TODO: Check double release (i.e. close application at stoped camera)
}

// Request buffers in kernel space
int V4L2Device::InitBuffers()
{
    int rc = 0;
    v4l2_requestbuffers req;
    CLEAR(req);

    /* 
     * Request buffers 
     */
    req.count = BUFFER_NUM;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    if (-1 == ioctl(dev_fd, VIDIOC_REQBUFS, &req)) {
        g_warning("Can't VIDIOC_REQBUFS (%d)", errno);
        return -1;
    }

    /*
     * Map DMA memory to the user space
     */

    for(int n = 0; n < BUFFER_NUM; n++) {
        v4l2_buffer buf;
        v4l2_plane mplanes[PLANE_NUM];
        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n;
        buf.m.planes = mplanes;
        buf.length = PLANE_NUM;

        // Query the information of the buffer with index=n into struct buf
        if (-1 == ioctl(dev_fd, VIDIOC_QUERYBUF, &buf)) {
            g_warning("Can't VIDIOC_QUERYBUF (%d)", errno);
            return -1;
        }

        // Record the length and mmap buffer to user space
        m_buffers[n].length = buf.m.planes[0].length;
        m_buffers[n].start = mmap(NULL, buf.m.planes[0].length, 
                PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, buf.m.planes[0].m.mem_offset); 

        if (MAP_FAILED == m_buffers[n].start) {
            g_warning("Can't MMAP (%d)", errno);
            return -1;
        }
    }

    for (int i = 0; i < BUFFER_NUM; i++) {
        rc != ioctlQueueBuffer(i);
    }

    return rc;
}

// Enqueue a buffer
int V4L2Device::ioctlQueueBuffer(int index)
{
    v4l2_buffer buf;
    v4l2_plane mplanes[PLANE_NUM];
    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.m.planes = mplanes;
    buf.length = PLANE_NUM;
    buf.index = index;

    // Buffer back to queue
    if (-1 == ioctl(dev_fd, VIDIOC_QBUF, &buf)) {
        std::cout << "VIDIOC_QBUF error: buffer " << index << " already enqueue." << std::endl;
        return -1;
    }
    return 0;
}

// Dequeue a buffer /read a frame
int V4L2Device::ioctlDequeueBuffer(int *index, struct timeval *ts)
{
    v4l2_buffer buf;
    v4l2_plane mplanes[PLANE_NUM];
    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.m.planes = mplanes;
    buf.length = PLANE_NUM;
    
    //Dequeue a buffer with captured frame
    if (-1 == ioctl(dev_fd, VIDIOC_DQBUF, &buf)) {
        perror("VIDIOC_DQBUF");
        return -1;
    }

    *index = buf.index;
    *ts = buf.timestamp;
//    frame = (u_char*)m_buffers[buf.index].start; // Point to the buffer where a frame is captured
//    size = m_buffers[buf.index].length;

    return 0;
}

// Start stream
int V4L2Device::StreamOn(int fmt_idx)
{
    int rc = 0;

    // TODO: get format by index
    //       v4l2_format fmt = fmt_enum[fmt_idx]
    //       ioctlSetStreamFmt(fmt)
    rc |= ioctlSetStreamFmt(V4L2_PIX_FMT_GREY, IMAGEWIDTH, IMAGEHEIGHT);

    if (rc) {
        return -1;
    }
    rc |= InitBuffers();

    if (m_isStreamOn) {
        return 0;
    }
    v4l2_buf_type type { V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
    if (-1 == ioctl(dev_fd, VIDIOC_STREAMON, &type)) {
        g_warning("VIDIOC_STREAMON");
        return -1;
    }
    
    m_isStreamOn = true;

    // TODO: Preserve currently used format ?

    io_in_tag = g_source_add_unix_fd((GSource*)io_in_src, dev_fd, (GIOCondition)(G_IO_IN | G_IO_ERR));

    return 0;
}

// Stop stream
void V4L2Device::ioctlSetStreamOff()
{
    v4l2_buf_type type {V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE};
    while (-1 == ioctl(dev_fd, VIDIOC_STREAMOFF, &type)) {
        perror("VIDIOC_STREAMOFF");
        return;
    }

    m_isStreamOn = false;
}


void* V4L2Device::buffGet(int i)
{
    return m_buffers[i].start;
}

void V4L2Device::Deinit()
{
    DeinitBuffers();

    Close();
}

int V4L2Device::Init(const char *dev_name)
{
    int rc;

    if (!dev_name) {
        g_warning("Device name isn't specified");
        return -1;
    }

    rc = Open(dev_name);
    if (rc) return rc;

    // TODO: Compare the current ENUM list in the configuration with the 
    //       received one. 
    //       What if differs? compare enum at particular index?
    //       Update Ctrl with formats enumerated from the device
    //       ? Ctrl must send this enum to the UI ?

    return 0;
}
