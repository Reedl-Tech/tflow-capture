// 2) Consider usage TFlowBuf[] inside of V4L2_Device
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

    int rc = cam->onBuff();
    if (rc) {
        // Close camera???
        // return G_SOURCE_REMOVE;
    }

    return G_SOURCE_CONTINUE;
}

V4L2Device::V4L2Device(GMainContext* _context, int _buffs_num, int _planes_num)
{
    context = _context;
    buffs_num = _buffs_num;
    planes_num = _planes_num;

    buf_srv = NULL;

    io_in_tag = NULL;
    io_in_src = NULL;
    CLEAR(gsource_funcs);

    gsource_funcs.dispatch = cam_io_in_dispatch;
    io_in_src = (GSourceCam*)g_source_new(&gsource_funcs, sizeof(GSourceCam));
    io_in_src->cam = this;
    g_source_attach((GSource*)io_in_src, _context);

    CLEAR(v4l2_buf_template);
    
    v4l2_plane empty_plane {};
    mplanes_template = std::vector<v4l2_plane>(planes_num, empty_plane);

    v4l2_buf_template.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    v4l2_buf_template.memory = V4L2_MEMORY_MMAP;
    v4l2_buf_template.m.planes = mplanes_template.data();
    v4l2_buf_template.length = planes_num;
    v4l2_buf_template.index = 0;
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

    if (is_streaming) {
        ioctlSetStreamOff();
    }

    Deinit();
}

/*
 * The function is a calback called on async VIDIOC_DQBUF completion
 */
int V4L2Device::onBuff()
{
    int rc;

    //Dequeue all buffers
    v4l2_buffer v4l2_buf = v4l2_buf_template; // TODO: Q: can it be reused without reinitialization?
    while (1) {
        // ? reinit v4l2_buf ?
        rc = ioctlDequeueBuffer(v4l2_buf);
        if (rc == 0) {

            if (v4l2_buf.index == -1) {
                return 0;       // No buffers - It is OK, everything read out. 
            }
            // TODO: check - is it possible to receive veeeery old buffer in
            //       case of buffer not enqueued back for a long time?
            //       If yes, the obsolete buffers need to be dropped.

            rc = buf_srv->buf_consume(v4l2_buf);
        }

        if (rc) {
            // Q: ? Close camera ?
            return rc;
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
        close(dev_fd);
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
    v4l2_requestbuffers req{};
    v4l2_requestbuffers req_delme;  // AV: TODO: check zero initialization
    CLEAR(req);

    /* 
     * Request buffers 
     */
    req.count = buffs_num;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    if (-1 == ioctl(dev_fd, VIDIOC_REQBUFS, &req)) {
        g_warning("Can't VIDIOC_REQBUFS (%d)", errno);
        return -1;
    }

    buf_srv->buf_create(buffs_num);

    return rc;
}

// Enqueue a buffer
int V4L2Device::ioctlQueueBuffer(int index)
{
    v4l2_buffer v4l2_buf = v4l2_buf_template;
    v4l2_buf.index = index;

    // Put the buffer back to kernel's queue
    if (-1 == ioctl(dev_fd, VIDIOC_QBUF, &v4l2_buf)) {
        g_warning("V4L2Device: Ooops VIDIOC_QBUF @%d", index);
        return -1;
    }
    return 0;
}

int V4L2Device::ioctlDequeueBuffer(v4l2_buffer &v4l2_buf)
{
    int rc;
    
    //Dequeue a buffer with captured frame
    rc = ioctl(dev_fd, VIDIOC_DQBUF, &v4l2_buf);
    if (rc == -1) {
        if (errno == EAGAIN) {
            v4l2_buf.index = -1;
            return 0;
        }
        else {
            g_warning("VIDIOC_DQBUF: unexpected error (%d) - %s", errno, strerror(errno));

            // TODO: Close the device?
            // ...
            return -1;
        }
    }

    return 0;
}

int V4L2Device::StreamOn(int fmt_idx)
{
    int rc = 0;

    // TODO: get format by index
    //       v4l2_format fmt = fmt_enum[fmt_idx]
    //       ioctlSetStreamFmt(fmt)
    rc |= ioctlSetStreamFmt(V4L2_PIX_FMT_GREY, IMAGEWIDTH, IMAGEHEIGHT);

    frame_format = V4L2_PIX_FMT_GREY;
    frame_width = IMAGEWIDTH;
    frame_height = IMAGEHEIGHT;
    
    if (rc) {
        return -1;
    }
    rc |= InitBuffers();

    if (is_streaming) {
        return 0;
    }
    v4l2_buf_type type { V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
    if (-1 == ioctl(dev_fd, VIDIOC_STREAMON, &type)) {
        g_warning("VIDIOC_STREAMON");
        return -1;
    }
    
    is_streaming = true;

    // TODO: Preserve currently used format ?

    io_in_tag = g_source_add_unix_fd((GSource*)io_in_src, dev_fd, (GIOCondition)(G_IO_IN | G_IO_ERR));
#if CODE_BROWSE
    cam_io_in_dispatch();
#endif

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

    is_streaming = false;
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
