#include <thread>
#include <iostream>
#include <fcntl.h>

#include <giomm.h>
#include <glib-unix.h>

#include "v4l2Device.h"

#define CAM_DESCR 1

V4L2Device::V4L2Device()
{
    m_buffers = 0;
}

V4L2Device::~V4L2Device()
{
    if (m_isStreamOn) {
        ioctlSetStreamOff();
    }

    unMmapBuffers();
    if (m_buffers) {
        free(m_buffers);
    }
    if (m_fname) free((void*)m_fname);

    closeDevice();
}

// Open video device
int V4L2Device::openDevice(const char* filename)
{
    cam_fd = open(filename, O_RDWR); // By default with block mode
    if (cam_fd == -1) {
        std::cerr << "Cann't open video device " << filename << std::endl;
        return -1;
    }
    m_fname = strdup(filename);

    return 0;
}

// Close video device
void V4L2Device::closeDevice()
{
    if (m_fd != -1) {
        if (close(m_fd) == -1) {
            std::cerr << m_fname << " close device failed" << std::endl;
        }
        m_fd = -1;
    }
}

// Get device's capabilities
int V4L2Device::ioctlQueryCapability()
{
    v4l2_capability capa;
    CLEAR(capa);

    if (-1 == ioctl(m_fd, VIDIOC_QUERYCAP, &capa)) {
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
    while (-1 != ioctl(m_fd, VIDIOC_ENUM_FMT, &fmtdesc)) {
        v4l2_frmsizeenum frmsize;
        CLEAR(frmsize);
        frmsize.pixel_format = fmtdesc.pixelformat;
        frmsize.index = 0;
        while (-1 != ioctl(m_fd, VIDIOC_ENUM_FRAMESIZES, &frmsize)) {
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
    if (-1 == ioctl(m_fd, VIDIOC_G_PARM, &streamparm)) {
        perror("VIDIOC_G_PARM");
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
    if (-1 == ioctl(m_fd, VIDIOC_S_PARM, &streamparm)) {
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
    if (-1 == ioctl(m_fd, VIDIOC_G_FMT, &fmt)) {
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

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (-1 == ioctl(m_fd, VIDIOC_G_FMT, &fmt)) {
        perror("VIDIOC_G_FMT");
        return -1;
    }

    if ((width > fmt.fmt.pix_mp.width) || (height > fmt.fmt.pix_mp.height)) {
        std::cerr << m_fname << ": out of support range." << std::endl;
        return -1;
    }

    fmt.fmt.pix_mp.width = width;
    fmt.fmt.pix_mp.height = height;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.num_planes = 1;
    fmt.fmt.pix_mp.pixelformat = pixelformat;
    fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
    if (-1 == ioctl(m_fd, VIDIOC_S_FMT, &fmt)) {
        perror("VIDIOC_S_FMT");
        return -1;
    }
    return 0;
}

// Request buffers in kernel space
int V4L2Device::ioctlRequestBuffers()
{
    v4l2_requestbuffers req;
    CLEAR(req);

    req.count = BUFFER_NUM;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    if (-1 == ioctl(m_fd, VIDIOC_REQBUFS, &req)) {
        perror("VIDIOC_REQBUFS");
        return -1;
    }

    m_buffers = (struct Buffer*)calloc(req.count, sizeof(struct Buffer));
    if (!m_buffers) {
        perror("calloc");
        return -1;
    }
    return 0;
}

// Query the infomation of buffers in kernel, and memory map them to user space
int V4L2Device::ioctlMmapBuffers()
{
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
        if (-1 == ioctl(m_fd, VIDIOC_QUERYBUF, &buf)) {
            perror("VIDIOC_QUERYBUF");
            return -1;
        }
        // Record the length and mmap buffer to user space
        m_buffers[n].length = buf.m.planes[0].length;
        m_buffers[n].start = mmap(NULL, buf.m.planes[0].length, 
                PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, buf.m.planes[0].m.mem_offset); 
        if (MAP_FAILED == m_buffers[n].start) {
            perror("mmap");
            return -1;
        }
    }

    return 0;
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
    if (-1 == ioctl(m_fd, VIDIOC_QBUF, &buf)) {
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
    if (-1 == ioctl(m_fd, VIDIOC_DQBUF, &buf)) {
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
int V4L2Device::ioctlSetStreamOn()
{
    if (m_isStreamOn) {
        return 0;
    }
    v4l2_buf_type type { V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
    if (-1 == ioctl(m_fd, VIDIOC_STREAMON, &type)) {
        perror("VIDIOC_STREAMON");
        return -1;
    }
    
    m_isStreamOn = true;
    return 0;
}

// Stop stream
void V4L2Device::ioctlSetStreamOff()
{
    v4l2_buf_type type {V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE};
    while (-1 == ioctl(m_fd, VIDIOC_STREAMOFF, &type)) {
        perror("VIDIOC_STREAMOFF");
        return;
    }

    m_isStreamOn = false;
}

 // Unmap buffers 
void V4L2Device::unMmapBuffers()
{
    if (m_buffers) {
        if (m_buffers[0].start != NULL) {
            for (int i{ 0 }; i < BUFFER_NUM; ++i) {
                munmap(m_buffers[i].start, m_buffers[i].length);
            }
            m_buffers[0].start = NULL;
        }
    }
}

void* V4L2Device::buffGet(int i)
{
    if (m_buffers) {
        return m_buffers[i].start;
    }
    return nullptr;
}

int V4L2Device::CaptureInit(const char *dev_name)
{
    int rc;

    rc = openDevice(dev_name);
    if (rc) return rc;

#if CAM_DESCR
    rc |= ioctlQueryCapability();
    rc |= ioctlGetStreamParm();
    rc |= ioctlEnumFmt();
    rc |= ioctlGetStreamFmt();
#endif
    rc |= ioctlSetStreamFmt(V4L2_PIX_FMT_GREY, IMAGEWIDTH, IMAGEHEIGHT);
    rc |= ioctlRequestBuffers();
    if (rc) return rc;

    rc = ioctlMmapBuffers();
    if (rc) return rc;

    for (u_int i = 0; i < BUFFER_NUM; i++) {
        rc = ioctlQueueBuffer(i);
        if (rc) return rc;
    }

    return 0;
}
