// 2) Consider usage TFlowBuf[] inside of V4L2_Device
#include <thread>
#include <iostream>
#include <fcntl.h>

#include <giomm.h>
#include <glib-unix.h>

#include "tflow-capture.h"
#include "v4l2Device.h"

#define CAM_DESCR 1

V4L2Device::V4L2Device(MainContextPtr _context, 
    int _buffs_num, int _planes_num,
    const TFlowCtrlCapture::cfg_v4l2_ctrls* _cfg)
{
    context = _context;
    buffs_num = _buffs_num;
    planes_num = _planes_num;
    cfg = _cfg;

    is_streaming = false;

    buf_srv = NULL;

    CLEAR(v4l2_buf_template);
    
    v4l2_plane empty_plane {};
    mplanes_template = std::vector<v4l2_plane>(planes_num, empty_plane);

    v4l2_buf_template.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    v4l2_buf_template.memory = V4L2_MEMORY_MMAP;
    v4l2_buf_template.m.planes = mplanes_template.data();
    v4l2_buf_template.length = planes_num;
    v4l2_buf_template.index = 0;

    stat_cnt_frames_num = 0;

    streaming_watchdog_cnt = 0;         // Watchdog is enabled on StreamOn
    streaming_watchdog_frames_num =0;

    dev_fd = -1;        // ISI control
    sub_dev_fd = -1;    // Camera sensor

    flyn_calib_is_on = -1;      // -1 Unknown (will be updated from ioctl); 0 - off; 1 - on

}

V4L2Device::~V4L2Device()
{
    if (is_streaming) {
        ioctlSetStreamOff();
    }

    Deinit();
}

/*
 * The function is a calback called on async VIDIOC_DQBUF completion
 */
int V4L2Device::onBuff(Glib::IOCondition G_IO_IN)
{

    // TODO: Check camera state 
    // ... 
   
    //Dequeue all buffers
    v4l2_buffer v4l2_buf = v4l2_buf_template; // TODO: Q: can it be reused without reinitialization?
    while (1) {
        int rc = ioctlDequeueBuffer(v4l2_buf);
        if (rc == 0) {

            if (v4l2_buf.index == -1) {
                return G_SOURCE_CONTINUE;       // No buffers - It is OK, everything read out. 
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

    return G_SOURCE_CONTINUE;

}
// Open video device
int V4L2Device::Open()
{
    int rc = 0;

    dev_fd = open(cfg->dev_name.v.str, O_RDWR | O_NONBLOCK);
    if (dev_fd == -1) {
        g_warning("Can't open the video device %s (%d) - %s", 
            cfg->dev_name.v.str, errno, strerror(errno));
        return -1;
    }

    // TODO: Get devices name as a function from /dev/videoX
    sub_dev_fd = open(cfg->sub_dev_name.v.str, O_RDWR | O_NONBLOCK);
    if (sub_dev_fd == -1) {
        g_warning("Can't open the sub video device %s (%d) - %s", 
            cfg->sub_dev_name.v.str, errno, strerror(errno));
        // Not critical. Probably we can continue without sensor configuration
    }

#if CAM_DESCR
    rc |= ioctlQueryCapability();
    rc |= ioctlGetStreamParm();
    rc |= ioctlEnumFmt();
    getDriverName();
    //    rc |= ioctlGetStreamFmt();
#endif

    return rc;
}

// Close video device
void V4L2Device::Close()
{
    if (sub_dev_fd != -1) {
        close(sub_dev_fd);
        sub_dev_fd = -1;
    }

    if (dev_fd != -1) {
        close(dev_fd);
        dev_fd = -1;
    }

    if (io_in_src) {
        io_in_src->destroy();
        io_in_src.reset();
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

void V4L2Device::getDriverName()
{
    v4l2_capability capa = { 0 };

    if (sub_dev_fd == -1) return;

    if (-1 == ioctl(sub_dev_fd, VIDIOC_QUERYCAP, &capa)) return;

    driver_name = std::string((char*)capa.driver);
}

// Query and list the formats supported
int V4L2Device::ioctlEnumFmt()
{
    v4l2_fmtdesc fmtdesc = { 0 };

    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    while (-1 != ioctl(dev_fd, VIDIOC_ENUM_FMT, &fmtdesc)) {
        v4l2_frmsizeenum frmsize = { 0 };

        frmsize.pixel_format = fmtdesc.pixelformat;
        frmsize.index = 0;
        while (-1 != ioctl(dev_fd, VIDIOC_ENUM_FRAMESIZES, &frmsize)) {
            struct fmt_info fmt_info_cam = {
                .fmt_cc = {.u32 = fmtdesc.pixelformat},
                .width = frmsize.discrete.width,
                .height = frmsize.discrete.height
            };
            fmt_info_enum.emplace_back(fmt_info_cam);
            //std::cout << "description=" << fmtdesc.description << "\tpixelformat=" << std::oct
            //    << char(fmtdesc.pixelformat&0xFF) << char((fmtdesc.pixelformat>>8)&0xFF)
            //    << char((fmtdesc.pixelformat>>16)&0xFF) << char((fmtdesc.pixelformat>>24)&0xFF)
            //    << std::dec << "\tWxH=" << frmsize.discrete.width << "x" << frmsize.discrete.height
            //    // << "\tflags=" << fmtdesc.flags << "\ttype=" << frmsize.type
            //    << "\n";

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

int V4L2Device::ioctlSetStreamFmt(const struct fmt_info *stream_fmt)
{
    v4l2_format fmt = {0};

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (-1 == ioctl(dev_fd, VIDIOC_G_FMT, &fmt)) {
        g_warning("Can't VIDIOC_G_FMT (%d)", errno);
        return -1;
    }

    if ((stream_fmt->width > fmt.fmt.pix_mp.width) || 
        (stream_fmt->height > fmt.fmt.pix_mp.height)) {
        g_warning("Bad WxH (%dx%d vs %dx%d)", 
            stream_fmt->width, stream_fmt->height,
            fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height);
        return -1;
    }

    fmt.fmt.pix_mp.width = stream_fmt->width;
    fmt.fmt.pix_mp.height = stream_fmt->height;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.num_planes = 1;
    fmt.fmt.pix_mp.pixelformat = stream_fmt->fmt_cc.u32;
    fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
    if (-1 == ioctl(dev_fd, VIDIOC_S_FMT, &fmt)) {
        g_warning("Can't VIDIOC_S_FMT (%d) %s", errno, strerror(errno));
        /* AV: Sometimes on error the device not released properly, only exit from the application releases the camera */
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

    stat_cnt_frames_num ++;
    streaming_watchdog_cnt = V4L2Device::STREAM_WDT_CNT;

    return 0;
}

int V4L2Device::is_stall()
{
    if (streaming_watchdog_cnt == 0) return 0;

    if (streaming_watchdog_frames_num == stat_cnt_frames_num) {
        // frame counter not updated since last check
        streaming_watchdog_cnt--;
        if (streaming_watchdog_cnt == 0) {
            g_warning("Camera stall on frame (%d)", streaming_watchdog_frames_num);
            return 1;
        }
    }
    
    streaming_watchdog_frames_num = stat_cnt_frames_num;
    return 0;

}
int V4L2Device::StreamOn(const struct fmt_info *_stream_fmt)
{
    int rc = 0;

    if (!_stream_fmt) return -1;

    stream_fmt = _stream_fmt;  //_stream_fmt points to one of enum fmt_info_enum
    rc |= ioctlSetStreamFmt(stream_fmt);
    if (rc) {
        return -1;
    }

    rc |= InitBuffers();

    if (is_streaming) {
        return 0;
    }

    stat_cnt_frames_num = 0;
    
    streaming_watchdog_cnt = V4L2Device::STREAM_WDT_CNT;
    streaming_watchdog_frames_num = 0;

    v4l2_buf_type type { V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
    if (-1 == ioctl(dev_fd, VIDIOC_STREAMON, &type)) {
        g_warning("VIDIOC_STREAMON");
        return -1;
    }

    /* AV: 
     *     FLYN catch congestion error on very first configuration.
     *     Either something wrong in MCU FW or in flyn driver. 
     *     1. Why congestion not processed by driver?
     *     2. How to get confirmation from MCU?
     *     Need to be investigated. 
     */
    
    is_streaming = true;

    io_in_src = Glib::IOSource::create(dev_fd, (Glib::IOCondition)(G_IO_IN | G_IO_ERR));
    io_in_src->connect(sigc::mem_fun(*this, &V4L2Device::onBuff));
    io_in_src->attach(context);

    return 0;
}

// Stop stream
void V4L2Device::ioctlSetStreamOff()
{
    // Enable back calibration as not using the stream anymore.
    ioctlSetControls_flyn_calib(1);

    // TODO: ??? Restore other parameters ???

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

int V4L2Device::ioctlSetControls()
{
    int rc; 

    if (0 == driver_name.compare("atic320")) {
        ioctlSetControls_atic();
    }
    else if (0 == driver_name.compare("flyn384")) {
        ioctlSetControls_flyn();
    }

    // Get controls from config or compose by the camera type?
    rc = ioctlSetControls_ISI();
    return rc;

}
int V4L2Device::ioctlSetControls_atic() 
{
    return 0;
}

int V4L2Device::ioctlSetControls_flyn_calib(int on_off)
{
    int rc;

    struct v4l2_ext_control flyn_ctrls_calib =
        { .id = V4L2_CID_FLYN384_TEMP_CALIB, .size = 0, .value64 = 0 };

    struct v4l2_ext_controls controls = {
        .which = V4L2_CTRL_WHICH_CUR_VAL, .count = 1, .error_idx = 0,
        .request_fd = sub_dev_fd, .controls = &flyn_ctrls_calib };

    if (sub_dev_fd == -1) return -1;

    rc = ioctl(sub_dev_fd, VIDIOC_G_EXT_CTRLS, &controls);
    if (rc) {
        // EACCES  || ENOSPC 
        g_warning("Error reading external controls");
        return -1;
    }

    // Preserve the state localy
    flyn_calib_is_on = on_off;

    // Modify Controls according to config
    flyn_ctrls_calib.value = on_off;
    rc = ioctl(sub_dev_fd, VIDIOC_S_EXT_CTRLS, &controls);
    if (rc) {
        // EACCES  || ENOSPC 
        g_warning("Error setting FLYN's controls");
        return -1;
    }
    else {
        g_warning( "FLYN384 controls: \r\n"\
            "\t TEMP_CALIB = %d\r\n",
            flyn_ctrls_calib.value);
    }

    return 0;
}

int V4L2Device::ioctlSetControls_flyn_calib_trig()
{
    int rc;

    struct v4l2_ext_control flyn_ctrls_calib_trig =
    { .id = V4L2_CID_FLYN384_TEMP_CALIB, .size = 0, .value64 = 0 };

    struct v4l2_ext_controls controls = {
        .which = V4L2_CTRL_WHICH_CUR_VAL, .count = 1, .error_idx = 0,
        .request_fd = sub_dev_fd, .controls = &flyn_ctrls_calib_trig };

        if (sub_dev_fd == -1) return -1;

    rc = ioctl(sub_dev_fd, VIDIOC_G_EXT_CTRLS, &controls);
    if (rc) {
        // EACCES  || ENOSPC 
        g_warning("Error reading external controls");
        return -1;
    }

    flyn_ctrls_calib_trig.value = 1;
    rc |= ioctl(sub_dev_fd, VIDIOC_S_EXT_CTRLS, &controls);

    flyn_ctrls_calib_trig.value = 0;
    rc |= ioctl(sub_dev_fd, VIDIOC_S_EXT_CTRLS, &controls);

    if (rc) {
        // EACCES  || ENOSPC 
        g_warning("Error setting FLYN's controls (CALIB_TRIG)");
        return -1;
    }
    else {
        g_info("FLYN384 controls: CALIB_TRIG");
    }

    return 0;
}

int V4L2Device::ioctlSetControls_flyn()
{
#define FLYN384_CTRL_DENOISE     0
#define FLYN384_CTRL_FILTER      1
#define FLYN384_CTRL_TEMP_CALIB  2
#define FLYN384_CTRL_BRIGHTNESS  3
#define FLYN384_CTRL_CONTRAST    4
#define FLYN384_CTRL_AUTOGAIN    5
#define FLYN384_CTRL_NUMS        6

    int rc;

    struct v4l2_ext_control flyn_ctrls[FLYN384_CTRL_NUMS] = {
        [FLYN384_CTRL_DENOISE   ] = {.id = V4L2_CID_FLYN384_DENOISE,    .size = 0, .value64 = 0 },
        [FLYN384_CTRL_FILTER    ] = {.id = V4L2_CID_FLYN384_FILTER,     .size = 0, .value64 = 0 },
        [FLYN384_CTRL_TEMP_CALIB] = {.id = V4L2_CID_FLYN384_TEMP_CALIB, .size = 0, .value64 = 0 },
        [FLYN384_CTRL_BRIGHTNESS] = {.id = V4L2_CID_BRIGHTNESS,         .size = 0, .value64 = 0 },
        [FLYN384_CTRL_CONTRAST  ] = {.id = V4L2_CID_CONTRAST,           .size = 0, .value64 = 0 },
        [FLYN384_CTRL_AUTOGAIN  ] = {.id = V4L2_CID_AUTOGAIN,           .size = 0, .value64 = 0 },
    };

    struct v4l2_ext_controls controls = {
        .which = V4L2_CTRL_WHICH_CUR_VAL,
        .count = FLYN384_CTRL_NUMS,
        .error_idx = 0,
        .request_fd = sub_dev_fd,
        .controls = flyn_ctrls
    };

    if (sub_dev_fd == -1) return -1;

    rc = ioctl(sub_dev_fd, VIDIOC_G_EXT_CTRLS, &controls);
    if (rc) {
        // EACCES  || ENOSPC 
        g_warning("Error reading external controls");
        return -1;
    }

    flyn_calib_is_on = flyn_ctrls[FLYN384_CTRL_TEMP_CALIB].value;

    const TFlowCtrlCapture::cfg_v4l2_ctrls_flyn* cfg_flyn = 
        (const TFlowCtrlCapture::cfg_v4l2_ctrls_flyn *)cfg->flyn384.v.ref;

    // Modify Controls according to config. 
    // All but TEMP_CALIB it will be disabled on takeoff and enabled after landing
    flyn_ctrls[FLYN384_CTRL_DENOISE   ].value = cfg_flyn->denoise.v.num;
    flyn_ctrls[FLYN384_CTRL_FILTER    ].value = cfg_flyn->filter.v.num;
    flyn_ctrls[FLYN384_CTRL_BRIGHTNESS].value = cfg_flyn->brightness.v.num;
    flyn_ctrls[FLYN384_CTRL_CONTRAST  ].value = cfg_flyn->contrast.v.num;
    flyn_ctrls[FLYN384_CTRL_AUTOGAIN  ].value = cfg_flyn->gain.v.num;

    rc = ioctl(sub_dev_fd, VIDIOC_S_EXT_CTRLS, &controls);
    if (rc) {
        // EACCES  || ENOSPC 
        g_warning("Error setting FLYN controls");
        return -1;
    }
    else {
        g_info(
            "FLYN384 controls: \r\n"\
            "\t DENOISE    = %d\r\n"\
            "\t FILTER     = %d\r\n"\
            "\t BRIGHTNESS = %d\r\n"\
            "\t CONTRAST   = %d\r\n"\
            "\t AUTOGAIN   = %d\r\n",
            flyn_ctrls[FLYN384_CTRL_DENOISE].value,
            flyn_ctrls[FLYN384_CTRL_FILTER].value,
            flyn_ctrls[FLYN384_CTRL_BRIGHTNESS].value,
            flyn_ctrls[FLYN384_CTRL_CONTRAST].value,
            flyn_ctrls[FLYN384_CTRL_AUTOGAIN].value);
    }

    return 0;
}

int V4L2Device::ioctlSetControls_ISI()
{
    int rc;

#define ISI_CTRL_HFLIP 0
#define ISI_CTRL_VFLIP 1
#define ISI_CTRL_NUMS  2
    
    struct v4l2_ext_control isi_ctrls[ISI_CTRL_NUMS] = {
        [ISI_CTRL_HFLIP] = {.id = V4L2_CID_HFLIP,  .size = 0,   .value64 = 0 },
        [ISI_CTRL_VFLIP] = {.id = V4L2_CID_VFLIP,  .size = 0,   .value64 = 0 },
    };

    struct v4l2_ext_controls controls = {
        .which = V4L2_CTRL_WHICH_CUR_VAL,
        .count = ISI_CTRL_NUMS,
        .error_idx = 0,
        .request_fd = dev_fd,
        .controls = isi_ctrls
    };

    if (dev_fd == -1) return -1;

    rc = ioctl(dev_fd, VIDIOC_G_EXT_CTRLS, &controls);
    if (rc) {
        // EACCES  || ENOSPC 
        g_warning("Error reading external controls");
        return -1;
    }
#if 0
    else {
        g_warning(
            "ISI Ctrls: \r\n"\
            "\t VFLIP = %d\r\n"\
            "\t HFLIP = %d\r\n",
        isi_ctrls[ISI_CTRL_VFLIP].value, 
        isi_ctrls[ISI_CTRL_HFLIP].value);
    }
#endif

    // Modify Controls according to config
    isi_ctrls[ISI_CTRL_VFLIP].value = cfg->vflip.v.num;
    isi_ctrls[ISI_CTRL_HFLIP].value = cfg->hflip.v.num;

    rc = ioctl(dev_fd, VIDIOC_S_EXT_CTRLS, &controls);
    if (rc) {
        // EACCES  || ENOSPC 
        g_warning("Error setting external controls");
        return -1;
    }
    else {
        g_info(
            "ISI Ctrls: \r\n"\
            "\t VFLIP = %d\r\n"\
            "\t HFLIP = %d\r\n",
        isi_ctrls[ISI_CTRL_VFLIP].value, 
        isi_ctrls[ISI_CTRL_HFLIP].value);
    }

    return 0;
}

int V4L2Device::Init()
{
    int rc;

    if (!cfg->dev_name.v.str) {
        g_warning("Device name isn't specified");
        return -1;
    }

    rc = Open();
    if (rc) return rc;

    // TODO: Compare the current ENUM list in the configuration with the 
    //       received one. 
    //       What if differs? compare enum at particular index?
    //       Update Ctrl with formats enumerated from the device
    //       ? Ctrl must send this enum to the UI ?

    ioctlSetControls();

    return 0;
}
