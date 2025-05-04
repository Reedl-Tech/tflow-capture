// 2) Consider usage TFlowBuf[] inside of V4L2_Device
#include <thread>
#include <iostream>
#include <fcntl.h>

#include <giomm.h>
#include <glib-unix.h>

#include "tflow-capture.hpp"
#include "tflow-capture-v4l2.hpp"

#define CAM_DESCR 1

TFlowCaptureV4L2::TFlowCaptureV4L2(MainContextPtr _context, 
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

    v4l2_buf_template.type = -1;
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

TFlowCaptureV4L2::~TFlowCaptureV4L2()
{
    if (is_streaming) {
        ioctlSetStreamOff();
    }

    Deinit();
}

/*
 * The function is a calback called on async VIDIOC_DQBUF completion
 */
int TFlowCaptureV4L2::onBuff(Glib::IOCondition G_IO_IN)
{
    // TODO: Check camera state 
    // ... 
   
    //Dequeue all buffers
    v4l2_buffer v4l2_buf = v4l2_buf_template;
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
int TFlowCaptureV4L2::Open()
{
    int rc = 0;

    /* TODO: Try to find proper camera device by the driver/card/bus name if 
     *       only one camera exists in the system.
     */
    dev_fd = -1;
    if ( strcmp(cfg->dev_name.v.str, "auto") == 0 ) {
        for( int dev_num = 0; dev_num < 5; dev_num++) {
            v4l2_capability capa = { 0 };
            char dev_name [ 16 ] = {};

            snprintf(dev_name, sizeof(dev_name), "/dev/video%d", dev_num);

            int _dev_fd = open(dev_name, O_RDWR | O_NONBLOCK);
            if ( _dev_fd == -1) continue;

            if ( 0 == ioctl(_dev_fd, VIDIOC_QUERYCAP, &capa) ) {
                // TFlow thermal cameras uses ISI interface, thus try to match
                // it by name
                if ( 0 == strcmp((char*)capa.driver, "mxc-isi-cap") ) {
                    // Found an isi capture!
                    g_warning("Dev %s: Card %s, Driver %s - selected", dev_name, capa.card, capa.driver);
                    dev_fd = _dev_fd;
                    break;
                }
                g_warning("Dev %s: Card %s, Driver %s - skipped", dev_name, capa.card, capa.driver);
            }
            close(_dev_fd);
            continue;
        }
    } 
    else {
        dev_fd = open(cfg->dev_name.v.str, O_RDWR | O_NONBLOCK);
    }

    if (dev_fd == -1) {
        g_warning("Can't open the video device %s (%d) - %s", 
            cfg->dev_name.v.str, errno, strerror(errno));
        return -1;
    }

    /* TODO: Get devices name as a function from /dev/videoX or by the 
     *       driver/card/bus name.
     */
    sub_dev_fd = -1;
    if ( strcmp(cfg->sub_dev_name.v.str, "auto") == 0 ) {
        for ( int sub_dev_num = 2; sub_dev_num < 5; sub_dev_num++ ) {
            v4l2_capability capa = { 0 };
            char sub_dev_name [ 32 ] = {};
            snprintf(sub_dev_name, sizeof(sub_dev_name), "/dev/v4l-subdev%d",
                sub_dev_num);

            int _sub_dev_fd = open(sub_dev_name, O_RDWR | O_NONBLOCK);
            if ( _sub_dev_fd == -1 ) continue;

            if ( 0 == ioctl(_sub_dev_fd, VIDIOC_QUERYCAP, &capa) ) {


                // TFlow thermal cameras uses ISI interface, thus try to match
                // it by name
                if ( 0 == strcmp((char*)capa.driver, "flyn384") ||
                     0 == strcmp((char*)capa.driver, "atic320") ) {
                    // Found a REEDL Tech cameras!
                    g_warning("SubDev %s: Driver %s - selected", sub_dev_name, capa.driver);
                    sub_dev_fd = _sub_dev_fd;
                    break;
                }
                g_warning("SubDev %s: Driver %s - skipped", sub_dev_name, capa.driver);
            }
            close(_sub_dev_fd);
            continue;
        }
    }
    else {
        sub_dev_fd = open(cfg->sub_dev_name.v.str, O_RDWR | O_NONBLOCK);
    }

    if (sub_dev_fd == -1) {
        g_warning("Can't open the sub video device %s (%d) - %s", 
            cfg->sub_dev_name.v.str, errno, strerror(errno));
        // Not critical. Probably we can continue without sensor configuration
    }

    rc = ioctlQueryCapability();

    if ( rc == 0 ) {
        if ( capa.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE ) {
            fmt_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        }
        else {
            fmt_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        }
        v4l2_buf_template.type = fmt_type;
    }

    rc |= ioctlGetStreamParm();
    rc |= ioctlEnumFmt();
    getDriverName();
#if CAM_DESCR
    //    rc |= ioctlGetStreamFmt();
#endif

    // Close the camera in case error on stream and fmt parameters set.
    if (rc) {
        if (dev_fd != -1) {
            dev_fd = -1;
            close(dev_fd);
        }

        if ( sub_dev_fd != -1 ) {
            sub_dev_fd = -1;
            close(sub_dev_fd);
        }
    }

    return rc;
}

// Close video device
void TFlowCaptureV4L2::Close()
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
int TFlowCaptureV4L2::ioctlQueryCapability()
{
    CLEAR(capa);

    if (-1 == ioctl(dev_fd, VIDIOC_QUERYCAP, &capa)) {
        g_critical("Can't VIDIOC_QUERYCAP (%d)", errno);
        return -1;
    }

    std::string caps_str = "";
        caps_str += (capa.capabilities & V4L2_CAP_VIDEO_CAPTURE         ) ? "\t\tVIDEO_CAPTURE           0x00000001\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VIDEO_OUTPUT          ) ? "\t\tVIDEO_OUTPUT            0x00000002\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VIDEO_OVERLAY         ) ? "\t\tVIDEO_OVERLAY           0x00000004\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VBI_CAPTURE           ) ? "\t\tVBI_CAPTURE             0x00000010\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VBI_OUTPUT            ) ? "\t\tVBI_OUTPUT              0x00000020\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_SLICED_VBI_CAPTURE    ) ? "\t\tSLICED_VBI_CAPTURE      0x00000040\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_SLICED_VBI_OUTPUT     ) ? "\t\tSLICED_VBI_OUTPUT       0x00000080\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_RDS_CAPTURE           ) ? "\t\tRDS_CAPTURE             0x00000100\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VIDEO_OUTPUT_OVERLAY  ) ? "\t\tVIDEO_OUTPUT_OVERLAY    0x00000200\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_HW_FREQ_SEEK          ) ? "\t\tHW_FREQ_SEEK            0x00000400\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_RDS_OUTPUT            ) ? "\t\tRDS_OUTPUT              0x00000800\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE  ) ? "\t\tVIDEO_CAPTURE_MPLANE    0x00001000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VIDEO_OUTPUT_MPLANE   ) ? "\t\tVIDEO_OUTPUT_MPLANE     0x00002000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE      ) ? "\t\tVIDEO_M2M_MPLANE        0x00004000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VIDEO_M2M             ) ? "\t\tVIDEO_M2M               0x00008000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_TUNER                 ) ? "\t\tTUNER                   0x00010000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_AUDIO                 ) ? "\t\tAUDIO                   0x00020000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_RADIO                 ) ? "\t\tRADIO                   0x00040000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_MODULATOR             ) ? "\t\tMODULATOR               0x00080000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_SDR_CAPTURE           ) ? "\t\tSDR_CAPTURE             0x00100000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_EXT_PIX_FORMAT        ) ? "\t\tEXT_PIX_FORMAT          0x00200000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_SDR_OUTPUT            ) ? "\t\tSDR_OUTPUT              0x00400000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_META_CAPTURE          ) ? "\t\tMETA_CAPTURE            0x00800000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_READWRITE             ) ? "\t\tREADWRITE               0x01000000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_ASYNCIO               ) ? "\t\tASYNCIO                 0x02000000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_STREAMING             ) ? "\t\tSTREAMING               0x04000000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_META_OUTPUT           ) ? "\t\tMETA_OUTPUT             0x08000000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_TOUCH                 ) ? "\t\tTOUCH                   0x10000000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_IO_MC                 ) ? "\t\tIO_MC                   0x20000000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_DEVICE_CAPS           ) ? "\t\tDEVICE_CAPS             0x80000000\r\n" : "";

    g_info("VIDIOC_QUERYCAP:\r\n"
        "\tDriver name  : %s\r\n"
        "\tCard         : %s\r\n"
        "\tBus info     : %s\r\n"
        "\tVersion      : %d\r\n"
        "\tCapabilities : 0x%08X\r\n%s"
        "\tDevice capab.: 0x%08X",
        capa.driver, capa.card, capa.bus_info, capa.version, 
    	capa.capabilities, caps_str.c_str(), capa.device_caps);

    if (!(capa.capabilities & V4L2_CAP_STREAMING)) {
        g_critical("Ooops, streaming is not supported by device");
    }
    return 0;
}

void TFlowCaptureV4L2::getDriverName()
{
    v4l2_capability capa = { 0 };

    if (sub_dev_fd == -1) return;

    if (-1 == ioctl(sub_dev_fd, VIDIOC_QUERYCAP, &capa)) return;

    driver_name = std::string((char*)capa.driver);
}

// Query and list the formats supported
int TFlowCaptureV4L2::ioctlEnumFmt()
{
    v4l2_fmtdesc fmtdesc = { 0 };

    fmtdesc.index = 0;
    fmtdesc.type = fmt_type;

    while (-1 != ioctl(dev_fd, VIDIOC_ENUM_FMT, &fmtdesc)) {
        v4l2_frmsizeenum frmsize = { 0 };

        frmsize.pixel_format = fmtdesc.pixelformat;
        frmsize.index = 0;
        while (-1 != ioctl(dev_fd, VIDIOC_ENUM_FRAMESIZES, &frmsize)) {
            struct fmt_info fmt_info_cam = {
                .fmt_cc = {.u32 = fmtdesc.pixelformat},
                .frmsize = frmsize,
            };

            uint32_t fourcc = fmtdesc.pixelformat;
            std::string fourcc_str = "";
            fourcc_str += (char)((fourcc       ) & 0xff);
            fourcc_str += (char)((fourcc >>  8 ) & 0xff);
            fourcc_str += (char)((fourcc >> 16 ) & 0xff);
            fourcc_str += (char)((fourcc >> 24 ) & 0xff);

            char frm_size[64] = {0};
            switch ( frmsize.type ) {
            case V4L2_FRMSIZE_TYPE_DISCRETE:
                snprintf(frm_size, sizeof(frm_size)-1, "DISCRETE %d x %d", frmsize.discrete.width, frmsize.discrete.height);
                fmt_info_cam.height = frmsize.discrete.height;
                fmt_info_cam.width = frmsize.discrete.width;
                break;
            case V4L2_FRMSIZE_TYPE_STEPWISE:
                snprintf(frm_size, sizeof(frm_size)-1, "STEPWISE %d+%d..%d x %d+%d..%d", 
                    frmsize.stepwise.min_width, frmsize.stepwise.step_width, frmsize.stepwise.max_width,
                    frmsize.stepwise.min_height, frmsize.stepwise.step_height, frmsize.stepwise.max_height);
                    // Needs to be updated from the user config on format selection 
                    fmt_info_cam.height = 0;
                    fmt_info_cam.width = 0;
                break;
            case V4L2_FRMSIZE_TYPE_CONTINUOUS:
                snprintf(frm_size, sizeof(frm_size)-1, "CONTINUOUS");
                break;
            }

            fmt_info_enum.emplace_back(fmt_info_cam);

            g_info("VIDIOC_ENUM_FMT: %-24s (%s, 0x%08X) %s Flags: 0x%08X",
                fmtdesc.description,  
                fourcc_str.c_str(), fmtdesc.pixelformat,
                frm_size,
                fmtdesc.flags);

            frmsize.index++;
        }
        fmtdesc.index++;
    }

    return 0;
}

// Get current format and frame rate
int TFlowCaptureV4L2::ioctlGetStreamParm()
{
    v4l2_streamparm streamparm;
    CLEAR(streamparm);

    streamparm.type = fmt_type;
    if (-1 == ioctl(dev_fd, VIDIOC_G_PARM, &streamparm)) {
        g_warning("Can't VIDIOC_G_PARM (%d)", errno);
        return -1;
    }
    
    g_info("VIDIOC_G_PARM:\r\n"
        "\tModes        : 0x%08X\r\n"
        "\tCurrent mode : 0x%08X\r\n"
        "\tFrame rate   : %d/%d\r\n"
        "\tBuffers      : %d",
        streamparm.parm.capture.capability,
        streamparm.parm.capture.capturemode,
        streamparm.parm.capture.timeperframe.numerator,
        streamparm.parm.capture.timeperframe.denominator,
        streamparm.parm.capture.readbuffers);

    return 0;
}

int TFlowCaptureV4L2::ioctlSetStreamParm(bool highQuality, u_int timeperframe)
{
    v4l2_streamparm streamparm;
    CLEAR(streamparm);
    streamparm.type = fmt_type;

    streamparm.parm.capture.capturemode = V4L2_CAP_TIMEPERFRAME | (u_char)highQuality;
    streamparm.parm.capture.timeperframe.numerator = 1;
    streamparm.parm.capture.timeperframe.denominator = timeperframe;
    if (-1 == ioctl(dev_fd, VIDIOC_S_PARM, &streamparm)) {
        g_critical("Can't VIDIOC_S_PARM (%d)", errno);
        return -1;
    }
    return 0;
}

// Get format
int TFlowCaptureV4L2::ioctlGetStreamFmt()
{
    v4l2_format fmt;
    CLEAR(fmt);

    fmt.type = fmt_type;
    if (-1 == ioctl(dev_fd, VIDIOC_G_FMT, &fmt)) {
        g_critical("Can't VIDIOC_G_FMT (%d)", errno);
        return -1;
    }
    uint32_t fourcc = fmt.fmt.pix_mp.pixelformat;
    std::string fourcc_str = "";
    fourcc_str += (char)((fourcc       ) & 0xff);
    fourcc_str += (char)((fourcc >>  8 ) & 0xff);
    fourcc_str += (char)((fourcc >> 16 ) & 0xff);
    fourcc_str += (char)((fourcc >> 24 ) & 0xff);

    g_info("VIDIOC_G_FMT:\r\n"
        "\tWxH            : %dx%d\r\n"
        "\tPixel fmt      : (%s, 0x%08X)\r\n"
  //      "\tField          : %s\r\n"
        "\tBytes per line : %d\r\n"
        "\tSize image     : %d\r\n"
        "\tColor space    : %d",
        fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
        fourcc_str.c_str(), fourcc,
//        fmt.fmt.pix_mp.field,
        fmt.fmt.pix.bytesperline,
        fmt.fmt.pix.sizeimage,
        fmt.fmt.pix_mp.colorspace);

    return 0;
}

int TFlowCaptureV4L2::ioctlSetStreamFmt(const struct fmt_info *stream_fmt)
{
    v4l2_format fmt = {0};

    fmt.type = fmt_type;
    if (-1 == ioctl(dev_fd, VIDIOC_G_FMT, &fmt)) {
        g_warning("Can't VIDIOC_G_FMT (%d)", errno);
        return -1;
    }

#if 0
    if ( stream_fmt->frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE ) {
        uint32_t w = stream_fmt->frmsize.discrete.width;
        uint32_t h = stream_fmt->frmsize.discrete.height;
        if ( w != fmt.fmt.pix_mp.width  || h != fmt.fmt.pix_mp.height ) {
            g_warning("Bad WxH (%dx%d vs %dx%d)", w, h, 
                fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height);
            return -1;
        }
    }
#endif

    fmt.type = fmt_type;
    fmt.fmt.pix_mp.width = stream_fmt->width;
    fmt.fmt.pix_mp.height = stream_fmt->height;
    fmt.fmt.pix_mp.pixelformat = stream_fmt->fmt_cc.u32;
    fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
    fmt.fmt.pix_mp.num_planes = 1;
    fmt.fmt.pix_mp.plane_fmt[0].bytesperline = stream_fmt->width;
    fmt.fmt.pix_mp.plane_fmt[0].sizeimage    = stream_fmt->width * stream_fmt->height;

    if (-1 == ioctl(dev_fd, VIDIOC_S_FMT, &fmt)) {
        g_warning("Can't VIDIOC_S_FMT (%d) %s", errno, strerror(errno));
        /* AV: Sometimes on error the device not released properly, only exit from the application releases the camera */
        return -1;
    }
    return 0;
}

void TFlowCaptureV4L2::DeinitBuffers()
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
    req.type = fmt_type;                // Is it necessary then count == 0?
    req.memory = V4L2_MEMORY_MMAP;      // Is it necessary then count == 0?
    rc = ioctl(dev_fd, VIDIOC_REQBUFS, &req);
    if (-1 == rc) {
        g_warning("clear me as we don't care");
    }
    // TODO: Check STREAMOFF
    // TODO: Check double release (i.e. close application at stoped camera)

}

// Request buffers in kernel space
int TFlowCaptureV4L2::InitBuffers()
{
    int rc = 0;
    v4l2_requestbuffers req{};
    v4l2_requestbuffers req_delme;  // AV: TODO: check zero initialization
    CLEAR(req);

    /* 
     * Request buffers 
     */
    req.count = buffs_num;
    req.type = fmt_type;
    req.memory = V4L2_MEMORY_MMAP;
    if (-1 == ioctl(dev_fd, VIDIOC_REQBUFS, &req)) {
        g_warning("Can't VIDIOC_REQBUFS (%d)", errno);
        return -1;
    }

    buf_srv->buf_create(buffs_num);

    return rc;
}

// Enqueue a buffer
int TFlowCaptureV4L2::ioctlQueueBuffer(int index)
{
    v4l2_buffer v4l2_buf = v4l2_buf_template;
    v4l2_buf.index = index;

    // Put the buffer back to kernel's queue
    if (-1 == ioctl(dev_fd, VIDIOC_QBUF, &v4l2_buf)) {
        g_warning("TFlowCaptureV4L2: Ooops VIDIOC_QBUF @%d", index);
        return -1;
    }
    return 0;
}

int TFlowCaptureV4L2::ioctlDequeueBuffer(v4l2_buffer &v4l2_buf)
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
    streaming_watchdog_cnt = TFlowCaptureV4L2::STREAM_WDT_CNT;

    return 0;
}

int TFlowCaptureV4L2::is_stall()
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
int TFlowCaptureV4L2::StreamOn(const struct fmt_info *_stream_fmt)
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
    
    streaming_watchdog_cnt = TFlowCaptureV4L2::STREAM_WDT_CNT;
    streaming_watchdog_frames_num = 0;

    v4l2_buf_type type { fmt_type };
    if (-1 == ioctl(dev_fd, VIDIOC_STREAMON, &type)) {
        g_warning("Error on VIDIOC_STREAMON");
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
    io_in_src->connect(sigc::mem_fun(*this, &TFlowCaptureV4L2::onBuff));
    io_in_src->attach(context);

    return 0;
}

// Stop stream
void TFlowCaptureV4L2::ioctlSetStreamOff()
{
    if (0 == driver_name.compare("flyn384")) {
    	// Enable back calibration as not using the stream anymore.
    	ioctlSetControls_flyn_calib(1);
    }

    // TODO: ??? Restore other parameters ???

    v4l2_buf_type type {fmt_type};
    
    if ( -1 == ioctl(dev_fd, VIDIOC_STREAMOFF, &type)) {
        g_warning("Error on VIDIOC_STREAMOFF");
    }

    is_streaming = false;
}


void TFlowCaptureV4L2::Deinit()
{
    DeinitBuffers();
    Close();
}

int TFlowCaptureV4L2::ioctlSetControls()
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
int TFlowCaptureV4L2::ioctlSetControls_atic() 
{
    return 0;
}

int TFlowCaptureV4L2::ioctlSetControls_flyn_calib(int on_off)
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
        g_warning( "FLYN384 controls: \n"\
            "\t TEMP_CALIB = %d\n",
            flyn_ctrls_calib.value);
    }

    return 0;
}

int TFlowCaptureV4L2::ioctlSetControls_flyn_calib_trig()
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

int TFlowCaptureV4L2::ioctlSetControls_flyn()
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

int TFlowCaptureV4L2::ioctlSetControls_ISI()
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

int TFlowCaptureV4L2::Init()
{
    int rc;

    if (!cfg->dev_name.v.str) {
        g_warning("Device name isn't specified.");
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
