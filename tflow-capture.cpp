#include "tflow-build-cfg.hpp"
#include <iostream>
#include <functional>

#include "tflow-glib.hpp"

#include <json11.hpp>

#include "tflow-buf-srv.hpp"

#include "tflow-capture.hpp"

using namespace json11;

v4l2_buffer g_buf;

#define IDLE_INTERVAL_MSEC 300

static struct timespec diff_timespec(
    const struct timespec* time1,
    const struct timespec* time0)
{
    assert(time1);
    assert(time0);
    struct timespec diff = { .tv_sec = time1->tv_sec - time0->tv_sec, //
        .tv_nsec = time1->tv_nsec - time0->tv_nsec };
    if (diff.tv_nsec < 0) {
        diff.tv_nsec += 1000000000; // nsec/sec
        diff.tv_sec--;
    }
    return diff;
}

static double diff_timespec_msec(
    const struct timespec* time1,
    const struct timespec* time0)
{
    struct timespec d_tp = diff_timespec(time1, time0);
    return d_tp.tv_sec * 1000 + (double)d_tp.tv_nsec / (1000 * 1000);
}


TFlowBuf::TFlowBuf()
{

    memset(&ts, 0, sizeof(ts));
    sequence = 0;

    /* Parameters obtained from Kernel on the Client side */
    index = -1;
    length = 0;
    start = MAP_FAILED;

    owners = 0;             // Bit mask of TFlowBufCli. Bit 0 - means buffer is in user space
}

TFlowBuf::~TFlowBuf()
{
    if (start != MAP_FAILED) {
        munmap(start, length);
        start = MAP_FAILED;
    }
}

int TFlowBuf::age() {
    int rc;
    struct timespec tp;
    unsigned long proc_frame_ms, now_ms;

    rc = clock_gettime(CLOCK_MONOTONIC, &tp);
    now_ms = tp.tv_sec * 1000 + tp.tv_nsec / 1000000;
    proc_frame_ms = ts.tv_sec * 1000 + ts.tv_usec / 1000;

    return (now_ms - proc_frame_ms);
}

#if CAPTURE_PLAYER
static int _onBufPlayer(void* ctx, TFlowBuf& buf)
{
    TFlowCapture* m = (TFlowCapture*)ctx;
    return m->onBufPlayer(buf);
}
#endif

int TFlowCapture::onCustomMsg(const TFlowBufPck::pck& in_msg)
{
    if (in_msg.hdr.id <= TFlowBufPck::TFLOWBUF_MSG_CUSTOM_) return 0;

    // Return -1 to close the cli_port
#if WITH_AP
    return autopilot->onCaptureMsgRcv(in_msg);
#endif

#if CODE_BROWSE
        TFlowMilesi::onCaptureMsgRcv(in_msg);
        TFlowFixar::onCaptureMsgRcv(in_msg);
#endif
    return 0;
}

#if CAPTURE_PLAYER
/* 
 * Add data aux to TFlow buffer, data updated externaly by another module,
 * i.e. TFlowPlayer
 */
int TFlowCapture::onBufPlayer(TFlowBuf& buf)
{
#if CODE_BROWSE
    // caled from 
    TFlowBufSrv::buf_consume();
#endif

    //  MJPEGCapture::ap_data and   TFlowCapture::ap_data are matched one-to-one
    MJPEGCapture::ap_data* player_imu = player->shm_tbl[buf.index].aux_data;

    buf.aux_data = (uint8_t*)&aux_ap_data;
    buf.aux_data_len = sizeof(aux_ap_data);
    return 0;
}
#endif

TFlowCapture::TFlowCapture(MainContextPtr _context, const std::string cfg_fname) :
    cam0(nullptr),
#if WITH_AP    
    autopilot(nullptr),
#endif
#if CAPTURE_PLAYER    
    player(nullptr)
#endif    
    context(_context),
    ctrl(*this, cfg_fname)        // Att!: Must be constructed after submodules' pointers initialization
{
    main_loop = Glib::MainLoop::create(context, false);

    Glib::signal_timeout().connect(sigc::mem_fun(*this, &TFlowCapture::onIdle), IDLE_INTERVAL_MSEC);

    cam0_last_check_ts.tv_sec = 0;
    cam0_last_check_ts.tv_nsec = 0;

#if CAPTURE_PLAYER    
    player_last_check_ts.tv_sec = 0;
    player_last_check_ts.tv_nsec = 0;
#endif

#if WITH_AP
    autopilot = TFlowAP::createAPInstance(context);
#endif

}

TFlowCapture::~TFlowCapture()
{

#if WITH_AP
    if (autopilot) {
        delete autopilot;
    }
#endif
}

#if CAPTURE_PLAYER
void TFlowCapture::checkPlayerState(struct timespec *now_ts)
{
    if (player_state_flag.v == Flag::SET) {
        return;
    }

    if (player_state_flag.v == Flag::CLR) {

        // Check media file name
        if (!ctrl.player_fname_is_valid()) return;

        /* Don't try open the file to often 
         * TODO: Should we try only once?
         */
        if (diff_timespec_msec(now_ts, &player_last_check_ts) > 1000) {
            player_last_check_ts = *now_ts;
            player_state_flag.v = Flag::RISE;
        }
    }
    
    if (player_state_flag.v == Flag::UNDEF || player_state_flag.v == Flag::RISE) {
        int rc;

        // Check player device name
        if (!ctrl.player_fname_is_valid()) return;

        do {
            int cfg_buffs_num = ctrl.cmd_flds_config.buffs_num.v.num;
            assert(player);
            player = new TFlowPlayer(context, cfg_buffs_num, 61.5f);

            /* Link Player and TFlowBufferServer*/
            player->buf_srv = buf_srv;

            buf_srv->player = player;
            buf_srv->registerOnBuf(this, _onBufPlayer);

            rc = player->Init(ctrl.player_fname_get());
            if (rc) break;

            // int player_state = ctrl.cmd_flds_config.player_state.v.num;
            // int player_state = 1;   // Play/Pause

            //if (player_state == 1) {
            //    rc = player->StreamOn();
            //    if (rc) break;
            //}

            player_state_flag.v = Flag::SET;
        }
        while (0);
        
        if (rc) {
            // Can't open media file - try again later 
            cam_state_flag.v = Flag::CLR;
        }
        else {
            /* Media file is open. Now we can advertise video buffers */
            buf_srv->sck_state_flag.v = Flag::RISE;
        }
        return;
    }

    if (player_state_flag.v == Flag::FALL) {
        
        buf_srv->sck_state_flag.v == Flag::FALL;

        // close the media file and the Player
        delete player;
        player = NULL;
        player_state_flag.v = Flag::CLR;
    }

}
#endif

const struct fmt_info* TFlowCapture::getCamFmt() 
{
    // Get format from config:
    //  Option 1.   By config idx 
    //  Option 1.1  In case of stepwise frame size, the actual size defined by WxH
    //  Option 2.   By WxH and format

    int cam_fmt_idx = -1;
    std::vector<struct fmt_info> cfg_fmt_enum;
    struct fmt_info cfg_fmt;

    int cfg_idx = ctrl.cam_fmt_get();
    int use_default_idx = 0;
    int use_default_format = 0;

    ctrl.cam_fmt_enum_get(cfg_fmt_enum);

    if (cfg_idx < 0) {
        use_default_idx = 1;
    }

    if (cfg_fmt_enum.empty()) {
        use_default_format = 1;
    }

    if (use_default_idx && !use_default_format) {
        g_warning("Use default config format [0]");
        cfg_idx = 0;
    }
    else if (use_default_idx && use_default_format) {
        // Get _Camera_ format index by mathcing fmt
        for (struct fmt_info &cam_fmt : cam0->fmt_info_enum) {
            if (cam_fmt.fmt_cc.u32 == V4L2_PIX_FMT_GREY){
                
                cam_fmt.height = cam_fmt.frmsize.discrete.height;
                cam_fmt.width = cam_fmt.frmsize.discrete.width;
                return &cam_fmt;
            }
        }

        if (cam0->fmt_info_enum[0].height != 0 || 
            cam0->fmt_info_enum[0].width  != 0 ){
            g_warning("Using default camera format [0]");
            return &cam0->fmt_info_enum[0];
        }
        else {
            g_warning("WxH must be specified.");
            return nullptr;
        }

    }
    else {
        // Both index and format are from config
        if (cfg_idx < cfg_fmt_enum.size()) {
            cfg_fmt = cfg_fmt_enum[cfg_idx];
        }
        else {
            g_warning("Bad index fmt index (%d vs %ld)", cfg_idx, cfg_fmt_enum.size());
            return nullptr;
        }
    }

    // Get _Camera_ format index by mathcing fmt
    for (struct fmt_info &cam_fmt : cam0->fmt_info_enum) {
        switch ( cam_fmt.frmsize.type ) {
        case V4L2_FRMSIZE_TYPE_DISCRETE:
            if ( ( cam_fmt.fmt_cc.u32 == cfg_fmt.fmt_cc.u32 ) &&
                ( cam_fmt.width == cfg_fmt.width ) &&
                ( cam_fmt.height == cfg_fmt.height ) ) {
                return &cam_fmt;
            }

        case V4L2_FRMSIZE_TYPE_STEPWISE:
        case V4L2_FRMSIZE_TYPE_CONTINUOUS:
            // TODO: need to be implemeneted for some RGB sensors
            break;
        }
        continue;
    }

    g_warning("Config format not matched (%d)", cfg_idx);

    return nullptr;
}

int TFlowCapture::onBufAP(TFlowBuf &buf)
{
    // Called on every received video frame.
    // Custom submodules can do some work here.
    // For ex. add custom data to frame's AUX section.

#if WITH_AP
    autopilot->onBuf(buf);
#endif
    // player->onBuf

    return 0;
}

void TFlowCapture::checkCamState(struct timespec* now_ts)
{
    if (cam0_state_flag.v == Flag::SET) {
        if (cam0->is_stall()) {
            cam0_state_flag.v = Flag::FALL;
        }
        return;
    }

    if (cam0_state_flag.v == Flag::CLR) {

        /* Don't try connect to camera to often */
        if (diff_timespec_msec(now_ts, &cam0_last_check_ts) > 5000) {
            cam0_state_flag.v = Flag::RISE;
        }
        return;
    }

    if (cam0_state_flag.v == Flag::UNDEF || cam0_state_flag.v == Flag::RISE) {
        int rc;

        // TODO: How to put all supported configuration into the config before 
        //       opening the camera?
        //       Update Ctrl Format enum on camera Open and sed it to the UI
        //       The Ctrl configuration stores number of Format in enumeration list.
        //       If index is valid then use this format to start the stream
        do {
            cam0_last_check_ts = *now_ts;

            int cfg_buffs_num = ctrl.cmd_flds_config.buffs_num.v.num;
            
            TFlowCtrlCapture::cfg_v4l2_ctrls *cfg_v4l2 = 
                (TFlowCtrlCapture::cfg_v4l2_ctrls *)ctrl.cmd_flds_config.v4l2.v.ref;

            cam0 = new TFlowCaptureV4L2(context,
                std::string("Capture0"),
                std::string("com.reedl.tflow.capture0.buf-server"),
                cfg_buffs_num, 1, cfg_v4l2,
                std::bind(&TFlowCapture::onBufAP, this, std::placeholders::_1),
                std::bind(&TFlowCapture::onCustomMsg, this, std::placeholders::_1));

            // AV: Sometimes camera can't initialize just after open.
            //     Probably it is related with Sensor initialization time with old drivers.
            //     I.e. non blocking initilization. Let's add some explicit delay
            //     If it won't help exit the process as a last resort. Systemctl should restart as again.
            //     Second time driver should starts faster as I2c registers already initialized.
            usleep(200000);

            rc = cam0->Init(); // TODO: Add camera configuration here (WxH, format, frame rate, etc)
            // if (rc) break;
            if (rc) {
                if (cam0->dev_fd == -1) {
                    g_error("Can't open camera - critical error");
                    exit(1);
                }
            }

            if (cam0->sensor_api_type == TFlowCaptureV4L2::SENSOR_API_TYPE::FLYN_COIN417G2) {
                cfg_v4l2->flyn384.ui_ctrl = &ui_group_def;
            }
            else if (cam0->sensor_api_type == TFlowCaptureV4L2::SENSOR_API_TYPE::FLYN_TWIN412) {
                cfg_v4l2->flyn384.ui_ctrl = &ui_group_def;
            }
            else if (cam0->sensor_api_type == TFlowCaptureV4L2::SENSOR_API_TYPE::FLYN_TWIN412G2) {
                cfg_v4l2->twin412g2.ui_ctrl = &ui_group_def;
            }

            rc = cam0->StreamOn(getCamFmt());
            if (rc) break;

            // Loop over all connected clients and advertise Camera's FD
            cam0->onSrcFD();

            cam0_state_flag.v = Flag::SET;

        } while (0);

        if (rc) {
            cam0_state_flag.v = Flag::FALL;
#if 0
            if ( cam->dev_fd != -1 || cam->sub_dev_fd != -1 ) {
                // Camera was opened but something goes wrong
                // TODO: Don't try to open camera again until config changed
            } 
            else {
                // Can't open camera - try again later 
                cam_state_flag.v = Flag::CLR;
            }
#endif
        }
        else {
            /* Camera is open. Now we can advertise video buffers */
            cam0->sck_state_flag.v = Flag::RISE;
        }
        return;
    }

    if (cam0_state_flag.v == Flag::FALL) {
        // All Cli ports need to be closed and buffers released 
        // prior the camera close
        if (cam0->sck_state_flag.v != Flag::CLR) {
            cam0->sck_state_flag.v = Flag::FALL;
        } else {
            // close the camera
            delete cam0;
            cam0 = nullptr;
            cam0_state_flag.v = Flag::CLR;
        }
    }

}

int TFlowCapture::onIdle()
{
    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);

    // TODO: Check configuration player or live
    //       Q: Should it be checked inside checkStateXxx() ?
    if (1) {
        checkCamState(&now_ts);
    }

#if CAPTURE_PLAYER
    else {
        // Draft - not tested
        checkPlayerState(&now_ts);
    }
#endif

    if (autopilot) {
        autopilot->onIdle(now_ts);
#if CODE_BROWSE
            // Milesi
            TFlowSerial::onIdle(now_ts);
            TFlowUDP::onIdle(now_ts);
            // Fixar
            TFlowSerial::onIdle(now_ts);
#endif

    }

    if (cam0) {
        cam0->onIdle(now_ts);
    }

    ctrl.ctrl_srv.onIdle(now_ts);

    return G_SOURCE_CONTINUE;
}

