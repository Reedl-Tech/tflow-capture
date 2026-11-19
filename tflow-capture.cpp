#include "tflow-build-cfg.hpp"
#include <iostream>
#include <functional>
#include <cmath>

#include "tflow-glib.hpp"
#include "tflow-common.hpp"

#include <json11.hpp>

#include "tflow-buf-srv.hpp"

#include "tflow-capture.hpp"

using namespace json11;
using namespace std;

v4l2_buffer g_buf;

// Values specific for the servo MODEL: RDS3218 @ PTT
std::vector<int> ptt_pan_dtc_range_value  = {  40, 195 };
std::vector<int> ptt_tilt_dtc_range_value = { 120, 160 };

struct TFlowCtrlUI::uictrl ui_sl_dtc_val = {
    .label = "Servo pos 0..1",
    .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
    .size = -1,
    .slider = {.min = 0, .max = 100}
};

struct TFlowCtrlUI::uictrl ui_sl2_pan_dtc_range = {
    .label = nullptr,
    .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER2,
    .size = -1,
    .slider = {.min = 0, .max = 200}
};

struct TFlowCtrlUI::uictrl ui_sl2_tilt_dtc_range = {
    .label = nullptr,
    .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER2,
    .size = -1,
    .slider = {.min = 0, .max = 200}
};

struct TFlowCtrlUI::uictrl ui_sw_force_en = {
    .label = "Force",
    .type = TFlowCtrlUI::UICTRL_TYPE::SWITCH,
};

TFlowPWMCfg::cfg_tflow_servo_cntrl ptt_pan_cmd_flds_cfg = {
        TFLOW_CMD_HEAD("PTT Pan"),
        .channel         = { "pan_channel",         TFlowCtrl::CFT_NUM,  0, {.num =              0 } },
        .period          = { "pan_period",          TFlowCtrl::CFT_NUM,  0, {.num =   20 * 1000000 } },   // In nano sec
        .initial_val     = { "pan_init_val",        TFlowCtrl::CFT_DBL,  0, {.dbl =   1.0          }, &ui_edit_def },
        .degr2dtc        = { "pan_degr2dtc",        TFlowCtrl::CFT_DBL,  0, {.dbl = (10.0/9*10000) }, &ui_edit_def },
        .move_speed      = { "pan_move_speed",      TFlowCtrl::CFT_DBL,  0, {.dbl =            1.0 }, &ui_edit_def },
        .update_time_min = { "pan_update_time_min", TFlowCtrl::CFT_DBL,  0, {.dbl =           30.0 }, &ui_edit_def },   // minimal PWM update time in ms
        .force_up        = { "pan_force_up",        TFlowCtrl::CFT_NUM,  0, {.num =              0 }, &ui_switch_def },
        .force_down      = { "pan_force_down",      TFlowCtrl::CFT_NUM,  0, {.num =              0 }, &ui_switch_def },
        .dtc_range       = { "pan_dtc_range",       TFlowCtrl::CFT_VNUM, 0, {.vnum = &ptt_pan_dtc_range_value}, &ui_sl2_pan_dtc_range },
        .force_dtc_en    = { "pan_force_dtc_en",    TFlowCtrl::CFT_NUM,  0, {.num =              0 }, &ui_sw_force_en },
        .force_dtc_degr  = { "pan_force_dtc_degr",  TFlowCtrl::CFT_NUM,  0, {.num =             20 }  },
        .force_dtc_val   = { "pan_force_dtc_val",   TFlowCtrl::CFT_NUM,  0, {.num =              0 }, &ui_sl_dtc_val  },
        TFLOW_CMD_EOMSG
    };

TFlowPWMCfg::cfg_tflow_servo_cntrl ptt_tilt_cmd_flds_cfg = {
        TFLOW_CMD_HEAD("PTT Tilt"),
        .channel         = { "tilt_channel",         TFlowCtrl::CFT_NUM,  0, {.num =              1 } },
        .period          = { "tilt_period",          TFlowCtrl::CFT_NUM,  0, {.num =   20 * 1000000 } },   // In nano sec
        .initial_val     = { "tilt_init_val",        TFlowCtrl::CFT_DBL,  0, {.dbl = 0.5            }, &ui_edit_def },
        .degr2dtc        = { "tilt_degr2dtc",        TFlowCtrl::CFT_DBL,  0, {.dbl = (10.0/9*10000) }, &ui_edit_def },
        .move_speed      = { "tilt_move_speed",      TFlowCtrl::CFT_DBL,  0, {.dbl =            1.0 }, &ui_edit_def },
        .update_time_min = { "tilt_update_time_min", TFlowCtrl::CFT_DBL,  0, {.dbl =           30.0 }, &ui_edit_def },   // minimal PWM update time in ms
        .force_up        = { "tilt_force_up",        TFlowCtrl::CFT_NUM,  0, {.num =              0 }, &ui_switch_def },
        .force_down      = { "tilt_force_down",      TFlowCtrl::CFT_NUM,  0, {.num =              0 }, &ui_switch_def },
        .dtc_range       = { "tilt_dtc_range",       TFlowCtrl::CFT_VNUM, 0, {.vnum = &ptt_tilt_dtc_range_value}, &ui_sl2_tilt_dtc_range },
        .force_dtc_en    = { "tilt_force_dtc_en",    TFlowCtrl::CFT_NUM,  0, {.num =              0 }, &ui_sw_force_en },
        .force_dtc_degr  = { "tilt_force_dtc_degr",  TFlowCtrl::CFT_NUM,  0, {.num =             20 }},
        .force_dtc_val   = { "tilt_force_dtc_val",   TFlowCtrl::CFT_NUM,  0, {.num =              0 }, &ui_sl_dtc_val },
        TFLOW_CMD_EOMSG
    };


#define IDLE_INTERVAL_MSEC 300
#define PTT_INTERVAL_MSEC 10

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

int TFlowCapture::onCustomMsg(const TFlowBufPck::pck& in_msg)
{
    /* Message received from a TFlowProcess */
    if (in_msg.hdr.id <= TFlowBufPck::TFLOWBUF_MSG_CUSTOM_) return 0;

    return 0;
}

int TFlowCapture::onPttGcMsg(const char* udp_msg, int udp_msg_len)
{
    // Get Pan/Tilt control message
    const struct ptt_gc_v0* ptt_gc = (const struct ptt_gc_v0*)udp_msg;
    assert(ptt_gc->sign == 0x50545430);               // PTT0
    
    if (isfinite(ptt_gc->pan_val)) {
        ptt_pan.move_target = MAX(0, MIN(ptt_gc->pan_val, 1));
    }

    if (isfinite(ptt_gc->tilt_val)) {
        ptt_tilt.move_target = MAX(0, MIN(ptt_gc->tilt_val, 1));
    }
    
    //ptt_pan.move_speed = 0.; // defined by config
    //ptt_tilt.move_speed = 0.; // defined by config
    return 0;
}

TFlowCapture::TFlowCapture(MainContextPtr _context, const std::string cfg_fname) :
    cam0(nullptr),
    context(_context),
    ptt_pan(&ptt_pan_cmd_flds_cfg),
    ptt_tilt(&ptt_tilt_cmd_flds_cfg),
    ptt_gc_link(std::bind(&TFlowCapture::onPttGcMsg, this, std::placeholders::_1, std::placeholders::_2)),
    ctrl(*this, cfg_fname)        // Att!: Must be constructed after submodules' pointers initialization
{
    main_loop = Glib::MainLoop::create(context, false);
   
    Glib::signal_timeout().connect(sigc::mem_fun(*this, &TFlowCapture::onIdle), IDLE_INTERVAL_MSEC);
    Glib::signal_timeout().connect(sigc::mem_fun(*this, &TFlowCapture::onPttTimer), PTT_INTERVAL_MSEC);

    cam0_last_check_ts.tv_sec = 0;
    cam0_last_check_ts.tv_nsec = 0;

    aux_data_size = 1024;
    aux_data_buf = (uint8_t*)calloc(1, aux_data_size);
}

TFlowCapture::~TFlowCapture()
{

}

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

int TFlowCapture::onBufPtt(TFlowBuf &buf)
{
    // Called on every received video frame.
    // Custom submodules can do some work here.
    // For ex. add custom data to frame's AUX section.

    // Fill PTT structure 
    // set pointer to PTT aux_data
    struct ptt_aux_v0* aux_ptt_v0 = (struct ptt_aux_v0*)aux_data_buf;
    aux_ptt_v0->sign = 0x50545430;               // PTT0
    aux_ptt_v0->tv_sec = buf.ts.tv_sec;
    aux_ptt_v0->tv_usec = buf.ts.tv_usec;
    aux_ptt_v0->pan_val = ptt_pan.get_val();
    aux_ptt_v0->tilt_val = ptt_tilt.get_val();

    buf.aux_data = aux_data_buf;
    buf.aux_data_len = sizeof(struct ptt_aux_v0);

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
                std::bind(&TFlowCapture::onBufPtt, this, std::placeholders::_1),
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

    if (cam0) {
        cam0->onIdle(now_ts);
    }

    ptt_gc_link.onIdle(now_ts);

    ctrl.ctrl_srv.onIdle(now_ts);

    return G_SOURCE_CONTINUE;
}

int TFlowCapture::onPttTimer()
{
    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);

    ptt_pan.move_update(now_ts);
    ptt_tilt.move_update(now_ts);

    return G_SOURCE_CONTINUE;
}
