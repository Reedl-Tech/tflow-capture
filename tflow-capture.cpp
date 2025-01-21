#include <iostream>
#include <functional>
#include <giomm.h>
#include <glib-unix.h>
#include <json11.hpp>

#include "tflow-capture.h"

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
    this->index = -1;
    this->length = 0;
    this->start = MAP_FAILED;

    memset(&ts, 0, sizeof(ts));
    sequence = 0;

    /* Parameters obtained from Kernel on the Client side */
    start = nullptr;        // Not used on Server side
    length = -1;            // Not used on Server side

    owners = 0;             // Bit mask of TFlowBufCli. Bit 0 - means buffer is in user space
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

static int _onCustomMsg(void* ctx, const TFlowBuf::pck_t& in_msg)
{
    TFlowCapture* m = (TFlowCapture*)ctx;
    return m->onCustomMsg(in_msg);
}

static int _onBufPlayer(void* ctx, TFlowBuf& buf)
{
    TFlowCapture* m = (TFlowCapture*)ctx;
    return m->onBufPlayer(buf);
}

static int _onBufAP(void* ctx, TFlowBuf &buf)
{
    TFlowCapture* m = (TFlowCapture*)ctx;
    return m->onBufAP(buf);
}


int TFlowCapture::onCustomMsgNavigator(const struct pck_navigator& in_msg_nav)
{
    // No logic here - just repack data to AP message

    static TFLOW_AP::out_msg msg = {
        .positioning = {
            .hdr = {
                .mark = 'T', 
                .src = 0, 
                .len = sizeof(TFLOW_AP::positioning) - sizeof(TFLOW_AP::hdr) 
            },
            .msg_id = 0x01
        }
    };

    msg.positioning.position_x       = in_msg_nav.position_x;
    msg.positioning.position_y       = in_msg_nav.position_y;
    msg.positioning.north_azimuth    = in_msg_nav.north_azimuth;
    msg.positioning.sync_time        = in_msg_nav.sync_time;
    msg.positioning.position_quality = in_msg_nav.position_quality;
    msg.positioning.video_quality    = in_msg_nav.video_quality;
    msg.positioning.sync_mode        = in_msg_nav.sync_mode;

    if (autopilot) autopilot->serialDataSend(msg);
    return 0;
}

int TFlowCapture::onCustomMsg(const TFlowBuf::pck_t& in_msg)
{
    if (in_msg.hdr.id <= TFLOWBUF_MSG_CUSTOM_) return 0;

    switch (in_msg.hdr.id) {
    case TFLOWBUF_MSG_CUSTOM_NAVIGATOR:
        onCustomMsgNavigator((const struct pck_navigator&)in_msg);
        break;
    default:
        break;
    }
    // Return -1 to close the cli_port
    return 0;
}

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

/* Add data from Autopilot to the temporary aux buffer */
int TFlowCapture::onBufAP(TFlowBuf& buf) 
{
#if CODE_BROWSE
    // caled from 
    TFlowBufSrv::buf_consume();
#endif

    // Frame time normally is ahead of AP time
    struct timeval dt;

    if (!autopilot) return 0;

    if (autopilot->last_sensors_ts_obsolete.tv_usec == autopilot->last_sensors_ts.tv_usec) {
        static int presc = 0;
        // sensors still not updated
        if ((presc++ & 0xff) == 0) {
            timersub(&buf.ts, &autopilot->last_sensors_ts_obsolete, &dt);
            g_warning("IMU isn't updated for %ldsec", dt.tv_sec);
        }
        return 0;
    }
    else {
        timersub(&buf.ts, &autopilot->last_sensors_ts, &dt);
        if ( dt.tv_sec > 0 || (dt.tv_sec == 0 && dt.tv_usec > (200 * 1000))) {
            // IMU data is too old - do not add them 
            buf.aux_data = nullptr;
            buf.aux_data_len = 0;
            g_warning("IMU too old - dt = %ldsec %ldusec",
                dt.tv_sec, dt.tv_usec);

            autopilot->last_sensors_ts_obsolete = autopilot->last_sensors_ts;
            return 0;
        }
        else if (autopilot->last_sensors_ts_obsolete.tv_usec != -1) {
            autopilot->last_sensors_ts_obsolete.tv_usec = -1;
            g_warning("IMU restored - dt = %ldsec %ldusec",
                dt.tv_sec, dt.tv_usec);
        }
    }     

    aux_ap_data.sign      = 0x32554D49;                            // "IMU2" -> IMU v.2
    aux_ap_data.tv_sec    = autopilot->last_sensors_ts.tv_sec;     // Local time
    aux_ap_data.tv_usec   = autopilot->last_sensors_ts.tv_usec;    // Local time

    aux_ap_data.hwHealthStatus       = autopilot->last_sensors.hwHealthStatus;
    aux_ap_data.rangefinder_val_cm   = autopilot->last_sensors.rangefinder_val_cm;
    aux_ap_data.rangefinder_type     = autopilot->last_sensors.rangefinder_type;
    aux_ap_data.stabilization_mode   = autopilot->last_sensors.stabilization_mode;
    aux_ap_data.board_attitude_roll  = autopilot->last_sensors.board_attitude_roll;
    aux_ap_data.board_attitude_yaw   = autopilot->last_sensors.board_attitude_yaw;
    aux_ap_data.board_attitude_pitch = autopilot->last_sensors.board_attitude_pitch;
    aux_ap_data.uav_attitude_roll    = autopilot->last_sensors.uav_attitude_roll;
    aux_ap_data.uav_attitude_yaw     = autopilot->last_sensors.uav_attitude_yaw;
    aux_ap_data.uav_attitude_pitch   = autopilot->last_sensors.uav_attitude_pitch;
    aux_ap_data.pe_baro_alt          = autopilot->last_sensors.pe_baro_alt;
    aux_ap_data.curr_pos_height      = autopilot->last_sensors.curr_pos_height;
    aux_ap_data.position_x           = autopilot->last_sensors.position_x;
    aux_ap_data.position_y           = autopilot->last_sensors.position_y;
    aux_ap_data.position_z           = autopilot->last_sensors.position_z;
    aux_ap_data.raw_yaw              = autopilot->last_sensors.raw_yaw;

    buf.aux_data = (uint8_t*)&aux_ap_data;
    buf.aux_data_len = sizeof(aux_ap_data);
    
    {
        static int presc = 0;
        if ((presc++ & 0x7f) == 0) {
            g_warning("TFlowAP[%d]: Health=0x%08X ROLL=%5.1f PITCH=%5.1f YAW=%5.1f Alt=%f (rf=%d) Mode=%d",
                autopilot->last_sensor_cnt,
                autopilot->last_sensors.hwHealthStatus,
                (float)autopilot->last_sensors.board_attitude_roll / 100,
                (float)autopilot->last_sensors.board_attitude_pitch / 100,
                (float)autopilot->last_sensors.board_attitude_yaw / 100,
                (float)autopilot->last_sensors.curr_pos_height / 100,
                autopilot->last_sensors.rangefinder_val_cm,
                autopilot->last_sensors.stabilization_mode);
        }
    }
    return 0;
}

TFlowCapture::TFlowCapture(MainContextPtr _context, const std::string cfg_fname) :
    context(_context),
    ctrl(*this, cfg_fname)
{
    main_loop = Glib::MainLoop::create(context, false);

    Glib::signal_timeout().connect(sigc::mem_fun(*this, &TFlowCapture::onIdle), IDLE_INTERVAL_MSEC);

    buf_srv = new TFlowBufSrv(context);

    buf_srv->registerOnCustomMsg(this, _onCustomMsg);

    cam = nullptr;
    cam_last_check_ts.tv_sec = 0;
    cam_last_check_ts.tv_nsec = 0;
    
    player = nullptr;
    player_last_check_ts.tv_sec = 0;
    player_last_check_ts.tv_nsec = 0;

    if (ctrl.serial_name_is_valid()) {
        autopilot = new TFlowAutopilot(context, ctrl.serial_name_get(), ctrl.serial_baud_get());
    } else {
        g_info("TFlowAP: Disabled - Serial port name not configured");
        autopilot = nullptr;
    }
}

TFlowCapture::~TFlowCapture()
{
    delete buf_srv;

    if (autopilot) {
        delete autopilot;
    }
}

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

const struct fmt_info* TFlowCapture::getCamFmt() 
{
    int cam_fmt_idx = -1;
    // Get format from config - in config enum by config idx
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
        for (struct fmt_info &cam_fmt : cam->fmt_info_enum) {
            if (cam_fmt.fmt_cc.u32 == V4L2_PIX_FMT_GREY){
                return &cam_fmt;
            }
        }

        g_warning("Use default camera format [0]");
        return &cam->fmt_info_enum[0];
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
    for (struct fmt_info &cam_fmt : cam->fmt_info_enum) {
        if ((cam_fmt.fmt_cc.u32 == cfg_fmt.fmt_cc.u32) &&
            (cam_fmt.width  == cfg_fmt.width) &&
            (cam_fmt.height == cfg_fmt.height)) {
            return &cam_fmt;
        }
    }

    g_warning("Config format not matched (%d)", cfg_idx);

    return nullptr;
}

void TFlowCapture::checkCamState(struct timespec* now_ts)
{
    if (cam_state_flag.v == Flag::SET) {
        if (cam->is_stall()) {
            cam_state_flag.v = Flag::FALL;
        }
        return;
    }

    if (cam_state_flag.v == Flag::CLR) {

        /* Don't try connect to camera to often */
        if (diff_timespec_msec(now_ts, &cam_last_check_ts) > 1000) {
            cam_last_check_ts = *now_ts;
            cam_state_flag.v = Flag::RISE;
        }
    }

    if (cam_state_flag.v == Flag::UNDEF || cam_state_flag.v == Flag::RISE) {
        int rc;

        // TODO: How to put all supported configuration into the config before 
        //       opening the camera?
        //       Update Ctrl Format enum on camera Open and sed it to the UI
        //       The Ctrl configuration stores number of Format in enumeration list.
        //       If index is valid then use this format to start the stream
        do {
            int cfg_buffs_num = ctrl.cmd_flds_config.buffs_num.v.num;
            cam = new V4L2Device(context, cfg_buffs_num, 1, 
                (const TFlowCtrlCapture::cfg_v4l2_ctrls*)ctrl.cmd_flds_config.v4l2.v.ref);

            /* Link Camera and TFlowBufferServer*/
            cam->buf_srv = buf_srv;

            buf_srv->cam = cam;
            buf_srv->registerOnBuf(this, _onBufAP);

            rc = cam->Init(); // TODO: Add camera configuration here (WxH, format, frame rate, etc)
            if (rc) break;

            rc = cam->StreamOn(getCamFmt());
            if (rc) break;

            cam_state_flag.v = Flag::SET;

        } while (0);

        if (rc) {
            // Can't open camera - try again later 
            cam_state_flag.v = Flag::CLR;
        }
        else {
            /* Camera is open. Now we can advertise video buffers */
            buf_srv->sck_state_flag.v = Flag::RISE;
        }
        return;
    }

    if (cam_state_flag.v == Flag::FALL) {
        // All Cli ports need to be closed and buffers released 
        // prior the camera close
        if (buf_srv->sck_state_flag.v != Flag::CLR) {
            buf_srv->sck_state_flag.v = Flag::FALL;
        }

        if (buf_srv->sck_state_flag.v == Flag::CLR) {
            // close the camera
            delete cam;
            cam = NULL;
            cam_state_flag.v = Flag::CLR;
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
    else {
        // Draft - not tested
        checkPlayerState(&now_ts);
    }

    if (autopilot) {
        autopilot->onIdle(&now_ts);

        // FLYN384 is a SHUTER camera, thus the calibration needs to be
        // disabled during the flight
        if (autopilot->last_sensors_ts_obsolete.tv_usec != autopilot->last_sensors_ts.tv_usec) {
            if (cam && (0 == cam->driver_name.compare("flyn384"))) {
                // TODO: Set calibration OFF on disarm and 
                //       ON on "Copter" or "Plane" stabilization mode.
                //       Stabilization mode isn't valid so far, thus let's use altitude with 
                //       some hysteresis as a workaround.
                if (autopilot->last_sensors.curr_pos_height < 5 && !cam->flyn_calib_is_on) {
                    cam->ioctlSetControls_flyn_calib(1);
                } 
                else if (autopilot->last_sensors.curr_pos_height > 10 && cam->flyn_calib_is_on) {
                    cam->ioctlSetControls_flyn_calib(0);
                }
            }
        }

    }

    buf_srv->onIdle(&now_ts);
    ctrl.ctrl_srv.onIdle(&now_ts);



    return G_SOURCE_CONTINUE;
}


