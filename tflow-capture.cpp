#include <iostream>
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
            g_warning("IMU isn't updated for %ldsec",
                dt.tv_sec);
        }
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

    aux_ap_data.sign      = 0x32554D49;                        // "IMU2" -> IMU v.2
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
        if ((presc++ & 0x3f) == 0) {
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

TFlowCapture::TFlowCapture(MainContextPtr _context) :
    context(_context),
    ctrl(*this)
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
            //int player_state = 1;   // Play/Pause

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

void TFlowCapture::checkCamState(struct timespec* now_ts)
{
    if (cam_state_flag.v == Flag::SET) {
        return;
    }

    if (cam_state_flag.v == Flag::CLR) {

        // Check camera device name
        if (!ctrl.dev_name_is_valid()) return;

        /* Don't try connect to camera to often */
        if (diff_timespec_msec(now_ts, &cam_last_check_ts) > 1000) {
            cam_last_check_ts = *now_ts;
            cam_state_flag.v = Flag::RISE;
        }
    }

    if (cam_state_flag.v == Flag::UNDEF || cam_state_flag.v == Flag::RISE) {
        int rc;

        // Check camera device name
        if (!ctrl.dev_name_is_valid()) return;

        // TODO: How to put all supported configuration into the config before 
        //       opening the camera?
        //       Update Ctrl Format enum on camera Open and sed it to the UI
        //       The Ctrl configuration stores number of Format in enumeration list.
        //       If index is valid then use this format to start the stream
        do {
            int cfg_buffs_num = ctrl.cmd_flds_config.buffs_num.v.num;
            cam = new V4L2Device(context, cfg_buffs_num, 1, 
                (const TFlowCtrlCapture::cfg_v4l2_ctrls*)ctrl.cmd_flds_config.v4l2_ctrls.v.ref);

            /* Link Camera and TFlowBufferServer*/
            cam->buf_srv = buf_srv;

            buf_srv->cam = cam;
            buf_srv->registerOnBuf(this, _onBufAP);

            rc = cam->Init(ctrl.cam_name_get()); // TODO: Add camera configuration here (WxH, format, frame rate, etc)
            if (rc) break;

            rc = cam->StreamOn(ctrl.cam_fmt_get());
            if (rc) break;

            cam_state_flag.v = Flag::SET;

            //rc = cam->onBuff();          // Trigger initial buffer readout - aka kick ???
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

        buf_srv->sck_state_flag.v == Flag::FALL;

        // close the camera
        delete cam;
        cam = NULL;
        cam_state_flag.v = Flag::CLR;
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
    }

    buf_srv->onIdle(&now_ts);
    ctrl.ctrl_srv.onIdle(&now_ts);

    return G_SOURCE_CONTINUE;
}


