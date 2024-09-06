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

    //  MJPEGCapture::imu_data and   TFlowCapture::imu_data are matched one-to-one
    MJPEGCapture::imu_data* player_imu = player->shm_tbl[buf.index].imu;

    aux_imu_data.sign      = player_imu->sign;
    aux_imu_data.tv_sec    = player_imu->tv_sec;
    aux_imu_data.tv_usec   = player_imu->tv_usec;
    aux_imu_data.log_ts    = player_imu->log_ts;
    aux_imu_data.roll      = player_imu->roll;
    aux_imu_data.pitch     = player_imu->pitch;
    aux_imu_data.yaw       = player_imu->yaw;

    aux_imu_data.altitude  = player_imu->altitude;
    aux_imu_data.pos_x     = player_imu->pos_x;
    aux_imu_data.pos_y     = player_imu->pos_y;
    aux_imu_data.pos_z     = player_imu->pos_z;

    buf.aux_data = (uint8_t*)&aux_imu_data;
    buf.aux_data_len = sizeof(aux_imu_data);
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

    if (autopilot->last_cas_ts_obsolete.tv_usec == autopilot->last_cas_ts.tv_usec) {
        static int presc = 0;
        // CAS still not updated
        if ((presc++ & 0xff) == 0) {
            timersub(&buf.ts, &autopilot->last_cas_ts_obsolete, &dt);
            g_warning("IMU isn't updated for %ldsec",
                dt.tv_sec);
        }
    }
    else {
        timersub(&buf.ts, &autopilot->last_cas_ts, &dt);
        if ( dt.tv_sec > 0 || (dt.tv_sec == 0 && dt.tv_usec > (200 * 1000))) {
            // IMU data is too old - do not add them 
            buf.aux_data = nullptr;
            buf.aux_data_len = 0;
            g_warning("IMU too old - dt = %ldsec %ldusec",
                dt.tv_sec, dt.tv_usec);

            autopilot->last_cas_ts_obsolete = autopilot->last_cas_ts;
            return 0;
        }
        else if (autopilot->last_cas_ts_obsolete.tv_usec != -1) {
            autopilot->last_cas_ts_obsolete.tv_usec = -1;
            g_warning("IMU restored - dt = %ldsec %ldusec",
                dt.tv_sec, dt.tv_usec);
        }
    }     

    aux_imu_data.sign      = 0x30554D49;                        // IMU0
    aux_imu_data.tv_sec    = autopilot->last_cas_ts.tv_sec;     // Local time
    aux_imu_data.tv_usec   = autopilot->last_cas_ts.tv_usec;    // Local time
    aux_imu_data.log_ts    = autopilot->last_cas.ts;            // AP time
    aux_imu_data.roll      = autopilot->last_cas.CAS_board_att_roll;
    aux_imu_data.pitch     = autopilot->last_cas.CAS_board_att_pitch;
    aux_imu_data.yaw       = autopilot->last_cas.CAS_board_att_yaw;

    aux_imu_data.altitude  = autopilot->last_pe.PE_baro_alt;
    aux_imu_data.pos_x     = autopilot->last_pe.PE_est_pos_x;
    aux_imu_data.pos_y     = autopilot->last_pe.PE_est_pos_y;
    aux_imu_data.pos_z     = autopilot->last_pe.PE_est_pos_z;

    buf.aux_data = (uint8_t*)&aux_imu_data;
    buf.aux_data_len = sizeof(aux_imu_data);
    return 0;
}

TFlowCapture::TFlowCapture(GMainContext* _context) :
    context(_context),
    ctrl(*this)
{
    g_main_context_push_thread_default(context);
    
    main_loop = g_main_loop_new(context, false);

    buf_srv = new TFlowBufSrv(context);

    cam = nullptr;
    cam_last_check_tp.tv_sec = 0;
    cam_last_check_tp.tv_nsec = 0;
    
    player = nullptr;
    player_last_check_tp.tv_sec = 0;
    player_last_check_tp.tv_nsec = 0;

    autopilot = new TFlowAutopilot(context, ctrl.serial_name_get(), ctrl.serial_baud_get());
}

TFlowCapture::~TFlowCapture()
{
    delete buf_srv;

    g_main_loop_unref(main_loop);
    main_loop = NULL;

    g_main_context_pop_thread_default(context);
    g_main_context_unref(context);
    context = NULL;
}


static gboolean tflow_capture_idle(gpointer data)
{
    TFlowCapture* app = (TFlowCapture*)data;

    app->onIdle();

    return G_SOURCE_CONTINUE;
}

void TFlowCapture::checkPlayerState(struct timespec *now_tp)
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
        if (diff_timespec_msec(now_tp, &player_last_check_tp) > 1000) {
            player_last_check_tp = *now_tp;
            player_state_flag.v = Flag::RISE;
        }
    }
    
    if (player_state_flag.v == Flag::UNDEF || player_state_flag.v == Flag::RISE) {
        int rc;

        // Check camera device name
        if (!ctrl.player_fname_is_valid()) return;

        do {
            int cfg_buffs_num = ctrl.cmd_flds_config.buffs_num.v.num;
            assert(player);
            player = new TFlowPlayer(context, cfg_buffs_num, 61.5f);

            /* Link Camera and TFlowBufferServer*/
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

void TFlowCapture::checkCamState(struct timespec* now_tp)
{
    if (cam_state_flag.v == Flag::SET) {
        return;
    }

    if (cam_state_flag.v == Flag::CLR) {

        // Check camera device name
        if (!ctrl.dev_name_is_valid()) return;

        /* Don't try connect to camera to often */
        if (diff_timespec_msec(now_tp, &cam_last_check_tp) > 1000) {
            cam_last_check_tp = *now_tp;
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
            cam = new V4L2Device(context, cfg_buffs_num, 1);

            /* Link Camera and TFlowBufferServer*/
            cam->buf_srv = buf_srv;

            buf_srv->cam = cam;
            buf_srv->registerOnBuf(this, _onBufAP);

            rc = cam->Init(ctrl.cam_name_get()); // TODO: Add camera configuration here (WxH, format, frame rate, etc)
            if (rc) break;

            rc = cam->StreamOn(ctrl.cam_fmt_get());
            if (rc) break;

            cam_state_flag.v = Flag::SET;
            rc = cam->onBuff();          // Trigger initial buffer readout - aka kick ???
#if CODE_BROWSE
            V4L2Device::onBuff();
#endif 
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

void TFlowCapture::onIdle()
{
    struct timespec now_tp;
    clock_gettime(CLOCK_MONOTONIC, &now_tp);

    // TODO: Check configuration player or live
    //       Q: Should it be checked inside checkStateXxx() ?
    if (1) {
        checkCamState(&now_tp);
    }
    else {
        // Draft - not tested
        checkPlayerState(&now_tp);
    }

    autopilot->onIdle(&now_tp);
    buf_srv->onIdle(&now_tp);
    ctrl.ctrl_srv.onIdle(&now_tp);
}

void TFlowCapture::AttachIdle()
{
    GSource* src_idle = g_timeout_source_new(IDLE_INTERVAL_MSEC);
    g_source_set_callback(src_idle, (GSourceFunc)tflow_capture_idle, this, nullptr);
    g_source_attach(src_idle, context);
    g_source_unref(src_idle);

    return;
}


