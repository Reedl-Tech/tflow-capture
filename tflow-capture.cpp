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

static int _onBuf(void* ctx, TFlowBuf &buf)
{
    TFlowCapture* m = (TFlowCapture*)ctx;
    return m->onBuf(buf);
}

int TFlowCapture::onBuf(TFlowBuf& buf) 
{

#if CODE_BROWSE
    // caled from 
    TFlowBufSrv::buf_consume();
#endif

    // Frame time is normally ahead of AP time
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
    aux_imu_data.pos_x = autopilot->last_pe.PE_est_pos_x;
    aux_imu_data.pos_y = autopilot->last_pe.PE_est_pos_y;
    aux_imu_data.pos_z = autopilot->last_pe.PE_est_pos_z;

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

    // TODO: Create camera in Idle loop on open attempt like a serial port?
    {
        int cfg_buffs_num = ctrl.cmd_flds_config.buffs_num.v.num;
        cam = new V4L2Device(context, cfg_buffs_num, 1);
    }

    /* Link Camera and TFlowBufferServer*/
    cam->buf_srv = buf_srv;
    buf_srv->cam = cam;
    buf_srv->registerOnBuf(this, _onBuf);

    cam_last_check_tp.tv_sec = 0;
    cam_last_check_tp.tv_nsec = 0;
    
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

void TFlowCapture::checkCamState(struct timespec *now_tp)
{

#if 0
    std::cout << "kuku5" << std::endl;
    cam.f_in_fd = open("/dev/ttyACM0", O_RDWR | O_NONBLOCK);

    cam.f_in_src = g_source_new(&f_gsf, sizeof(CamFdSource));
    cam.f_in_tag = g_source_add_unix_fd(cam.f_in_src, cam.f_in_fd, (GIOCondition)(G_IO_OUT | G_IO_IN | G_IO_PRI | G_IO_ERR | G_IO_HUP | G_IO_NVAL));
    g_source_attach(cam.f_in_src, context);

    char b[10];
    int rc = read(cam.f_in_fd, b, 10);
    int err = errno;
#endif

    if (cam_state_flag.v == Flag::SET) {
        return;
    }

    if (cam_state_flag.v == Flag::CLR) {
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
        //       opening the camera.
        //       Update Ctrl Format enum on camera Open and sed it to the UI
        //       The Ctrl configuration stores number of Format in enumeration list.
        //       If index is valid then use this format to start the stream
        do {
            rc = cam->Init(ctrl.cam_name_get()); // TODO: Add camera configuration here (WxH, format, frame rate, etc)
            if (rc) break;

            rc = cam->StreamOn(ctrl.cam_fmt_get());
            if (rc) break;

            cam_state_flag.v = Flag::SET;
            rc = cam->onBuff();          // Trigger initial buffer readout - aka kick ???
        }
        while (0);
        
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

    checkCamState(&now_tp);

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


