// adsa
#include <iostream>
#include <giomm.h>
#include <glib-unix.h>
#include <json11.hpp>

#include "tflow-capture.h"

using namespace json11;

v4l2_buffer g_buf;

TFlowBuf::TFlowBuf()
{
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

TFlowCapture::TFlowCapture() : 
    ctrl(*this)
{
    context = g_main_context_new();
    g_main_context_push_thread_default(context);
    
    main_loop = g_main_loop_new(context, false);

    ctrl.Init(); // Q: ? Should it be part of constructor ?

    buf_srv = new TFlowBufSrv(context);
    {
        int cfg_buffs_num = ctrl.cmd_flds_config.buffs_num.v.u32;
        cam = new V4L2Device(context, cfg_buffs_num, 1);
    }

    /* Link Camera and TFlowBufferServer*/
    cam->buf_srv = buf_srv;
    buf_srv->cam = cam;

    last_cam_check = clock();
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

    app->OnIdle();

    return G_SOURCE_CONTINUE;
}

void TFlowCapture::checkCamState(clock_t now)
{
    clock_t dt = now - last_cam_check;

    /* Do not check to often*/
    if (dt < 3 * CLOCKS_PER_SEC ) return;
    last_cam_check = now;

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

    if (cam_state_flag.v == Flag::SET || cam_state_flag.v == Flag::CLR) {
        return;
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
            cam_state_flag.v = Flag::RISE;
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

void TFlowCapture::OnIdle()
{
    clock_t now = clock();

    checkCamState(now);     // 
    buf_srv->onIdle(now);

}

void TFlowCapture::AttachIdle()
{
    g_info("App Streamer Started");

    GSource* src_idle = g_idle_source_new();
    g_source_set_callback(src_idle, (GSourceFunc)tflow_capture_idle, this, nullptr);
    g_source_attach(src_idle, context);
    g_source_unref(src_idle);

    return;
}


