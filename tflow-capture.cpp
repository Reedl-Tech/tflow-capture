#include <giomm.h>
#include <glib-unix.h>

#include "tflow-capture.h"

TFlowCapture::TFlowCapture() : 
    ctrl(this)
{
    context = g_main_context_new();
    g_main_context_push_thread_default(context);
    
    main_loop = g_main_loop_new(context, false);

    //dbg  = new AticDbg(*this);
    //dbg->Init(); // Q: ? Should it be part of constructor ?

    // ctrl = new TFlowCtrlCapture(*this);
    ctrl.Init(); // Q: ? Should it be part of constructor ?

    last_cam_check = clock();
}

TFlowCapture::~TFlowCapture()
{
    g_main_loop_unref(main_loop);
    main_loop = NULL;

    if (ctrl) {
        delete ctrl;
    }

    if (dbg) {
        delete dbg;
    }

    if (gst) {
        delete gst;
    }

    g_main_context_pop_thread_default(context);
    g_main_context_unref(context);
    context = NULL;
}


static gboolean tflow_capture_idle(gpointer data)
{
    TFlowCapture* app = (TFlowCapture*)data;

    app->OnIdle();

    return true;
}

void TFlowCapture::checkCamState(clock_t now)
{
    clock_t dt = now - last_cam_check;

    /* Do not check to often*/
    if (dt < 3 * CLOCKS_PER_SEC ) return;

    if (cam_state_flag == FL_SET || cam_state_flag == FL_CLR) {
        return;
    }
    
    if (cam_state_flag == FL_UNDEF || cam_state_flag == FL_RISE) {
        // Check camera device name
        if (!ctrl.cam_name_is_valid()) return;

        int rc = cam.CaptureInit(ctrl.cam_name_get());
        if (rc == 0) {
            cam_state_flag == FL_SET;
        }
        else {
            // Can't open camera - try again later 
            cam_state_flag = FL_RISE;
        }
    }

    if (cam_state_flag == FL_FALL) {
        // close the camera
        cam_state_flag = FL_FALL;
    }

}

void TFlowCapture::OnIdle()
{
    clock_t now = clock();

    checkCamState(now);

}

void TFlowCapture::Start()
{
    g_info("App Streamer Started");

    GSource* src_idle = g_idle_source_new();
    g_source_set_callback(src_idle, (GSourceFunc)tflow_capture_idle, &app, nullptr);
    g_source_attach(src_idle, app.context);
    g_source_unref(src_idle);

    return;
}

