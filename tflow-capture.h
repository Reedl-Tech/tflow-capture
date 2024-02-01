#pragma once

#include <cassert>
#include <time.h>
#include <giomm.h>

#include "tflow-common.h"
#include "tflow-ctrl-capture.h"
#include "tflow-buf-srv.h"
#include "v4l2Device.h"

class TFlowCapture {
public:
    TFlowCapture(GMainContext* context);
    ~TFlowCapture();

    GMainContext* context;
    GMainLoop *main_loop;
    
    void AttachIdle();
    void OnIdle();

    TFlowBufSrv *buf_srv;

private:
    void checkCamState(clock_t now);

    TFlowCtrlCapture ctrl;

    V4L2Device *cam;

    Flag   cam_state_flag;     // FL_SET -> camera opened; FL_CLR -> camera closed
    clock_t last_cam_check;
};

