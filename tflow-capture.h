#pragma once

#include <cassert>
#include <time.h>
#include <giomm.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

typedef Glib::RefPtr<Glib::MainLoop> MainLoopPtr;
typedef Glib::RefPtr<Glib::MainContext> MainContextPtr;

/* Buffer that is passed to TFlowBufSrv */
class TFlowBuf {
public:
    TFlowBuf();

    int index = -1;
    struct timeval ts = { 0 };
    uint32_t subscribers = 0;   // Bit mask of TFlowBufCli. Bit 0 - means buffer is in user space

    int age();
};

class Flag {
public:
    enum states {
        UNDEF,
        CLR,
        SET,
        FALL,
        RISE
    };
    enum states v = Flag::UNDEF;
};

#include "tflow-ctrl-capture.h"
#include "tflow-buf-srv.h"
#include "v4l2Device.h"


class TFlowBufCli {
public:
};


class TFlowCapture {
public:
    TFlowCapture();
    ~TFlowCapture();

    GMainContext *context;
    GMainLoop *main_loop;
    
    void Attach();
    void OnIdle();

    TFlowBufSrv *buf_srv;

private:
    void checkCamState(clock_t now);

    TFlowCtrlCapture ctrl;

    V4L2Device *cam;

    Flag   cam_state_flag;     // FL_SET -> camera opened; FL_CLR -> camera closed
    clock_t last_cam_check;
};

