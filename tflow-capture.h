#pragma once

#include <cassert>
#include <time.h>
#include <giomm.h>

#include "tflow-ctrl-capture.h"
#include "v4l2Device.h"

typedef Glib::RefPtr<Glib::MainLoop> MainLoopPtr;
typedef Glib::RefPtr<Glib::MainContext> MainContextPtr;

// Structure to contain all information so that we can pass it to callback

class Flags {
    enum states {
        FL_UNDEF,
        FL_CLR,
        FL_SET,
        FL_FALL,
        FL_RISE
    } 
}
class TFlowCapture {
public:
    TFlowCapture();
    ~TFlowCapture();

    GMainContext *context;
    GMainLoop *main_loop;
    
    void Start();
    void OnIdle();

    TFlowCtrlCapture ctrl;

    V4L2Device cam;

    Flags   cam_state_flag;     // FL_SET -> camera opened; FL_CLR -> camera closed
    clock_t last_cam_check;
};

