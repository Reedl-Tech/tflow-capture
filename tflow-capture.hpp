#pragma once

#include <cassert>
#include <time.h>
#include <giomm.h>

#include "tflow-build-cfg.hpp"
#include "tflow-glib.hpp"
#include "tflow-common.hpp"
#include "tflow-ctrl-capture.hpp"
#include "tflow-buf-srv.hpp"
#include "tflow-buf-pck.hpp"
#include "tflow-ap.hpp"
#include "tflow-player.h"
#include "tflow-capture-v4l2.hpp"

struct fmt_info {
    union {
        uint32_t u32;
        char c[4];
    }fmt_cc;

    v4l2_frmsizeenum frmsize;       // Info returned by driver

    uint32_t width;                 // Set by getCamFmt() from user config
    uint32_t height;
};

class TFlowCapture {
public:

    TFlowCapture(MainContextPtr context, const std::string cfg_fname);
    ~TFlowCapture();

    MainContextPtr context;
    MainLoopPtr main_loop;
    
    void AttachIdle();
    int onIdle();

    // Functions that fill aux_ap_data from the proper source - AP or media file
    int onBufAP(TFlowBuf& buf);
#if CAPTURE_PLAYER
    int onBufPlayer(TFlowBuf& buf);
#endif


    // Custom messages handler from buffer server's clients.
    // For instance, the tflow-process->navigator returns result of algorithm 
    // processing to send it back to the Auto Pilot.
    int onCustomMsg(const TFlowBufPck::pck &in_msg);

    TFlowBufSrv *buf_srv;

    TFlowCaptureV4L2 *cam;
private:
    void checkCamState(struct timespec *now_tp);

    TFlowCtrlCapture ctrl;

    Flag   cam_state_flag;     // FL_SET -> camera opened; FL_CLR -> camera closed
    struct timespec cam_last_check_ts;

#if CAPTURE_PLAYER
    void checkPlayerState(struct timespec* now_tp);
    TFlowPlayer* player;
    Flag   player_state_flag;     // FL_SET -> player active; FL_CLR -> player disabled
#endif

#if WITH_AP
    TFlowAP *autopilot;
#endif 

    const struct fmt_info* getCamFmt();

};

