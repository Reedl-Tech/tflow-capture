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
#include "tflow-pwm.hpp"
#include "tflow-udp.hpp"
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
    
    int onIdle();

    // Custom messages handler from buffer server's clients.
    // For instance, the tflow-process->navigator returns result of algorithm 
    // processing to send it back to the Auto Pilot.
    int onCustomMsg(const TFlowBufPck::pck &in_msg);

    TFlowCaptureV4L2 *cam0;
    //TFlowCaptureV4L2 *cam1;

    struct ptt_aux_data {

    } aux;

    int onPttGcMsg(const char* udp_msg, int udp_msg_len);

    // Functions that fill aux_data with PTT specific 
    int onBufPtt(TFlowBuf& buf);
    TFlowPWM ptt_pan;       // Servo control
    TFlowPWM ptt_tilt;      // Servo control
    TFlowUDP ptt_gc_link;   // Connection with Ground Control

    int aux_data_size;
    uint8_t* aux_data_buf;

#pragma pack(push, 1)
    // AUX data within video frame
    struct ptt_aux_v0 {
        uint32_t sign;
        uint32_t tv_sec;      // Local timestamp
        uint32_t tv_usec;     // Local timestamp
        float pan_val;
        float pan_speed;
        float pan_trgt;
        float tilt_val;
        float tilt_speed;
        float tilt_trgt;
    };

    // Control message from PTT GC (IP/UDP link)
    struct ptt_gc_v0 {
        uint32_t sign;
        float pan_val;
        float pan_speed;
        float tilt_val;
        float tilt_speed;
        uint32_t mode;          // Scan vertical/horizontal/manual/etc
    };
#pragma pack(pop)

private:
    void checkCamState(struct timespec *now_tp);

    TFlowCtrlCapture ctrl;

    Flag   cam0_state_flag;     // FL_SET -> camera opened; FL_CLR -> camera closed
    struct timespec cam0_last_check_ts;

    const struct fmt_info* getCamFmt();

    int onPttTimer();

    
};

