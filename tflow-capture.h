#pragma once

#include <cassert>
#include <time.h>
#include <giomm.h>

#include "tflow-glib.hpp"
#include "tflow-common.h"
#include "tflow-ctrl-capture.h"
#include "tflow-buf-srv.h"
#include "tflow-ap.h"
#include "tflow-player.h"
#include "v4l2Device.h"

#define TFLOWBUF_MSG_CUSTOM_NAVIGATOR (TFLOWBUF_MSG_CUSTOM_ + 1)    // 0x80 + 1
struct pck_navigator {
    TFlowBuf::pck_hdr hdr;
    int32_t	    position_x;	        // Drone position relative to takeoff point	In meters			
    int32_t		position_y;	        // 
    int32_t	    north_azimuth;	    // Azimuth angle to North	In degrees
    uint32_t	sync_time;	        // Last position synchronization time, type of synchronization is in sync_mode	In ms
    uint8_t	    position_quality;	// Position confidence coefficient, accuracy	0 - position can’t be used, 255 - position max accuracy
    uint8_t	    video_quality;	    // Video quality	0 - bad quality … 255 - best quality
    uint8_t	    sync_mode;	        // TFlowNavigator synchronization mode	0 - GPS assisted, 1 - IMU assisted, 2 - Standalone
};

struct fmt_info {
    union {
        uint32_t u32;
        char c[4];
    }fmt_cc;
    uint32_t width;
    uint32_t height;
};

class TFlowCapture {
public:


    TFlowCapture(MainContextPtr context);
    ~TFlowCapture();

    MainContextPtr context;
    MainLoopPtr main_loop;
    
    void AttachIdle();
    int onIdle();

    // Functions that fill aux_ap_data from the proper source - AP or media file
    int onBufAP(TFlowBuf& buf);
    int onBufPlayer(TFlowBuf& buf);


    // Custom messages handler from buffer server's clients.
    // For instance, the tflow-process->navigator returns result of algorithm 
    // processing to send it back to the Auto Pilot.
    int onCustomMsg(const TFlowBuf::pck_t &in_msg);
    int onCustomMsgNavigator(const struct pck_navigator& in_msg);

    TFlowBufSrv *buf_srv;

    /*
     * Data shared between TFlowCapture and TFlowBufSrv. It will be copied to
     * TFlowBuf on each frame and sent to all TFlowBuf clients.
     * Accordingly, client's hosts (for ex. tflow-vstream) needs to be in sync
     * with the structure.
     */
    // Is it part of TFlowNav?
#pragma pack(push, 1)
    struct ap_data {
        uint32_t sign;
        uint32_t tv_sec;      // Local timestamp
        uint32_t tv_usec;     // Local timestamp
        uint32_t hwHealthStatus;
        int16_t  rangefinder_val_cm;
        uint8_t  rangefinder_type;
        uint8_t  stabilization_mode;
        int32_t  board_attitude_roll;
        int32_t  board_attitude_yaw;
        int32_t  board_attitude_pitch;
        int32_t  uav_attitude_roll;
        int32_t  uav_attitude_yaw;
        int32_t  uav_attitude_pitch;
        int32_t  pe_baro_alt;
        int32_t  curr_pos_height;
        int32_t  position_x;
        int32_t  position_y;
        int32_t  position_z;
        int32_t  raw_yaw;

    } aux_ap_data; // Temporary local copy
#pragma pack(pop)

private:
    void checkCamState(struct timespec *now_tp);
    void checkPlayerState(struct timespec* now_tp);

    TFlowCtrlCapture ctrl;

    V4L2Device *cam;
    Flag   cam_state_flag;     // FL_SET -> camera opened; FL_CLR -> camera closed
    struct timespec cam_last_check_ts;

    TFlowPlayer* player;
    Flag   player_state_flag;     // FL_SET -> player active; FL_CLR -> player disabled
    struct timespec player_last_check_ts;

    TFlowAutopilot *autopilot;
    
    const struct fmt_info* getCamFmt();

};

