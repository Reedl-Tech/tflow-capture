#pragma once

#include <cassert>
#include <time.h>
#include <giomm.h>

#include "tflow-common.h"
#include "tflow-ctrl-capture.h"
#include "tflow-buf-srv.h"
#include "tflow-ap.h"
#include "tflow-player.h"
#include "v4l2Device.h"

class TFlowCapture {
public:
    TFlowCapture(GMainContext* context);
    ~TFlowCapture();

    GMainContext* context;
    GMainLoop *main_loop;
    
    void AttachIdle();
    void onIdle();

    // Functions that fill aux_imu_data from the proper source - AP or media file
    int onBufAP(TFlowBuf& buf);
    int onBufPlayer(TFlowBuf& buf);

    TFlowBufSrv *buf_srv;

    /*
     * Data shared between TFlowCapture and TFlowBufSrv. It will be copied to
     * TFlowBuf on each frame and sent to all TFlowBuf clients.
     */
    // Is it part of TFlowNav?
#pragma pack(push, 1)
    struct imu_data {
        uint32_t sign;
        uint32_t tv_sec;      // Local timestamp
        uint32_t tv_usec;     // Local timestamp
        uint32_t log_ts;      // Timestamp received from AP
        int32_t roll;
        int32_t pitch;
        int32_t yaw;
        int32_t altitude;
        int32_t pos_x;
        int32_t pos_y;
        int32_t pos_z;
    } aux_imu_data; // Temporary local copy of IMU data 
#pragma pack(pop)

private:

    void checkCamState(struct timespec *now_tp);
    void checkPlayerState(struct timespec* now_tp);

    TFlowCtrlCapture ctrl;

    V4L2Device *cam;
    Flag   cam_state_flag;     // FL_SET -> camera opened; FL_CLR -> camera closed
    struct timespec cam_last_check_tp;

    TFlowPlayer* player;
    Flag   player_state_flag;     // FL_SET -> player active; FL_CLR -> player disabled
    struct timespec player_last_check_tp;

    TFlowAutopilot *autopilot;

};

