#pragma once

#include <cassert>
#include <time.h>
#include <giomm.h>

#include "tflow-glib.hpp"
#include "tflow-common.hpp"
#include "tflow-ctrl-capture.hpp"
#include "tflow-buf-srv.hpp"
#include "tflow-ap.h"
#include "tflow-player.h"
#include "tflow-capture-v4l2.hpp"

#define TFLOWBUF_MSG_CUSTOM_NAVIGATOR (TFLOWBUF_MSG_CUSTOM_ + 1)    // 0x80 + 1

#pragma pack(push, 1)
struct pck_navigator {
    TFlowBuf::pck_hdr hdr;
    int32_t    position_x;            // Drone position relative to takeoff point    In meters            
    int32_t    position_y;            // 
    int32_t    north_azimuth;         // Azimuth angle to North n degrees
    uint8_t    position_quality;      // Position confidence coefficient, accuracy    0 - position can’t be used, 255 - position max accuracy

    int32_t    velocity_x;            // Movement direction
    int32_t    velocity_y;            // 
    int32_t    velocity_heading;      // Azimuth angle to North in degrees
    uint8_t    velocity_is_valid;     // 

    uint32_t   sync_time;             // Last position synchronization time, type of synchronization is in sync_mode    In ms
    uint8_t    video_quality;         // Video quality    0 - bad quality … 255 - best quality
    uint8_t    sync_mode;             // TFlowNavigator synchronization mode    0 - GPS assisted, 1 - IMU assisted, 2 - Standalone

#if 0
    TFlowBuf::pck_hdr hdr;
    int32_t	    position_x;	        // Drone position relative to takeoff point	In meters			
    int32_t		position_y;	        // 
    int32_t	    north_azimuth;	    // Azimuth angle to North	In degrees
    uint32_t	sync_time;	        // Last position synchronization time, type of synchronization is in sync_mode	In ms
    uint8_t	    position_quality;	// Position confidence coefficient, accuracy	0 - position can’t be used, 255 - position max accuracy
    uint8_t	    video_quality;	    // Video quality	0 - bad quality … 255 - best quality
    uint8_t	    sync_mode;	        // TFlowNavigator synchronization mode	0 - GPS assisted, 1 - IMU assisted, 2 - Standalone
#endif
};
#pragma pack(pop)

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

        /* GPS raw data */
        uint8_t  gps_flags;
        uint8_t  gps_fix_type;
        uint8_t  gps_numSat;
        uint8_t  gps_is_new;
        uint16_t gps_hdop;
        uint16_t gps_eph;
        uint16_t gps_epv;
        uint16_t gps_groundCourse;
        uint16_t gps_groundSpeed;
        uint32_t gps_lat;
        uint32_t gps_lon;
        uint32_t gps_alt;

    } aux_ap_data; // Temporary local copy
#pragma pack(pop)

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


    TFlowAutopilot *autopilot;
    
    const struct fmt_info* getCamFmt();

};

