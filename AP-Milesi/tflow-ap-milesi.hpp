#pragma once
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "../mavlink/common/mavlink.h"
#pragma GCC diagnostic pop

#include "../mongoose.h"
#include "../tflow-buf.hpp"
#include "../tflow-buf-pck.hpp"
#include "../tflow-ap.hpp" 
#include "../tflow-udp.hpp"
#include "../tflow-serial.hpp"
#include "tflow-ap-milesi-cfg.hpp"

#define JSTCK_DEAD_ZONE 10

#define MILESI_PILOTING_EN         (1 << 10)    // TODO: Rename to TARGETING_ASSIST

#define MILESI_TRGT_SEL_EN         (1 << 9)
#define MILESI_TRGT_SEL_UP         (1 << 11)
#define MILESI_TRGT_SEL_DOWN       (1 << 12)
#define MILESI_TRGT_SEL_LEFT       (1 << 13)
#define MILESI_TRGT_SEL_RIGHT      (1 << 14)

class TFlowMilesi : public TFlowSerial, public TFlowAP, public TFlowUDP {
public:

    // Convert input message to Manual control
#pragma pack(push, 1)
    struct mg_tlv_hdr {
        uint32_t magic;
        uint16_t type;
        uint16_t len;
        uint8_t  value[0];
    };

    struct mg_userctrl_tlv {
        mg_tlv_hdr hdr;
        mavlink_manual_control_t mav_man_ctrl;
    };

    union mg_tlv_u {
        struct mg_tlv_hdr hdr;
        struct mg_userctrl_tlv userctrl;
    } in_mg_tlv, tmp_mg_tlv;


    // Filled from incoming ATTITUDE message from autopilot on serial interface
    // Based on ox4 Mavlink messages
    struct imu_milesi_v0 {
        
        uint32_t sign;        // IMU1   0x494D5531
        uint32_t tv_sec;      // Local timestamp
        uint32_t tv_usec;     // Local timestamp

        /* Direct translation from Mavlink ATTITUDE_QUATERNION */
        float qw;
        float qx;
        float qy;
        float qz;
        float rollspeed;    /*< [rad/s] Roll angular speed*/
        float pitchspeed;   /*< [rad/s] Pitch angular speed*/
        float yawspeed;     /*< [rad/s] Yaw angular speed*/

        /* Direct translation from Mavlink ATTITUDE */ 
        float roll;         /*< [rad] Roll angle (-pi..+pi)*/
        float pitch;        /*< [rad] Pitch angle (-pi..+pi)*/
        float yaw;          /*< [rad] Yaw angle (-pi..+pi)*/

        /* Poke from GIMBAL_DEVICE_ATTITUDE_STATUS */
        float gimbal_qw;
        float gimbal_qx;
        float gimbal_qy;
        float gimbal_qz;
        uint16_t gimbal_flags;     /* GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME or GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME */

    } ap_imu;                                                                                                       

    // Filled from TFlow aux GC mav message on UDP interface
    struct jstk_ctrl {
        uint32_t sign;        // JSTK 0x4A53544B
        uint32_t tv_sec;      // Local timestamp
        uint32_t tv_usec;     // Local timestamp

        int x;
        int y;
        int z;
        int r;
    } user_jstk_ctrl;
    
    int user_jstk_ctrl_updated;

#pragma pack(pop)

    int in_mg_tlv_bytes_read;
    int dbg_cnt = 0;

    TFlowMilesi(MainContextPtr context, const TFlowMilesiCfg::cfg_milesi *cfg);

    ~TFlowMilesi();

    const TFlowMilesiCfg::cfg_milesi *cfg;

    int onSerialDataRcv() override;
    void onUDPMsg(const char *udp_msg, int udp_msg_len) override;

    void onMavlinkAP(const mavlink_message_t &msg, const mavlink_status_t &status);  // Mavlink message from Autopilot
    void onMavlinkAP_GimbalAttitude(const mavlink_gimbal_device_attitude_status_t &gimb_att);
    void onMavlinkAP_Attitude(const mavlink_attitude_t &att);
    void onMavlinkAP_AttitudeQ(const mavlink_attitude_quaternion_t &attq);

    void onMavlinkGC(const mavlink_message_t &msg, const mavlink_status_t &status);  // Mavlink message from Ground Control
    int onMavlinkGCManualControl(const mavlink_manual_control_t &man_ctrl);

    /* === Mongoose WebSocket interface related === */
    // TODO: ??? Move to a dedicated class WS User Control ???
    //void onIdleMg(struct timespec now_ts);
    int OpenMg();
    void CloseMg();

    gboolean onMgSrc(Glib::IOCondition);
    int onMgTLVMsg();
    int onMgTLVCheckErr(int res);
    int parseMgMsg(union mg_tlv_u *mg_tlv, const char* in_mg_msg, size_t in_mg_msg_len);

    /* ========== Capture Interface ======== */
    int onCaptureMsgRcv(const TFlowBufPck::pck &in_msg);
    void onIdle(const struct timespec &now_ts);
    int onBuf(TFlowBuf &buf);
    /* ====================================== */

    int onBufTargeting(uint8_t *buf);
    int onBufAPIMU(uint8_t *buf);
    int onBufUserctrl(uint8_t *buf);

private:

    MainContextPtr context;

#pragma pack(push, 1)
    struct targeting_data {
        uint32_t sign;        // TGT1   0x54475431
        uint32_t tv_sec;      // Local timestamp
        uint32_t tv_usec;     // Local timestamp
        float    cursor_x;    // Normalized cursor position
        float    cursor_y;
        uint8_t  flags;       // EN/DIS etc
        uint16_t evt;         // Button press event
        int16_t  evt_id;      // Button press event id
    };
#pragma pack(pop)

    // Copied into the captured buffer
    uint32_t aux_data_size;
    uint32_t aux_data_len;
    uint8_t *aux_data_buf;

    uint8_t ap_raw_buf [ 512 ];
    mavlink_status_t ap_status;
    mavlink_message_t ap_msg;
    int ap_mav_chan = 0;

    mavlink_status_t gc_status;
    mavlink_message_t gc_msg;
    int gc_mav_chan = 0;

    // === control over WS (aka Mongoose) ===
    // TODO: move to a dedicated class?
    Flag             mg_pipe_state_flag;
    int              mg_pipe[2];
    IOSourcePtr      mg_pipe_src;
    //size_t           in_mg_msg_size;
    //char*            in_mg_msg;

    pthread_t        mg_th;
    pthread_cond_t   mg_th_cond;
    struct mg_mgr    mg_manager;

    static void* _mg_thread(void* ctx);
    static void _on_mg_msg(struct mg_connection* c, int ev, void* ev_data);


    // === Targeting 
    // Data generated from MavLink MANUAL CONTROL and added to each frame    
    int targeting_en;
    float cursor_x;
    float cursor_y;

    uint16_t selection_evt; // Choose target from left/right/up/down
    int selection_evt_id;   // Incremented on each new selection_evt

    struct timespec last_manual_control_ts;

    float jstck2pos(int j, float dt_ms);
    int targetingCtrl(int jstck_x, int jstck_y, uint16_t butt, uint16_t prev_butt);
};
