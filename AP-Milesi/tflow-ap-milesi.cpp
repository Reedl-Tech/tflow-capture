#include <cassert>
#include <string>

#include <json11.hpp>

#include "../tflow-build-cfg.hpp"

#include "../tflow-buf-pck.hpp"

#include "tflow-ap-milesi-cfg.hpp"
#include "tflow-ap-milesi.hpp"

using namespace json11;

#define TFLOWBUF_MSG_CUSTOM_MAV_MANUAL_CONTROL (TFLOWBUF_MSG_CUSTOM_ + 1)    // 0x80 + 1

TFlowMilesiCfg tflow_milesi_cfg;

static struct timespec diff_timespec(
    const struct timespec* time1,
    const struct timespec* time0)
{
    assert(time1);
    assert(time0);
    struct timespec diff = { .tv_sec = time1->tv_sec - time0->tv_sec, //
        .tv_nsec = time1->tv_nsec - time0->tv_nsec };
    if (diff.tv_nsec < 0) {
        diff.tv_nsec += 1000000000; // nsec/sec
        diff.tv_sec--;
    }
    return diff;
}

static double diff_timespec_msec(
    const struct timespec* time1,
    const struct timespec* time0)
{
    struct timespec d_tp = diff_timespec(time1, time0);
    return d_tp.tv_sec * 1000 + (double)d_tp.tv_nsec / (1000 * 1000);
}

TFlowMilesi::~TFlowMilesi()
{
    if (aux_data_buf) {
        free(aux_data_buf);
        aux_data_size = 0;
    }

    CloseMg();
}

TFlowMilesi::TFlowMilesi(MainContextPtr _context, const TFlowMilesiCfg::cfg_milesi *_cfg) :
    TFlowSerial(_context, _cfg->serial_name.v.c_str, _cfg->serial_baud.v.num),
    TFlowUDP(_context, _cfg->udp_remote_addr.v.c_str, _cfg->udp_local_addr.v.c_str)
{
    context = _context;
    cfg = _cfg;

    targeting_en = 0;
    cursor_x = 0.5f;
    cursor_y = 0.5f;
    selection_evt = 0;     
    selection_evt_id = 0;

    aux_data_len = 0;
    aux_data_size = 1024*1024;
    aux_data_buf = (uint8_t*)calloc(1, aux_data_size);

    ap_imu.sign = 0x494D5531;           // IMU1
    user_jstk_ctrl.sign = 0x4A53544B;   // JSTK

    OpenMg();
}

int TFlowMilesi::onCaptureMsgRcv(const TFlowBufPck::pck &in_msg)
{
    return 0;
}

void TFlowMilesi::onIdle(const struct timespec &now_ts)
{
    TFlowSerial::onIdle(now_ts);

    TFlowUDP::onIdle(now_ts);
}

int TFlowMilesi::onBufTargeting(uint8_t* buf)
{
    struct targeting_data *aux_targeting_data = (struct targeting_data *)buf;

    aux_targeting_data->sign = 0x54475431;        // TGT1   0x54475431

    // Fill Targeting message what will be send to tracker
    aux_targeting_data->tv_sec   = last_manual_control_ts.tv_sec;
    aux_targeting_data->tv_usec  = last_manual_control_ts.tv_nsec / 1000;

    aux_targeting_data->cursor_x = cursor_x;
    aux_targeting_data->cursor_y = cursor_y;

    aux_targeting_data->flags = 0;
    aux_targeting_data->flags |= targeting_en ? (1 << 0) : 0; 

    aux_targeting_data->evt      = selection_evt;
    aux_targeting_data->evt_id   = selection_evt_id;

    return sizeof(struct targeting_data);
}


int TFlowMilesi::onBufUserctrl(uint8_t* buf)
{
    memcpy(buf, &user_jstk_ctrl, sizeof(jstk_ctrl));
    return sizeof(jstk_ctrl);
}

int TFlowMilesi::onBufAPIMU(uint8_t* buf)
{
    memcpy(buf, &ap_imu, sizeof(imu_milesi_v0));
    return sizeof(imu_milesi_v0);
}

int TFlowMilesi::onBuf(TFlowBuf &buf)
{
#if CODE_BROWSE
    // caled from 
    TFlowBufSrv::buf_consume();
#endif

    aux_data_len = 0;
    aux_data_len += onBufTargeting(&aux_data_buf[aux_data_len]);
    aux_data_len += onBufAPIMU    (&aux_data_buf[aux_data_len]);
    aux_data_len += onBufUserctrl (&aux_data_buf[aux_data_len]);

    // The data will be copied into a frame packet
    buf.aux_data_len = aux_data_len;
    buf.aux_data = aux_data_buf;

    return 0;
}

#if 0
Quternion to rotation matrix. W must be normalized to 1
Rotating point in a frame
[
    [1 - 2q₂² - 2q₃², 2q₁q₂ - 2q₀q₃, 2q₁q₃ + 2q₀q₂],
    [2q₁q₂ + 2q₀q₃, 1 - 2q₁² - 2q₃², 2q₂q₃ - 2q₀q₁],
    [2q₁q₃ - 2q₀q₂, 2q₂q₃ + 2q₀q₁, 1 - 2q₁² - 2q₂²]
]
#endif
inline float quaternion_yaw(float q_w, float q_x, float q_y, float q_z)
{
    return (float)atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y*q_y + q_z*q_y));
}

inline float quaternion_pitch(float q_w, float q_x, float q_y, float q_z)
{
    return (float)asin(2 * (q_w * q_y - q_z * q_x));
}

inline float quaternion_roll(float q_w, float q_x, float q_y, float q_z)
{
    return (float)atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x * q_x + q_y*q_y));
}

void TFlowMilesi::onMavlinkAP_Attitude(const mavlink_attitude_t& att)
{
    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);
    ap_imu.tv_sec = now_ts.tv_sec; 
    ap_imu.tv_usec = now_ts.tv_nsec / 1000;

    ap_imu.roll  = att.roll;
    ap_imu.pitch = att.pitch;
    ap_imu.yaw   = att.yaw;
    g_info("ATT_E: roll=%5.1f pitch=%5.1f yaw=%5.1f", 
        RAD2DEG(att.roll), RAD2DEG(att.pitch), RAD2DEG(att.yaw));
}

void TFlowMilesi::onMavlinkAP_AttitudeQ(const mavlink_attitude_quaternion_t &attq)
{
    // TODO: define which one is easier to use Euler angles or Quaternions
    //struct timespec now_ts;
    //clock_gettime(CLOCK_MONOTONIC, &now_ts);
    //ap_imu.tv_sec = now_ts.tv_sec; 
    //ap_imu.tv_usec = now_ts.tv_nsec / 1000;

    ap_imu.qw = attq.q1;
    ap_imu.qx = attq.q2;
    ap_imu.qy = attq.q3;
    ap_imu.qz = attq.q4;
    ap_imu.rollspeed  = attq.rollspeed;
    ap_imu.pitchspeed = attq.pitchspeed;
    ap_imu.yawspeed   = attq.yawspeed;

    float q_roll, q_pitch, q_yaw;

    q_roll  = quaternion_roll (attq.q1, attq.q2, attq.q3, attq.q4);
    q_pitch = quaternion_pitch(attq.q1, attq.q2, attq.q3, attq.q4);
    q_yaw   = quaternion_yaw  (attq.q1, attq.q2, attq.q3, attq.q4);

    {
        static int presc = 0;
        g_info("ATT_Q: roll=%5.1f pitch=%5.1f yaw=%5.1f",
            RAD2DEG(q_roll), RAD2DEG(q_pitch), RAD2DEG(q_yaw));
    }

    
}
void TFlowMilesi::onMavlinkAP(const mavlink_message_t &msg, const mavlink_status_t &status)
{
    // TODO: 1) rework to zero-copy packets
    //       2) ??? Multiple Mavlink packets per transfer
    //       3) split high priority and othe packets to reduce ATTITUDE response latency.
    //       4) avoid ATTITUDE and other STATUS (battery, RSSI, etc.) packets duplication
    //       5) Collect attitude

    uint8_t buf[300];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    int rc = UDPDataSend(buf, len);
    
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
    switch (msg.msgid){
        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
            onMavlinkAP_AttitudeQ(*(mavlink_attitude_quaternion_t*)&msg.payload64); break;
        case MAVLINK_MSG_ID_ATTITUDE:
            onMavlinkAP_Attitude(*(mavlink_attitude_t*)&msg.payload64); break;
        default:
            break;
    }
#pragma GCC diagnostic pop

    if (rc == 0) {
        static struct timespec now_ts;
        static struct timespec last_dbg_ts;
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        if (diff_timespec_msec(&now_ts, &last_dbg_ts) > 300) {
            const mavlink_message_info_t *msg_info = mavlink_get_message_info(&msg);
            g_info("UAV -> GCS [%-40s] %d seq=%d src=%d:%d",
                msg_info->name, msg.msgid, msg.seq, msg.sysid, msg.compid);
            last_dbg_ts = now_ts;
        }
    }


}

float TFlowMilesi::jstck2pos(int j, float dt_sec)
{
    // Converts joystick position to cursor increment
    // 1000 - max value
    // 3 sec per full screen (1.0) -> 0.33 
    // TODO: Make speed exponential?
    static float speed_pix_per_sec = 0.33;
    return ((float)j / 1000) * speed_pix_per_sec * dt_sec;
}

int TFlowMilesi::targetingCtrl(int jstck_x, int jstck_y, uint16_t butt, uint16_t prev_butt)
{
    // Process MANUAL CONTROL packet
    // Convert Joystick position to cursor as well as keep targeting ON/OFF 
    // state + button debouncing?
    // TODO:
    //    Should it be part of TFlowProcess? 
    //    Why capture should care about targeting control logic?
    //    Is processing rate that important (UDP packets vs Frame Capturing)?

    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);

    float dt_sec = (float)diff_timespec_msec(&now_ts, &last_manual_control_ts) / 1000;
    last_manual_control_ts = now_ts;

    if (0 == (butt & MILESI_TRGT_SEL_EN)) {
        // Targeting disabled
        targeting_en = 0;
        return 0;
    }

    // Process jstk_movement
    targeting_en = 1;
    if (abs(jstck_x) > JSTCK_DEAD_ZONE) {      // Is it handled by QGC?
        cursor_x += jstck2pos(jstck_x, dt_sec);
        cursor_x = MAX(0, cursor_x);
        cursor_x = MIN(1, cursor_x);
    }
    if (abs(jstck_y) > JSTCK_DEAD_ZONE) {      // Is it handled by QGC?
        cursor_y += jstck2pos(jstck_y, dt_sec);
        cursor_y = MAX(0, cursor_y);
        cursor_y = MIN(1, cursor_y);
    }

    uint16_t trgt_sel_butt = 
        MILESI_TRGT_SEL_UP    |
        MILESI_TRGT_SEL_DOWN  |
        MILESI_TRGT_SEL_LEFT  |
        MILESI_TRGT_SEL_RIGHT;

    uint16_t butt_diff = (butt ^ prev_butt);
    if (butt_diff & trgt_sel_butt) {                // Button changed
        if (0 == (prev_butt & trgt_sel_butt)) {     // Button pressed
            selection_evt = butt & trgt_sel_butt;
            selection_evt_id++;
        }
    }

    {
        static int presc = 0;
        if ((presc++ & 0x0F) == 0) {
            g_debug("TRGT: pos=%5.3f,%5.3f evt=%c (%d) dt=%5.1f",
                cursor_x, cursor_y,
                selection_evt & MILESI_TRGT_SEL_UP    ? 'U' :        // Up
                selection_evt & MILESI_TRGT_SEL_DOWN  ? 'D' :        // Down
                selection_evt & MILESI_TRGT_SEL_LEFT  ? 'L' :        // Left
                selection_evt & MILESI_TRGT_SEL_RIGHT ? 'R' : '-',   // Right
                selection_evt_id,
                dt_sec * 1000);
        }
    }

    return 0;
}

int TFlowMilesi::onMavlinkGCManualControl(const mavlink_manual_control_t &man_ctrl)
{
    static uint16_t prev_buttons = 0;
    if ((man_ctrl.buttons | prev_buttons) & MILESI_TRGT_SEL_EN) {

#define F310 1
#if F310
        int jstck_x = man_ctrl.y;
        int jstck_y = -man_ctrl.x;
#endif
        targetingCtrl(jstck_x, jstck_y, man_ctrl.buttons, prev_buttons);
        prev_buttons = man_ctrl.buttons;
#if 0
        {
            static int presc = 0;
            if ((++presc & 0x1F) == 0) {
                g_info("GCS -> UAV [MANUAL_CONTROL] x=%5d y=%5d butt=[%c%c%c%c%c], 0x%04X",
                    man_ctrl.x, man_ctrl.y,
                    man_ctrl.buttons & MILESI_TRGT_SEL_UP    ? 'U' : ' ',        // Up
                    man_ctrl.buttons & MILESI_TRGT_SEL_DOWN  ? 'D' : ' ',        // Down
                    man_ctrl.buttons & MILESI_TRGT_SEL_LEFT  ? 'L' : ' ',        // Left
                    man_ctrl.buttons & MILESI_TRGT_SEL_RIGHT ? 'R' : ' ',        // Right
                    man_ctrl.buttons & MILESI_TRGT_SEL_EN    ? '+' : ' ',
                    man_ctrl.buttons);
            }
        }
#endif
        return 0;   // Consumed
    } 
    if ((man_ctrl.buttons | prev_buttons) & MILESI_PILOTING_EN) {
        // Joysticks controls the flight 
        // Upate the data to be send to TFlow Process for rendering
        struct timespec now_ts;
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        user_jstk_ctrl.tv_sec = now_ts.tv_sec;
        user_jstk_ctrl.tv_usec = now_ts.tv_nsec / 1000;
        user_jstk_ctrl.x = man_ctrl.y;
        user_jstk_ctrl.y = -man_ctrl.x;
        user_jstk_ctrl.z = man_ctrl.z;
        user_jstk_ctrl.r = man_ctrl.r;
        
        // return 1;   // Send to AP
        return 0;   // Temporary. Remove it as soon as the AP is ready to accept this packet
    }
    else {
        g_info("GCS -> UAV [MANUAL_CONTROL] x=%5d y=%5d z=%5d r=%5d s=%5d t=%5d butt=0x%04X",
            man_ctrl.x, man_ctrl.y, man_ctrl.z, man_ctrl.r,  man_ctrl.s, man_ctrl.t, man_ctrl.buttons);
    }
    return 0;   // Consumed

}                                               

void TFlowMilesi::onMavlinkGC(const mavlink_message_t &msg, const mavlink_status_t &status)
{
    // TODO: 1) rework to zero-copy packets
    //       2) ??? Multiple Mavlink packets per transfer
    //       3) split high priority and othe packets to reduce manual control latency.
    //          ??? Is it important @1M baud?
    //       4) avoid manual control packets duplication

    uint8_t buf[300];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    int forward_to_ap = 1;
    
#define TFLOW_AUXGC_MAV_SYSID 2

    // Check is message received from TFlow auxilary ground control.
    if (msg.msgid == MAVLINK_MSG_ID_MANUAL_CONTROL && msg.sysid == TFLOW_AUXGC_MAV_SYSID) {
        forward_to_ap = onMavlinkGCManualControl(*(mavlink_manual_control_t*)&msg.payload64);

        // Set msg.sysid and msg.compid to ??? .
        //msg.sysid = ???;
        //msg.compid = ???;
    }

    if (forward_to_ap) {
        int rc = TFlowSerial::serialDataSend(buf, len);

        if (rc == 0) {
            static struct timespec now_ts;
            static struct timespec last_dbg_ts;
            clock_gettime(CLOCK_MONOTONIC, &now_ts);
            if (diff_timespec_msec(&now_ts, &last_dbg_ts) > 1000) {
                const mavlink_message_info_t *msg_info = mavlink_get_message_info(&msg);
                g_info("GCS -> UAV [%-40s] %d seq=%d src=%d:%d",
                    msg_info->name, msg.msgid, msg.seq, msg.sysid, msg.compid);
                last_dbg_ts = now_ts;
            }

#if 0
            const mavlink_message_info_t *msg_info = mavlink_get_message_info(&msg);
            if (msg_info->msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                mavlink_command_long_t *cmd_long = (mavlink_command_long_t*)&msg.payload64;
                g_info("GCS -> UAV [%-40s] %d seq=%d src=%d:%d CMD=%d",
                    msg_info->name, msg.msgid, msg.seq, msg.sysid, msg.compid,
                    cmd_long->command);
                /*
                * param1  = 0-1
                * param2 =  1
                * param5 = 1421
                * command =310
                * sys = 1
                * comp= 1
                * conf = 0
                */
            }
            else {
                g_info("GCS -> UAV [%-40s] %d seq=%d src=%d:%d",
                    msg_info->name, msg.msgid, msg.seq, msg.sysid, msg.compid);
            }
#endif
        }
    }
   
}

void TFlowMilesi::onUDPMsg(const char *udp_msg, int udp_msg_len)
{
    for ( int i = 0; i < udp_msg_len; i++ ) {
        if ( mavlink_parse_char(gc_mav_chan, udp_msg[i], &gc_msg, &gc_status) ) {
            onMavlinkGC(this->gc_msg, this->gc_status);
        }
    }

}

int TFlowMilesi::onSerialDataRcv()
{
    ssize_t bytes_read = read(serial_sck_fd, ap_raw_buf, sizeof(ap_raw_buf));

    for ( int i = 0; i < bytes_read; i++ ) {
        if ( mavlink_parse_char(ap_mav_chan, ap_raw_buf [ i ], &this->ap_msg, &this->ap_status) ) {
            onMavlinkAP(this->ap_msg, this->ap_status);
        }
    }
    
    return 0;
}

void TFlowMilesi::CloseMg()
{
    // TODO: Close the thread
    // ...

    if (mg_pipe[0] != -1) {
        close(mg_pipe[0]);
        mg_pipe[0] = -1;
    }

    if (mg_pipe[1] != -1) {
        close(mg_pipe[1]);
        mg_pipe[1] = -1;
    }

    if (mg_pipe_src) {
        mg_pipe_src->destroy();
        mg_pipe_src.reset();
    }

    return;
}

struct user {
  const char *name, *pass, *access_token;
};

static struct user *authenticate(struct mg_http_message *hm) {
  // In production, make passwords strong and tokens randomly generated
  // In this example, user list is kept in RAM. In production, it can
  // be backed by file, database, or some other method.
  static struct user users[] = {
      {"admin", "admin", "admin_token"},
      {"user1", "user1", "user1_token"},
      {"user2", "user2", "user2_token"},
      {NULL, NULL, NULL},
  };
  char user[64], pass[64];
  struct user *u, *result = NULL;
  mg_http_creds(hm, user, sizeof(user), pass, sizeof(pass));
  MG_VERBOSE(("user [%s] pass [%s]", user, pass));

  if (user[0] != '\0' && pass[0] != '\0') {
    // Both user and password is set, search by user/password
    for (u = users; result == NULL && u->name != NULL; u++)
      if (strcmp(user, u->name) == 0 && strcmp(pass, u->pass) == 0) result = u;
  } else if (user[0] == '\0') {
    // Only password is set, search by token
    for (u = users; result == NULL && u->name != NULL; u++)
      if (strcmp(pass, u->access_token) == 0) result = u;
  }
  return result;
}


void TFlowMilesi::_on_mg_msg(struct mg_connection* c, int ev, void* ev_data)
{
    TFlowMilesi *milesi = (TFlowMilesi *)c->fn_data;

    if (ev == MG_EV_OPEN && c->is_listening) {
        // Connection created
    }
    else if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message* hm = (struct mg_http_message*)ev_data;
        struct user *u = authenticate(hm);  // TODO: try to remove

        if (mg_http_match_uri(hm, "/websocket")) {
            mg_ws_upgrade(c, hm, NULL);  // Upgrade HTTP to Websocket
            c->data[0] = 'W';            // Set some unique mark on a connection
        }
    }
    if (ev == MG_EV_ACCEPT) {
        size_t cert_len = 0;
        size_t key_len = 0;
        struct mg_tls_opts opts = {
            .cert = mg_file_read(&mg_fs_posix, "/home/root/cert/server.crt", &cert_len),
            .key  = mg_file_read(&mg_fs_posix, "/home/root/cert/server.key", &key_len),
            // .ca   = mg_file_read(&mg_fs_posix, "/home/root/cert/ca.crt", NULL)
        };
        opts.cert.len = cert_len;
        opts.key.len = key_len;
        mg_tls_init(c, &opts);
    }
    else if (ev == MG_EV_WS_OPEN) {
        c->data[0] = 'W';  // Mark this connection as an established WS client
    }
    else if (ev == MG_EV_WS_MSG) {
        // Got websocket frame. Received data is wm->data
        struct mg_ws_message* wm = (struct mg_ws_message*)ev_data;
        
        int rc = milesi->parseMgMsg(&milesi->tmp_mg_tlv, 
            wm->data.ptr, wm->data.len);

        if (0 == rc) {
            int res = write(milesi->mg_pipe[1], &milesi->tmp_mg_tlv, 
                sizeof(milesi->tmp_mg_tlv.hdr) + milesi->tmp_mg_tlv.hdr.len);
        }
        else {
            // Can't parse
        }

        mg_iobuf_del(&c->recv, 0, c->recv.len);
    }
    else if (ev == MG_EV_WAKEUP) {
                  ;
    }

}

void* TFlowMilesi::_mg_thread(void* ctx)
{
    TFlowMilesi* m = (TFlowMilesi*)ctx;

    /* Mongoose main thread */
    mg_mgr_init(&m->mg_manager);        // Initialise event manager
    mg_log_set(MG_LL_INFO);             // Set log level
    mg_http_listen(&m->mg_manager, "http://0.0.0.0:8021", m->_on_mg_msg, (void*)m);
    mg_wakeup_init(&m->mg_manager);     // Initialise wakeup socket pair
    for (;;) {                          // Event loop
        mg_mgr_poll(&m->mg_manager, 1000);
    }
    mg_mgr_free(&m->mg_manager);

    return nullptr;
}

int TFlowMilesi::OpenMg()
{
    int rc;
   
    rc = pipe2(mg_pipe, O_NONBLOCK);
    if (rc) {
        g_warning("TFlow error in Mongoose initiation (mg_pipe)");
        return -1;
    }

    mg_pipe_src = Glib::IOSource::create(mg_pipe[0], (Glib::IOCondition)(G_IO_IN | G_IO_ERR | G_IO_HUP));
    mg_pipe_src->connect(sigc::mem_fun(*this, &TFlowMilesi::onMgSrc));
    mg_pipe_src->attach(context);

     /* Create mongoose thread */
    pthread_attr_t attr;

    pthread_cond_init(&mg_th_cond, nullptr);
    pthread_attr_init(&attr);

    pthread_create(&mg_th, &attr, _mg_thread, this);
    pthread_attr_destroy(&attr);

    in_mg_tlv_bytes_read = 0;
    dbg_cnt = 0;

    return 0;
}
gboolean TFlowMilesi::onMgSrc(Glib::IOCondition io_cond)
{
    if (io_cond == Glib::IOCondition::IO_ERR) {
        assert(0);  // Implement something or remove condition from the source
    }

    if (io_cond == Glib::IOCondition::IO_HUP) {
        assert(0);  // Implement something or remove condition from the source
    }

    int rc = onMgTLVMsg();
    if (rc) {
        mg_pipe_state_flag.v = Flag::FALL;
        return G_SOURCE_REMOVE;
    }
    return G_SOURCE_CONTINUE;
}

int TFlowMilesi::parseMgMsg(union mg_tlv_u *mg_tlv,
    const char* in_mg_msg, size_t in_mg_msg_len)
{
    std::string j_err;

    const Json j_in_msg = Json::parse({in_mg_msg, in_mg_msg_len}, j_err);

    if (j_in_msg.is_null()) {
        g_warning("TFlowMilesi: Bad Userctrl packet - %s", j_err.c_str());
        return -1;
    }

    const json11::Json j_userctl_type = j_in_msg["type"];

    if (!j_userctl_type.is_string()) {
        // Bad format
        return -1;
    }

    mg_tlv->hdr.magic = 0xFEEDBEAD;

    if (0 == strcmp("controller", j_userctl_type.string_value().c_str())) {
/*
        {
          "type": "controller",
          "data": {
            "timestamp": 120974.5,
            "mapping": "standard",
            "buttons": [<array(17)>],
            "axes": [<array>(4)]
          }
        }
*/
        mg_userctrl_tlv* tlv_userctrl = &mg_tlv->userctrl;
        mavlink_manual_control_t* mav_man_ctrl = &tlv_userctrl->mav_man_ctrl;

        mg_tlv->hdr.type = 0x0100 | MAVLINK_MSG_ID_MANUAL_CONTROL;
        mg_tlv->hdr.len = sizeof(mavlink_manual_control_t);

        const json11::Json controller_data = j_in_msg["data"];
        if (!controller_data.is_object()) {
            return -1;
        }

        // Ignore mapping for a while
        // ...

        const json11::Json controller_buttons = controller_data["buttons"];
        if (!controller_buttons.is_array()) {
            return -1;
        }

        const json11::Json controller_axes = controller_data["axes"];
        if (controller_axes.is_array() == false || 
            controller_axes.array_items().size() < 4 ||
            controller_axes.array_items().at(0).is_number() == false) {
            // Not an array of 4+ integers
            return -1;
        }

        // Message is OK fill mavlink packet
        // Convert buttons to UINT32 bit mask
        memset(mav_man_ctrl, 0, sizeof(*mav_man_ctrl));

        uint32_t mav_buttons = 0;
        // int i = 0;
        //for (const json11::Json b : controller_buttons.array_items()) {
        //    if (!b.is_number()) return -1;
        //    
        //    if (b.int_value()) {
        //        mav_buttons |= (!!b.int_value() << i);
        //    }
        //    i++;
        //}

        // Manual mapping i to n
        int i, n;
        int v;
        
        int x = controller_buttons.array_items().at(4).int_value();
        
        i =  4; n = MILESI_TRGT_SEL_EN;    v = !!controller_buttons.array_items().at(i).int_value(); mav_buttons |= v ? n : 0;
        i = 12; n = MILESI_TRGT_SEL_UP;    v = !!controller_buttons.array_items().at(i).int_value(); mav_buttons |= v ? n : 0;
        i = 13; n = MILESI_TRGT_SEL_DOWN;  v = !!controller_buttons.array_items().at(i).int_value(); mav_buttons |= v ? n : 0;
        i = 14; n = MILESI_TRGT_SEL_LEFT;  v = !!controller_buttons.array_items().at(i).int_value(); mav_buttons |= v ? n : 0;
        i = 15; n = MILESI_TRGT_SEL_RIGHT; v = !!controller_buttons.array_items().at(i).int_value(); mav_buttons |= v ? n : 0;

        mav_man_ctrl->buttons = mav_buttons;

        mav_man_ctrl->y =  lround(1000 * controller_axes.array_items().at(0).number_value());
        mav_man_ctrl->x = -lround(1000 * controller_axes.array_items().at(1).number_value());
        mav_man_ctrl->z =  lround(1000 * controller_axes.array_items().at(2).number_value());
        mav_man_ctrl->r =  lround(1000 * controller_axes.array_items().at(3).number_value());

        mav_man_ctrl->s = lround(1000 * controller_buttons.array_items().at(6).number_value());
        mav_man_ctrl->t = lround(1000 * controller_buttons.array_items().at(7).number_value());

        // TODO: ?make it configurable from config?
        mav_man_ctrl->target = 1; /*<  The system to be controlled .*/

        // TODO: Get [s, t] from Analog Buttons ?
        //mav_man_ctrl.s = controller_buttons.array_items().at(???).int_value();
        //mav_man_ctrl.t = controller_buttons.array_items().at(???).int_value();
        return 0;
    }

    return -1;
}


int TFlowMilesi::onMgTLVCheckErr(int res)
{
   if (res <= 0) {
        int err = errno;
        if (err == EAGAIN) {
            return 0;
        }
        if (err == ECONNRESET) {
            g_warning("TFlowMilesi: disconnected (%d, %s) - closing", errno, 
                strerror(errno));
        }
        else {
            g_warning("TFlowMilesi: unexpected error (%d, %s) - closing", errno,
                strerror(errno));
        }
        return -1;
   }
   return 0;
}

int TFlowMilesi::onMgTLVMsg()
{
    int is_tlv_received = 0;

    // New message. Get TLV header
    int bytes_to_read = sizeof(in_mg_tlv.hdr);
    int bytes_read = read(mg_pipe[0], &in_mg_tlv, bytes_to_read);
    if (onMgTLVCheckErr(bytes_read)) return -1;

    if (bytes_read != bytes_to_read) return -1;

    {
        static int presc = 0;
        if ((presc++ & 0xFF) == 0) {
            g_info("====================== CNT %d", dbg_cnt);
        }
    }

    do {
        // Header received. Get TLV value
        bytes_to_read = in_mg_tlv.hdr.len;
        bytes_read = read(mg_pipe[0], &in_mg_tlv.hdr.value, bytes_to_read);
        if (onMgTLVCheckErr(bytes_read)) return -1;

        if (bytes_read != bytes_to_read ) return -1;
        is_tlv_received = 1;

        // Read next message if any
        bytes_to_read = sizeof(in_mg_tlv.hdr);
        bytes_read = read(mg_pipe[0], &in_mg_tlv, sizeof(in_mg_tlv.hdr));
        if (onMgTLVCheckErr(bytes_read)) return -1;

        if (bytes_read == -1) {
            break;    // -1 && EAGAIN - all data read
        }

        // Yet another message header received - continue with body read
        // 
        dbg_cnt++;
        is_tlv_received = 0;
        
    } while(1);

    if (is_tlv_received) {
        // TLV message received
        assert(in_mg_tlv.hdr.magic == 0xFEEDBEAD);
        if (in_mg_tlv.hdr.type & (uint16_t)0x0100) {
            // Pure mavlink
            uint8_t mav_msg_id = (in_mg_tlv.hdr.type & 0xFF);
            switch (mav_msg_id) {
            case MAVLINK_MSG_ID_MANUAL_CONTROL:
                onMavlinkGCManualControl(in_mg_tlv.userctrl.mav_man_ctrl);
                break;
            default:
                break;
            }
        }
    }

    // Prepare for next message
    in_mg_tlv_bytes_read = 0;

    return 0;
    
}
