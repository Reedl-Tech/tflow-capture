#pragma once 
#include <stddef.h>
#include <termios.h>

#include "tflow-ap-fixar.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

class TFlowAutopilot {

public:

    TFlowAutopilot(GMainContext* app_context, const char* serial_name, int baud_rate);
    ~TFlowAutopilot();

    int open_dev();
    void close_dev();
    
    typedef struct {
        GSource g_source;
        TFlowAutopilot* ap;
    } GSourceAP;

    GMainContext* context;
    std::string serial_name;
    int baud_rate;

    GSourceAP* serial_sck_src;
    gpointer serial_sck_tag;
    GSourceFuncs serial_sck_gsfuncs;
    
    void onIdle(struct timespec* now_ts);
    void onConfigUpdate();
    int onSerialData();

    struct AP_FIXAR::ap_fixar_pe last_pe;
    struct timeval last_pe_ts;

    struct AP_FIXAR::ap_fixar_status last_status;   // Sensors healf, flight mode, etc.
    struct timeval last_status_ts;

    struct AP_FIXAR::ap_fixar_sensors last_sensors; // Sensors values (IMU, VN100, rangefinder, etc)
    struct timeval last_sensors_ts;

    struct AP_FIXAR::ap_fixar_cas last_cas;        // Current Axis State
    struct timeval last_cas_ts;
    struct timeval last_cas_ts_obsolete;

private:

    struct timespec last_serial_check_tp;

    Flag serial_state_flag;
    int serial_sck_fd = -1;

    enum {
        READER_STATE_SYNCING = 0,
        READER_STATE_SYNCED = 1,
    } state;


    // ***************************************************************
    int set_baudrate();
    int write_dev(const void *src, size_t len);

    AP_FIXAR ap_fixar;

    void onFixarMsg(AP_FIXAR::ap_fixar_msg& msg);
};

