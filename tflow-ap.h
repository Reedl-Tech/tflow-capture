#pragma once 
#include <stddef.h>
#include <termios.h>

#include "tflow-glib.hpp"
#include "tflow-ap-fixar.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

class TFlowAutopilot {

public:

    TFlowAutopilot(MainContextPtr context, const char* serial_name, int baud_rate);
    ~TFlowAutopilot();

    int open_dev();
    void close_dev();
    
    std::string serial_name;
    int baud_rate;

    IOSourcePtr serial_sck_src;
    
    void onIdle(struct timespec* now_ts);
    void onConfigUpdate();
    int onSerialData(Glib::IOCondition io_cond);
    int onSerialDataRcv();

    int serialDataSend(TFLOW_AP::out_msg &msg);

    struct TFLOW_AP::sensors last_sensors;
    struct timeval last_sensors_ts;
    struct timeval last_sensors_ts_obsolete;
    int last_sensor_cnt;

private:
    MainContextPtr context;

    struct timespec last_serial_check_tp;

    Flag serial_state_flag;
    int serial_sck_fd = -1;

    enum {
        READER_STATE_SYNCING = 0,
        READER_STATE_SYNCED = 1,
    } state;


    // ***************************************************************

    int set_baudrate();

    TFLOW_AP ap_fixar;
                            
    void onTFlowAPMsg(TFLOW_AP::in_msg &msg);
};

