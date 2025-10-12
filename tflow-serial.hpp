#pragma once 
#include <cstddef>
#include <termios.h>

#include "tflow-glib.hpp"
#include "tflow-common.hpp"

class TFlowSerial {

public:

    TFlowSerial(MainContextPtr context, const char* serial_name, int baud_rate);
    ~TFlowSerial();

    int open_dev();
    void close_dev();
    
    std::string serial_name;
    int baud_rate;

    IOSourcePtr serial_sck_src;
    
    void onIdle(const struct timespec &now_ts);
    void onConfigUpdate();
    int onSerialData(Glib::IOCondition io_cond);
    int serialDataSend(uint8_t *buf, size_t buf_len);
    ssize_t serialRead(uint8_t *raw_data_in, size_t btr);

    virtual int onSerialDataRcv() = 0;

    int serial_name_is_valid();
    const char* serial_name_get() { return serial_name_is_valid() ? serial_name.c_str() : nullptr; }
    int serial_baud_get()   { return baud_rate; }

    int serial_sck_fd = -1;

private:

    struct timespec last_serial_check_tp;

    MainContextPtr context;

    Flag serial_state_flag;

    int set_baudrate();

    static speed_t get_baud(int baud);
    static void raw_hex_dump(const uint8_t * buf, size_t buf_len);
};

