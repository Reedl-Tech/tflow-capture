
#include <features.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>

#include <sys/ioctl.h>

#include <giomm.h>
#include <glib-unix.h>

using namespace std;

//#include "reedl-generic.h"
#include "tflow-capture.h"
#include "tflow-ap-fixar.h"
#include "tflow-ap.h"

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

static speed_t get_baud(int baud)
{
    switch (baud) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    case 4000000:
        return B4000000;
    default:
        return -1;
    }
}

int TFlowAutopilot::onSerialData(Glib::IOCondition io_cond)
{
    int rc = onSerialDataRcv();
    if (rc) {
        // Close / Reopen the port ?
        return G_SOURCE_REMOVE;
    }
    return G_SOURCE_CONTINUE;
}

void TFlowAutopilot::onConfigUpdate()
{
    // Upon falling flag the port will be closed and reopened 
    // with the new configuration.
    serial_state_flag.v = Flag::FALL;
}
void TFlowAutopilot::onIdle(struct timespec* now_ts)
{
    if (serial_state_flag.v == Flag::SET) {
        return;
    }

    if (serial_state_flag.v == Flag::CLR) {
        if (serial_name.empty()) return;

        if (diff_timespec_msec(now_ts, &last_serial_check_tp) > 1000) {
            last_serial_check_tp = *now_ts;
            serial_state_flag.v = Flag::RISE;
        }
    }

    if (serial_state_flag.v == Flag::UNDEF || serial_state_flag.v == Flag::RISE) {
        int rc;

        if (serial_name.empty()) return;

        rc = open_dev();
        if (0 == rc) {
            serial_state_flag.v = Flag::SET;
        } else {
            // Can't open serial port - try again later 
            serial_state_flag.v = Flag::CLR;
        }
        return;
    }

    if (serial_state_flag.v == Flag::FALL) {
        // close the serial port
        close_dev();
        serial_state_flag.v = Flag::CLR;
    }

}

TFlowAutopilot::TFlowAutopilot(MainContextPtr app_context, const char* _serial_name, int _baud_rate)
{
    struct timespec now;
    context = app_context;

    serial_sck_fd = -1;

    serial_name = std::string(_serial_name);
    baud_rate = _baud_rate;

    last_serial_check_tp.tv_sec = 0;
    last_serial_check_tp.tv_nsec = 0;

    clock_gettime(CLOCK_MONOTONIC, &now);

    last_sensors_ts.tv_sec = now.tv_sec;
    last_sensors_ts.tv_usec = now.tv_nsec / 1000;

    last_sensors_ts_obsolete = last_sensors_ts;

    last_sensor_cnt = 0;
}

TFlowAutopilot::~TFlowAutopilot()
{
    close_dev();

    if (serial_sck_src) {
        serial_sck_src->destroy();
        serial_sck_src.reset();
    }

}

void TFlowAutopilot::onTFlowAPMsg(TFLOW_AP::in_msg &msg)
{
    struct timespec tp;
    struct timeval  now_ts;

    clock_gettime(CLOCK_MONOTONIC, &tp);

    now_ts.tv_sec = tp.tv_sec;
    now_ts.tv_usec = tp.tv_nsec / 1000;

    // Keep Position Estimation and Current Axis State messages only
    switch (msg.common.msg_id) {
    case TFLOW_AP_MSG_ID_SENSORS: 
        last_sensors = TFLOW_AP::sensors(msg.sensors);
        last_sensors_ts = now_ts;
        last_sensor_cnt++;
        break;
    
    default:
        return;
    }

}
int TFlowAutopilot::serialDataSend(TFLOW_AP::out_msg &out_msg)
{
    uint8_t *raw_data_out = (uint8_t*)&out_msg;
    
    TFLOW_AP::chcksum_set(out_msg);

    if (serial_sck_fd == -1) {
        // Serial port not ready yet or was intentionally closed.
        // Not an error
        return 0;
    }

    size_t bytes_to_wr = sizeof(TFLOW_AP::hdr) + out_msg.hdr.len + 2;
    ssize_t bytes_written = write(serial_sck_fd, raw_data_out, bytes_to_wr);

    {
        static char out_str[512] = { 0 };
        char *out_str_p = &out_str[0];
        for (int i = 0; i < bytes_to_wr; i++) {
            int n = snprintf(out_str_p, sizeof(out_str) - 1, " %02X", raw_data_out[i]);
            out_str_p += n;
        }
        *out_str_p = 0;
        //g_warning(" POSITIONING: %s", out_str);
    }

    if (bytes_written <= 0) {
        // In case of USB<->UART converter the port might disappear suddenly.
        g_warning("TFlowAutopilot: Can't write to serial (%s): %s", 
            serial_name.c_str(), strerror(errno));
        close_dev();
        serial_state_flag.v = Flag::FALL;
        return -1;
    }

    if (bytes_written != bytes_to_wr) {
        g_warning("TFlowAutopilot: Can't write whole packet to (%s): %s", 
            serial_name.c_str(), strerror(errno));
        serial_state_flag.v = Flag::FALL;
        return -1;
    }

    return 0;
}

int TFlowAutopilot::onSerialDataRcv()
{
    uint8_t *raw_data_in;

    size_t btr = ap_fixar.get_btr_dst(&raw_data_in);

    ssize_t bytes_read = read(serial_sck_fd, raw_data_in, btr);

    if (bytes_read <= 0) {
        int err = errno;

        // Check error codes on USB serial converter conn/disc
        if (err == EAGAIN) {
            g_warning("TFlowAutopilot: ???");
            return 0;
        }
        else {
            g_warning("TFlowAutopilot: unexpected error (%d) - %s",
                errno, strerror(errno));
            serial_state_flag.v = Flag::FALL;
            return -1;
        }
    }

    int res = ap_fixar.data_in(bytes_read);

    if (res) {
        onTFlowAPMsg(ap_fixar.ap_in_msg);
    }

    return 0;
}

int TFlowAutopilot::set_baudrate()
{
    int ret;
    struct termios tio;

    ret = tcgetattr(serial_sck_fd, &tio);
    if (ret) {
        g_warning("ERROR during tcgetattr(): %d", errno);
        return -1;
    }


    speed_t brate = get_baud(baud_rate);
    if (brate == -1) {
        g_warning("ERROR during tcgetattr(): %d", errno);
        return -1;
    }

    tio.c_cflag = (CS8 | CLOCAL | CREAD);
    cfsetispeed(&tio, brate);
    cfsetospeed(&tio, brate);

    // set input mode (non-canonical, no echo,...)
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cc[VTIME] = 0;
    tio.c_cc[VMIN] = 0;

    g_warning("Baudrate: %x-%X", cfgetispeed(&tio), tio.c_cflag);
    ret = tcsetattr(serial_sck_fd, TCSANOW, &tio);
    if (ret) {
        g_warning("Baudrate: ERROR during tcsetattr(): %d", errno);
        return -1;
    }
    return 0;
}

void TFlowAutopilot::close_dev()
{
    if (serial_sck_fd != -1) {
        close(serial_sck_fd);
        serial_sck_fd = -1;
    }
}

int TFlowAutopilot::open_dev()
{
    int rc;
    int foo;

    serial_sck_fd = open(serial_name.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);
    if (serial_sck_fd < 0) {
        g_warning("Unable to open serial port (%s): %s",
            serial_name.c_str(), strerror(errno));
        serial_sck_fd = -1;
        return -1;
    }

    rc = ioctl(serial_sck_fd, FIONBIO, &foo, sizeof(foo));
    if (rc < 0) {
        g_warning("Unable set FIONBIO on serial port (%s): %s",
            serial_name.c_str(), strerror(errno));
        goto on_error;
    }

    if (0 != set_baudrate()) {
        goto on_error;
    }

    serial_sck_src = Glib::IOSource::create(serial_sck_fd, (Glib::IOCondition)(G_IO_IN | G_IO_ERR | G_IO_HUP));
    serial_sck_src->connect(sigc::mem_fun(*this, &TFlowAutopilot::onSerialData));
    serial_sck_src->attach(context);

    return 0;

on_error:
    close_dev();
    return -1;
}
