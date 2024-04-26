
#include <features.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <assert.h>
#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>

#include <arpa/inet.h>
#include <sys/ioctl.h>

#include <giomm.h>
#include <glib-unix.h>

using namespace std;

//#include "reedl-generic.h"
#include "tflow-capture.h"
#include "tflow-ap-fixar.h"
#include "tflow-ap.h"

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
gboolean tflow_ap_serial_dispatch(GSource* g_source, GSourceFunc callback, gpointer user_data)
{
    TFlowAutopilot::GSourceAP* source = (TFlowAutopilot::GSourceAP*)g_source;
    TFlowAutopilot* ap = source->ap;

    int rc = ap->onSerialData();
    if (rc) {
        // Close the port ?
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
void TFlowAutopilot::onIdle(clock_t now)
{
    clock_t dt = now - last_idle_check;

    /* Do not check to often*/
    if (dt < 3 * CLOCKS_PER_SEC) return;
    last_idle_check = now;

    if (serial_state_flag.v == Flag::SET || serial_state_flag.v == Flag::CLR) {
        return;
    }

    if (serial_state_flag.v == Flag::UNDEF || serial_state_flag.v == Flag::RISE) {
        int rc;

        // Check camera device name
        if (serial_name.empty()) return;

        do {
            rc = open_dev();
            if (rc) break;

            serial_state_flag.v = Flag::SET;
        } while (0);

        if (rc) {
            // Can't open serial port - try again later 
            serial_state_flag.v = Flag::RISE;
        }
        return;
    }

    if (serial_state_flag.v == Flag::FALL) {
        // close the camera
        close_dev();
        serial_state_flag.v = Flag::RISE;
    }

}

TFlowAutopilot::TFlowAutopilot(GMainContext* app_context, const char* _serial_name, int _baud_rate)
{
    context = app_context;

    serial_sck_fd = -1;
    serial_sck_tag = NULL;
    serial_sck_src = NULL;
    CLEAR(serial_sck_gsfuncs);

    serial_name = std::string(_serial_name);
    baud_rate = _baud_rate;

    /* Assign g_source on the socket */
    serial_sck_gsfuncs.dispatch = tflow_ap_serial_dispatch;
    serial_sck_src = (GSourceAP*)g_source_new(&serial_sck_gsfuncs, sizeof(GSourceAP));
    serial_sck_src->ap = this;
    g_source_attach((GSource*)serial_sck_src, context);
}

TFlowAutopilot::~TFlowAutopilot()
{
    close_dev();
}

void TFlowAutopilot::onFixarMsg(AP_FIXAR::ap_fixar_msg &msg)
{
    struct timespec tp;
    struct timeval  now_ts;

    clock_gettime(CLOCK_MONOTONIC, &tp);

    now_ts.tv_sec = tp.tv_sec;
    now_ts.tv_usec = tp.tv_nsec / 1000;

    // Keep Position Estimation and Current Axis State messages only
    switch (msg.common.msg_id) {
    case AP_FIXAR_MSG_ID_PE: 
        last_pe = AP_FIXAR::ap_fixar_pe(msg.pe);
        last_pe_ts = now_ts;
        break;
    case AP_FIXAR_MSG_ID_CAS:
        //AP_FIXAR::ap_fixar_cas_betoh(last_cas, msg.cas);
        //{
        //    static int presc = 0;
        //    if ((presc++ & 0x1f) == 0) {
        //        g_warning("--- Pitch: 0x%08X  ROLL: 0x%08X", 
        //            *(uint32_t*)&msg.cas.CAS_board_att_pitch,
        //            *(uint32_t*)&msg.cas.CAS_board_att_roll);
        //    }
        //}
        last_cas = AP_FIXAR::ap_fixar_cas(msg.cas);
        last_cas_ts = now_ts;
        break;
    default:
        return;
    }

}
int TFlowAutopilot::onSerialData()
{
    uint8_t *raw_data_in;

    size_t btr = ap_fixar.get_btr_dst(&raw_data_in);

    ssize_t bytes_read = read(serial_sck_fd, raw_data_in, btr);

    if (bytes_read <= 0) {
        int err = errno;

        // Check error codes on USB serial converter conn/disc
        if (err == EAGAIN) {
            g_warning("TFlowAP: ???");
            return 0;
        }
        else {
            g_warning("TFlowAP: unexpected error (%d) - %s",
                errno, strerror(errno));
            serial_state_flag.v = Flag::FALL;
            return -1;
        }
    }

    int res = ap_fixar.data_in(bytes_read);

    if (res) {
        onFixarMsg(ap_fixar.in_bb_msg);
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

int TFlowAutopilot::write_dev(const void *src, size_t len)
{
    int rc;
    
    if (serial_sck_fd == -1) return 0;

    rc = write(serial_sck_fd, src, len);
    if (rc == -1) {
        g_warning("Can't write to serial (%s): %s", 
            serial_name.c_str(), strerror(errno));
        close_dev();
        return -1;
    }
    if (rc != len) {
        g_warning("Can't write whole packet to (%s): %s", 
            serial_name.c_str(), strerror(errno));
        // AV: What to do next? Flash tx buffer?
        // ...
        return -1;
    }

    return 0;
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

    serial_sck_tag = g_source_add_unix_fd((GSource*)serial_sck_src, serial_sck_fd,
        (GIOCondition)(G_IO_IN | G_IO_ERR | G_IO_HUP));

    return 0;

on_error:
    close_dev();
    return -1;
}
