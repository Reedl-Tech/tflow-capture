#pragma once

#include <cassert>
#include <time.h>
#include <linux/videodev2.h> //V4L2 stuff

//#include <giomm.h>
#include <glib-unix.h>

#define TFLOWBUFSRV_SOCKET_NAME "com.reedl.tflowbufsrv"
#include "tflow-capture.h"

//class Flag;

class V4L2Device;

class TFlowBufSrv {
public:
    TFlowBufSrv(GMainContext* context);
    ~TFlowBufSrv();
    int StartListening();
    void onIdle(clock_t now);
    int consume(v4l2_buffer &v4l2_buf);

    char* sck_name;
    int sck_fd = -1;
    Flag sck_state_flag;

    V4L2Device *cam;

    typedef struct
    {
        GSource g_source;
        TFlowBufSrv* srv;
    } GSourceSrv;

    GSourceSrv* sck_src;
    gpointer sck_tag;
    GSourceFuncs sck_gsfuncs;

private:
    GMainContext* context;
    clock_t last_idle_check = 0;

    struct cli_port {
        char* sck_name;
        int sck_fd;
        uint32_t subscriber_mask;
        int consumed_cnt;
        int redeemed_cnt;
        struct timeval last_act;
    };

    static constexpr int PORT_VPROCESS = 0;
    static constexpr int PORT_STREAMER = 1;
    static constexpr int PORT_LAST = 2;

    std::array<struct cli_port, PORT_LAST> cli_ports{};

    static constexpr int TFLOW_BUF_NUM = 4; // AV: Should be the same as V4L2 bufs?
    std::array<TFlowBuf, TFLOW_BUF_NUM> bufs{};
};

