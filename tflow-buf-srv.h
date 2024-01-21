#pragma once

#include <cassert>
#include <time.h>
#include <linux/videodev2.h> //V4L2 stuff

#include <glib-unix.h>

#include "tflow-buf.h"

#define TFLOWBUFSRV_SOCKET_NAME "com.reedl.tflow.buf-server"

class TFlowBufSrv;

class TFlowBufCliPort {

public:
    TFlowBufCliPort(TFlowBufSrv* srv, uint32_t mask, int fd);
    ~TFlowBufCliPort();

    GMainContext* context;      // AV: Q: ? is in use ? Use server context instead?
    clock_t last_idle_check = 0;

    char* signature;

    TFlowBufSrv* srv;

    int onMsg();

    uint32_t cli_port_mask;     // TODO: Q: ? Use sck_fd as a mask ?

    typedef struct
    {
        GSource g_source;
        TFlowBufCliPort* cli_port;
    } GSourceCliPort;

    int                 sck_fd;
private:

    int onSign(struct TFlowBuf::pck_sign *sign_msg);
    int SendCamFD();

    GSourceCliPort*     sck_src;
    gpointer            sck_tag;
    GSourceFuncs        sck_gsfuncs;

    int msg_seq_num;
};

class V4L2Device;

class TFlowBufSrv  {
public:

    TFlowBufSrv(GMainContext* context, int _buffs_num);
    ~TFlowBufSrv();
    int StartListening();
    void onIdle(clock_t now);
    
    void add_new(int index);                    // Called by Camera device upon new V4L2 buffer allocation
    int  consume(v4l2_buffer &v4l2_buf);        // Pass newly incoming frame to Client Ports
    void redeem(TFlowBuf& buf, uint32_t mask);  // Called when CliPort returns buffers back to TFlow Buffer Server

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

    int SendCamFD(TFlowBufCliPort *cli_port);
    int onConnect();
    void releaseCliPort(TFlowBufCliPort* cli_port);

    GMainContext* context;

    int buffs_num;

private:
    clock_t last_idle_check = 0;

    std::array<TFlowBufCliPort*, 4> cli_ports{};

    std::vector<TFlowBuf> bufs{};                       // TODO: Q: ? Use external oject shared between V4L2_Device and TFlowBufSrv/CliPort ?

};

