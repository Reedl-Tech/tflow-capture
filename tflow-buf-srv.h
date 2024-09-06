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

    std::string signature;

    TFlowBufSrv* srv;       // Is used to access the Camera device.

    int onMsg();

    uint32_t cli_port_mask;     // TODO: Q: ? Use sck_fd as a mask ?

    typedef struct
    {
        GSource g_source;
        TFlowBufCliPort* cli_port;
    } GSourceCliPort;

    int                 sck_fd;

    int request_cnt;

    int SendConsume(TFlowBuf& tflow_buf);

private:
    struct timespec last_idle_check_tp = { 0 };

    int onRedeem(struct TFlowBuf::pck_redeem* pck_redeem);
    int onSign(struct TFlowBuf::pck_sign *pck_sign);
    int SendCamFD();

    GSourceCliPort*     sck_src;
    gpointer            sck_tag;
    GSourceFuncs        sck_gsfuncs;

    int msg_seq_num;
};

class V4L2Device;
class TFlowPlayer;

class TFlowBufSrv  {
public:

    TFlowBufSrv(GMainContext* context);
    ~TFlowBufSrv();
    int StartListening();
    void onIdle(struct timespec* now_tp);

    void buf_create(int buff_num);                  // Called by Camera device upon new V4L2 buffers allocation
    void buf_redeem(int index, uint32_t mask);
    void buf_redeem(TFlowBuf& buf, uint32_t mask);  // Called when CliPort returns buffers back to TFlow Buffer Server
    int  buf_consume(v4l2_buffer &v4l2_buf);        // Pass newly incoming frame to Client Ports

    char* sck_name;
    int sck_fd = -1;
    Flag sck_state_flag;

    // TODO: Rework to a generic source with overloaded functions get_fd, redeem
    // Uses only 2 functions - 
    //     xxx->dev_fd and 
    //     xxx->ioctlQueueBuffer    // Redeem buffer back to owner
    V4L2Device *cam;
    TFlowPlayer *player;

    typedef struct
    {
        GSource g_source;
        TFlowBufSrv* srv;
    } GSourceSrv;

    GSourceSrv* sck_src;
    gpointer sck_tag;
    GSourceFuncs sck_gsfuncs;

    void onConnect();
    void releaseCliPort(TFlowBufCliPort* cli_port);

    int registerOnBuf(void* ctx, std::function<int(void* ctx, TFlowBuf& tflow_buf)> cb);

    GMainContext* context;

private:
    struct timespec last_idle_check_tp = { 0 };

    std::array<TFlowBufCliPort*, 4> cli_ports{};

    std::vector<TFlowBuf> bufs;     // Actual buffers will be created by Camera device upon successeful request from Kernel
                                    // TODO: Q: ? Use external oject shared between V4L2_Device and TFlowBufSrv/CliPort ?

    void* onBuf_ctx;
    size_t onBuf_aux_data_len;
    std::function<int(void* ctx, TFlowBuf &tflow_buf)> onBuf_cb;
};

