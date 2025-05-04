#pragma once

#include <cassert>
#include <time.h>
#include <linux/videodev2.h> //V4L2 stuff

#include <glib-unix.h>

#include "tflow-glib.hpp"
#include "tflow-buf.hpp"

#define TFLOWBUFSRV_SOCKET_NAME "com.reedl.tflow.capture.buf-server"

class TFlowBufSrv;

class TFlowBufCliPort {

public:
    TFlowBufCliPort(TFlowBufSrv* srv, uint32_t mask, int fd);
    ~TFlowBufCliPort();

    MainContextPtr context;      // AV: Q: ? is in use ? Use server context instead?

    std::string signature;

    TFlowBufSrv* srv;       // Is used to access the Camera device.

    gboolean onMsg(Glib::IOCondition io_cond);
    int onMsgRcv();

    uint32_t cli_port_mask;     // TODO: Q: ? Use sck_fd as a mask ?

    int sck_fd;

    int request_cnt;

    int SendConsume(TFlowBuf& tflow_buf);

private:
    struct timespec last_idle_check_ts;

    int onRedeem(struct TFlowBuf::pck_redeem* pck_redeem);
    int onSign(struct TFlowBuf::pck_sign *pck_sign);
    int onPing(struct TFlowBuf::pck_ping *pck_ping);
    int SendCamFD();

    IOSourcePtr sck_src;

    int msg_seq_num;
};

class TFlowCaptureV4L2;
class TFlowPlayer;

class TFlowBufSrv  {
public:

    TFlowBufSrv(MainContextPtr context);
    ~TFlowBufSrv();
    int StartListening();
    void onIdle(struct timespec* now_ts);

    void buf_create(int buff_num);                  // Called by Camera device upon new V4L2 buffers allocation
    void buf_redeem(int index, uint32_t mask);
    void buf_redeem(TFlowBuf& buf, uint32_t mask);  // Called when CliPort returns buffers back to TFlow Buffer Server
    int  buf_consume(v4l2_buffer &v4l2_buf);        // Pass newly incoming frame to Client Ports

    int sck_fd = -1;
    Flag sck_state_flag;

    // TODO: Rework to a generic source with overloaded functions get_fd, redeem
    // Uses only 2 functions - 
    //     xxx->dev_fd and 
    //     xxx->ioctlQueueBuffer    // Redeem buffer back to owner
    TFlowCaptureV4L2 *cam;
    TFlowPlayer *player;

    IOSourcePtr sck_src;

    gboolean onConnect(Glib::IOCondition io_cond);
    void releaseCliPort(TFlowBufCliPort* cli_port);

    int registerOnBuf(void* ctx, std::function<int(void* ctx, TFlowBuf& tflow_buf)> cb);
    int registerOnCustomMsg(void* ctx, std::function<int(void* ctx, const TFlowBuf::pck_t& in_msg)> cb);

    MainContextPtr context;

    std::string my_name;

    // Called from CliPort
    void *onCustomMsg_ctx;
    std::function<int(void* ctx, const TFlowBuf::pck_t& in_msg)> onCustomMsg_cb;

private:

    std::string srv_name;

    struct timespec last_idle_check_ts;

    std::array<TFlowBufCliPort*, 4> cli_ports{};

    std::vector<TFlowBuf> bufs;     // Actual buffers will be created by Camera device upon successeful request from Kernel
                                    // TODO: Q: ? Use external oject shared between V4L2_Device and TFlowBufSrv/CliPort ?

    void* onBuf_ctx;
    std::function<int(void* ctx, TFlowBuf &tflow_buf)> onBuf_cb;
};

