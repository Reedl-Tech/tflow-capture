#include <sys/socket.h>
#include <sys/un.h>

#include <giomm.h>
#include <glib-unix.h>

#include "tflow-buf.h"
#include "tflow-capture.h"

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

gboolean TFlowBufCliPort::onMsg(Glib::IOCondition io_cond)
{
    int rc = onMsgRcv();
    if (rc) {
        srv->releaseCliPort(this);
        return G_SOURCE_REMOVE;
    }
    return G_SOURCE_CONTINUE;
}

void TFlowBufSrv::buf_redeem(int index, uint32_t mask)
{
    assert(index >= 0 && index < bufs.size());
    auto &tflow_buf = bufs.at(index);

    assert(tflow_buf.owners & mask);
    buf_redeem(bufs.at(index), mask);
}

void TFlowBufSrv::buf_redeem(TFlowBuf &tflow_buf, uint32_t mask)
{
    assert(tflow_buf.owners != 0);   // Attempt to redeem a free buffer

    tflow_buf.owners &= ~mask;
    if (tflow_buf.owners == 0) {
        tflow_buf.owners = 1;
        if (cam) cam->ioctlQueueBuffer(tflow_buf.index);
        else if (player) player->shmQueueBuffer(tflow_buf.index);
    }
}

void TFlowBufSrv::releaseCliPort(TFlowBufCliPort* cli_port)
{
    uint32_t mask = cli_port->cli_port_mask;

    g_warning("TFlowBufSrv: Release port [%s] mask=%d",
        cli_port->signature.c_str(), mask);

    for (auto& tflow_buf : bufs) {
        if (tflow_buf.owners & mask) {
            g_warning("TFlowBufSrv: redeem idx=%d, owners=%d", tflow_buf.index, tflow_buf.owners);
            buf_redeem(tflow_buf, mask);
        }
    }

    for (auto& cli_port_p : cli_ports) {
        if (cli_port_p == cli_port) {
            delete cli_port;
            cli_port_p = NULL;
            break;
        }
    }

}

int TFlowBufSrv::registerOnBuf(void* ctx, std::function<int(void* ctx, TFlowBuf& tflow_buf)> cb) 
{
    onBuf_ctx = ctx;
    onBuf_cb = cb;

    return 0;
}

int TFlowBufSrv::registerOnCustomMsg(void* ctx, std::function<int(void* ctx, const TFlowBuf::pck_t &in_msg)> cb)
{
    onCustomMsg_ctx = ctx;
    onCustomMsg_cb = cb;

    return 0;
}

TFlowBufCliPort::~TFlowBufCliPort()
{
    if (sck_fd != -1) {
        close(sck_fd);
        sck_fd = -1;
    }

    if (sck_src) {
        sck_src->destroy();
        sck_src.reset();
    }
}

TFlowBufCliPort::TFlowBufCliPort(TFlowBufSrv* _srv, uint32_t mask, int fd)
{
    context = _srv->context;
    srv = _srv;

    sck_fd = fd;

    cli_port_mask = mask;
    msg_seq_num = 0;
    request_cnt = 0;

    sck_src = Glib::IOSource::create(sck_fd, (Glib::IOCondition)(G_IO_IN | G_IO_ERR | G_IO_HUP));
    sck_src->connect(sigc::mem_fun(*this, &TFlowBufCliPort::onMsg));
    sck_src->attach(context);

    last_idle_check_ts.tv_nsec = 0;
    last_idle_check_ts.tv_sec = 0;
}

TFlowBufSrv::~TFlowBufSrv()
{
    if (sck_fd > 0) {
        close(sck_fd);
    }

    if (sck_src) {
        sck_src->destroy();
        sck_src.reset();
    }
}

TFlowBufSrv::TFlowBufSrv(MainContextPtr app_context)
{
    context = app_context;

    last_idle_check_ts.tv_nsec = 0;
    last_idle_check_ts.tv_sec = 0;

    sck_fd = -1;
    sck_state_flag.v = Flag::UNDEF;

    cam = nullptr;
    player = nullptr;

    onBuf_ctx = nullptr;
    onBuf_cb = nullptr;
}

void TFlowBufSrv::buf_create(int buf_num)
{
    bufs = std::vector<TFlowBuf>(buf_num, TFlowBuf());

    for (int i = 0; i < buf_num; i++) {
        auto& tflow_buf = bufs.at(i);
        tflow_buf.index = i;

        // Pass all newly created buffers to the Kernel
        tflow_buf.owners = 1;
        if (cam) cam->ioctlQueueBuffer(i);
        else if (player) player->shmQueueBuffer(i);
    }

}

/*
 * TODO: Q: ? Make input buffer agnostic as we need only buffer index, 
 *           v4l2_buf.timestamp and v4l2_buf.sequence ?
 */
int TFlowBufSrv::buf_consume(v4l2_buffer &v4l2_buf) 
{
    int rc = 0;

    // Sanity check
    if (v4l2_buf.index < 0 || v4l2_buf.index >= bufs.size()) {
        g_error("Ooops... at %s (%d) index mismatch %d ", __FILE__, __LINE__, v4l2_buf.index);
    }

    auto &tflow_buf = bufs[v4l2_buf.index];

    // Sanity - ensure buff is just from the driver
    assert(tflow_buf.owners == 1);

    // new buffer received normally
    {
        static int cnt = 0;
        cnt++;
        if ((cnt & 0xFF) == 0) {
            g_warning("Consumed %d frames", cnt);
        }
    }

    tflow_buf.owners = 0;
    tflow_buf.ts = v4l2_buf.timestamp;
    tflow_buf.sequence = v4l2_buf.sequence;

    // call owner's callback to update buffer's aux_data
    if (onBuf_cb) {
        onBuf_cb(onBuf_ctx, tflow_buf);
#if CODE_BROWSE
        static int _onBufAP();
        TFlowCapture::onBufAP(tflow_buf);
        TFlowCapture::onBufPlayer(tflow_buf);
#endif
    }

    // loop over subscribers 
    for (auto &cli_port_p : cli_ports) {
        if (!cli_port_p) continue;

        cli_port_p->SendConsume(tflow_buf);
    }

    if (tflow_buf.owners == 0) {
        // If packet is not consumed, i.e. all subscribers already are
        // filled-up or no subscribers at all, then return the buffer back to 
        // the Camera driver (Kernel)
        tflow_buf.owners = 1;

        if (cam) rc = cam->ioctlQueueBuffer(tflow_buf.index);
        else if (player) rc = player->shmQueueBuffer(tflow_buf.index);
    }
    return 0;
}

int TFlowBufCliPort::SendConsume(TFlowBuf &tflow_buf)
{
    if (request_cnt == 0) {
        return 0;
    }

    ssize_t res;
    TFlowBuf::pck_t tflow_pck{};

    tflow_pck.hdr.id = TFLOWBUF_MSG_CONSUME;
    tflow_pck.hdr.seq = msg_seq_num++;

    tflow_pck.consume.buff_index = tflow_buf.index;
    tflow_pck.consume.ts = tflow_buf.ts;
    tflow_pck.consume.seq = tflow_buf.sequence;

    assert(tflow_buf.aux_data_len <= sizeof(tflow_pck.consume.aux_data));
    tflow_pck.consume.aux_data_len = tflow_buf.aux_data_len;
    if (tflow_buf.aux_data_len) {
        memcpy(tflow_pck.consume.aux_data, tflow_buf.aux_data, tflow_buf.aux_data_len);
    }

    res = send(sck_fd, &tflow_pck, offsetof(TFlowBuf::pck_consume, aux_data) + tflow_buf.aux_data_len, 0);
    int err = errno;

    if (res == -1) {
        if (errno == EAGAIN) {
            g_warning("---------------- EAGAIN on CliPort SEND -------");
            return 0;
        }
        else {
            g_warning("TFlowBufSrv: send error (%d) - %s", errno, strerror(errno));
            return -1;
        }
    }

    assert(this->request_cnt > 0);

    tflow_buf.owners |= this->cli_port_mask;
    this->request_cnt --;

    return 0;
}
int TFlowBufCliPort::SendCamFD()
{
    struct msghdr   msg;
    struct iovec    iov[1];
    ssize_t res;

    TFlowBuf::pck_t tflow_pck {};

    tflow_pck.hdr.id = TFLOWBUF_MSG_CAM_FD;
    tflow_pck.hdr.seq = msg_seq_num++;

    if (srv->cam) {
        tflow_pck.cam_fd.planes_num = srv->cam->planes_num;
        tflow_pck.cam_fd.buffs_num  = srv->cam->buffs_num;
        tflow_pck.cam_fd.width      = srv->cam->stream_fmt->width;
        tflow_pck.cam_fd.height     = srv->cam->stream_fmt->height;
        tflow_pck.cam_fd.format     = srv->cam->stream_fmt->fmt_cc.u32;
    } else if (srv->player) {
        tflow_pck.cam_fd.planes_num = 0;
        tflow_pck.cam_fd.buffs_num  = srv->player->buffs_num;
        // WxH from config file of file meta info
        tflow_pck.cam_fd.width      = srv->player->frame_width;
        tflow_pck.cam_fd.height     = srv->player->frame_height;
        tflow_pck.cam_fd.format     = srv->player->frame_format;
    }
    else {
        assert(0);
    }

#if CODE_BROWSE
    V4L2_PIX_FMT_GREY
#endif

    char buf[CMSG_SPACE(sizeof(int))];  /* ancillary data buffer */

    int dev_fd =
        srv->cam ? srv->cam->dev_fd :
        srv->player ? srv->player->shm_fd : -1;

    if (dev_fd == -1) {
        g_warning("TFlowBufCliPort: Ooops - fd is not valid");
    }

    msg.msg_name = NULL;
    msg.msg_namelen = 0;

    msg.msg_control = buf;
    msg.msg_controllen = sizeof(buf);

    struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
    cmsg->cmsg_level = SOL_SOCKET;
    cmsg->cmsg_type = SCM_RIGHTS;
    cmsg->cmsg_len = CMSG_LEN(sizeof(int));
    *(int*)CMSG_DATA(cmsg) = dev_fd;
    msg.msg_controllen = cmsg->cmsg_len;

    iov[0].iov_base = (void*)&tflow_pck;
    iov[0].iov_len = sizeof(tflow_pck.cam_fd);

    msg.msg_iov = iov;
    msg.msg_iovlen = 1;

    res = sendmsg(sck_fd, &msg, 0);
    int err = errno;

    if (res == -1) {
        if (errno == EAGAIN) {
            return 0;
        }
        else {
            g_warning("TFlowBufSrv: sendmsg error (%d) - %s", errno, strerror(errno));
            return -1;
        }
    }

    return 0;

}
int TFlowBufCliPort::onRedeem(TFlowBuf::pck_redeem* pck_redeem)
{
    if (pck_redeem->buff_index != -1) {
        srv->buf_redeem(pck_redeem->buff_index, cli_port_mask);
    }

    if (pck_redeem->need_more) {
        request_cnt++;
    }

    return 0;
}

int TFlowBufCliPort::onPing(TFlowBuf::pck_ping* pck_ping)
{
    int rc = 0;

    std::string ping_from = std::string(pck_ping->cli_name);
    g_warning("TFlowBufCliPort: Ping on port %d from [%s]",
        this->cli_port_mask, ping_from.c_str());

//    rc = SendPong();

    return rc;
}

int TFlowBufCliPort::onSign(TFlowBuf::pck_sign *pck_sign)
{
    int rc;

    this->signature = std::string(pck_sign->cli_name);
    g_warning("TFlowBufCliPort: Signature for port %d - [%s]",
        this->cli_port_mask, this->signature.c_str());

    rc = SendCamFD();

    return rc;
}

int TFlowBufCliPort::onMsgRcv()
{
    TFlowBuf::pck_t in_msg;

    int res = recv(sck_fd, &in_msg, sizeof(in_msg), 0); //MSG_DONTWAIT 

    if (res <= 0) {
        int err = errno;
        if (err == ECONNRESET || err == EAGAIN) {
            g_warning("TFlowBufCliPort: [%s] disconnected (%d) - closing",
                this->signature.c_str(), errno);
        }
        else {
            g_warning("TFlowBufCliPort: [%s] unexpected error (%d) - %s",
                signature.c_str(), errno, strerror(errno));
        }
        return -1;
    }

    switch (in_msg.hdr.id) {
        case TFLOWBUF_MSG_SIGN_ID:  
            return onSign((TFlowBuf::pck_sign*)&in_msg);
        case TFLOWBUF_MSG_PING:
            return onPing((TFlowBuf::pck_ping*)&in_msg);
        case TFLOWBUF_MSG_REDEEM:
            return onRedeem((TFlowBuf::pck_redeem*)&in_msg);
        default:
            if ((in_msg.hdr.id > TFLOWBUF_MSG_CUSTOM_) && srv->onCustomMsg_cb) {
                return srv->onCustomMsg_cb(srv->onCustomMsg_ctx, in_msg);
#if CODE_BROWSE
    static int _onCustomMsg(void* ctx, const TFlowBuf::pck_t & in_msg);
    int TFlowCapture::onCustomMsg(const TFlowBuf::pck_t & in_msg);
    int TFlowCapture::onCustomMsgNavigator(const struct pck_navigator& in_msg_nav);
#endif
            }
            else {
                g_warning("TFlowBufCliPort: unexpected message received (%d)", in_msg.hdr.id);
            }
    }

    return 0;
}

gboolean TFlowBufSrv::onConnect(Glib::IOCondition io_cond)
{
    int cli_port_fd;
    
    g_info("TFlowBuf: Incoming connection (cond %d)", (int)io_cond);

    /* Get new empty Client port */
    uint32_t mask = 2;
    TFlowBufCliPort** cli_port_empty_pp = NULL;
    for (auto &cli_port_p : cli_ports) {
        if (cli_port_p == NULL) {
            cli_port_empty_pp = &cli_port_p;
            break;
        }
        mask <<= 1;
    }

    if (cli_port_empty_pp == NULL) {
        g_warning("No more free TFlow Buf Client Ports");
        return G_SOURCE_CONTINUE;
    }

    struct sockaddr_un peer_addr = { 0 };
    socklen_t sock_len = sizeof(peer_addr);

    cli_port_fd = accept(sck_fd, (struct sockaddr*)&peer_addr, &sock_len);
    if (cli_port_fd == -1) {
        g_warning("TFlowBufSrv: Can't connect a TFlow Buffer Client");
        return G_SOURCE_CONTINUE;
    }

    //int flags = fcntl(cli_port_fd, F_GETFL, 0);
    //fcntl(cli_port_fd, F_SETFL, flags | O_NONBLOCK);

    auto cli_port = new TFlowBufCliPort(this, mask, cli_port_fd);
    *cli_port_empty_pp = cli_port;

    g_warning("TFlowBufSrv: TFlow Buffer Client %d (%d) is connected",
        cli_port->cli_port_mask, cli_port->sck_fd);

    return G_SOURCE_CONTINUE;
}

int TFlowBufSrv::StartListening()
{
    int rc;
    struct sockaddr_un sock_addr;
    struct timeval tv;

    if (sck_fd != -1) {
        // Already listening
        return 0;
    }

    // Open local UNIX socket
    sck_fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0);
    if (sck_fd == -1) {
        g_warning("TFlowBufSrv: Can't open socket for local server (%d) - %s", errno, strerror(errno));
        return -1;
    }

    // Set to listen mode
    // Initialize socket address
    memset(&sock_addr, 0, sizeof(struct sockaddr_un));
    sock_addr.sun_family = AF_UNIX;
    memcpy(sock_addr.sun_path, "\0" TFLOWBUFSRV_SOCKET_NAME, strlen(TFLOWBUFSRV_SOCKET_NAME) + 1);  // NULL termination excluded

    socklen_t sck_len = sizeof(sock_addr.sun_family) + strlen(TFLOWBUFSRV_SOCKET_NAME) + 1;
    rc = bind(sck_fd, (const struct sockaddr*)&sock_addr, sck_len);
    if (rc == -1) {
        g_warning("TFlowBufSrv: Can't bind (%d) - %s", errno, strerror(errno));
        close(sck_fd);
        sck_fd = -1;
        return -1;
    }

    rc = listen(sck_fd, 1);
    if (rc == -1) {
        g_warning("TFlowBufSrv: Can't listen (%d) - %s", errno, strerror(errno));
        close(sck_fd);
        sck_fd = -1;
        return -1;
    }

    sck_src = Glib::IOSource::create(sck_fd, (Glib::IOCondition)(G_IO_IN | G_IO_ERR | G_IO_HUP));
    sck_src->connect(sigc::mem_fun(*this, &TFlowBufSrv::onConnect));
    sck_src->attach(context);

    return 0;
}

void TFlowBufSrv::onIdle(struct timespec *now_ts)
{
    if (sck_state_flag.v == Flag::SET) {
        // Normal operation. Check buffer are occupied for too long
        // ...
        for (auto &tflow_buf : bufs) {
            if (tflow_buf.age() > 3000 && (tflow_buf.owners & ~1)) {
                g_warning("TFlowBuf client(s) stall: 0x%02X", 
                    tflow_buf.owners);
                // Disqualify the client, close connection or ???
            }
        }

        return;
    }

    if (sck_state_flag.v == Flag::CLR) {
        if (diff_timespec_msec(now_ts, &last_idle_check_ts) > 1000) {
            last_idle_check_ts = *now_ts;
            sck_state_flag.v = Flag::RISE;
        }
    }

    if (sck_state_flag.v == Flag::RISE) {
        int rc;

        // Sanity check;
        int dev_fd =
            cam ? cam->dev_fd :
            player ? player->shm_fd : -1;

        if (dev_fd == -1) {
            g_warning("TFlowBufSrv: Ooops - listening at no source");
            sck_state_flag.v = Flag::CLR;
            return;
        }

        rc = StartListening();
        if (rc) {
            // Can't open local UNIX socket - try again later. 
            // It won't help, but anyway ...
            sck_state_flag.v = Flag::CLR;
        }
        else {
            sck_state_flag.v = Flag::SET;
        }
        return;
    }

    if (sck_state_flag.v == Flag::FALL) {
        // We can't provide buffers any more.
        // Probably camera is closed
        
        // Close all Clients Ports as the frame format might be changed then camera reopened
        for (auto &cli_port_p : cli_ports) {
            if (cli_port_p) {
                delete cli_port_p;
                cli_port_p = nullptr;
            }
        }
        sck_state_flag.v = Flag::CLR;
    }

}

#if 0
struct v4l2_ext_control ctrl;
CLEAR(ctrl);
CLEAR(ctrls);
ctrls.controls = &ctrl;
ctrls.count = 1;
ctrls.which = V4L2_CTRL_WHICH_CUR_VAL;

ctrl.id = TEGRA_CAMERA_CID_EXPOSURE;
ctrl.value = ;
if (xioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls) == -1) {
     fprintf(stderr, "Error VIDIOC_S_EXT_CTRLS %s\n",strerror(errno));
 }
#endif