#include <sys/socket.h>
#include <sys/un.h>

#include <giomm.h>
#include <glib-unix.h>

#include "tflow-buf.h"
#include "tflow-capture.h"

gboolean tflow_buf_cli_port_dispatch(GSource* g_source, GSourceFunc callback, gpointer user_data)
{
    TFlowBufCliPort::GSourceCliPort* source = (TFlowBufCliPort::GSourceCliPort*)g_source;
    TFlowBufCliPort* cli_port = source->cli_port;

    int rc = cli_port->onMsg();
    if (rc) {
        TFlowBufSrv* srv = cli_port->srv;
        g_warning("Client port [%s] need to be closed", cli_port->signature);
        srv->releaseCliPort(cli_port);
        cli_port = NULL;
        return G_SOURCE_REMOVE;
    }
    return G_SOURCE_CONTINUE;
}

gboolean tflow_buf_srv_dispatch(GSource* g_source, GSourceFunc callback, gpointer user_data)
{
    TFlowBufSrv::GSourceSrv* source = (TFlowBufSrv::GSourceSrv*)g_source;
    TFlowBufSrv* srv = source->srv;

    g_info("TFlowBuf: Incoming connection");

    srv->onConnect();
    
    return G_SOURCE_CONTINUE;
}

void TFlowBufSrv::redeem(TFlowBuf &tflow_buf, uint32_t mask)
{
    assert(tflow_buf.owners != 0);   // Attempt to redeem a free buffer

    tflow_buf.owners &= ~mask;
    if (tflow_buf.owners == 0) {
        tflow_buf.owners = 1;
        cam->ioctlQueueBuffer(tflow_buf.index);
        
//        free_usr_bufs.push_back(&buf);
    }
}

void TFlowBufSrv::releaseCliPort(TFlowBufCliPort* cli_port)
{
    uint32_t mask = cli_port->cli_port_mask;

    for (auto &tflow_buf : bufs) {
        if (tflow_buf.owners & mask) {
            redeem(tflow_buf, cli_port->cli_port_mask);
        }
    }
    delete cli_port;
}

TFlowBufSrv::~TFlowBufSrv()
{
    if (sck_src) {
        if (sck_tag) {
            g_source_remove_unix_fd((GSource*)sck_src, sck_tag);
            sck_tag = nullptr;
        }
        g_source_destroy((GSource*)sck_src);
        g_source_unref((GSource*)sck_src);
        sck_src = nullptr;
    }
}
TFlowBufCliPort::~TFlowBufCliPort()
{
    if (signature) {
        free(signature);
    }

    if (sck_src) {
        if (sck_tag) {
            g_source_remove_unix_fd((GSource*)sck_src, sck_tag);
            sck_tag = nullptr;
        }
        g_source_destroy((GSource*)sck_src);
        g_source_unref((GSource*)sck_src);
        sck_src = nullptr;
    }

}

TFlowBufCliPort::TFlowBufCliPort(TFlowBufSrv* _srv, uint32_t mask, int fd)
{
    context = _srv->context;
    srv = _srv;

    sck_fd = fd;
    sck_tag = NULL;
    sck_src = NULL;

    signature = NULL;
    
    CLEAR(sck_gsfuncs);

    cli_port_mask = mask;
    msg_seq_num = 0;

    /* Assign g_source on the socket */
    sck_gsfuncs.dispatch = tflow_buf_cli_port_dispatch;
    sck_src = (GSourceCliPort*)g_source_new(&sck_gsfuncs, sizeof(GSourceCliPort));
    sck_tag = g_source_add_unix_fd((GSource*)sck_src, sck_fd, (GIOCondition)(G_IO_IN | G_IO_ERR | G_IO_HUP));
    sck_src->cli_port = this;
    g_source_attach((GSource*)sck_src, srv->context);

}

TFlowBufSrv::TFlowBufSrv(GMainContext* app_context, int _buffs_num)
{
    context = app_context;

    buffs_num = _buffs_num;

    sck_fd = -1;
    sck_tag = NULL;
    sck_src = NULL;
    CLEAR(sck_gsfuncs);

}

/*
int TFlowBufSrv::get_free() {

    if (free_usr_bufs.empty()) return -1;
    auto& buf_p = free_usr_bufs.front();
    free_usr_bufs.pop_front();
    return buf_p->index;
}
*/

void TFlowBufSrv::add_new(int index)
{
    auto& tflow_buf = *(bufs.insert(bufs.begin() + index, TFlowBuf(index)));

    tflow_buf.owners = 1;
    cam->ioctlQueueBuffer(index);

//    free_usr_bufs.push_back(&buf);
}

int TFlowBufSrv::consume(v4l2_buffer &v4l2_buf) 
{
    if (v4l2_buf.index == -1) {
        return 0;       // No buffers - It is OK, everything read out. 
    }

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
        if ((cnt & 0x3F) == 0) {
            g_info("Consumed %d frames", cnt);
        }
    }

    tflow_buf.owners = 0;
    tflow_buf.ts = v4l2_buf.timestamp;

    // loop over subscribers 
    if (0) {
        // pass buffer to the client
        // tflow_buf.subscribers = subscriber mask
        // 
    }

    if (tflow_buf.owners == 0) {
        // If packet is not consumed, i.e. all subscribers already are
        // filled-up or no subscribers at all, then return buffer back to the 
        // Camera
        tflow_buf.owners = 1;
        cam->ioctlQueueBuffer(tflow_buf.index);
    }
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
    tflow_pck.cam_fd.buffs_num = srv->cam->buffs_num;
    tflow_pck.cam_fd.planes_num = srv->cam->planes_num;

    char buf[CMSG_SPACE(sizeof(int))];  /* ancillary data buffer */
    int* fdptr;

    if (srv->cam->dev_fd == -1) {
        g_warning("TFlowBufCliPort: Ooops - camera fd is not valid");
    }

    msg.msg_name = NULL;
    msg.msg_namelen = 0;

    msg.msg_control = buf;
    msg.msg_controllen = sizeof(buf);

    struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
    cmsg->cmsg_level = SOL_SOCKET;
    cmsg->cmsg_type = SCM_RIGHTS;
    cmsg->cmsg_len = CMSG_LEN(sizeof(int));
    *(int*)CMSG_DATA(cmsg) = srv->cam->dev_fd;
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

int TFlowBufCliPort::onSign(struct TFlowBuf::pck_sign *sign_msg)
{
    int rc;

    this->signature = strdup(sign_msg->cli_name);

    rc = SendCamFD();

    return rc;
}
int TFlowBufCliPort::onMsg()
{
    g_info("TFlowBufCliPort: Check incoming message");

    TFlowBuf::pck_t in_msg;

    int res = recv(sck_fd, &in_msg, sizeof(in_msg), 0); //MSG_DONTWAIT 
    int err = errno;

    if (res <= 0) {
        g_warning("TFlowBufCliPort: unexpected error (%d) - %s", errno, strerror(errno));
        return -1;
    }

    switch (in_msg.hdr.id) {
        case TFLOWBUF_MSG_SIGN_ID:  return onSign((struct TFlowBuf::pck_sign*)&in_msg);
    default:
        g_warning("TFlowBufCliPort: unexpected message received (%d)", in_msg.hdr.id);
    }

    return 0;
}

int TFlowBufSrv::onConnect()
{
    int cli_port_fd;
    int rc;
    
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
        g_warning("No more free TFlow Client Ports");
        return -1;
    }

    cli_port_fd = accept(sck_fd, NULL, 0);
    if (cli_port_fd != -1) {
        g_warning("TFlowBufSrv: TFlow Buffer Client is connected");
    }
    else {
        g_warning("TFlowBufSrv: Can't connect a TFlow Buffer Client");
    }

    //int flags = fcntl(cli_port_fd, F_GETFL, 0);
    //fcntl(cli_port_fd, F_SETFL, flags | O_NONBLOCK);

    auto cli_port = new TFlowBufCliPort(this, mask, cli_port_fd);
    *cli_port_empty_pp = cli_port;

    return 0;
}

int TFlowBufSrv::StartListening()
{
    int rc;
    struct sockaddr_un sock_addr;
    struct timeval tv;

    // Open local UNIX socket
    sck_fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0);
    if (sck_fd == -1) {
        g_warning("TFlowBufSrv: Can't open socket for local server (%d) - %s", errno, strerror(errno));
        return -1;
    }

    //int flags = fcntl(sck_fd, F_GETFL, 0);
    //fcntl(sck_fd, F_SETFL, flags | O_NONBLOCK);

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
        g_warning("TFlowBufSrv: Can't bind (%d) - %s", errno, strerror(errno));
        close(sck_fd);
        return -1;
    }

    /* Assign g_source on the socket */
    sck_gsfuncs.dispatch = tflow_buf_srv_dispatch;
    sck_src = (GSourceSrv*)g_source_new(&sck_gsfuncs, sizeof(GSourceSrv));
    sck_tag = g_source_add_unix_fd((GSource*)sck_src, sck_fd, (GIOCondition)(G_IO_IN | G_IO_ERR | G_IO_HUP));
    sck_src->srv = this;
    g_source_attach((GSource*)sck_src, context);

    return 0;
}

// AV: Is TFlowCapture funtionality ??? Similarly as with Camera (V4L2Device)
void TFlowBufSrv::onIdle(clock_t now)
{
    clock_t dt = now - last_idle_check;

    /* Do not check to often*/
    if (dt < 3 * CLOCKS_PER_SEC) return;
    last_idle_check = now;

    if (sck_state_flag.v == Flag::SET || sck_state_flag.v == Flag::CLR) {
        // Normal operation. Check buffer are occupied for too long
        // ...
        for (auto &tflow_buf : bufs) {
            if (tflow_buf.age() > 3000) {
                g_warning("TFlowBuf client(s) stall: 0x%02X", 
                    tflow_buf.owners);
                // Disqualify the client, close connection or ???
            }
        }

        return;
    }

    if (sck_state_flag.v == Flag::RISE) {
        int rc;

        // Sanity check;
        if (cam->dev_fd == -1) {
            g_warning("TFlowBufSrv: Ooops - listening  at no camera");
            sck_state_flag.v = Flag::CLR;
            return;
        }

        rc = StartListening();
        if (rc) {
            // Can't open local UNIX socket - try again later. 
            // It won't help, but anyway ...
            sck_state_flag.v = Flag::RISE;
        }
        else {
            sck_state_flag.v = Flag::SET;
        }
        return;
    }

    if (sck_state_flag.v == Flag::FALL) {
        // We can't provide buffers any more.
        // Probably camera is closed
        
        // Close all Clients Ports
        
        // Close the socket? but why?
        sck_state_flag.v = Flag::CLR;
    }

}
