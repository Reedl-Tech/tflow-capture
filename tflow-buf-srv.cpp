#include <sys/socket.h>
#include <sys/un.h>

#include <giomm.h>
#include <glib-unix.h>

#include "tflow-capture.h"

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

TFlowBufSrv::TFlowBufSrv(GMainContext* app_context)
{
    context = app_context;

    uint32_t subscr_mask = 2;
    for (auto &cli_port : cli_ports) {
        cli_port.subscriber_mask = subscr_mask;
        subscr_mask <<= 1;
    }

    sck_tag = NULL;
    sck_src = NULL;
    CLEAR(sck_gsfuncs);

    int i = 0;
    for (auto& buf : bufs) {
        buf.index = i++;
    }

#if 0
    // set to listen mode
    // Initialize socket address

#define SOCKET_NAME "com.ubnt.dsdp.local-server"
    memset(&sock_addr, 0, sizeof(struct sockaddr_un));
    sock_addr.sun_family = AF_UNIX;
    memcpy(sock_addr.sun_path, "\0"SOCKET_NAME, strlen(SOCKET_NAME) + 1);  // NULL termination excluded

    cli_ports[PORT_VPROCESS].sck_name =
#endif
}

int TFlowBufSrv::consume(v4l2_buffer &v4l2_buf) 
{
    int is_consumed = 0;

    // Get TFlow buffer from by v4l2_buf's index
    // 
    // Sanity check
    if (v4l2_buf.index < 0 || v4l2_buf.index >= bufs.size()) {
        g_error("Ooops... at %s (%d) index mismatch %d ", __FILE__, __LINE__, v4l2_buf.index);
    }
    auto &tflow_buf = bufs[v4l2_buf.index];

    // Sanity
    assert(tflow_buf.subscribers == 0);

    tflow_buf.ts = v4l2_buf.timestamp;

    // loop over subscribers 
    if (0) {
        // pass buffer to the client
        // tflow_buf.subscribers = subscriber mask
        // 
    }

    if (!is_consumed) {
        // If packet is not consumed, i.e. all subscribers already 
        // filled-up or no subscribers at all, then return buffer back to the 
        // Camera
        cam->redeem(tflow_buf.index);
    }
    return 0;
}

gboolean tflow_buf_srv_dispatch(GSource* g_source, GSourceFunc callback, gpointer user_data)
{
    TFlowBufSrv::GSourceSrv* source = (TFlowBufSrv::GSourceSrv*)g_source;
    TFlowBufSrv* srv = source->srv;

    g_info("TFlowBuf: Incoming connection");

    //int index;
    //timeval frame_ts;
    //cam->ioctlDequeueBuffer(&index, &frame_ts);

    return G_SOURCE_CONTINUE;
}

int TFlowBufSrv::StartListening()
{
    int rc;
    struct sockaddr_un sock_addr;

    struct timeval tv;

    // Open local UNIX socket
    sck_fd = socket(AF_UNIX, SOCK_STREAM, 0);
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
                    tflow_buf.subscribers);
                // Disqualify the client, close connection or ???
            }
        }

        return;
    }

    if (sck_state_flag.v == Flag::UNDEF || sck_state_flag.v == Flag::RISE) {
        int rc;

        rc = StartListening();
        if (rc) {
            // Can't open local UNIX socket - try again later. 
            // It won't, help but anyway ...
            sck_state_flag.v = Flag::RISE;
        }

        sck_state_flag.v = Flag::SET;
        return;
    }

    if (sck_state_flag.v == Flag::FALL) {
        // Close the socket? but why?
        sck_state_flag.v = Flag::CLR;
    }

}
