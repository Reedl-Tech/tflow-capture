#include "tflow-build-cfg.hpp"
#include <cassert>
#include <functional>

#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/ioctl.h>

#include <arpa/inet.h>
#include <netinet/ip.h>


#include "tflow-common.hpp"
#include "tflow-glib.hpp"
//#include "tflow-perfmon.hpp"

#include "tflow-udp.hpp"

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

int TFlowUDP::parseCfgAddrPort(struct sockaddr_in *sock_addr_out, const char *cfg_addr_port_in)
{
    const char *colon = strchr(cfg_addr_port_in, ':');
    char addr_clone[128];
    const char *addr;
    uint16_t port = 0;

    if ( colon ) {
        // Port specified - lets split port and address
        strncpy(addr_clone, cfg_addr_port_in, (colon - cfg_addr_port_in));
        addr_clone[(colon - cfg_addr_port_in)] = 0;
        addr = addr_clone;
        char *endptr, *str;
        port = strtol(colon+1, &endptr, 10);
        if ( endptr == 0 || endptr > colon + 1 ) {
            // port OK or some garbage at the end
            sock_addr_out->sin_port = htons(port);
        } 
    }
    else {
        // No port, only address
        addr = cfg_addr_port_in;
    }

    if ( port == 0 ) {
        sock_addr_out->sin_port = htons(14550);
    }

    unsigned char buf[sizeof(in_addr)];
    sock_addr_out->sin_family = AF_INET;
    if (1 > inet_pton(sock_addr_out->sin_family, addr, &sock_addr_out->sin_addr) ) {
        // SISO mode
        inet_pton(sock_addr_out->sin_family, "127.0.0.1", &sock_addr_out->sin_addr);
        return -1;
    }

    return 0;
}

TFlowUDP::TFlowUDP(MainContextPtr app_context,
    const char *_remote_addr, const char *_local_addr)
{
    context = app_context;
    sck_state_flag.v = Flag::UNDEF;
    
    clock_gettime(CLOCK_MONOTONIC, &last_send_ts);
    last_conn_check_ts.tv_sec = 0;
    last_conn_check_ts.tv_nsec = 0;

    udp_in_msg_size = 1024 * 1024;
    udp_in_msg = (char*)g_malloc(udp_in_msg_size);

    addr_nok = 0;
    addr_nok |= parseCfgAddrPort(&remote_addr, _remote_addr);
    addr_nok |= parseCfgAddrPort(&local_addr, _local_addr);

}

TFlowUDP::~TFlowUDP()
{
    Disconnect();

    if (udp_in_msg) {
        g_free(udp_in_msg);
        udp_in_msg = nullptr;
        udp_in_msg_size = 0;
    }
}

void TFlowUDP::Disconnect()
{
    if (sck_fd != -1) {
        close(sck_fd);
        sck_fd = -1;
    }

    if (sck_src) {
        sck_src->destroy();
        sck_src.reset();
    }

    return;
}

int TFlowUDP::Connect()
{
    int rc;

    if ( addr_nok ) {
        g_warning("TFlowUDP: Bad address format");
        return -1;
    }

    sck_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, IPPROTO_UDP);
    if (sck_fd == -1) {
        g_warning("TFlowUDP: Can't create UDP socket (%d) - %s", errno, strerror(errno));
        return -1;
    }

	//{
	//	int so_bc_flag = 1;
	//	socklen_t len = sizeof(so_bc_flag);
	//	rc = setsockopt(sck_fd, SOL_SOCKET, SO_BROADCAST, &so_bc_flag, len);
	//}

    char addr_str_buf[128];
    const char *addr_str = inet_ntop(
        local_addr.sin_family, (const void*)&local_addr.sin_addr,
        addr_str_buf, sizeof(addr_str_buf));

	//bind socket to port
    rc = bind(sck_fd, (const struct sockaddr*)&local_addr, sizeof(local_addr));
	if ( rc == -1) {
        g_warning("TFlowUDP: Can't bind to %s (%d) - %s", addr_str, errno, strerror(errno));
        addr_nok = 1;
        return -1;
	}

	g_info("TFlowUDP: Bond sucessfully");

    sck_src = Glib::IOSource::create(sck_fd, (Glib::IOCondition)(G_IO_IN | G_IO_ERR | G_IO_HUP));
    sck_src->connect(sigc::mem_fun(*this, &TFlowUDP::onMsg));
    sck_src->attach(context);

    memset(udp_in_msg, 0, udp_in_msg_size);
    return 0;
}

void TFlowUDP::onIdle_no_ts()
{
    // Called as a kick 
    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);
    onIdle(now_ts);
}

void TFlowUDP::onIdle(struct timespec now_ts)
{
    if (sck_state_flag.v == Flag::CLR) {
        if (diff_timespec_msec(&now_ts, &last_conn_check_ts) > 1000) {
            last_conn_check_ts = now_ts;
            sck_state_flag.v = Flag::RISE;
        }
        return;
    }

    if (sck_state_flag.v == Flag::SET) {
        // Normal operation. 
        if (diff_timespec_msec(&now_ts, &last_send_ts) > 1000) {
            // Do something lazy;
        }
        return;
    }

    if (sck_state_flag.v == Flag::UNDEF || sck_state_flag.v == Flag::RISE) {
        int rc;

        rc = Connect();
        if (rc) {
            sck_state_flag.v = Flag::FALL;
        }
        else {
            sck_state_flag.v = Flag::SET;
            //app_onConnect();
        }
        return;
    }

    if (sck_state_flag.v == Flag::FALL) {
        // Connection aborted.
//        if () app_onSrcGone();
//        if () app_onDisconnect();

        Disconnect();

        // Try to reconnect later
        sck_state_flag.v = Flag::CLR;
    }
}

gboolean TFlowUDP::onMsg(Glib::IOCondition io_cond)
{
    if (io_cond == Glib::IOCondition::IO_ERR) {
        assert(0);  // Implement something or remove condition from the source
    }

    if (io_cond == Glib::IOCondition::IO_HUP) {
        assert(0);  // Implement something or remove condition from the source
    }

    int rc = onMsgRcv();
    if (rc) {
        sck_state_flag.v = Flag::FALL;
        return G_SOURCE_REMOVE;
    }
    return G_SOURCE_CONTINUE;
}

int TFlowUDP::onMsgRcv()
{
    // TODO: use recvfrom. 
    //     If QGC address is specified, then filter only our messages
    //     If QGC address isn't specified, then preserve for ???

    int res = recv(sck_fd, udp_in_msg, udp_in_msg_size, 0); //MSG_DONTWAIT 

    if (res <= 0) {
        int err = errno;
        if (err == ECONNRESET || err == EAGAIN) {
            g_warning("TFlowUDP: disconnected (%d, %s) - closing", errno, 
                strerror(errno));
        }
        else {
            g_warning("TFlowUDP: unexpected error (%d, %s) - closing", errno,
                strerror(errno));
        }
        return -1;
    }
    
    onUDPMsg(udp_in_msg, res);
#if CODE_BROWSE
        TFlowMilesi::onUDPMsg(in_msg);
#endif

    return 0;
    
}

int TFlowUDP::UDPDataSend(uint8_t *buf, size_t buf_len)
{
    if (sck_state_flag.v != Flag::SET) {
        // UDP port not ready yet or was intentionally closed.
        // Not an error
        return 0;
    }

    size_t bytes_to_wr = buf_len;
    ssize_t bytes_written = sendto(sck_fd, buf, bytes_to_wr, 0, 
        (struct sockaddr*)&remote_addr, sizeof(struct sockaddr_in));

    // raw_hex_dump(buf, bytes_written);

    if (bytes_written <= 0) {
        g_warning("TFlowUDP: Can't write to port - %s  (%d)",  
            strerror(errno), errno);

        Disconnect();

        // Try to reconnect later
        sck_state_flag.v = Flag::CLR;
        return -1;
    }

    if (bytes_written != bytes_to_wr) {
        g_warning("TFlowSerial: Can't write whole packet - %s (%d)", 
            strerror(errno), errno);

        Disconnect();

        // Try to reconnect later
        sck_state_flag.v = Flag::CLR;
        return -1;
    }

    return 0;
}
