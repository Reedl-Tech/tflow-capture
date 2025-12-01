#pragma once
#include <arpa/inet.h>
#include <netinet/ip.h>

#include "tflow-common.hpp"
#include "tflow-glib.hpp"

class TFlowUDP {

public:
    TFlowUDP(
        MainContextPtr app_context,
        const char *remote_addr, const char *local_addr);

    ~TFlowUDP();

    void onIdle_no_ts();
    void onIdle(const struct timespec &now_ts);

    int Connect();
    void Disconnect();
    gboolean onMsg(Glib::IOCondition);
    int onMsgRcv();
    int UDPDataSend(uint8_t *buf, size_t buf_len);

    virtual void onUDPMsg (const char *udp_msg, int udp_msg_len) = 0;

private:
    MainContextPtr context;

    int addr_nok;
    struct sockaddr_in local_addr;
    struct sockaddr_in remote_addr;

    // int last_err;

    int sck_fd;
    Flag sck_state_flag;

    struct timespec last_send_ts;
    struct timespec last_conn_check_ts;

    IOSourcePtr sck_src;
    size_t udp_in_msg_size;
    char* udp_in_msg;

    static int parseCfgAddrPort(struct sockaddr_in *sock_addr_out, const char *cfg_addr_port_in);
};
