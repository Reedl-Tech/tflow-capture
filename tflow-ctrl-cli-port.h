#pragma once

#include <time.h>
#include <string>
#include <glib-unix.h>
#include <json11.hpp>

class TFlowCtrlSrv;
class TFlowCtrlCliPort {

public:
    TFlowCtrlCliPort(MainContextPtr context, TFlowCtrlSrv &srv, int fd);
    ~TFlowCtrlCliPort();

    std::string signature;

private:
    MainContextPtr context;

    TFlowCtrlSrv &srv;      // is used to report socket error to the Server

    struct timespec last_idle_check_ts;
    struct timespec last_send_ts;
                                                        
    int pid;

    int sck_fd;

    gboolean onMsg(Glib::IOCondition io_cond);
    int onMsgRcv();
    int onMsgSign(const json11::Json& j_params);

    int sendResp(const char *cmd, int err, const json11::Json::object& j_resp_params);
    IOSourcePtr sck_src;

    char in_msg[4096];
};
