#pragma once

#include <cassert>
#include <time.h>

#include <glib-unix.h>
#include <json11.hpp>

#include "tflow-common.h"
#include "tflow-glib.hpp"

#include "tflow-ctrl-cli-port.h" 

class TFlowCtrlSrv {
public:

    TFlowCtrlSrv(const std::string &my_name, const std::string & srv_sck_name, MainContextPtr context);
    ~TFlowCtrlSrv();
    int StartListening();
    void onIdle(struct timespec* now_ts);

    virtual int onCliPortConnect(int fd) { return 0; };
    virtual void onCliPortError(int fd) {};

    virtual void onSignature(json11::Json::object& j_params, int& err) {};
    virtual void onTFlowCtrlMsg(const std::string& cmd, const json11::Json& j_in_params, json11::Json::object& j_out_params, int &err) {};
    
    MainContextPtr context;
    std::string my_name;
    struct timespec last_idle_check_ts;

private:
    std::string ctrl_srv_name;

    char* sck_name;
    int sck_fd = -1;
    Flag sck_state_flag;

    IOSourcePtr sck_src;

    gboolean onConnect(Glib::IOCondition io_cond);
};
