#pragma once

#include <cassert>
#include <time.h>

#include <glib-unix.h>
#include <json11.hpp>

#include "tflow-common.h"
#include "tflow-ctrl-cli-port.h" 

class TFlowCtrlSrv {
public:

    TFlowCtrlSrv(const std::string &my_name, const std::string & srv_sck_name, GMainContext* context);
    ~TFlowCtrlSrv();
    int StartListening();
    void onIdle(struct timespec* now_tp);

    virtual int onCliPortConnect(int fd) { return 0; };
    virtual void onCliPortError(int fd) {};

    virtual void onSignature(json11::Json::object& j_params, int& err) {};
    virtual void onTFlowCtrlMsg(const std::string& cmd, const json11::Json& j_in_params, json11::Json::object& j_out_params, int &err) {};
    
    GMainContext* context;
    std::string my_name;
    struct timespec last_idle_check_tp = { 0 };

private:
    std::string ctrl_srv_name;

    char* sck_name;
    int sck_fd = -1;
    Flag sck_state_flag;

    typedef struct {
        GSource g_source;
        TFlowCtrlSrv* srv;
    } GSourceSrv;

    GSourceSrv* sck_src;
    gpointer sck_tag;
    GSourceFuncs sck_gsfuncs;

    static gboolean tflow_ctrl_srv_dispatch(GSource* g_source, GSourceFunc callback, gpointer user_data);
    void onConnect();
};
