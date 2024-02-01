#pragma once 

#include <stdint.h>
#include <vector>
#include <unordered_map>
#include <functional>

#include <json11.hpp>
using namespace json11;

#include "tflow-ctrl.h"
#include "tflow-ctrl-srv.h"

class TFlowCtrlCapture;
/*
static int _cmd_cb_sign      (TFlowCtrlCapture* obj, Json& json);
static int _cmd_cb_config    (TFlowCtrlCapture* obj, Json& json);
static int _cmd_cb_set_as_def(TFlowCtrlCapture* obj, Json& json);
*/
class TFlowCapture;

//class TFlowCtrlPortCapture : public TFlowCtrlCliPort {
//public:
//    TFlowCtrlSrvCapture& srv;
//
//    TFlowCtrlPortCapture(TFlowCtrlSrvCapture& _srv, GMainContext* context, int fd);
//
//};

class TFlowCtrlSrvCapture : public TFlowCtrlSrv {
public:
    TFlowCtrlSrvCapture(TFlowCtrlCapture &_ctrl_capture, GMainContext* context);
    int onCliPortConnect(int fd) override;
    void onCliPortError(int fd) override;

    void onSignature(Json::object& j_out_params, int &err) override;
    void onTFlowCtrlMsg(const std::string& cmd, const json11::Json& j_in_params, Json::object& j_out_params, int &err) override;

private:
    TFlowCtrlCapture &ctrl_capture;

    std::unordered_map<int, TFlowCtrlCliPort> ctrl_clis;
};


class TFlowCtrlCapture : private TFlowCtrl {
public:

    TFlowCtrlCapture(TFlowCapture& app);

    TFlowCapture& app;      // AV: For access to context 

    void InitConfig();
    void InitServer();

    int dev_name_is_valid();
    char* cam_name_get();
    int cam_fmt_get();

    struct tflow_cmd_flds_sign {
        tflow_cmd_field_t   eomsg;
    } cmd_flds_version = {
        TFLOW_CMD_EOMSG
    };

    struct tflow_cmd_flds_config {
        tflow_cmd_field_t   state;
        tflow_cmd_field_t   buffs_num;
        tflow_cmd_field_t   dev_name;
        tflow_cmd_field_t   fmt_enum;   // Enumeration of all camera supported formats. Obtained on camera opening
        tflow_cmd_field_t   fmt_idx;    // The index of the currently used format from the fmt_enum
        tflow_cmd_field_t   eomsg;
    } cmd_flds_config = {
        .state     = { "state",      CFT_NUM, 0, {.num = 0} },
        .buffs_num = { "buffs_num",  CFT_NUM, 0, {.num = 0} },
        .dev_name  = { "dev_name",   CFT_STR, 0, {.num = 0} },
        .fmt_enum  = { "fmt_enum",   CFT_NUM, 0, {.num = 0} },
        .fmt_idx   = { "fmt_idx",    CFT_NUM, 0, {.num = 0} },
        TFLOW_CMD_EOMSG
    };

    struct tflow_cmd_flds_set_as_def {
        tflow_cmd_field_t   eomsg;
    } cmd_flds_set_as_def = {
        TFLOW_CMD_EOMSG
    };

    //TFlowCtrlOnCmd cmd_cb_version;
    int cmd_cb_version   (const json11::Json& j_in_params, Json::object& j_out_params);
    int cmd_cb_config    (const json11::Json& j_in_params, Json::object& j_out_params);
    int cmd_cb_set_as_def(const json11::Json& j_in_params, Json::object& j_out_params);

#define TFLOW_CAPTURE_RPC_CMD_VERSION    0
#define TFLOW_CAPTURE_RPC_CMD_CONFIG     1
#define TFLOW_CAPTURE_RPC_CMD_SET_AS_DEF 2
#define TFLOW_CAPTURE_RPC_CMD_LAST       3

    typedef struct {
        const char* name;
        tflow_cmd_field_t* fields;
        //int (*cb)(TFlowCtrlCapture* obj, Json& in_params);
        //int (TFlowCtrlCapture::*cb)(Json& in_params);
        std::function<int(Json& json, Json::object& j_out_params)> cb;
    } tflow_cmd_t;

    //#define THIS_M(_f) std::bind(&TFlowCtrlCapture::_f, this, std::placeholders::_1,)
    #define THIS_M(_f) std::bind(&TFlowCtrlCapture::_f, this, std::placeholders::_1, std::placeholders::_2)

    tflow_cmd_t ctrl_capture_rpc_cmds[TFLOW_CAPTURE_RPC_CMD_LAST + 1] = {
        [TFLOW_CAPTURE_RPC_CMD_VERSION   ] = { "version",    (tflow_cmd_field_t*)&cmd_flds_version    /*, _cmd_cb_sign      */ , THIS_M(cmd_cb_version)   },
        [TFLOW_CAPTURE_RPC_CMD_CONFIG    ] = { "config",     (tflow_cmd_field_t*)&cmd_flds_config     /*, _cmd_cb_config    */ , THIS_M(cmd_cb_config)    },
        [TFLOW_CAPTURE_RPC_CMD_SET_AS_DEF] = { "set_as_def", (tflow_cmd_field_t*)&cmd_flds_set_as_def /*, _cmd_cb_set_as_def*/ , THIS_M(cmd_cb_set_as_def)},
        [TFLOW_CAPTURE_RPC_CMD_LAST] = { nullptr , nullptr, nullptr }
    };
    //tflow_cmd_t ctrl_capture_rpc_cmds[TFLOW_CAPTURE_RPC_CMD_LAST + 1] = {
    //    [TFLOW_CAPTURE_RPC_CMD_SIGN] = { "sign",        (tflow_cmd_field_t*)&cmd_flds_sign       , _cmd_cb_sign       },
    //    [TFLOW_CAPTURE_RPC_CMD_CONFIG] = { "config",      (tflow_cmd_field_t*)&cmd_flds_config     , _cmd_cb_config     },
    //    [TFLOW_CAPTURE_RPC_CMD_SET_AS_DEF] = { "set_as_def",  (tflow_cmd_field_t*)&cmd_flds_set_as_def , _cmd_cb_set_as_def },
    //    [TFLOW_CAPTURE_RPC_CMD_LAST] = { nullptr , nullptr, nullptr }
    //};

    void getSignResponse(Json::object& j_cmd_obj);

    TFlowCtrlSrvCapture ctrl_srv;
private:


    const char* cfg_fname = "tflow-capture-config.json";

    void getCmdInfo(tflow_cmd_t* cmd, Json::object& j_cmd_info);

};

