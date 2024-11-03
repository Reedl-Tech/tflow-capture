#pragma once 
#include <stdint.h>
#include <vector>
#include <unordered_map>

#include "tflow-ctrl.h"
#include "tflow-ctrl-srv.h"

class TFlowCtrlCapture;
class TFlowCapture;

class TFlowCtrlSrvCapture : public TFlowCtrlSrv {
public:
    TFlowCtrlSrvCapture(TFlowCtrlCapture &_ctrl_capture, MainContextPtr context);
    int onCliPortConnect(int fd) override;
    void onCliPortError(int fd) override;

    void onSignature(json11::Json::object& j_out_params, int &err) override;
    void onTFlowCtrlMsg(const std::string& cmd, const json11::Json& j_in_params, json11::Json::object& j_out_params, int &err) override;

private:
    TFlowCtrlCapture &ctrl_capture;

    std::unordered_map<int, TFlowCtrlCliPort> ctrl_clis;
};


class TFlowCtrlCapture : private TFlowCtrl {
public:

    TFlowCtrlCapture(TFlowCapture& app);

    TFlowCapture& app;      // AV: For access to context. Passed to CtrlServer

    int parseConfig(tflow_cmd_t* config_cmd, const std::string& cfg_fname, const std::string& raw_cfg_default);
    void InitServer();

    int player_fname_is_valid();
    char* player_fname_get() { return player_fname_is_valid() ? cmd_flds_config.player_fname.v.str : nullptr; }
    int player_fmt_get() { return 0; }  // Must be obtained from a input file

    int dev_name_is_valid();
    char* cam_name_get() { return dev_name_is_valid() ? cmd_flds_config.dev_name.v.str : nullptr; }
    int cam_fmt_get()    { return cmd_flds_config.fmt_idx.v.num; }

    int serial_name_is_valid();
    char* serial_name_get() { return serial_name_is_valid() ? cmd_flds_config.serial_name.v.str : nullptr; }
    int serial_baud_get()   { return cmd_flds_config.serial_baud.v.num; }

    struct tflow_cmd_flds_sign {
        tflow_cmd_field_t   eomsg;
    } cmd_flds_version = {
        TFLOW_CMD_EOMSG
    };

    struct cfg_v4l2_ctrls {
        tflow_cmd_field_t   head;
        tflow_cmd_field_t   vflip;
        tflow_cmd_field_t   hflip;
        tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_v4l2_ctrls = {
        .head          = { "v4l2-ctrls",     CFT_STR, 0, {.str = nullptr} },
        .vflip         = { "vflip",          CFT_NUM, 0, {.num = 0} },
        .hflip         = { "hflip",          CFT_NUM, 0, {.num = 0} },
        TFLOW_CMD_EOMSG
    };

    struct tflow_cmd_flds_config {
        tflow_cmd_field_t   state;
        tflow_cmd_field_t   buffs_num;
        tflow_cmd_field_t   player_fname;   // File name of media file then in player mode
        tflow_cmd_field_t   dev_name;       // Camera device name 
        tflow_cmd_field_t   fmt_enum;       // Enumeration of all camera supported formats. Obtained on camera opening
        tflow_cmd_field_t   fmt_idx;        // The index of the currently used format from the fmt_enum
        tflow_cmd_field_t   serial_name;
        tflow_cmd_field_t   serial_baud;
        tflow_cmd_field_t   v4l2_ctrls;
        tflow_cmd_field_t   eomsg;
    } cmd_flds_config = {
        .state        = { "state",        CFT_NUM, 0, {.num = 0} },
        .buffs_num    = { "buffs_num",    CFT_NUM, 0, {.num = 0} },
        .player_fname = { "player_fname", CFT_STR, 0, {.num = 0} },
        .dev_name     = { "dev_name",     CFT_STR, 0, {.num = 0} },
        .fmt_enum     = { "fmt_enum",     CFT_NUM, 0, {.num = 0} },
        .fmt_idx      = { "fmt_idx",      CFT_NUM, 0, {.num = 0} },
        .serial_name  = { "serial_name",  CFT_STR, 0, {.num = 0} },
        .serial_baud  = { "serial_baud",  CFT_NUM, 0, {.num = 0} },
        .v4l2_ctrls   = { "v4l2_ctrls",   CFT_REF, 0, {.ref = &(cmd_flds_cfg_v4l2_ctrls.head)} },
        TFLOW_CMD_EOMSG
    };

    struct tflow_cmd_flds_set_as_def {
        tflow_cmd_field_t   eomsg;
    } cmd_flds_set_as_def = {
        TFLOW_CMD_EOMSG
    };

    int cmd_cb_version   (const json11::Json& j_in_params, json11::Json::object& j_out_params);
    int cmd_cb_config    (const json11::Json& j_in_params, json11::Json::object& j_out_params);
    int cmd_cb_set_as_def(const json11::Json& j_in_params, json11::Json::object& j_out_params);

#define TFLOW_CAPTURE_RPC_CMD_VERSION    0
#define TFLOW_CAPTURE_RPC_CMD_CONFIG     1
#define TFLOW_CAPTURE_RPC_CMD_SET_AS_DEF 2
#define TFLOW_CAPTURE_RPC_CMD_LAST       3

    tflow_cmd_t ctrl_capture_rpc_cmds[TFLOW_CAPTURE_RPC_CMD_LAST + 1] = {
        [TFLOW_CAPTURE_RPC_CMD_VERSION   ] = { "version",    (tflow_cmd_field_t*)&cmd_flds_version   , THIS_M(&TFlowCtrlCapture::cmd_cb_version)   },
        [TFLOW_CAPTURE_RPC_CMD_CONFIG    ] = { "config",     (tflow_cmd_field_t*)&cmd_flds_config    , THIS_M(&TFlowCtrlCapture::cmd_cb_config)    },
        [TFLOW_CAPTURE_RPC_CMD_SET_AS_DEF] = { "set_as_def", (tflow_cmd_field_t*)&cmd_flds_set_as_def, THIS_M(&TFlowCtrlCapture::cmd_cb_set_as_def)},
        [TFLOW_CAPTURE_RPC_CMD_LAST] = { nullptr , nullptr, nullptr }
    };

    TFlowCtrlSrvCapture ctrl_srv;

    static void getSignResponse(const tflow_cmd_t* cmd_p, json11::Json::object& j_params) { TFlowCtrl::getSignResponse(cmd_p, j_params); }

private:

    const std::string cfg_fname{ "tflow-capture-config.json" };
};

