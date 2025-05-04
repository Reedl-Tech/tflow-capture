#pragma once 
#include <stdint.h>
#include <vector>
#include <unordered_map>

#include "tflow-ctrl.hpp"
#include "tflow-ctrl-srv.hpp"

class TFlowCtrlCapture;
class TFlowCapture;
struct fmt_info;

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

    TFlowCtrlCapture(TFlowCapture& app, const std::string _cfg_fname);

    TFlowCapture& app;      // AV: For access to context. Passed to CtrlServer

    void InitServer();

    //int player_fname_is_valid();
    //char* player_fname_get() { return player_fname_is_valid() ? cmd_flds_config.player_fname.v.str : nullptr; }
    //int player_fmt_get() { return 0; }  // Must be obtained from a input file

    int dev_name_is_valid();
    int sub_dev_name_is_valid();
    int cam_fmt_get()    { return cmd_flds_config.fmt_idx.v.num; }

    void cam_fmt_enum_get(std::vector<struct fmt_info> &cfg_fmt_enum);

    int serial_name_is_valid();
    char* serial_name_get() { return serial_name_is_valid() ? cmd_flds_config.serial_name.v.str : nullptr; }
    int serial_baud_get()   { return cmd_flds_config.serial_baud.v.num; }

    struct tflow_cmd_flds_sign {
        tflow_cmd_field_t   eomsg;
    } cmd_flds_version = {
        TFLOW_CMD_EOMSG
    };

    struct cfg_v4l2_ctrls_flyn {
        tflow_cmd_field_t   head;
        tflow_cmd_field_t   contrast;
        tflow_cmd_field_t   brightness;
        tflow_cmd_field_t   filter;
        tflow_cmd_field_t   denoise;
        tflow_cmd_field_t   gain;
        tflow_cmd_field_t   calib;
        tflow_cmd_field_t   calib_trig;
        tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_v4l2_ctrls_flyn = {
        .head       = { "v4l2-ctrls-flyn", CFT_STR, 0, {.str = nullptr} },
        .contrast   = { "contrast",        CFT_NUM, 0, {.num = 100} },
        .brightness = { "brightness",      CFT_NUM, 0, {.num = 8} },
        .filter     = { "filter",          CFT_NUM, 0, {.num = 0} },
        .denoise    = { "denoise",         CFT_NUM, 0, {.num = 0} },
        .gain       = { "gain",            CFT_NUM, 0, {.num = 0} },
        .calib      = { "calib",           CFT_NUM, 0, {.num = 0} },
        .calib_trig = { "calib_trig",      CFT_NUM, 0, {.num = 0} },
         TFLOW_CMD_EOMSG
     };

    struct cfg_v4l2_ctrls_atic {
        tflow_cmd_field_t   head;
        tflow_cmd_field_t   xz;
        tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_v4l2_ctrls_atic = {
        .head = { "v4l2-ctrls-atic", CFT_STR, 0, {.str = nullptr} },
        .xz   = { "xz",              CFT_NUM, 0, {.num = 0} },
         TFLOW_CMD_EOMSG
    };

    struct cfg_v4l2_ctrls {
        tflow_cmd_field_t   head;
        tflow_cmd_field_t   dev_name;       // Camera ISI device name. For ex. /dev/video0
        tflow_cmd_field_t   sub_dev_name;   // Camera sensor device name. For ex. /dev/v4l-subdev1 

        /* ISI common */
        tflow_cmd_field_t   vflip;
        tflow_cmd_field_t   hflip;

        /* ATIC specific */
        tflow_cmd_field_t   flyn384;

        /* ATIC specific */
        tflow_cmd_field_t   atic320;

        tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_v4l2 = {
        .head          = { "v4l2",         CFT_STR, 0, {.str = nullptr} },
        .dev_name      = { "dev_name",     CFT_STR, 0, {.str = nullptr} },  // TODO: Add auto get by name "ISI"
        .sub_dev_name  = { "sub_dev_name", CFT_STR, 0, {.str = nullptr} },  // TODO: Add auto get by known name FLYN/ATIC
        /* Common (ISI) */
        .vflip   = { "vflip",    CFT_NUM, 0, {.num = 0} },
        .hflip   = { "hflip",    CFT_NUM, 0, {.num = 0} },
        .flyn384 = { "flyn384",  CFT_REF, 0, {.ref = &( cmd_flds_cfg_v4l2_ctrls_flyn.head )} },
        .atic320 = { "atic320",  CFT_REF, 0, {.ref = &( cmd_flds_cfg_v4l2_ctrls_atic.head )} },
        TFLOW_CMD_EOMSG
    };

    struct tflow_cmd_flds_config {
        tflow_cmd_field_t   state;
        tflow_cmd_field_t   buffs_num;
//        tflow_cmd_field_t   player_fname;   // File name of media file then in player mode
        tflow_cmd_field_t   fmt_enum;       // Enumeration of all camera supported formats. Obtained on camera opening
        tflow_cmd_field_t   fmt_idx;        // The index of the currently used format from the fmt_enum
        tflow_cmd_field_t   preffered_w;    // Prefered width/height selected by a user in case of 
        tflow_cmd_field_t   preffered_h;    //      STEPWISE and CONTINUOS frame size
        tflow_cmd_field_t   serial_name;
        tflow_cmd_field_t   serial_baud;
        tflow_cmd_field_t   v4l2;
        tflow_cmd_field_t   eomsg;
    } cmd_flds_config = {
        .state        = { "state",        CFT_NUM, 0, {.num = 0} },
        .buffs_num    = { "buffs_num",    CFT_NUM, 0, {.num = 0} },
//        .player_fname = { "player_fname", CFT_STR, 0, {.str = nullptr} },
        .fmt_enum     = { "fmt_enum",     CFT_STR, 0, {.str = nullptr} },     // WxH CCCC, ...
        .fmt_idx      = { "fmt_idx",      CFT_NUM, 0, {.num = -1} },          // Invalidate index
        .preffered_w  = { "pref_width",   CFT_NUM, 0, {.num = -1} },          // 
        .preffered_h  = { "pref_height",  CFT_NUM, 0, {.num = -1} },          // 
        .serial_name  = { "serial_name",  CFT_STR, 0, {.str = nullptr} },
        .serial_baud  = { "serial_baud",  CFT_NUM, 0, {.num = 0} },
        .v4l2         = { "v4l2",         CFT_REF, 0, {.ref = &(cmd_flds_cfg_v4l2.head)} },
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
        ARRAY_INIT_IDX(TFLOW_CAPTURE_RPC_CMD_VERSION   ) { "version",    (tflow_cmd_field_t*)&cmd_flds_version   , THIS_M(&TFlowCtrlCapture::cmd_cb_version)   },
        ARRAY_INIT_IDX(TFLOW_CAPTURE_RPC_CMD_CONFIG    ) { "capture",    (tflow_cmd_field_t*)&cmd_flds_config    , THIS_M(&TFlowCtrlCapture::cmd_cb_config)    },
        ARRAY_INIT_IDX(TFLOW_CAPTURE_RPC_CMD_SET_AS_DEF) { "set_as_def", (tflow_cmd_field_t*)&cmd_flds_set_as_def, THIS_M(&TFlowCtrlCapture::cmd_cb_set_as_def)},
        ARRAY_INIT_IDX(TFLOW_CAPTURE_RPC_CMD_LAST) { nullptr , nullptr, nullptr }
    };

    TFlowCtrlSrvCapture ctrl_srv;

    void getSignResponse(json11::Json::object &j_params);

private:

    int collectCtrls(const tflow_cmd_field_t *cmd_fld, json11::Json::array &j_out_params);
    int collectCtrlsCustom(const tflow_cmd_field_t *cmd_fld, json11::Json::array &j_out_params);
    const std::string cfg_fname;
};

