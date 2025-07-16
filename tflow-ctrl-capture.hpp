#pragma once 
#include <stdint.h>
#include <vector>
#include <unordered_map>

#include "tflow-ctrl.hpp"

#include "tflow-ctrl-srv.hpp"

#include "tflow-ctrl-capture-ui.hpp"

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

//extern class TFlowCtrlUI ctrl_ui;
//TFlowCtrlProcessUI
class TFlowCtrlCapture : private TFlowCtrlCaptureUI, private TFlowCtrl {
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

    struct cfg_v4l2_ctrls_flip {
        tflow_cmd_field_t   head;
        tflow_cmd_field_t   hflip;
        tflow_cmd_field_t   vflip;
        tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_v4l2_ctrls_flip = {
        TFLOW_CMD_HEAD("flip"),
        .hflip = { "hflip",  CFT_NUM, 0, {.num =       0}},
        .vflip = { "vflip",  CFT_NUM, 0, {.num =       0}},
         TFLOW_CMD_EOMSG
     };

    struct cfg_v4l2_ctrls_flyn_testpatt {
        tflow_cmd_field_t   head;
        tflow_cmd_field_t   gst_testpatt;
        tflow_cmd_field_t   mipi_testpatt;
        tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_v4l2_ctrls_flyn_testpatt = {
        // Test pattern initialized to disabled by default (GST=Live, MIPI=0). 
        // Not recommended for configuration vi config file.
        TFLOW_CMD_HEAD("testpatt"),
        .gst_testpatt  = { "gst_testpatt",  CFT_STR, 0, {.str = nullptr}, &ui_gst_testpatt },
        .mipi_testpatt = { "mipi_testpatt", CFT_NUM, 0, {.num =       0}, &ui_mipi_testpatt},
         TFLOW_CMD_EOMSG
     };

    struct cfg_v4l2_ctrls_flyn_compression {
        tflow_cmd_field_t   head;
        tflow_cmd_field_t   compr_en;
        tflow_cmd_field_t   histogramm_en;
        tflow_cmd_field_t   level;
        tflow_cmd_field_t   reset;
        tflow_cmd_field_t   p1x;
        tflow_cmd_field_t   p1y;
        tflow_cmd_field_t   p2x;
        tflow_cmd_field_t   p2y;
        tflow_cmd_field_t   p3x;
        tflow_cmd_field_t   p3y;
        tflow_cmd_field_t   p4x;
        tflow_cmd_field_t   p4y;
        tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_v4l2_ctrls_flyn_compression = {
        // Test pattern initialized to disabled by default (GST=Live, MIPI=0). 
        // Not recommended for configuration via config file.
        TFLOW_CMD_HEAD("compression"),
        .compr_en      = { "compr_en",       CFT_NUM, 0, {.num =      0}, &ui_switch_def },
        .histogramm_en = { "histogramm_en",  CFT_NUM, 0, {.num =      0}, &ui_switch_def },
        .level         = { "level",          CFT_STR, 0, {.str = strdup(ui_dd_compression_level.dropdown.val[0]) }, &ui_dd_compression_level},
        .reset         = { "reset",          CFT_NUM, 0, {.num =      0}, &ui_butt_def   },
        .p1x           = { "p1x",            CFT_NUM, 0, {.num =   1000}, &ui_edit_def   },
        .p1y           = { "p1y",            CFT_NUM, 0, {.num =     50}, &ui_edit_def   },
        .p2x           = { "p2x",            CFT_NUM, 0, {.num =   3000}, &ui_edit_def   },
        .p2y           = { "p2y",            CFT_NUM, 0, {.num =    100}, &ui_edit_def   },
        .p3x           = { "p3x",            CFT_NUM, 0, {.num =   5000}, &ui_edit_def   },
        .p3y           = { "p3y",            CFT_NUM, 0, {.num =    150}, &ui_edit_def   },
        .p4x           = { "p4x",            CFT_NUM, 0, {.num =  10000}, &ui_edit_def   },
        .p4y           = { "p4y",            CFT_NUM, 0, {.num =    200}, &ui_edit_def   },
         TFLOW_CMD_EOMSG
     };

    struct cfg_v4l2_ctrls_flyn {
        tflow_cmd_field_t   head;
        tflow_cmd_field_t   compression;
        tflow_cmd_field_t   test_patt;
        tflow_cmd_field_t   contrast;
        tflow_cmd_field_t   brightness;
        tflow_cmd_field_t   filter;
        tflow_cmd_field_t   denoise;
        tflow_cmd_field_t   gain;
        tflow_cmd_field_t   calib;
        tflow_cmd_field_t   calib_trig;

        tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_v4l2_ctrls_flyn = {
        TFLOW_CMD_HEAD("FLYN384"),
        .compression = { "compression", CFT_REF, 0, {.ref = &(cmd_flds_cfg_v4l2_ctrls_flyn_compression.head) }, &ui_custom_flyn_compression},
        .test_patt   = { "testpatt",    CFT_REF, 0, {.ref = &(cmd_flds_cfg_v4l2_ctrls_flyn_testpatt.head)    }, &ui_custom_flyn_testpatt},
        .contrast    = { "contrast",    CFT_NUM, 0, {.num = 100}    , &ui_flyn_contrast  },
        .brightness  = { "brightness",  CFT_NUM, 0, {.num =   8}    , &ui_flyn_brightness},
        .filter      = { "filter",      CFT_NUM, 0, {.num =   0}    , &ui_flyn_filter},
        .denoise     = { "denoise",     CFT_NUM, 0, {.num =   0}    , nullptr},
        .gain        = { "gain",        CFT_NUM, 0, {.num =   0}    , &ui_flyn_gain},
        .calib       = { "calib",       CFT_NUM, 0, {.num =   0}    , &ui_ll_edit_def},
        .calib_trig  = { "calib_trig",  CFT_NUM, 0, {.num =   0}    , &ui_butt_def},
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
        tflow_cmd_field_t   flip;

        /* ATIC specific */
        tflow_cmd_field_t   flyn384;

        /* ATIC specific */
        tflow_cmd_field_t   atic320;

        tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_v4l2 = {
        .head          = { "v4l2",         CFT_STR, 0, {.str = nullptr} },
        .dev_name      = { "dev_name",     CFT_STR, 0, {.str = nullptr} },
        .sub_dev_name  = { "sub_dev_name", CFT_STR, 0, {.str = nullptr} },
        /* Common (ISI) */
        .flip    = { "flip",     CFT_REF, 0, {.ref = &( cmd_flds_cfg_v4l2_ctrls_flip.head )}, &ui_custom_flip},
        .flyn384 = { "flyn384",  CFT_REF, 0, {.ref = &( cmd_flds_cfg_v4l2_ctrls_flyn.head )}, &ui_group_def},
        .atic320 = { "atic320",  CFT_REF, 0, {.ref = &( cmd_flds_cfg_v4l2_ctrls_atic.head )} },
        TFLOW_CMD_EOMSG
    };

    struct tflow_cmd_flds_config {
        tflow_cmd_field_t   state;
        tflow_cmd_field_t   buffs_num;
        tflow_cmd_field_t   fmt_enum;       // Enumeration of all camera supported formats. Obtained on camera opening
        tflow_cmd_field_t   fmt_idx;        // The index of the currently used format from the fmt_enum
        tflow_cmd_field_t   preffered_w;    // Prefered width/height selected by a user in case of 
        tflow_cmd_field_t   preffered_h;    // STEPWISE and CONTINUOS frame size
        tflow_cmd_field_t   serial_name;
        tflow_cmd_field_t   serial_baud;
        tflow_cmd_field_t   v4l2;
        tflow_cmd_field_t   eomsg;
    } cmd_flds_config = {
        .state        = { "state",        CFT_NUM, 0, {.num = 0}       },
        .buffs_num    = { "buffs_num",    CFT_NUM, 0, {.num = 0},                         &ui_ll_edit_def},
        .fmt_enum     = { "fmt_enum",     CFT_STR, 0, {.str = nullptr} },     // WxH CCCC, ...
        .fmt_idx      = { "fmt_idx",      CFT_NUM, 0, {.num = -1}      },     // Invalidate index
        .preffered_w  = { "pref_width",   CFT_NUM, 0, {.num = -1}      },     // 
        .preffered_h  = { "pref_height",  CFT_NUM, 0, {.num = -1}      },     // 
        .serial_name  = { "serial_name",  CFT_STR, 0, {.str = nullptr},                   &ui_edit_def    },
        .serial_baud  = { "serial_baud",  CFT_NUM, 0, {.num = 0},                         &ui_serial_baud },
        .v4l2         = { "v4l2",         CFT_REF, 0, {.ref = &(cmd_flds_cfg_v4l2.head)}, &ui_group_def   },
        TFLOW_CMD_EOMSG
    };

    struct tflow_cmd_flds_set_as_def {
        tflow_cmd_field_t   eomsg;
    } cmd_flds_set_as_def = {
        TFLOW_CMD_EOMSG
    };

    int onConfigV4L2();
    int onConfigFLYN();
    int onConfigATIC();

    int cmd_cb_version   (const json11::Json& j_in_params, json11::Json::object& j_out_params);
    int cmd_cb_ui_sign   (const json11::Json& j_in_params, json11::Json::object& j_out_params);
    int cmd_cb_config    (const json11::Json& j_in_params, json11::Json::object& j_out_params);
    int cmd_cb_set_as_def(const json11::Json& j_in_params, json11::Json::object& j_out_params);

#define TFLOW_CAPTURE_RPC_CMD_VERSION    0
#define TFLOW_CAPTURE_RPC_CMD_CONTROLS   1
#define TFLOW_CAPTURE_RPC_CMD_SET_AS_DEF 2
#define TFLOW_CAPTURE_RPC_CMD_CONFIG     3
#define TFLOW_CAPTURE_RPC_CMD_LAST       4

    tflow_cmd_t ctrl_capture_rpc_cmds[TFLOW_CAPTURE_RPC_CMD_LAST + 1] = {
        ARRAY_INIT_IDX(TFLOW_CAPTURE_RPC_CMD_VERSION   ) { "version",    (tflow_cmd_field_t*)&cmd_flds_version   , THIS_M(&TFlowCtrlCapture::cmd_cb_version)   },
        ARRAY_INIT_IDX(TFLOW_CAPTURE_RPC_CMD_CONTROLS  ) { "ui_sign",    (tflow_cmd_field_t*)&cmd_flds_config    , THIS_M(&TFlowCtrlCapture::cmd_cb_ui_sign)   },
        ARRAY_INIT_IDX(TFLOW_CAPTURE_RPC_CMD_SET_AS_DEF) { "set_as_def", (tflow_cmd_field_t*)&cmd_flds_set_as_def, THIS_M(&TFlowCtrlCapture::cmd_cb_set_as_def)},
        ARRAY_INIT_IDX(TFLOW_CAPTURE_RPC_CMD_CONFIG    ) { "config",     (tflow_cmd_field_t*)&cmd_flds_config    , THIS_M(&TFlowCtrlCapture::cmd_cb_config)    },
        ARRAY_INIT_IDX(TFLOW_CAPTURE_RPC_CMD_LAST) { nullptr , nullptr, nullptr }
    };

    TFlowCtrlSrvCapture ctrl_srv;

    void getSignResponse(json11::Json::object &j_params);

private:

    void collectCtrlsCustom(UICTRL_TYPE type, const char *fld_name, 
        const tflow_cmd_field_t *cmd_fld, json11::Json::array &j_out_params) override;

    const std::string cfg_fname;
};

