#pragma once 

#include <stdint.h>
#include <vector>

#include "tflow-ctrl.h"

#define TFLOW_CMD_EOMSG .eomsg = {.name = nullptr, .type = CFT_LAST, .max_len = 0, .v = {.u32 = 0} }

class TFlowCapture;

static int ctrl_capture_cmd_cb_sign_s(void* ctx, Json& json_cfg);
static int ctrl_capture_cmd_cb_set_as_def_s(void* ctx, Json& json_cfg);
static int ctrl_capture_cmd_cb_config_s(void* ctx, Json& json_cfg);

class TFlowCtrlCapture : private TFlowCtrl {
public:

    TFlowCtrlCapture(TFlowCapture& app);
    void Init();

    
    int dev_name_is_valid();
    char* cam_name_get();
    int cam_fmt_get();

    struct tflow_cmd_flds_sign {
        tflow_cmd_field_t   eomsg;
    } cmd_flds_sign = {
        TFLOW_CMD_EOMSG
    };

    struct tflow_cmd_flds_config {
        tflow_cmd_field_t   state;
        tflow_cmd_field_t   dev_name;
        tflow_cmd_field_t   fmt_enum;   // Enumeration of all camera supported formats. Obtained on camera opening
        tflow_cmd_field_t   fmt_idx;    // The index of the currently used format from the fmt_enum
        tflow_cmd_field_t   eomsg;
    } cmd_flds_config = {
        .state    = { "state",    CFT_NUM, 0, {.u32 = 0} },
        .dev_name = { "dev_name", CFT_TXT, 0, {.u32 = 0} },
        .fmt_enum = { "fmt_enum", CFT_NUM, 0, {.u32 = 0} },
        .fmt_idx  = { "fmt_idx",  CFT_NUM, 0, {.u32 = 0} },
        TFLOW_CMD_EOMSG
    };

    struct tflow_cmd_flds_set_as_def {
        tflow_cmd_field_t   eomsg;
    } cmd_flds_set_as_def = {
        TFLOW_CMD_EOMSG
    };

    // int tflow_cmd_cb_sign(void* ctx);
    int cmd_cb_config(Json& json);
    //int tflow_cmd_cb_set_as_def(void* ctx);

#define TFLOW_CAPTURE_RPC_CMD_SIGN       0
#define TFLOW_CAPTURE_RPC_CMD_CONFIG     1
#define TFLOW_CAPTURE_RPC_CMD_SET_AS_DEF 2
#define TFLOW_CAPTURE_RPC_CMD_LAST       3

    tflow_cmd_t ctrl_capture_rpc_cmds[TFLOW_CAPTURE_RPC_CMD_LAST + 1] = {
        [TFLOW_CAPTURE_RPC_CMD_SIGN      ] = { "sign",        (tflow_cmd_field_t*)&cmd_flds_sign,    &ctrl_capture_cmd_cb_sign_s },
        [TFLOW_CAPTURE_RPC_CMD_CONFIG    ] = { "config",      (tflow_cmd_field_t*)&cmd_flds_config,  &ctrl_capture_cmd_cb_config_s },
        [TFLOW_CAPTURE_RPC_CMD_SET_AS_DEF] = { "set_as_def",  (tflow_cmd_field_t*)&cmd_flds_set_as_def, &ctrl_capture_cmd_cb_set_as_def_s },
        [TFLOW_CAPTURE_RPC_CMD_LAST] = { nullptr , nullptr, nullptr }
    };

private:

    TFlowCapture& app;      // AV: Why?

    const char* cfg_fname = "tflow-capture-config.json";
};
