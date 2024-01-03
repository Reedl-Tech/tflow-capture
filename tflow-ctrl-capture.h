#pragma once 

#include <stdint.h>
#include <vector>

#include "tflow-ctrl.h"

#define TFLOW_CMD_EOMSG .eomsg = {.name = nullptr, .type = CFT_LAST, .max_len = 0, .v = {.u32 = 0} }

class TFlowCtrlCapture : private TFlowCtrl {
public:

    TFlowCtrlCapture(TFlowCapture& app);
    void Init();

    
    int cam_name_is_valid();
    char* cam_name_get();

    struct tflow_cmd_flds_sign {
        tflow_cmd_field_t   eomsg;
    } tflow_cmd_flds_sign = {
        TFLOW_CMD_EOMSG
    };

    struct tflow_cmd_flds_config {
        tflow_cmd_field_t   state;
        tflow_cmd_field_t   cam_name;
        tflow_cmd_field_t   eomsg;
    } cmd_flds_config = {
        .state    = { "state", CFT_NUM, 0, {.u32 = 0} },
        .cam_name = { "cam_name",  CFT_TXT, 0, {.u32 = 0} },
        TFLOW_CMD_EOMSG
    };

    struct tflow_cmd_flds_set_as_def {
        tflow_cmd_field_t   eomsg;
    } tflow_cmd_flds_set_as_def = {
        TFLOW_CMD_EOMSG
    };

    int tflow_cmd_cb_sign(void* ctx);
    int tflow_cmd_cb_config(void* ctx);
    int tflow_cmd_cb_set_as_def(void* ctx);

#define TFLOW_CAPTURE_RPC_CMD_SIGN       0
#define TFLOW_CAPTURE_RPC_CMD_CONFIG     1
#define TFLOW_CAPTURE_RPC_CMD_SET_AS_DEF 2
#define TFLOW_CAPTURE_RPC_CMD_LAST       3

    tflow_cmd_t tflow_rpc_cmds[4] = {
            [TFLOW_CAPTURE_RPC_CMD_SIGN      ] = { "sign",        (tflow_cmd_field_t*)&tflow_cmd_flds_sign,       &TFlowCtrlCapture::tflow_cmd_cb_sign       },
            [TFLOW_CAPTURE_RPC_CMD_CONFIG    ] = { "config",      (tflow_cmd_field_t*)&tflow_cmd_flds_config,     &TFlowCtrlCapture::tflow_cmd_cb_config     },
            [TFLOW_CAPTURE_RPC_CMD_SET_AS_DEF] = { "set_as_def",  (tflow_cmd_field_t*)&tflow_cmd_flds_set_as_def, &TFlowCtrlCapture::tflow_cmd_cb_set_as_def },
            [TFLOW_CAPTURE_RPC_CMD_LAST] = { 0, 0 }
    };


private:

    TFlowCapture& app;
    const char* cfg_fname = "tflow-capture-config.json";
};
