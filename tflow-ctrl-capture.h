#pragma once 

#include <stdint.h>
#include <vector>

#include "tflow-ctrl.h"

#define TFLOW_CMD_EOMSG .eomsg = {.name = nullptr, .type = CFT_LAST, .max_len = 0, .v = {.u32 = 0} }

class TFlowCtrlCapture;
static int _cmd_cb_sign      (TFlowCtrlCapture* obj, Json& json);
static int _cmd_cb_config    (TFlowCtrlCapture* obj, Json& json);
static int _cmd_cb_set_as_def(TFlowCtrlCapture* obj, Json& json);

class TFlowCapture;
class TFlowCtrlCapture : private TFlowCtrl<TFlowCtrlCapture> {
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
        tflow_cmd_field_t   buffs_num;
        tflow_cmd_field_t   dev_name;
        tflow_cmd_field_t   fmt_enum;   // Enumeration of all camera supported formats. Obtained on camera opening
        tflow_cmd_field_t   fmt_idx;    // The index of the currently used format from the fmt_enum
        tflow_cmd_field_t   eomsg;
    } cmd_flds_config = {
        .state      = { "state",      CFT_NUM, 0, {.u32 = 0} },
        .buffs_num  = { "buffs_num",  CFT_NUM, 0, {.u32 = 0} },
        .dev_name   = { "dev_name",   CFT_TXT, 0, {.u32 = 0} },
        .fmt_enum   = { "fmt_enum",   CFT_NUM, 0, {.u32 = 0} },
        .fmt_idx    = { "fmt_idx",    CFT_NUM, 0, {.u32 = 0} },
        TFLOW_CMD_EOMSG
    };

    struct tflow_cmd_flds_set_as_def {
        tflow_cmd_field_t   eomsg;
    } cmd_flds_set_as_def = {
        TFLOW_CMD_EOMSG
    };

    int cmd_cb_sign(Json& json);
    int cmd_cb_config(Json& json);
    int cmd_cb_set_as_def(Json& json);

#define TFLOW_CAPTURE_RPC_CMD_SIGN       0
#define TFLOW_CAPTURE_RPC_CMD_CONFIG     1
#define TFLOW_CAPTURE_RPC_CMD_SET_AS_DEF 2
#define TFLOW_CAPTURE_RPC_CMD_LAST       3

    //typedef struct {
    //    tflow_cmd_t cmd;
    //} tflow_cmd_capture_t ;

    tflow_cmd_t ctrl_capture_rpc_cmds[TFLOW_CAPTURE_RPC_CMD_LAST + 1] = {
        [TFLOW_CAPTURE_RPC_CMD_SIGN      ] = { "sign",        (tflow_cmd_field_t*)&cmd_flds_sign       , _cmd_cb_sign       },
        [TFLOW_CAPTURE_RPC_CMD_CONFIG    ] = { "config",      (tflow_cmd_field_t*)&cmd_flds_config     , _cmd_cb_config     },
        [TFLOW_CAPTURE_RPC_CMD_SET_AS_DEF] = { "set_as_def",  (tflow_cmd_field_t*)&cmd_flds_set_as_def , _cmd_cb_set_as_def },
        [TFLOW_CAPTURE_RPC_CMD_LAST] = { nullptr , nullptr, nullptr }
    };

private:

    TFlowCapture& app;      // AV: Why?

    const char* cfg_fname = "tflow-capture-config.json";
};
