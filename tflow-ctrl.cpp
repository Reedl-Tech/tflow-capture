#include <glib-unix.h>

#include "tflow-ctrl.h"

TFlowCtrl::TFlowCtrl()
{
    

}


int TFlowCtrl::set(tflow_cmd_field_t* cmd_field, Json &cfg_param)
{

    switch (cmd_field->type) {
    case CFT_TXT:
        if (!cfg_param.is_string()) return -1;
        if (cmd_field->v.str) free(cmd_field->v.str);
        cmd_field->v.str = strdup(cfg_param.string_value().data());
        return 0;
    case CFT_NUM:
        if (!cfg_param.is_number()) {
            cmd_field->v.u32 = cfg_param.number_value();
        }
        else if (!cfg_param.is_string()) {
            cmd_field->v.u32 = atoi(cfg_param.string_value().data());
        }
        else if (!cfg_param.is_bool()) {
            cmd_field->v.u32 = cfg_param.bool_value;
        }
        else {
            return -1;
        }
        return 0;
    default:
        g_info("Ooops... at %s (%d) Field name %s", __FILE__, __LINE__,
            cmd_field->name, cfg_param.object[0].first);
    }

    return -1;
}
