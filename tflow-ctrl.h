#pragma once 

#include <stdint.h>
#include <vector>
#include <json11.hpp>

using namespace json11;

#define TFLOW_CMD_EOMSG .eomsg = {.name = nullptr, .type = CFT_LAST, .max_len = 0, .v = {.u32 = 0} }

class TFlowCtrl {
public:

    TFlowCtrl();

    typedef enum {
        CFT_NUM,
        CFT_TXT,
        CFT_RESERVED = 4,
        CFT_LAST = -1
    } tflow_cmd_field_type_t;

    typedef struct {
        const char* name;
        tflow_cmd_field_type_t type;

        int max_len;
        union {
            uint32_t u32;
            char* str;
        } v;
    } tflow_cmd_field_t;

    typedef struct {
        const char* name;
        tflow_cmd_field_t* fields;
        int (*cb)(void *ctx, Json &in_params);
    } tflow_cmd_t;

    int set_cmd_fields(tflow_cmd_field_t* cmd_field, Json& in_params);
private:
    int set_field(tflow_cmd_field_t *cmd_field, Json& in_param);
};

