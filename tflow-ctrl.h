#pragma once 

#include <stdint.h>
#include <string.h>
#include <vector>
#include <functional>

#include <json11.hpp>
 
#include <glib-unix.h>

#define TFLOW_CMD_EOMSG .eomsg = {.name = nullptr, .type = CFT_LAST, .max_len = 0, .v = {.num = 0} }

#define THIS_M(_f) std::bind(_f, this, std::placeholders::_1, std::placeholders::_2)

class TFlowCtrl {
public:

    TFlowCtrl();

    typedef enum {
        CFT_NUM,
        CFT_STR,
        CFT_DBL,
        CFT_RESERVED = 4,
        CFT_LAST = -1
    } tflow_cmd_field_type_t;

    typedef struct {
        const char* name;
        tflow_cmd_field_type_t type;

        int max_len;
        union {
            int    num;
            char*  str;
            double dbl;
        } v;
    } tflow_cmd_field_t;

    typedef struct {
        const char* name;
        tflow_cmd_field_t* fields;
        std::function<int(json11::Json& json, json11::Json::object& j_out_params)> cb;
    } tflow_cmd_t;

    int set_cmd_fields(tflow_cmd_field_t* cmd_field, const json11::Json& in_params);

    static void getSignResponse(const tflow_cmd_t* cmd_p, json11::Json::object& j_params);
    static void getCmdInfo(const tflow_cmd_t* cmd, json11::Json::object& j_cmd_info);

private:
    int set_field(tflow_cmd_field_t *cmd_field, const json11::Json& in_param);
};

