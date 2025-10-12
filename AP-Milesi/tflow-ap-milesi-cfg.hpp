#pragma once

#include "../tflow-ctrl.hpp"

class TFlowMilesiUI : public TFlowCtrlUI {

public:

    // TODO: add grep mask for IP adress
    struct uictrl ui_edit_udp_remote_addr = {
        // 192.168.123.123:12345
        .label = "QGC <addr:port>",
        .type = TFlowCtrlUI::UICTRL_TYPE::EDIT,
        .size = 21
    };

    struct uictrl ui_edit_udp_local_addr = {
        .label = "Bind to <addr:port>",
        .type = TFlowCtrlUI::UICTRL_TYPE::EDIT,
        .size = 21
    };

    struct uictrl ui_edit_serial_name = {
        .label = "Serial port name",
        .type = TFlowCtrlUI::UICTRL_TYPE::EDIT,
        .size = 15
    };

    struct uictrl ui_edit_serial_baud = {
        .label = "Baud rate",
        .type = TFlowCtrlUI::UICTRL_TYPE::EDIT,
        .size = 7,
    };

};

class TFlowMilesiCfg : public TFlowMilesiUI {

public:

    struct cfg_milesi {
        TFlowCtrl::tflow_cmd_field_t   head;
        TFlowCtrl::tflow_cmd_field_t   serial_name;
        TFlowCtrl::tflow_cmd_field_t   serial_baud;
        TFlowCtrl::tflow_cmd_field_t   udp_local_addr;
        TFlowCtrl::tflow_cmd_field_t   udp_remote_addr;
        TFlowCtrl::tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_milesi = {
        TFLOW_CMD_HEAD("Milesi"),
        .serial_name     = { "serial_name", TFlowCtrl::CFT_STR, 0, {.str = strdup("/dev/ttymxc0")},  &ui_edit_serial_name },
        .serial_baud     = { "serial_baud", TFlowCtrl::CFT_NUM, 0, {.num = 921600},                  &ui_edit_serial_baud },
        .udp_local_addr  = { "local_addr",  TFlowCtrl::CFT_STR, 0, {.str = strdup("0.0.0.0:14550")}, &ui_edit_udp_local_addr},
        .udp_remote_addr = { "remote_addr", TFlowCtrl::CFT_STR, 0, {.str = strdup("192.168.2.10:14550")}, &ui_edit_udp_remote_addr},
        TFLOW_CMD_EOMSG
    };
};

extern TFlowMilesiCfg tflow_milesi_cfg;
