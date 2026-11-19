#pragma once
#include "tflow-build-cfg.hpp"
#include <cstdint>
#include <ctime>

#include "tflow-ctrl.hpp"

#include "tflow-common.hpp"

class TFlowPWMUI : public TFlowCtrlUI {
public:

    // Pitch hold slider2 controls vertical line within full frame height
    struct uictrl ui_sl2_dtc_range = {
        .label = "Duty cycle range (1:10000)",
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER2,
        .size = -1,
        .slider = {.min = 0, .max = 200}
    };

    struct uictrl ui_sw_force_en = {
        .label = "Force",
        .type = TFlowCtrlUI::UICTRL_TYPE::SWITCH,
    };

    TFlowCtrlUI::uictrl ui_sl_force_dtc_val = {
        .label = "Val",
        .label_pos = 0,
        .type = TFlowCtrlUI::SLIDER,
        .size = -1,
        .slider = {0, 100}
    };

};

class TFlowPWMCfg : public TFlowPWMUI {
public:

    struct cfg_tflow_servo_cntrl {
        TFlowCtrl::tflow_cmd_field_t   head;
        TFlowCtrl::tflow_cmd_field_t   channel;
        TFlowCtrl::tflow_cmd_field_t   period;
        TFlowCtrl::tflow_cmd_field_t   initial_val;
        TFlowCtrl::tflow_cmd_field_t   degr2dtc;
        TFlowCtrl::tflow_cmd_field_t   move_speed;
        TFlowCtrl::tflow_cmd_field_t   update_time_min;
        TFlowCtrl::tflow_cmd_field_t   force_up;
        TFlowCtrl::tflow_cmd_field_t   force_down;
        TFlowCtrl::tflow_cmd_field_t   dtc_range;
        TFlowCtrl::tflow_cmd_field_t   force_dtc_en;
        TFlowCtrl::tflow_cmd_field_t   force_dtc_degr;
        TFlowCtrl::tflow_cmd_field_t   force_dtc_val;
        TFlowCtrl::tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg; // Is instance is in use???
};

class TFlowPWM {
public:
    TFlowPWM(const TFlowPWMCfg::cfg_tflow_servo_cntrl* _cfg);
    ~TFlowPWM();

    int cfg_dtc_min() const { return (cfg->dtc_range.v.vnum->at(0) * 10000);}
    int cfg_dtc_max() const { return (cfg->dtc_range.v.vnum->at(1) * 10000);}

    const TFlowPWMCfg::cfg_tflow_servo_cntrl* cfg;

    void onConfig();

    int degr2dutecycle(float degr);
    float dutecycle2degr(int dtc);
    float get_dutecycle_rad() const;

    int val2dutecycle(float val);
    float get_val() const;

    std::string last_err_str;

    int fd_period;
    int fd_duty_cycle;
    int fd_enable;

    enum MOVE_DIR {
        STOP = 0,
        UP   = 1,
        DOWN = 2 
    } move_dir;

    int duty_cycle;

    struct timespec last_pwm_update_ts;

    double move_speed;      // Is in use only if move_target is NAN
    float  move_target;     // NAN - no target. 0..1 - DTC min..max

    void move_update(const struct timespec &now_ts);                                             // Update srvo angle according to currently set speed and direction
    void move_set(MOVE_DIR dir, float speed);                      // Set current speed and direction

    std::string pwm_chip;
};

