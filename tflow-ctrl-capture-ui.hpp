#pragma once
#include <stdint.h>
#include "tflow-ctrl.hpp"

class TFlowCtrlCaptureUI {
private:
    const char *serial_baud_entries[4] = {
            " 57600",
            "115200",
            "921600",
            nullptr 
    };

    const char *gst_testpatt_entries[5] = {
            "Live",
            "Bars V",
            "Bars H",
            "Chess ",
            nullptr 
    };

public:
    enum UICTRL_TYPE_CUSTOM  {
        BASE = TFlowCtrlUI::UICTRL_TYPE::CUSTOM,     // 
        FLIP,
        SW_DROPDOWN,
        FLYN_COMPRESSION,
        POINT
    };

    struct TFlowCtrlUI::uictrl ui_custom_flip = {
            .type = (TFlowCtrlUI::UICTRL_TYPE)UICTRL_TYPE_CUSTOM::FLIP,
    };

    struct TFlowCtrlUI::uictrl ui_custom_flyn_testpatt = {
            .type = (TFlowCtrlUI::UICTRL_TYPE)UICTRL_TYPE_CUSTOM::SW_DROPDOWN,
    };

    struct TFlowCtrlUI::uictrl ui_custom_flyn_compression = {
            .type = (TFlowCtrlUI::UICTRL_TYPE)UICTRL_TYPE_CUSTOM::FLYN_COMPRESSION,
    };

    /*********** Capture top ****************/

    struct TFlowCtrlUI::uictrl ui_serial_baud = {
            .label = "UART baud",
            .label_pos = 1,
            .type = TFlowCtrlUI::UICTRL_TYPE::DROPDOWN,
            .size = 7,
            .dropdown = {.val = (const char **)&serial_baud_entries }
    };

    /*********** CAPTURE/V4L2 ***********************/
    struct TFlowCtrlUI::uictrl ui_custom_flip_def = {
            .type = TFlowCtrlUI::UICTRL_TYPE::CUSTOM,
    };

    /*********** CAPTURE/V4L2/FLYN **********************/

    const char *compression_level_entries[2] = {
            "14bit",
            nullptr 
    };


    TFlowCtrlUI::uictrl ui_dd_compression_level = {
            .type = TFlowCtrlUI::UICTRL_TYPE::DROPDOWN,
            .dropdown = {.val = (const char **)&compression_level_entries }
    };

    TFlowCtrlUI::uictrl ui_gst_testpatt = {
            .type = TFlowCtrlUI::UICTRL_TYPE::DROPDOWN,
            .size = 7,
            .dropdown = {.val = (const char **)&gst_testpatt_entries }
    };

    TFlowCtrlUI::uictrl ui_mipi_testpatt = {
            .type = TFlowCtrlUI::UICTRL_TYPE::SWITCH,
    };

    TFlowCtrlUI::uictrl ui_flyn_contrast = {
        .label = "Contrast",
        .label_pos = 1,
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
        .size = 10,
        .slider = {0, 100}
    };

    TFlowCtrlUI::uictrl ui_flyn_brightness = {
        .label = "Brightness",
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
        .size = 10,
        .slider = {0, 100}
    };

    TFlowCtrlUI::uictrl ui_flyn_filter = {
        .label = "Filter",
        .label_pos = 1,
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
        .size = 10,
        .slider = {0, 10}       // 0 - OFF
    };

    TFlowCtrlUI::uictrl ui_flyn_comp_time = {
        .label = "Comp. (min)",
        .label_pos = 0,     // UP
        .type = TFlowCtrlUI::UICTRL_TYPE::EDIT,
        .size = 5,
    };


    TFlowCtrlUI::uictrl ui_flyn_gain = {
        .label = "Gain",
        .label_pos = 1,
        .type = TFlowCtrlUI::UICTRL_TYPE::SWITCH,
    };

    /*********** CAPTURE/V4L2/ATIC/COMPRESSION **********************/
#if 0
    TFlowCtrlUI::uictrl ui_compr_en = {
            .name = "compr_en",
            .type = TFlowCtrlUI::UICTRL_TYPE::SWITCH,
    };

    TFlowCtrlUI::uictrl ui_compr_rst = {
            .name = "reset",
            .type = TFlowCtrlUI::UICTRL_TYPE::BUTTON,
    };

    TFlowCtrlUI::uictrl ui_hist_en = {
            .name = "hist_en",
            .type = TFlowCtrlUI::UICTRL_TYPE::SWITCH,
    };

    TFlowCtrlUI::uictrl ui_custom_histogramm = {
            .name = "hist",
            .type = (TFlowCtrlUI::UICTRL_TYPE)UICTRL_TYPE_CUSTOM::HISTOGRAMM,
    };

    TFlowCtrlUI::uictrl ui_custom_p1 = {
            .name = "p1",
            .type = (TFlowCtrlUI::UICTRL_TYPE)UICTRL_TYPE_CUSTOM::POINT,
    };
    TFlowCtrlUI::uictrl ui_custom_p2 = {
            .name = "p2",
            .type = (TFlowCtrlUI::UICTRL_TYPE)UICTRL_TYPE_CUSTOM::POINT,
    };
    TFlowCtrlUI::uictrl ui_custom_p3 = {
            .name = "p3",
            .type = (TFlowCtrlUI::UICTRL_TYPE)UICTRL_TYPE_CUSTOM::POINT,
    };
    TFlowCtrlUI::uictrl ui_custom_p4 = {
            .name = "p4",
            .type = (TFlowCtrlUI::UICTRL_TYPE)UICTRL_TYPE_CUSTOM::POINT,
    };
#endif
};


