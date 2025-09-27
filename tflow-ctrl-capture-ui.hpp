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

    // Tailore UI handles FPGA and Sensor test pattern enabling
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

    
    const char *twin412g2_denoise2d_entries[4] = {
            "OFF",
            "1",
            "2",
            nullptr 
    };

    /* AV: description probably doesn't correspond to actual behavior 
     *     See FW sources for details
     */
    enum TWIN412G2_IMAGE_MODE {
        IMAGE_MODE_USER  = 0,
        IMAGE_MODE_SOFT  = 1,
        IMAGE_MODE_STD   = 2,
        IMAGE_MODE_ENH   = 3,
        IMAGE_MODE_LAST  = 4,
        NUM   = IMAGE_MODE_LAST+1
    };

    const char *twin412g2_image_mode_entries[5] = {
        [TWIN412G2_IMAGE_MODE::IMAGE_MODE_USER] = "User",
        [TWIN412G2_IMAGE_MODE::IMAGE_MODE_SOFT] = "Soft",
        [TWIN412G2_IMAGE_MODE::IMAGE_MODE_STD ] = "Standard",
        [TWIN412G2_IMAGE_MODE::IMAGE_MODE_ENH ] = "Enhanced",
        [TWIN412G2_IMAGE_MODE::IMAGE_MODE_LAST] = nullptr 
    };

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

    TFlowCtrlUI::uictrl ui_flyn_contrast_twin412g2 = {
        .label = "Contrast",
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
        .size = -1,
        .slider = {0, 100}
    };

    TFlowCtrlUI::uictrl ui_flyn_brightness_twin412g2 = {
        .label = "Brightness",
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
        .size = -1,
        .slider = {0, 100}
    };

    TFlowCtrlUI::uictrl ui_flyn_enh_detail_twin412g2 = {
        .label = "Enhanced details",
        .label_pos = 0,     // UP
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
        .size = 10,
        .slider = {0, 255}
    };

    TFlowCtrlUI::uictrl ui_flyn_denoise2d_twin412g2 = {
            .label = "Denoise 2D",
            .label_pos = 0,
            .type = TFlowCtrlUI::UICTRL_TYPE::DROPDOWN,
            .dropdown = {.val = (const char **)&twin412g2_denoise2d_entries}
    };

    TFlowCtrlUI::uictrl ui_flyn_image_mode_twin412g2 = {
        .label = "Image mode",
        .label_pos = 0,
        .type = TFlowCtrlUI::UICTRL_TYPE::DROPDOWN,
        .dropdown = {.val = (const char **)&twin412g2_image_mode_entries}

    };

    TFlowCtrlUI::uictrl ui_flyn_denoise2d = {
        .label = "Denoise 2D",
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
        .size = 10,
        .slider = {0, 2}
    };

    TFlowCtrlUI::uictrl ui_flyn_contrast = {
        .label = "Contrast",
        .label_pos = 0,
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
        .size = 10,
        .slider = {0, 255}
    };

    TFlowCtrlUI::uictrl ui_flyn_brightness = {
        .label = "Brightness",
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
        .size = 10,
        .slider = {0, 16}
    };

    TFlowCtrlUI::uictrl ui_flyn_denoise = {
        .label = "Denoise",
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
        .size = 10,
        .slider = {0, 9}
    };

    TFlowCtrlUI::uictrl ui_flyn_filter = {
        .label = "Filter",
        .label_pos = 0,
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
        .size = 10,
        .slider = {0, 3}       // 0 - OFF
    };

    TFlowCtrlUI::uictrl ui_sw_flyn_comp_en = {
        .label = "Comp.",
        .label_pos = 0,     // UP
        .type = TFlowCtrlUI::UICTRL_TYPE::SWITCH
    };

    TFlowCtrlUI::uictrl ui_edit_flyn_comp_time = {
        .label = "Comp. time (min)",
        .label_pos = 0,     // UP
        .type = TFlowCtrlUI::UICTRL_TYPE::EDIT,
        .size = 5,
    };

    TFlowCtrlUI::uictrl ui_butt_flyn_comp_trig = {
        .label = "Comp. trig",
        .type = TFlowCtrlUI::UICTRL_TYPE::BUTTON,
        .size = 5,
    };

    TFlowCtrlUI::uictrl ui_flyn_gain = {
        .label = "Auto Gain",
        .label_pos = 1,
        .type = TFlowCtrlUI::UICTRL_TYPE::SWITCH,
    };

    TFlowCtrlUI::uictrl ui_flyn_freeze = {
        .label = "Freeze",
        .label_pos = 1,
        .type = TFlowCtrlUI::UICTRL_TYPE::SWITCH,
    };

#if 0
    /*********** CAPTURE/V4L2/ATIC/COMPRESSION **********************/
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


