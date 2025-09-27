#include "tflow-build-cfg.hpp"
#include <sys/stat.h>

#include <json11.hpp>

#include "tflow-glib.hpp"
#include "tflow-capture.hpp"

using namespace json11;
using namespace std;

static const std::string capture_raw_cfg_default { R"( 
{
    "config" : {
        "buffs_num"    : 4,
        "v4l2" : {
            "dev_name"     : "auto",
            "sub_dev_name" : "auto",
            "flip" : {
                "vflip"    : 1,
                "hflip"    : 1
            },
            "flyn384" : {
                "filter"     : 0,
                "denoise"    : 0,
                "contrast"   : 100,
                "brightness" : 8,
                "gain"       : 0,
                "calib"      : 1,
                "calib_trig" : 0
            },
            "atic320" : {
                "xz"     : 0
            },
        }
    }
}
)" };

/*******************************************************************************/

TFlowCtrlSrvCapture::TFlowCtrlSrvCapture(TFlowCtrlCapture& _ctrl_capture, MainContextPtr context) :
    TFlowCtrlSrv(
        string("Capture"),
        string("_com.reedl.tflow.ctrl-server-capture"),
        context),
    ctrl_capture(_ctrl_capture)
{
}

int TFlowCtrlSrvCapture::onCliPortConnect(int fd)
{
    auto cli_port_p = new TFlowCtrlCliPort(context, *this, fd);
    ctrl_clis.emplace(fd, *cli_port_p);

    return 0;
}

void TFlowCtrlSrvCapture::onCliPortError(int fd)
{
    auto ctrl_cli_it = ctrl_clis.find(fd);

    if (ctrl_cli_it == ctrl_clis.end()) {
        g_error("Ooops in %s, (fd=%d)", __FUNCTION__, fd);  // Triggered on CLI close. Needs to be debugged
    }

    TFlowCtrlCliPort& cli_port = ctrl_cli_it->second;

    g_warning("TFlowCtrlSrvCapture: Release port [%s] (%d)",
        cli_port.signature.c_str(), fd);

    ctrl_clis.erase(fd);

#if CODE_BROWSE
    TFlowCtrlCliPort::~TFlowCtrlCliPort();
#endif

    return;
}

void TFlowCtrlSrvCapture::onTFlowCtrlMsg(const std::string& cmd, const json11::Json& j_in_params, Json::object& j_out_params, int& err)
{
    // Find command by name
    // Call command's processor from table
    TFlowCtrl::tflow_cmd_t *ctrl_cmd_p = ctrl_capture.ctrl_capture_rpc_cmds;

    while ( ctrl_cmd_p->name ) {
        if ( 0 == strncmp(ctrl_cmd_p->name, cmd.c_str(), cmd.length()) ) {
            err = ctrl_cmd_p->cb(j_in_params, j_out_params);
#if CODE_BROWSE
            TFlowCtrlCapture::cmd_cb_config();
            TFlowCtrlCapture::cmd_cb_ui_sign();
#endif
            return;
        }
        ctrl_cmd_p++;
    }
    err = -100;
    return;
}

void TFlowCtrlSrvCapture::onSignature(Json::object& j_out_params, int& err)
{
    err = 0;
    ctrl_capture.getSignResponse(j_out_params);

    return;
}


/*******************************************************************************/
TFlowCtrlCapture::TFlowCtrlCapture(TFlowCapture& _app, const std::string _cfg_fname) :
    app(_app),
    cfg_fname(_cfg_fname),
    ctrl_srv(*this, _app.context)  // ??? pass Ctrl Commands to the server?
{
    parseConfig(ctrl_capture_rpc_cmds, cfg_fname, capture_raw_cfg_default);
    InitServer();
}

void TFlowCtrlCapture::InitServer()
{
}

#if 0
int TFlowCtrlCapture::parseConfig(
    tflow_cmd_t* config_cmd_in, const std::string& _cfg_fname, const std::string& raw_cfg_default)
{
    struct stat sb;
    int cfg_fd = -1;
    bool use_default_cfg = 0;
    Json json_cfg;
    
    cfg_fd = open(cfg_fname.c_str(), O_RDWR);

    if (fstat(cfg_fd, &sb) < 0) {
        g_warning("Can't open configuration file %s", cfg_fname.c_str());
        use_default_cfg = true;
    }
    else if (!S_ISREG(sb.st_mode)) {
        g_warning("Config name isn't a file %s", cfg_fname.c_str());
        use_default_cfg = true;
    }

    if (!use_default_cfg) {
        char* raw_cfg = (char*)g_malloc(sb.st_size + 1);
        int bytes_read = read(cfg_fd, raw_cfg, sb.st_size);
        if (bytes_read != sb.st_size) {
            g_warning("Can't read config file %s", cfg_fname.c_str());
            use_default_cfg = true;
        }

        if (!use_default_cfg) {
            std::string err;
            
            raw_cfg[bytes_read] = 0;
            json_cfg = Json::parse(raw_cfg, err);
            if (json_cfg.is_null()) {
                g_warning("Error in JSON format - %s\n%s", (char*)err.data(), raw_cfg);
                use_default_cfg = true;
            }
        }
        free(raw_cfg);
        close(cfg_fd);
    }

    int rc = 0;
    do {
        tflow_cmd_t* config_cmd = config_cmd_in;
        if (use_default_cfg) {
            std::string err;
            json_cfg = Json::parse(raw_cfg_default.c_str(), err);
            if (!err.empty()) {
                g_error("Can't parse default config");
                return -1;  // won't hit because of g_error
            }                      
        }

        // Top level processing 
        while (config_cmd->fields) {
            const Json& in_params = json_cfg[config_cmd->name];
            if (!in_params.is_null() && in_params.is_object()) {
                rc = setCmdFields(config_cmd->fields, in_params);
                if (rc) break;
            }
            config_cmd++;
        }

        if (rc) {
            g_critical("Can't parse config %s",
                (use_default_cfg == 0) ? "- try to use defaults" : "- defaults fail");
            if (use_default_cfg) break; // Default just tried and fails.
            use_default_cfg = 1;        // Try to use default.
        }

    } while (rc);

    return 0;

}
#endif

void TFlowCtrlCapture::cam_fmt_enum_get(std::vector<struct fmt_info> &cfg_fmt_enum)
{
    // get cfg string comma separated in format
    // WWWWxHHHH CCCC
    struct fmt_info fmt_info_parsed;
    size_t strmax = 0;
    char *next_token = NULL;
    char *token;
    
    if (!cmd_flds_config.fmt_enum.v.str) return;

    char *fmt_enum_str = strdup(cmd_flds_config.fmt_enum.v.str);
    token = strtok_r(fmt_enum_str, ",", &next_token);
    while(token) {
        int parsed = sscanf(token, "%dx%d %c%c%c%c", 
            &fmt_info_parsed.width, &fmt_info_parsed.height, 
            &fmt_info_parsed.fmt_cc.c[0],
            &fmt_info_parsed.fmt_cc.c[1],
            &fmt_info_parsed.fmt_cc.c[2],
            &fmt_info_parsed.fmt_cc.c[3]);
        if (parsed == 6) {
            cfg_fmt_enum.emplace_back(fmt_info_parsed);
        }
        else {
            g_warning("Bad fmt: %s - \"WxH CCCC,...\" expected)", token);
            break;
        }
        token = strtok_r(fmt_enum_str, ",", &next_token);
    }
    free(fmt_enum_str);

}

int TFlowCtrlCapture::dev_name_is_valid()
{ 
    // check cmd_flds_config.dev_name.v.str;
    return 1;
}

int TFlowCtrlCapture::sub_dev_name_is_valid()
{ 
    // check cmd_flds_config.sub_dev_name.v.str;
    return 1;
}

#if CAPTURE_PLAYER
int TFlowCtrlCapture::player_fname_is_valid()
{
    // check cmd_flds_config.player_fname.v.str;
    return 1;
}
#endif
/*********************************/
/*** Application specific part ***/
/*********************************/
void TFlowCtrlCapture::getUISignResponse(json11::Json::object &j_out_params)
{
    getSignResponse(j_out_params);

    const tflow_cmd_t *cmd_config = &ctrl_capture_rpc_cmds[TFLOW_CAPTURE_RPC_CMD_CONFIG];

    Json::array j_resp_controls_arr;
    collectCtrls(cmd_config->fields, j_resp_controls_arr);
    j_out_params.emplace("controls", j_resp_controls_arr);

#if 0
    const tflow_cmd_t *cmd_p = &ctrl_capture_rpc_cmds[0];
    while (cmd_p->name) {
        tflow_cmd_field_t *cmd_fld = (tflow_cmd_field_t *)cmd_p->fields;
        Json::object j_cmd_fields;
        
        // Ignore common actions - version and set_as_default
        if ( 0 == strcmp(cmd_p->name, "version") ||
             0 == strcmp(cmd_p->name, "set_as_def") ) {
            cmd_p++;
            continue;
        }

        // Add header control for the command
        // ...
        // Add other controls for the command
        Json::array j_resp_controls_arr;
        collectCtrls(cmd_fld, j_resp_controls_arr);
        j_cmd_fields.emplace("controls", j_resp_controls_arr);

        // getCmdInfo(cmd_p->fields, j_cmd_fields);
        j_out_params.emplace(cmd_p->name, j_cmd_fields);
        cmd_p++;
    }
#endif

    //Json test = Json(j_out_params);
    //std::string s_msg = test.dump();
    //g_critical("signature: %s", s_msg.c_str());
}

void TFlowCtrlCapture::getSignResponse(json11::Json::object &j_out_params)
{
    j_out_params.emplace("state", "OK");
    j_out_params.emplace("version", "v0");  // TODO: replace for version from git or signature hash or both?
    j_out_params.emplace("config_id", config_id);  
}

void TFlowCtrlCapture::collectCtrlsCustom(UICTRL_TYPE _custom_type,
    const tflow_cmd_field_t *cmd_fld, Json::array &j_out_ctrl_arr)
{
    UICTRL_TYPE_CUSTOM custom_type = (UICTRL_TYPE_CUSTOM)_custom_type;
    assert((UICTRL_TYPE)custom_type > UICTRL_TYPE::CUSTOM);

    json11::Json::object j_custom;
    Json::array j_custom_arr; 

    // Custom controls - array which contains predefined set of control objects
    if ( 0 == strcmp(cmd_fld->name, "name_of_custom_control") ) {
        //j_custom_arr.emplace_back(<controls>);
        //j_custom.emplace("type", "name_of_custom_control");
        //j_custom.emplace("name", "name_of_custom_control");
        // Template ^^^
    }
    else if ( custom_type == UICTRL_TYPE_CUSTOM::FLYN_COMPRESSION) {

        // Compression control
        const cfg_v4l2_ctrls_flyn_compression *cmd_cc = 
            (cfg_v4l2_ctrls_flyn_compression*)cmd_fld;

        Json::array j_flyn_histogramm_arr; 
        j_flyn_histogramm_arr.emplace_back(10);
        j_flyn_histogramm_arr.emplace_back(20);
        j_flyn_histogramm_arr.emplace_back(120);
        j_flyn_histogramm_arr.emplace_back(50);
        j_flyn_histogramm_arr.emplace_back(10);
        j_flyn_histogramm_arr.emplace_back(150);
        j_flyn_histogramm_arr.emplace_back(30);
        j_flyn_histogramm_arr.emplace_back(10);
        
        json11::Json::object j_histogramm;
        
        j_histogramm.emplace("name", std::string("histogramm"));
        j_histogramm.emplace("type", std::string("histogramm"));
        j_histogramm.emplace("value", j_flyn_histogramm_arr );

        // Collect UI controls from configuration
        collectCtrls(cmd_fld + 1, j_custom_arr);  // +1 to skip header

        // Add runtime value as an array
        j_custom_arr.emplace_back(j_histogramm);
                                    
        j_custom.emplace("type", "compression");
    }
    else if ( custom_type == UICTRL_TYPE_CUSTOM::FLIP ) {

        // Flip control - is a dual check box with specific icons
        const cfg_v4l2_ctrls_flip *cmd_flip = (cfg_v4l2_ctrls_flip*)cmd_fld;

        json11::Json::object j_vflip, j_hflip;
        
        j_vflip.emplace("name", std::string(cmd_flip->vflip.name));
        j_vflip.emplace("value", cmd_flip->vflip.v.num );
        j_custom_arr.emplace_back(j_vflip);

        j_hflip.emplace("name", std::string(cmd_flip->hflip.name));
        j_hflip.emplace("value", cmd_flip->hflip.v.num );
        j_custom_arr.emplace_back(j_hflip);

        j_custom.emplace("type", "flip");
    }
    else if ( custom_type == UICTRL_TYPE_CUSTOM::SW_DROPDOWN) {

        // FLYN test pattern is custom only by logic on UI side all controls 
        // are standard.
        collectCtrls(cmd_fld + 1, j_custom_arr);  // +1 to skip header

        j_custom.emplace("type", "sw_dropdown");
    }

    j_custom.emplace("name", cmd_fld->name);
    j_custom.emplace("value", j_custom_arr);
    j_out_ctrl_arr.emplace_back(j_custom);
    //auto del_me = json11::Json(j_custom).dump();
}

int TFlowCtrlCapture::cmd_cb_version(const json11::Json& j_in_params, Json::object& j_out_params)
{
    j_out_params.emplace("version", "v0");
    return 0;
}

int TFlowCtrlCapture::cmd_cb_set_as_def(const json11::Json& j_in_params, Json::object& j_out_params)
{
    return 0;
}

int TFlowCtrlCapture::cmd_cb_ui_sign(const json11::Json& j_in_params, Json::object& j_out_params)
{
    g_info("UI Sign command\n");

    getUISignResponse(j_out_params);

    return 0;
}

int TFlowCtrlCapture::onConfigFLYN_TWIN412G2()
{
    cfg_v4l2_ctrls_twin412g2 *cfg_twin412g2 = &cmd_flds_cfg_v4l2_ctrls_twin412g2;

    if (cfg_twin412g2->comp_en.flags & FIELD_FLAG::CHANGED) 
        app.cam->ioctlSetControls_flyn_comp_en(cfg_twin412g2->comp_en.v.num);
    
    if (cfg_twin412g2->comp_time.flags & FIELD_FLAG::CHANGED) 
        app.cam->ioctlSetControls_flyn_comp_time(cfg_twin412g2->comp_time.v.num);

    if (cfg_twin412g2->comp_trig.flags & FIELD_FLAG::CHANGED) 
        app.cam->ioctlSetControls_flyn_comp_trig();

    if ( cfg_twin412g2->test_patt.flags & FIELD_FLAG::CHANGED ) {
        // on the FLYN the test patterns can be triggered either on FPGA 
        // (aka MIPI) or on the sensor (aka GST).
        // In case of test pattern enabled on the sensor side the calibration 
        // (aka compensation) should be disabled. Otherwise the algortihm will 
        // compensate the testpattern.
        // TODO: Should it be part of driver's logic?

        const struct cfg_v4l2_ctrls_flyn_testpatt *flyn_testpatt = 
            (struct cfg_v4l2_ctrls_flyn_testpatt *)cfg_twin412g2->test_patt.v.ref;

        if ( flyn_testpatt->mipi_testpatt.flags & FIELD_FLAG::CHANGED) {
            app.cam->ioctlSetControls_flyn_int(&flyn_testpatt->mipi_testpatt, V4L2_CID_TEST_PATTERN);
        }

        if ( flyn_testpatt->gst_testpatt.flags & FIELD_FLAG::CHANGED) {
            if ( flyn_testpatt->gst_testpatt.v.num ) {
                // Disable calibration (compensation).
                // Enable test pattern
                app.cam->ioctlSetControls_flyn_int(&cfg_twin412g2->comp_en, 0, V4L2_CID_FLYN384_TEMP_CALIB);
                app.cam->ioctlSetControls_flyn_int(&flyn_testpatt->gst_testpatt, V4L2_CID_TEST_PATTERN);
            }
            else {
                // Disable test patterm. Restore calib settings from config
                app.cam->ioctlSetControls_flyn_int(&cfg_twin412g2->comp_en, V4L2_CID_FLYN384_TEMP_CALIB);
                app.cam->ioctlSetControls_flyn_int(&flyn_testpatt->gst_testpatt, 0, V4L2_CID_TEST_PATTERN);
            }
        }
    }

    if ( cfg_twin412g2->image_mode.flags & FIELD_FLAG::CHANGED ) {
        app.cam->ioctlSetControls_flyn_int(&cfg_twin412g2->image_mode, V4L2_CID_FLYN384_IMG_MODE);
    }

    if ( cfg_twin412g2->contrast.flags & FIELD_FLAG::CHANGED ) {
        app.cam->ioctlSetControls_flyn_int(&cfg_twin412g2->contrast, V4L2_CID_CONTRAST);
    }
    if ( cfg_twin412g2->brightness.flags & FIELD_FLAG::CHANGED ) {
        app.cam->ioctlSetControls_flyn_int(&cfg_twin412g2->brightness, V4L2_CID_BRIGHTNESS);
    }
    if ( cfg_twin412g2->enh_detail.flags & FIELD_FLAG::CHANGED ) {
        app.cam->ioctlSetControls_flyn_int(&cfg_twin412g2->enh_detail, V4L2_CID_FLYN384_ENH_DETAIL);
    }

    if ( cfg_twin412g2->denoise2d.flags & FIELD_FLAG::CHANGED ) {
        app.cam->ioctlSetControls_flyn_int(&cfg_twin412g2->denoise2d, V4L2_CID_FLYN384_DENOISE_2D);
    }

    tflow_cmd_field_t   enh_detail;
        tflow_cmd_field_t   denoise2d;

    return 0;

}
int TFlowCtrlCapture::onConfigFLYN()
{
    cfg_v4l2_ctrls_flyn *cfg_flyn = &cmd_flds_cfg_v4l2_ctrls_flyn;

    if (cfg_flyn->comp_en.flags & FIELD_FLAG::CHANGED) 
        app.cam->ioctlSetControls_flyn_comp_en(cfg_flyn->comp_en.v.num);
    
    if (cfg_flyn->comp_time.flags & FIELD_FLAG::CHANGED) 
        app.cam->ioctlSetControls_flyn_comp_time(cfg_flyn->comp_time.v.num);

    if (cfg_flyn->comp_trig.flags & FIELD_FLAG::CHANGED) 
        app.cam->ioctlSetControls_flyn_comp_trig();

    if ( cfg_flyn->test_patt.flags & FIELD_FLAG::CHANGED ) {
        // on the FLYN the test patterns can be triggered either on FPGA or
        // on sensor.
        // In case of test pattern enabled on the sensor side the calibration 
        // (aka compensation) should be disabled. Otherwise the algortihm will 
        // compensate testpattern.
        // TODO: Should it be part of driver's logic?

        const struct cfg_v4l2_ctrls_flyn_testpatt *flyn_testpatt = 
            (struct cfg_v4l2_ctrls_flyn_testpatt *)cfg_flyn->test_patt.v.ref;

        if ( flyn_testpatt->mipi_testpatt.flags & FIELD_FLAG::CHANGED) {
            app.cam->ioctlSetControls_flyn_int(&flyn_testpatt->mipi_testpatt, V4L2_CID_TEST_PATTERN);
        }

        if ( flyn_testpatt->gst_testpatt.flags & FIELD_FLAG::CHANGED) {
            if ( flyn_testpatt->gst_testpatt.v.num ) {
                // Enable. Preserve calibration setting and disable it
                app.cam->ioctlSetControls_flyn_comp_en(0);
                app.cam->ioctlSetControls_flyn_int(&flyn_testpatt->gst_testpatt, V4L2_CID_TEST_PATTERN);
            }
            else {
                // Disable. Restore calib settings
                app.cam->ioctlSetControls_flyn_comp_en(cfg_flyn->comp_en.v.num);
                app.cam->ioctlSetControls_flyn_int(&flyn_testpatt->gst_testpatt, 0, V4L2_CID_TEST_PATTERN);
            }
        }
    }

    if ( cfg_flyn->contrast.flags & FIELD_FLAG::CHANGED ) {
        app.cam->ioctlSetControls_flyn_int(&cfg_flyn->contrast, V4L2_CID_CONTRAST);
    }
    if ( cfg_flyn->brightness.flags & FIELD_FLAG::CHANGED ) {
        app.cam->ioctlSetControls_flyn_int(&cfg_flyn->brightness, V4L2_CID_BRIGHTNESS);
    }

    if ( cfg_flyn->filter.flags & FIELD_FLAG::CHANGED ) {
        app.cam->ioctlSetControls_flyn_int(&cfg_flyn->filter, V4L2_CID_FLYN384_FILTER);
    }

    if ( cfg_flyn->denoise.flags & FIELD_FLAG::CHANGED ) {
        app.cam->ioctlSetControls_flyn_int(&cfg_flyn->denoise, V4L2_CID_FLYN384_DENOISE);
    }

    if ( cfg_flyn->gain.flags & FIELD_FLAG::CHANGED ) {
        app.cam->ioctlSetControls_flyn_int(&cfg_flyn->gain, V4L2_CID_AUTOGAIN);
    }

    return 0;
}

int TFlowCtrlCapture::onConfigATIC()
{
    return 0;
}
int TFlowCtrlCapture::onConfigV4L2()
{
    cfg_v4l2_ctrls *cfg_v4l2 = &cmd_flds_cfg_v4l2;

    if ( ( cfg_v4l2->dev_name.flags & FIELD_FLAG::CHANGED ) ||
         ( cfg_v4l2->sub_dev_name.flags & FIELD_FLAG::CHANGED ) ) {
        // app.cam->xxx;
        // In theory it is meaningless to proceed with other configuration
        // if device was changed, thus just return.
        g_warning("Device changed via UI not implemented yet");
        return 0;
    }

    if ( cfg_v4l2->flip.flags & FIELD_FLAG::CHANGED ) {
        app.cam->ioctlSetControls_ISI();
    }

    if ( cfg_v4l2->flyn384.flags & FIELD_FLAG::CHANGED ) onConfigFLYN();

    if ( cfg_v4l2->twin412g2.flags & FIELD_FLAG::CHANGED ) onConfigFLYN_TWIN412G2();

    if ( cfg_v4l2->atic320.flags & FIELD_FLAG::CHANGED ) onConfigATIC();

    return 0;
}

int TFlowCtrlCapture::cmd_cb_config(const json11::Json& j_in_params, Json::object& j_out_params)
{
    tflow_cmd_field_t* flds = (tflow_cmd_field_t*)&cmd_flds_config;

    g_info("Config command: %s", j_in_params.dump().c_str());

    // Fill config fields with values from Json input object
    int was_changed = 0;
    int rc = setCmdFields(flds, j_in_params, was_changed);

    if ( rc != 0 ) {
        // TODO: Add notice or error to out_params in case of error.
        //       We can't just return from here, because some parameters
        //       might be already changed and we don't have rollback functionality
        //       So, finger cross and just continue...
    }

    if ( cmd_flds_config.v4l2.flags & FIELD_FLAG::CHANGED) {
        onConfigV4L2();
    }

    // Composes all required config params and clears changed flag.
    // Also advance config ID on configuration change.
    // If previous config_id doesn't match with one receive in command, then
    // collect _all_ controls.
    // TODO: Collect UI exposed controls only?
    collectRequestedChangesTop(flds, j_in_params, j_out_params);

    return 0;
}




