#include <string.h>
#include <stdio.h>
#include <sys/stat.h>

#include <glib-unix.h>

#include <json11.hpp>

#include "tflow-capture.hpp"

using namespace json11;
using namespace std;

static const std::string capture_raw_cfg_default { R"( 
{
    "capture" : {
        "buffs_num"    : 4,
        "serial_name"  : "/dev/ttymxc1",
        "serial_baud"  : 921600,
        "v4l2" : {
            "dev_name"     : "auto",
            "sub_dev_name" : "auto",
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
            "vflip"   : 1,
            "hflip"   : 1
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
    int t = j_in_params.is_object();
    if ( !j_in_params.is_object() ) {
        // Empty request - reply with all controls
    }
    // Find command by name
    // Call command's processor from table
    TFlowCtrl::tflow_cmd_t *ctrl_cmd_p = ctrl_capture.ctrl_capture_rpc_cmds;
    while ( ctrl_cmd_p->name ) {
        if ( 0 == strncmp(ctrl_cmd_p->name, cmd.c_str(), cmd.length()) ) {
            err = ctrl_cmd_p->cb(j_in_params, j_out_params);
#if CODE_BROWSE
            TFlowCtrlCapture::cmd_cb_cfg_...();
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
    
    json11::Json::object xx;
    getSignResponse(xx);
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
int TFlowCtrlCapture::serial_name_is_valid()
{
    if (cmd_flds_config.serial_name.v.str == nullptr) return 0;
    return 1;
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

void TFlowCtrlCapture::getSignResponse(json11::Json::object &j_out_params)
{
    const tflow_cmd_t *cmd_p = &ctrl_capture_rpc_cmds[0];

    j_out_params.emplace("state", "OK");
    j_out_params.emplace("version", "v0");  // TODO: replace for version from git or signature hash or both?

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

    Json test = Json(j_out_params);
    std::string s_msg = test.dump();
    g_critical("signature: %s", s_msg.c_str());
}


int TFlowCtrlCapture::collectCtrlsCustom(const tflow_cmd_field_t *cmd_fld, Json::array &j_out_ctrl_arr)
{
    if ( 0 == strcmp(cmd_fld->name, "name_of_custom_control") ) {
        return 1;
    }
    // cmd_fld isn't a custom Control - proceed with default
    return 0;
}

int TFlowCtrlCapture::collectCtrls(const tflow_cmd_field_t *cmd_fld, Json::array &j_out_ctrl_arr)
{
    // Json json = Json::array{ Json::object { { "k", "v" } } };
    // Loop over all config parameters, add default control description for
    // all parameters except ones processed in collectCtrlsCustom().
    // Parameter's references are processed recursivly
    while (cmd_fld->name) {
        if ( cmd_fld->type == TFlowCtrl::CFT_REF ) {
            const tflow_cmd_field_t *cmd_fld_ref_hdr = cmd_fld->v.ref;
            Json::array j_ctrl_ref_arr;
            collectCtrls(cmd_fld_ref_hdr + 1, j_ctrl_ref_arr);  // +1 to skip header
            
            Json::object j_ctrl_params;
            addCtrlRef(cmd_fld, cmd_fld_ref_hdr->name, j_ctrl_ref_arr, j_ctrl_params);
            j_out_ctrl_arr.emplace_back(j_ctrl_params);

            cmd_fld++;
            continue;
        }

        if (!collectCtrlsCustom(cmd_fld, j_out_ctrl_arr)) {
            addCtrlDef(cmd_fld, j_out_ctrl_arr);
        }
        cmd_fld++;
    }

    return 0;
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

int TFlowCtrlCapture::cmd_cb_config(const json11::Json& j_in_params, Json::object& j_out_params)
{
    g_info("Config command\n    params:\t");

    int rc = setCmdFields((tflow_cmd_field_t*)&cmd_flds_config, j_in_params);

    if (rc != 0) return -1;

    // TODO: Add changed flag or trivial (lambda?) callback function  into field definition
    // Check something was changes
    //  - Camera name
    //  - Video fmt
    //  - Serial port name 
    //  - Serial baud rate

    if (rc != 0) return -1;
    
    std::string del_me = j_in_params.dump();

    if ( app.cam && ( 0 == app.cam->driver_name.compare("flyn384") ) ) {

        const Json j_v4l2 = j_in_params [ "v4l2" ];
        if ( j_v4l2.is_object() ) {
            cfg_v4l2_ctrls *v4l2 = ( cfg_v4l2_ctrls * ) ( cmd_flds_config.v4l2.v.ref );
            const Json j_flyn384 = j_v4l2 [ "flyn384" ];

            if ( j_flyn384.is_object() ) {
                cfg_v4l2_ctrls_flyn *flyn384 = ( cfg_v4l2_ctrls_flyn * ) ( v4l2->flyn384.v.ref );
                const Json j_calib = j_flyn384 [ "calib" ];
                const Json j_calib_trig = j_flyn384 [ "calib_trig" ];

                if ( j_calib.is_number() ) {
                    app.cam->ioctlSetControls_flyn_calib(j_calib.int_value());
                }
                if ( j_calib_trig.is_number() && j_calib_trig.int_value() ) {
                    app.cam->ioctlSetControls_flyn_calib_trig();
                    // One shot parameter. Clear after use.
                    flyn384->calib_trig.v.num = 0;
                }
            }
        }
    }

    return 0;
}



