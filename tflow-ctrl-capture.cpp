#include <string.h>
#include <stdio.h>
#include <sys/stat.h>

#include <glib-unix.h>

#include <json11.hpp>

#include "tflow-capture.h"

using namespace json11;
using namespace std;

static const std::string capture_raw_cfg_default{ R"( 
{
    "config" : {
        "buffs_num"    : 4,
        "player_fname" : "/home/root/4",
        "serial_name"  : "/dev/ttymxc0",
        "serial_baud"  : 921600,
        "v4l2" : {
            "dev_name"     : "/dev/video0",
            "sub_dev_name" : "/dev/v4l-subdev1",
            "flyn384" : {
                "filter"  : 0,
                "denoise" : 0,
                "contrast" : 100,
                "brightness" : 8
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
        g_error("Ooops in %s", __FUNCTION__); 
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
    // 
    // Call command's processor from table
    //
}

void TFlowCtrlSrvCapture::onSignature(Json::object& j_out_params, int& err)
{
    err = 0;
    ctrl_capture.getSignResponse(&ctrl_capture.ctrl_capture_rpc_cmds[0], j_out_params);
    return;
}


/*******************************************************************************/
TFlowCtrlCapture::TFlowCtrlCapture(TFlowCapture& _app) :
    app(_app),
    ctrl_srv(*this, _app.context)  // ??? pass Ctrl Commands to the server?
{
    parseConfig(ctrl_capture_rpc_cmds, cfg_fname, capture_raw_cfg_default);
    InitServer();
}

void TFlowCtrlCapture::InitServer()
{
}
int TFlowCtrlCapture::parseConfig(
    tflow_cmd_t* config_cmd, const std::string& _cfg_fname, const std::string& raw_cfg_default)
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

    if (use_default_cfg) {
        std::string err;
        json_cfg = Json::parse(raw_cfg_default.c_str(), err);
        if (!err.empty()) {
            g_error("Can't parse default config");
            return -1;  // won't hit because of g_error
        }
    }

    //std::string xz = json_cfg.dump();
    //g_warning("CFG DUMP : %s", xz.c_str());

    // Top level processing 
    while (config_cmd->fields) {
        const Json& in_params = json_cfg[config_cmd->name];
        if (!in_params.is_null() && in_params.is_object()) {
            int rc = setCmdFields(config_cmd->fields, in_params);
            if (rc) return -1;
        }
        config_cmd++;
    }
    return 0;

}

void TFlowCtrlCapture::cam_fmt_enum_get(std::vector<struct fmt_info> &cfg_fmt_enum)
{
    // get cfg string comma separated in format
    // WWWWxHHHH CCCC
    struct fmt_info fmt_info_parsed;
    size_t strmax = 0;
    char *next_token = NULL;
    char *token;
    
    if (!cmd_flds_config.fmt_enum.v.str) return;

    token = strtok_r(cmd_flds_config.fmt_enum.v.str, ",", &next_token);
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
        token = strtok_r(cmd_flds_config.fmt_enum.v.str, ",", &next_token);
    }

}
int TFlowCtrlCapture::serial_name_is_valid()
{
    // check cmd_flds_config.serial_name.v.str;
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


int TFlowCtrlCapture::player_fname_is_valid()
{
    // check cmd_flds_config.player_fname.v.str;
    return 1;
}

/*********************************/
/*** Application specific part ***/
/*********************************/

int TFlowCtrlCapture::cmd_cb_version(const json11::Json& j_in_params, Json::object& j_out_params)
{
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

    return 0;
}



