#include <sys/stat.h>

#include <glib-unix.h>

#include <json11.hpp>

#include "tflow-capture.h"

using namespace json11;
using namespace std;

static const char *raw_cfg_default =  R"( 
    {
        "buffs_num"  : 4,
        "dev_name" : "/dev/video0",
        "serial_name" : "/dev/ttymxc0",
        "serial_baud" : 921600
    } 
)";

/*******************************************************************************/

TFlowCtrlSrvCapture::TFlowCtrlSrvCapture(TFlowCtrlCapture& _ctrl_capture, GMainContext* context) :
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
    ctrl_srv(TFlowCtrlSrvCapture(*this, _app.context))  // ??? pass Ctrl Commands to the server?
{
    InitConfig();
    InitServer();
}

void TFlowCtrlCapture::InitServer()
{
}
void TFlowCtrlCapture::InitConfig()
{
    struct stat sb;
    int cfg_fd = -1;
    bool use_default_cfg = 0;
    Json json_cfg;
    
    cfg_fd = open(cfg_fname, O_RDWR);

    if (fstat(cfg_fd, &sb) < 0) {
        g_warning("Can't open configuration file %s", cfg_fname);
        use_default_cfg = true;
    }
    else if (!S_ISREG(sb.st_mode)) {
        g_warning("Config name isn't a file %s", cfg_fname);
        use_default_cfg = true;
    }

    if (!use_default_cfg) {
        char* raw_cfg = (char*)g_malloc(sb.st_size);
        int bytes_read = read(cfg_fd, raw_cfg, sb.st_size);
        if (bytes_read != sb.st_size) {
            g_warning("Can't read config file %s", cfg_fname);
            use_default_cfg = true;
        }

        if (!use_default_cfg) {
            std::string err;
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
        json_cfg = Json::parse(raw_cfg_default, err);
    }


    set_cmd_fields((tflow_cmd_field_t*)&cmd_flds_config, json_cfg);
}

int TFlowCtrlCapture::serial_name_is_valid()
{
    // check cmd_flds_config.dev_name.v.str;
    return 1;
}

int TFlowCtrlCapture::dev_name_is_valid()
{ 
    // check cmd_flds_config.dev_name.v.str;
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

    int rc = set_cmd_fields((tflow_cmd_field_t*)&cmd_flds_config, j_in_params);

    if (rc != 0) return -1;

    // TODO: Add changed flag or trivial (lambda?) callback function  into field definition
    // Check something was changes
    //  - Camera name
    //  - Video fmt
    //  - Serial port name 
    //  - Serial baud rate
    return 0;
}



