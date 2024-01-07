#include <sys/stat.h>
#include <glib-unix.h>
//#include <glib/gmem.h>

#include <json11.hpp>
#include "tflow-ctrl-capture.h"

using namespace json11;

static const char *raw_cfg_default =  R"( 
    {"dev_name" : "/dev/video0"} 
)";

TFlowCtrlCapture::TFlowCtrlCapture(TFlowCapture& parent) :
    app(parent)
{
    
}

void TFlowCtrlCapture::Init()
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

int TFlowCtrlCapture::dev_name_is_valid()
{ 
    // check cmd_flds_config.dev_name.v.str;
    return 1;
}
char* TFlowCtrlCapture::cam_name_get()
{
    return dev_name_is_valid() ? cmd_flds_config.dev_name.v.str : nullptr;
}
int TFlowCtrlCapture::cam_fmt_get()
{
    return (int)cmd_flds_config.fmt_idx.v.u32;
}

/*********************************/
/*** Application specific part ***/
/*********************************/

static int ctrl_capture_cmd_cb_sign_s(void* ctx, Json& in_params)
{
    return 0;
}

#if 0
int TFlowCtrlCapture::tflow_cmd_cb_sign(void* ctx)
{
    using namespace std;
    cout << "\nset_as_def\n";
    if (ctx != nullptr) {
        json11::Json::array &params = *static_cast<json11::Json::array *>(ctx);

        if (!params.empty()) {
            cout << "has PARAMS when shouldn't" << endl;
            return -1;
        }
    }
    cout << "EMPTY PARAMS (ALL OK)" << endl;

    // Reply with the module signature
    // ...
    return 0;
}

int TFlowCtrlCapture::tflow_cmd_cb_set_as_def(void* ctx)
{
    using namespace std;
    cout << "\nset_as_def\n";
    if (ctx != nullptr) {
        json11::Json::array& params = *static_cast<json11::Json::array*>(ctx);

        if (!params.empty()) {
            cout << "has PARAMS when shouldn't" << endl;
            return -1;
        }
    }
    cout << "EMPTY PARAMS (ALL OK)" << endl;
    return 0;
}

#endif


static int ctrl_capture_cmd_cb_set_as_def_s(void* ctx, Json& json_cfg)
{
    return 0;
}

static int ctrl_capture_cmd_cb_config_s(void* ctx, Json& in_params)
{
    TFlowCtrlCapture *ctrl_capture = (TFlowCtrlCapture*)ctx;
    return ctrl_capture->cmd_cb_config(in_params);
}

int TFlowCtrlCapture::cmd_cb_config(Json &in_params)
{
    g_info("Config command\n    params:\t");

    int rc = set_cmd_fields((tflow_cmd_field_t*)&cmd_flds_config, in_params);

    if (rc != 0) return -1;

    return 0;
}


