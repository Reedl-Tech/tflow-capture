#include <glib-unix.h>
#include <gst/gst.h>

#include <json11.h>
#include "tflow-ctrl-capture.h"
#include "atic-app-streamer.h"

using namespace json11;

static const char raw_cfg_default =  R"( 
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
        g_error("Can't open configuration file %s", cfg_name);
        use_default_cfg = true;
    }
    else if (!S_ISREG(sb.st_mode)) {
        g_error("Config name isn't a file %s", cfg_name);
        use_default_cfg = true;
    }

    if (!use_default_cfg) {
        const char *raw_cfg = alloc(sb.st_size);
        int bytes_read = read(cfg_fd, raw_cfg, sb.st_size);
        if (bytes_read != sb.st_size) {
            g_error("Can't read config file %s", cfg_name);
            use_default_cfg = true;
        }

        if (!use_default_cfg) {
            std::string err;
            json_cfg = Json::parse(raw_cfg, err);
            if (json_cfg.is_null()) {
                g_error("Error in JSON format - %s\n%s", (char*)err.data(), raw_cfg);
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

    tflow_cmd_cb_config(json_cfg)

}

int TFlowCtrlCapture::cam_name_is_valid()
{ 
    // check cmd_flds_config.cam_name.v.str;
    return 0;
}
char* TFlowCtrlCapture::cam_name_get()
{
    return cam_name_is_valid() ? cmd_flds_config.cam_name.v.str : nullptr;
}

/*********************************/
/*** Application specific part ***/
/*********************************/

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

int TFlowCtrlCapture::tflow_cmd_cb_config(Json &json_cfg)
{
    g_info("Config command\n    params:\t";

    // Loop over all config cpmmand fields and check json_cfg
    tflow_cmd_field_t* cfg_field = cmd_flds_config;
    while (cfg_field->name != nullptr) {
        Json cfg_param = json_cfg[cfg_field->name];
        if (!cfg_param.is_null()) {
            // Configuration parameter is found in Json config
            // Check field type is valid
            cfg_field.set(cfg_param);
        }

    }

    if (ctx != nullptr) {
//        auto* param_arr_ptr = static_cast<vector<variant<int, string>>*>(ctx);
        json11::Json::array &paramsArray = *static_cast<json11::Json::array *>(ctx);
        if (paramsArray.empty()) {
            cout << "EMPTY PARAMS" << endl;
            return -1;
        } else {
            cout << json11::Json(paramsArray).dump() << endl;
            if (paramsArray.size() == 3) {
                state = paramsArray[0].int_value();
                cam_name = paramsArray[1].int_value();
                ip = paramsArray[2].int_value();
            } else {
                cout << "WRONG NUM OF ARGS" << endl;
                return -1;
            }
        }

        cout << endl;
    } else {
        cout << "EMPTY PARAMS" << endl;
        return -1;
    }

    cout << "state:\t\t" << to_string(state)
         << "\nport:\t\t" << to_string(port)
         << "\nip:\t\t" << to_string(ip) << endl;


//    app.gst->onGstConfig();
    return 0;
}

int TFlowCtrlCapture::tflow_cmd_cb_set_as_def(void* ctx)
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
    return 0;
}

