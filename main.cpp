#include <giomm.h>
#include <glib-unix.h>
#include <thread>

#include "tflow-capture.h"

//using namespace reedl;

TFlowCapture *gp_app;

gboolean handle_signal(gpointer ctx)
{
    g_info("Got INT or TERM signal, terminating...");

    TFlowCapture *app = (TFlowCapture*)ctx;
    g_main_loop_quit(app->main_loop);

    return true;
}


static void setup_sig_handlers()
{
    // Register a function that GLib will call 4 times per second
    GSource* src_sigint, * src_sigterm;

    src_sigint = g_unix_signal_source_new(SIGINT);
    src_sigterm = g_unix_signal_source_new(SIGTERM);

    g_source_set_callback(src_sigint, (GSourceFunc)handle_signal, gp_app, NULL);
    g_source_set_callback(src_sigterm, (GSourceFunc)handle_signal, gp_app, NULL);

    g_source_attach(src_sigint, gp_app->context);
    g_source_attach(src_sigterm, gp_app->context);

    g_source_unref(src_sigint);
    g_source_unref(src_sigterm);
}

int main(int argc, char** argv)
{
    Gio::init();

    g_info("TFlow capture started");

    gp_app = new TFlowCapture;

    setup_sig_handlers();

    gp_app->Attach();

#if CODE_BROWSE
    gp_app->OnIdle();   
#endif

    g_main_loop_run(gp_app->main_loop);

    delete gp_app;

    g_info("App thread exited");

    return 0;
}
