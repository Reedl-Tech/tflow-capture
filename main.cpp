#include <giomm.h>
#include <glib-unix.h>
#include <thread>
#include <signal.h>

#include "tflow-capture.h"

TFlowCapture *gp_app;

gboolean handle_signal(gpointer ctx)
{
    g_info("Got INT or TERM signal, terminating...");

    TFlowCapture *app = (TFlowCapture*)ctx;
    app->main_loop->quit();

    return true;
}

int main(int argc, char** argv)
{
    Gio::init();

    g_info("TFlow Capture started");

    MainContextPtr context = Glib::MainContext::get_default();

    gp_app = new TFlowCapture(context);

    guint int_id = g_unix_signal_add(SIGINT, handle_signal, gp_app);
    guint term_id = g_unix_signal_add(SIGTERM, handle_signal, gp_app);

    gp_app->main_loop->run();

    g_source_remove(int_id);
    g_source_remove(term_id);

    delete gp_app;

    g_info("TFlow Capture exited");

    return 0;
}
