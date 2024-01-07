#pragma once 

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h> //V4L2 stuff
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <string.h>

#define BUFFER_NUM  4   // Number of buffers
#define PLANE_NUM   1   // Number of planes

// AV: TODO: HxW are part of of configuration!!!
#define IMAGEHEIGHT 240
#define IMAGEWIDTH  320

gboolean g_cam_io_in_cb(gpointer user_data);
//gboolean cam_io_in_dispatch(GSource* source, GSourceFunc callback, gpointer    user_data);
//gboolean f_cam_io_in_dispatch(GSource* source, GSourceFunc callback, gpointer    user_data);

class TFlowCapture;

class V4L2Device {
public:
//    V4L2Device(TFlowCapture *parent);

    V4L2Device(GMainContext* context);
    ~V4L2Device();

    int Init(const char* dev_name);
    void Deinit();
   
    void* buffGet(int i);   // AV: not in use

    // Query device information
    int  ioctlQueryCapability();
    int  ioctlEnumFmt();

    // Get/set paramters
    int  ioctlGetStreamParm();
    int  ioctlSetStreamParm(bool highQuality = false, u_int timeperframe = 30);
    int  ioctlGetStreamFmt();
    int  ioctlSetStreamFmt(u_int pixelformat, u_int width, u_int height); 

    // TODO: Rework to InitBuffers/DeinitBuffers
    int  InitBuffers();
    void DeinitBuffers();

    //Frame capture
    int  ioctlQueueBuffer(int index);
    int  ioctlDequeueBuffer(int *index, struct timeval *ts);

    int  StreamOn(int fmt_idx);
    void ioctlSetStreamOff();

    int dev_fd {-1};

    int      f_in_fd;
    GSource* f_in_src;
    gpointer f_in_tag;

    int onBuff();       // Called at new buffer available from the driver
    int onBuffRet();    // Returns the buffer back to driver
    void redeem(int);

    typedef struct {
        GSource g_source;
        V4L2Device* cam;
    } GSourceCam;

    TFlowBufSrv *buf_srv;  // Server to pass V4L2 buffers
private:

    int  Open(const char* filename = "/dev/video0");
    void Close();


  //  TFlowCapture* parent;
    GMainContext* context;  // Context for pending events

    const char* m_fname {nullptr};
    volatile bool m_isStreamOn {false};

    struct Buffer {
        void* start;
        size_t length;
    };
    std::array<struct Buffer, BUFFER_NUM> m_buffers {};

    GSourceCam* io_in_src;
    gpointer io_in_tag;
    GSourceFuncs gsource_funcs;

};
