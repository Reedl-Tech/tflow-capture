#pragma once 

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <memory.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h> //V4L2 stuff

#include "tflow-glib.hpp"

// AV: TODO: FMT@HxW are part of of configuration!!!
#define IMAGEHEIGHT 240
#define IMAGEWIDTH  320

class V4L2Device {
public:

    V4L2Device(MainContextPtr context, int buffs_num, int planes_num,
        const TFlowCtrlCapture::cfg_v4l2_ctrls* cfg);

    ~V4L2Device();

    int Init(const char* dev_name);
    void Deinit();
   
    // Query device information
    int  ioctlQueryCapability();
    int  ioctlEnumFmt();

    // Get/set paramters
    int  ioctlSetControls();
    int  ioctlSetControls_ISI();

    int  ioctlGetStreamParm();
    int  ioctlSetStreamParm(bool highQuality = false, u_int timeperframe = 30);

    int  ioctlGetStreamFmt();
    int  ioctlSetStreamFmt(u_int pixelformat, u_int width, u_int height); 

    int  InitBuffers();
    void DeinitBuffers();

    //Frame capture
    int  ioctlQueueBuffer(int index);
    int  ioctlDequeueBuffer(v4l2_buffer &v4l2_buf);

    int  StreamOn(int fmt_idx);
    void ioctlSetStreamOff();
    
    int onBuff(Glib::IOCondition io_cond);       // Called at new buffer available from the driver

    int dev_fd {-1};

    int f_in_fd;

    //typedef struct {
    //    GSource g_source;
    //    V4L2Device* cam;
    //} GSourceCam;

    TFlowBufSrv *buf_srv;   // Server to pass V4L2 buffers
    int buffs_num;          // Initialized on creation by callee 
    int planes_num;         // Initialized on creation by callee 
                            // TODO: AV: Planes_num is used for ioctl only. 
                            //           Consider usage of the same v4l2_buffer for all operation (4 times).
                            //           This will reduce the code size slightly
                            // 
    // AV: TODO: Rework to get frame format params from enum
    //           Q: ? use v4l2_format ?
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t frame_format;       // 4c V4L2_PIX_FMT_GREY

private:

    // AV: TODO: There is observable scenarios when camera can be closed and 
    //           then reopened without complete reinit of internal data.
    //           Thus, it is worth to consider dev_name specifed upon creation
    //           as a constructor parameter.
    //           Also consider camera open/close from v4l2Device constructor/desctructor.
    int  Open(const char* filename);
    void Close();

    MainContextPtr context;          // Context for pending events

    const char* m_fname {nullptr};  // Just a local copy of the current device name
    bool is_streaming {false};

    IOSourcePtr io_in_src;

    v4l2_buffer v4l2_buf_template;
    std::vector<v4l2_plane> mplanes_template;

    const TFlowCtrlCapture::cfg_v4l2_ctrls *cfg;
};
