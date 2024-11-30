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

#ifndef V4L2_CID_FLYN384_BASE
#define V4L2_CID_FLYN384_BASE (V4L2_CID_LASTP1)
#define V4L2_CID_FLYN384_COMPRESS_EN	    (V4L2_CID_FLYN384_BASE + 0)
#define V4L2_CID_FLYN384_DENOISE	        (V4L2_CID_FLYN384_BASE + 1)
#define V4L2_CID_FLYN384_FILTER		        (V4L2_CID_FLYN384_BASE + 2)
#define V4L2_CID_FLYN384_PALETTE	        (V4L2_CID_FLYN384_BASE + 5)
#define V4L2_CID_FLYN384_V_STRIP            (V4L2_CID_FLYN384_BASE + 6)
#define V4L2_CID_FLYN384_FREEZE	            (V4L2_CID_FLYN384_BASE + 7)
#define V4L2_CID_FLYN384_TEMP_CALIB	        (V4L2_CID_FLYN384_BASE + 8)
#define V4L2_CID_FLYN384_SHUT_CALIB_PERIOD  (V4L2_CID_FLYN384_BASE + 9)
#define V4L2_CID_FLYN384_SHUT_CALIB_TRIG    (V4L2_CID_FLYN384_BASE + 10)
#endif 

struct fmt_info;

class V4L2Device {
public:

    V4L2Device(MainContextPtr context, int buffs_num, int planes_num,
        const TFlowCtrlCapture::cfg_v4l2_ctrls* cfg);

    ~V4L2Device();

    int Init();
    void Deinit();
   
    // Query device information
    int  ioctlQueryCapability();
    void getDriverName();
    int  ioctlEnumFmt();

    // Get/set paramters
    int  ioctlSetControls();
    int  ioctlSetControls_ISI();
    int  ioctlSetControls_flyn();
    int  ioctlSetControls_flyn_calib(int on_off);
    int  ioctlSetControls_flyn_calib_trig();
    int  ioctlSetControls_atic();

    int  ioctlGetStreamParm();
    int  ioctlSetStreamParm(bool highQuality = false, u_int timeperframe = 30);

    int  ioctlGetStreamFmt();
    int  ioctlSetStreamFmt(const struct fmt_info *stream_fmt); 

    int  InitBuffers();
    void DeinitBuffers();

    //Frame capture
    int  ioctlQueueBuffer(int index);
    int  ioctlDequeueBuffer(v4l2_buffer &v4l2_buf);

    int StreamOn(const struct fmt_info *_stream_fmt);
    void ioctlSetStreamOff();
    
    int onBuff(Glib::IOCondition io_cond);       // Called at new buffer available from the driver

    int is_stall();                              //  Should be called periodiacally. For ex. from the idle loop.

    int dev_fd;                                  // Camera ISI device
    int sub_dev_fd;                              // Camera sensor device

    int f_in_fd;

    TFlowBufSrv *buf_srv;   // Server to pass V4L2 buffers
    int buffs_num;          // Initialized on creation by callee 
    int planes_num;         // Initialized on creation by callee 
                            // TODO: AV: Planes_num is used for ioctl only. 
                            //           Consider usage of the same v4l2_buffer for all operation (4 times).
                            //           This will reduce the code size slightly
                            // 

    const struct fmt_info *stream_fmt;   // one of fmt_info_enum
    std::vector<struct fmt_info>    fmt_info_enum;

    std::string driver_name;

    int flyn_calib_is_on;

private:

    // AV: TODO: There is no observable scenarios when camera can be closed and 
    //           then reopened without complete reinit of internal data.
    //           Thus, it is worth to consider dev_name specifed upon creation
    //           as a constructor parameter.
    //           Also consider camera open/close from v4l2Device constructor/desctructor.
    int  Open();
    void Close();

    MainContextPtr context;          // Context for pending events

    bool is_streaming;

    int streaming_watchdog_cnt;
    int streaming_watchdog_frames_num;  // Number or frames received at the moment of previous WD test
    static constexpr int STREAM_WDT_CNT = 10;

    IOSourcePtr io_in_src;

    v4l2_buffer v4l2_buf_template;
    std::vector<v4l2_plane> mplanes_template;

    const TFlowCtrlCapture::cfg_v4l2_ctrls *cfg;

    /* Statistics counters */
    int stat_cnt_frames_num;
};
