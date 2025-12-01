#pragma once 

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <memory.h>
#include <string.h>
#include <functional>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h> //V4L2 stuff

#include "tflow-glib.hpp"

#include "tflow-buf-srv.hpp"
#include "tflow-buf-pck.hpp"

// Att: Keep the following in sync with kernel defines in 
//      kernel/drivers/staging/media/reedl-flyn384/flyn384.c
#ifndef V4L2_CID_FLYN384_BASE
#define V4L2_CID_FLYN384_BASE (V4L2_CID_LASTP1)
#define V4L2_CID_FLYN384_COMPRESS_EN	    (V4L2_CID_FLYN384_BASE +  0)
#define V4L2_CID_FLYN384_DENOISE	        (V4L2_CID_FLYN384_BASE +  1)
#define V4L2_CID_FLYN384_FILTER		        (V4L2_CID_FLYN384_BASE +  2)
#define V4L2_CID_FLYN384_PALETTE	        (V4L2_CID_FLYN384_BASE +  5)
#define V4L2_CID_FLYN384_V_STRIP            (V4L2_CID_FLYN384_BASE +  6)
#define V4L2_CID_FLYN384_FREEZE	            (V4L2_CID_FLYN384_BASE +  7)
#define V4L2_CID_FLYN384_TEMP_CALIB	        (V4L2_CID_FLYN384_BASE +  8)
#define V4L2_CID_FLYN384_SHUT_CALIB_PERIOD  (V4L2_CID_FLYN384_BASE +  9)
#define V4L2_CID_FLYN384_SHUT_CALIB_TRIG    (V4L2_CID_FLYN384_BASE + 10)
#define V4L2_CID_FLYN384_IMG_MODE           (V4L2_CID_FLYN384_BASE + 11)
#define V4L2_CID_FLYN384_ENH_DETAIL         (V4L2_CID_FLYN384_BASE + 12)
#define V4L2_CID_FLYN384_DENOISE_2D         (V4L2_CID_FLYN384_BASE + 13)

#define V4L2_CID_FLYN384_COMPRESS_FIRST (V4L2_CID_FLYN384_BASE + 0x10)
#define V4L2_CID_FLYN384_COMPRESS_Y0 (V4L2_CID_FLYN384_BASE + 0x10)
#define V4L2_CID_FLYN384_COMPRESS_Y1 (V4L2_CID_FLYN384_BASE + 0x11)
#define V4L2_CID_FLYN384_COMPRESS_Y2 (V4L2_CID_FLYN384_BASE + 0x12)
#define V4L2_CID_FLYN384_COMPRESS_Y3 (V4L2_CID_FLYN384_BASE + 0x13)
#define V4L2_CID_FLYN384_COMPRESS_Y4 (V4L2_CID_FLYN384_BASE + 0x14)

#define V4L2_CID_FLYN384_COMPRESS_A0 (V4L2_CID_FLYN384_BASE + 0x20)
#define V4L2_CID_FLYN384_COMPRESS_A1 (V4L2_CID_FLYN384_BASE + 0x21)
#define V4L2_CID_FLYN384_COMPRESS_A2 (V4L2_CID_FLYN384_BASE + 0x22)
#define V4L2_CID_FLYN384_COMPRESS_A3 (V4L2_CID_FLYN384_BASE + 0x23)
#define V4L2_CID_FLYN384_COMPRESS_A4 (V4L2_CID_FLYN384_BASE + 0x24)

#define V4L2_CID_FLYN384_COMPRESS_B0 (V4L2_CID_FLYN384_BASE + 0x30)
#define V4L2_CID_FLYN384_COMPRESS_B1 (V4L2_CID_FLYN384_BASE + 0x31)
#define V4L2_CID_FLYN384_COMPRESS_B2 (V4L2_CID_FLYN384_BASE + 0x32)
#define V4L2_CID_FLYN384_COMPRESS_B3 (V4L2_CID_FLYN384_BASE + 0x33)
#define V4L2_CID_FLYN384_COMPRESS_B4 (V4L2_CID_FLYN384_BASE + 0x34)
#define V4L2_CID_FLYN384_COMPRESS_LAST (V4L2_CID_FLYN384_BASE + 0x34)

#define V4L2_CID_FLYN384_DEVICE_ID   (V4L2_CID_FLYN384_BASE + 0x40)
#define V4L2_CID_FLYN384_SERIAL_SENS (V4L2_CID_FLYN384_BASE + 0x41)
#define V4L2_CID_FLYN384_SERIAL_FLYN (V4L2_CID_FLYN384_BASE + 0x42)
#define V4L2_CID_FLYN384_REVISION    (V4L2_CID_FLYN384_BASE + 0x43)
#define V4L2_CID_FLYN384_SENSOR_TYPE (V4L2_CID_FLYN384_BASE + 0x48)

#endif 

struct fmt_info;

class TFlowCaptureV4L2 : public TFlowBufSrv {
public:

    TFlowCaptureV4L2(MainContextPtr context,
        const std::string& _buf_srv_name,
        const std::string& _buf_sck_name,
        int buffs_num, int planes_num,
        const TFlowCtrlCapture::cfg_v4l2_ctrls* cfg,
        std::function<int(TFlowBuf &buf)> _app_onBuf_cb,
        std::function<int(const TFlowBufPck::pck& in_msg)> _app_onCustomMsg_cb);

    ~TFlowCaptureV4L2();

    // Str
    int onCustomMsg(const TFlowBufPck::pck& in_msg);

    int Init();
    void Deinit();
   
    // Query device information
    int  ioctlQueryCapability();
    void getDriverName();
    void getSensorType();
    int  ioctlEnumFmt();

    // Get/set paramters
    int  ioctlSetControls();
    int  ioctlSetControls_ISI();
    int  ioctlSetControls_flyn();
    int  ioctlSetControls_flyn_coin417g2();
    int  ioctlSetControls_flyn_twin412g2();
    int  ioctlSetControls_flyn_twin412();
    int  ioctlSetControls_flyn_comp_en(int on_off);
    int  ioctlSetControls_flyn_comp_time(int minutes);
    int  ioctlSetControls_flyn_comp_trig();
    int  ioctlSetControls_flyn_int(const TFlowCtrl::tflow_cmd_field_t *fld, uint32_t id);
    int  ioctlSetControls_flyn_int(const TFlowCtrl::tflow_cmd_field_t *fld, int val, uint32_t id);

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

    // Interface to TFlowBufSrv 
    void buf_queue(int index) override;
    int  buf_dev_fd() override;
    void buf_dev_fmt(TFlowBufPck::pck_fd* pck_fd) override;

    int is_stall();                              //  Should be called periodiacally. For ex. from the idle loop.

    enum class SENSOR_API_TYPE {
        UNDEF          = 0,
        ATIC           = 1,
        FLYN_COIN417G2 = 2,
        FLYN_TWIN412   = 3,
        FLYN_TWIN412G2 = 4
    } sensor_api_type;

    int dev_fd;                                  // Camera ISI device
    int sub_dev_fd;                              // Camera sensor device
    int sensor_type;                             // Read from the FLYN device via IOCTL

    int f_in_fd;

    int buffs_num;          // Initialized on creation by callee 
    int planes_num;         // Initialized on creation by callee 

    const struct fmt_info *stream_fmt;   // one of fmt_info_enum
    std::vector<struct fmt_info>    fmt_info_enum;

    std::string driver_name;

    v4l2_capability capa;
    v4l2_buf_type fmt_type;

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
    static constexpr int STREAM_WDT_CNT = 10;  // In Idle intervals (IDLE_INTERVAL_MSEC 300)

    IOSourcePtr io_in_src;

    v4l2_buffer v4l2_buf_template;
    std::vector<v4l2_plane> mplanes_template;

    const TFlowCtrlCapture::cfg_v4l2_ctrls *cfg;

    /* Statistics counters */
    int stat_cnt_frames_num;
};
