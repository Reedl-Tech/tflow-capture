#pragma once 

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stddef.h>
#include <memory.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <jpeglib.h>

#include "tflow-capture.h"

class MJPEGCapture
{
public:
    ~MJPEGCapture();

#pragma pack(push, 1)
    struct ap_data {
        uint32_t sign;
        uint32_t tv_sec;      // Local timestamp (struct timeval)
        uint32_t tv_usec;     // Local timestamp
        uint32_t log_ts;      // Timestamp received from AP
        int32_t roll;         // In degrees * 100
        int32_t pitch;        // In degrees * 100
        int32_t yaw;          // In degrees * 100
        int32_t altitude;     // In meters * 100 ?
        int32_t pos_x;
        int32_t pos_y;
        int32_t pos_z;

        uint32_t flightModeFlags;
        uint32_t stateFlags;
        uint32_t hwHealthStatus;
        uint8_t  failsafePhase;
        uint8_t  receiver_status;
    };

#pragma pack(pop)

    std::string fname;
    uint32_t frames_count;

    uint32_t width;
    uint32_t height;
    uint32_t format;

    fpos_t offset_pos;
    int offset_frame;

    int open(const std::string& fname, int offset);
    int setupJPEGLib(int buffs_num);
    int decompressNext(int buff_idx, MJPEGCapture::ap_data *aux_data);
    void setupRowPointers(int idx, uint8_t *buf);
    int rewind(int frame_num);

private:
    FILE* f;

#pragma pack(push, 1)
    struct tflow_mjpeg_frame {
        char sign[4];
        uint32_t width;
        uint32_t height;
        uint32_t format;
        struct timeval ts;
        struct ap_data aux_data;
        uint32_t jpeg_sz;
    };
#pragma pack(pop)

    struct jpeg_decompress_struct   cinfo = { 0 };
    struct jpeg_error_mgr           jerr = { 0 };

    unsigned char* in_buff;
    uint32_t                        in_buff_size_max;

    struct  tflow_mjpeg_frame       frame_info;
    JSAMPARRAY                      row_pointers;   // Pointers to frame's rows

    long                            out_size;

    std::vector<fpos_t>             frame_offset_map;

    void cleanJPEGLib();

    void getIMU(MJPEGCapture::ap_data *aux_data);

};

class TFlowPlayer {
public:

    TFlowPlayer(MainContextPtr context, int buffs_num, float fps);
    ~TFlowPlayer();

    int Init(const char* media_file_name);
    void Deinit();

    int  StreamOn();
    int onTick();

    int media_file_fd { -1 };

    TFlowBufSrv* buf_srv;   // Server to pass V4L2 buffers
    int buffs_num;          // Initialized on creation by callee 
    
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t frame_format;       // 4c V4L2_PIX_FMT_GREY

    // Filled by MJPEGCapture, used by TFlowCapture on TFlowBuf preparation
    struct shm_entry {
        uint8_t* data;
        MJPEGCapture::ap_data *aux_data;
        int owner_player;
    };

    void *shm_obj;
    off_t  shm_size;
    struct shm_entry *shm_tbl;
    int shm_fd;
    
    int shmQuery();
    int getNextFrame(int buff_idx);
    int shmQueueBuffer(int buff_idx);

private:

    int  Open(const char* media_filename);
    void Close();

    MainContextPtr context;  // Context for pending events

    int frames_count;
    const char* m_fname{ nullptr };
    bool is_streaming{ false };

    MJPEGCapture mjpegCapture;
    v4l2_buffer v4l2_buf_template;
};

