/*
 * Copyright (C) All rights reserved.
 */
#ifndef __V4L2DEVICE_H__
#define __V4L2DEVICE_H__

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

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define BUFFER_NUM  4   // Number of buffers
#define PLANE_NUM   1   // Number of planes

#define IMAGEHEIGHT 240
#define IMAGEWIDTH  320

class V4L2Device {
public:
    V4L2Device();
    ~V4L2Device();

    int CaptureInit(const char* dev_name);
    void* buffGet(int i);

    int  openDevice(const char* filename="/dev/video0");
    void closeDevice();

    // Query device information
    int  ioctlQueryCapability();
    int  ioctlEnumFmt();

    // Get/set paramters
    int  ioctlGetStreamParm();
    int  ioctlSetStreamParm(bool highQuality = false, u_int timeperframe = 30);
    int  ioctlGetStreamFmt();
    int  ioctlSetStreamFmt(u_int pixelformat, u_int width, u_int height); 

    //Initialize buffers
    int  ioctlRequestBuffers();
    int  ioctlMmapBuffers();
    void unMmapBuffers();

    //Frame capture
    int  ioctlQueueBuffer(int index);
    int  ioctlDequeueBuffer(int *index, struct timeval *ts);

    int  ioctlSetStreamOn();
    void ioctlSetStreamOff();

    int cam_fd {-1};

private:
    const char* m_fname {nullptr};
    volatile bool m_isStreamOn {false};

    struct Buffer {
        void* start;
        size_t length;
    };
    struct Buffer* m_buffers {nullptr};
};

#endif //__V4L2DEVICE_H__

