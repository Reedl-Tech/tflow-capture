#include <features.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h> /* For mode constants */
#include <sys/types.h>

#include <giomm.h>
#include <glib-unix.h>

using namespace std;

#include "tflow-capture.h"
#include "tflow-player.h"

static gboolean player_timer_dispatch(gpointer data)
{
    TFlowPlayer* player = (TFlowPlayer*)data;

    return G_SOURCE_CONTINUE;

    int rc = player->onTick();
    if (rc) {
        // Close the file???
        // return G_SOURCE_REMOVE;
    }

    return G_SOURCE_CONTINUE;
}


int MJPEGCapture::rewind(int frame_num)
{
    if (frame_offset_map.size() <= frame_num) {
        return -1;
    }

    fpos_t new_pos = frame_offset_map.at(frame_num);
    fsetpos(f, &new_pos);
    return 0;
}

void MJPEGCapture::getIMU(MJPEGCapture::imu_data *imu)
{
    memset(imu, 0, sizeof(MJPEGCapture::imu_data));
    memcpy(imu, &frame_info.imu, sizeof(MJPEGCapture::imu_data));
}

int MJPEGCapture::decompressNext(int buff_idx, MJPEGCapture::imu_data *imu)
{
    size_t res;
    res = fread(&frame_info, sizeof(frame_info), 1, f);

    if (res <= 0) {
        if (feof(f)) {
            g_debug("--- end of file ---\r\n");
            return 0;
        }
        else if (ferror(f)) {
            g_warning("Error reading file - %s (%d)\r\n", strerror(errno), errno);
            return -1;
        }
    }
    if (*(uint32_t*)&frame_info.sign != 0x302E5452) {        // RT.0
        g_warning("File %s corrupted (bad sign)\r\n", fname.c_str());
        assert(0);
    }

    res = fread(in_buff, 1, frame_info.jpeg_sz, f);

    if (res < 0) {
        if (feof(f)) {
            g_debug("--- end of file ---\r\n");
            return -1;
        }
        else if (ferror(f)) {
            g_warning("Error reading file - %s (%d)\r\n", strerror(errno), errno);
            return -1;
        }
    }

    if (res != frame_info.jpeg_sz) {
        g_warning("Error reading file - %s (%d)\r\n", strerror(errno), errno);
        return -1;
    }

    getIMU(imu);

    jpeg_mem_src(&cinfo, in_buff, (unsigned long)frame_info.jpeg_sz);
    jpeg_save_markers(&cinfo, JPEG_COM, 1024);
    jpeg_read_header(&cinfo, TRUE);

    // Sanity check - JPEG parameters (color space & resolution) must 
    // corresponds the allocated out_buff size.
    assert(out_size == cinfo.image_height * cinfo.image_width * cinfo.num_components);

    JSAMPARRAY buff_row_pointers = &row_pointers[buff_idx * height];

    jpeg_start_decompress(&cinfo);
    while (cinfo.output_scanline < cinfo.output_height) {
        // ATT!: All lines must be read out from the image! Otherwise the critical error will be hit.
        //       Critical error can be suppressed by custom error handler.
        // ATT!: jpeglib process not the number of lines specified, but
        //       UP TO that number. Actually it processes 1 line per call.
        jpeg_read_scanlines(&cinfo, &buff_row_pointers[cinfo.output_scanline], 16);
    }
    jpeg_finish_decompress(&cinfo);

    return 1;
}

static void jpeg_error_exit(j_common_ptr cinfo)
{
    assert(1);
}

void MJPEGCapture::setupRowPointers(int buff_idx, uint8_t *buf)
{
    // Create out buffer & Initialize output buffer's rows table
    JSAMPARRAY buff_row_pointers = &row_pointers[buff_idx * height];

    JSAMPROW  row_ptr = (JSAMPROW)buf;
    for (int row = 0; row < height; row++) {
        buff_row_pointers[row] = row_ptr;
        row_ptr += width;
    }
}

MJPEGCapture::~MJPEGCapture()
{
    cleanJPEGLib();
}

void MJPEGCapture::cleanJPEGLib()
{
    if (row_pointers) {
        free(row_pointers);
        row_pointers = nullptr;
    }

    if (in_buff) {
        free(in_buff);
        in_buff = nullptr;
    }
    
    jpeg_destroy_decompress(&cinfo);

}

int MJPEGCapture::setupJPEGLib(int buffs_num)
{
    JSAMPROW    row_ptr;

    assert(height != -1);

    row_pointers = (JSAMPARRAY)malloc(sizeof(JSAMPROW) * height * buffs_num);
    if (row_pointers == NULL) return -1;

    // Create in buffer. Should be big enough to hold a whole compressed picture
    in_buff = (unsigned char*)malloc(in_buff_size_max);

    memset(&cinfo, 0, sizeof(cinfo));
    cinfo.err = jpeg_std_error(&jerr);

    jerr.error_exit = jpeg_error_exit;
    cinfo.client_data = nullptr;

    jpeg_create_decompress(&cinfo);

    return 0;
}

int MJPEGCapture::open(const std::string& fname, int _offset_frame)
{
    uint32_t _width = -1;
    uint32_t _height = -1;
    uint32_t _format = -1;

    tflow_mjpeg_frame frame;

    frame_offset_map.reserve(20 * 60 * 60); // ~20mins at 60Hz

    f = fopen(fname.c_str(), "rb");
    offset_frame = _offset_frame;
    // Index the file 
    fgetpos(f, &offset_pos);
    frames_count = 0;
    in_buff_size_max = 0;
    while (true) {
        fpos_t frame_pos;
        fgetpos(f, &frame_pos);

        size_t rd_res = fread(&frame, sizeof(frame), 1, f);
        if (rd_res <= 0) {
            if (feof(f)) {
                g_debug("File %s parsed ok - %d frames\r\n",
                    fname.c_str(), frames_count);
                break;
            }
            else if (ferror(f)) {
                g_warning("Error reading file %s - %s (%d)\r\n",
                    fname.c_str(), strerror(errno), errno);
                return -1;
            }
        }

        // check sign
        if (*(uint32_t*)&frame.sign != 0x302E5452) {        // RT.0
            g_warning("File %s corrupted (bad sign)\r\n", fname.c_str());
            return -1;
        }

        int seek_res = fseek(f, frame.jpeg_sz, SEEK_CUR);
        if (seek_res < 0) {
            if (feof(f)) {
                g_debug("Unexpected EOF %s\r\n", fname.c_str());
                return -1;//    TODO: truncated file should be processed as well
            }
            else if (ferror(f)) {
                g_warning("Error reading file %s - %s (%d)\r\n",
                    fname.c_str(), strerror(errno), errno);
                return -1;
            }
        }

        if (offset_frame && (offset_frame - 1) == frames_count) {
            fgetpos(f, &offset_pos);
        }

        if (frames_count >= (uint32_t)offset_frame) {
            frame_offset_map.push_back(frame_pos);
        }
        in_buff_size_max = std::max(frame.jpeg_sz, in_buff_size_max);
        //if (frame.jpeg_sz > in_buff_size_max) in_buff_size_max = frame.jpeg_sz;

        if (_width == -1) {
            _width = frame.width;
            _height = frame.height;
            _format = frame.format;
        }
        else {
            if ((_width != frame.width) ||
                (_height != frame.height) ||
                (_format != frame.format)) {

                g_warning("Error - frame params changed\r\n");
                return -1;
            }
        }

        frames_count++;
    }

    // File fully parsed - rewind to specified offset
    fsetpos(f, &offset_pos);

    width  = _width;
    height = _height;
    format = _format;

    return 0;
}

TFlowPlayer::TFlowPlayer(GMainContext* _context, int _buffs_num, float fps)
{
    context = _context;
    buffs_num = _buffs_num;

    buf_srv = NULL;

    if (fps != 0.f) {
        int fps_interval = (int)roundf(1 / fps * 1000.f);

        fps_timer_src = g_timeout_source_new(fps_interval);
        g_source_set_callback(fps_timer_src, (GSourceFunc)player_timer_dispatch, this, nullptr);
        g_source_attach(fps_timer_src, _context);
    }

    CLEAR(v4l2_buf_template);

    v4l2_buf_template.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    v4l2_buf_template.memory = V4L2_MEMORY_MMAP;
    v4l2_buf_template.m.planes = NULL;
    v4l2_buf_template.length = 0;
    v4l2_buf_template.index = 0;

    // Set on media file open
    frame_width = -1;
    frame_height = -1;
    frame_format = -1;       // 4c V4L2_PIX_FMT_GREY

}

int TFlowPlayer::getNextFrame(int buff_idx)
{
    int res;

    res = mjpegCapture.decompressNext(buff_idx, shm_tbl[buff_idx].imu);
    if (res != 1) {
        return res;
    }

    msync(shm_obj, shm_size, MS_SYNC);

    return 1;
}

int TFlowPlayer::shmQueueBuffer(int buff_idx)
{
    // Buffer returned by TFlowBufSrv
    shm_tbl[buff_idx].owner_player = 1;
    return 0;
}

int TFlowPlayer::onTick()
{
    int free_buff_idx = -1;
    // Loop over all buffers and get 1st available
    // Fill it and pass to buf_srv
    for (int i = 0; i < buffs_num; i++) {
        struct shm_entry *shm = &shm_tbl[i];
        if (shm->owner_player) {
            free_buff_idx = i;
            break;
        }
    }

    if (free_buff_idx == -1) {
        // No more free buffers - nothing to do.
        return 0;
    }

    getNextFrame(free_buff_idx);
    
    shm_tbl[free_buff_idx].owner_player = 0;

    // Get empty shared mem buffer
    v4l2_buffer v4l2_buf = v4l2_buf_template; // TODO: Q: can it be reused without reinitialization?
    v4l2_buf.timestamp.tv_sec = 0;
    v4l2_buf.timestamp.tv_usec = 0;
    v4l2_buf.sequence = 0;
    v4l2_buf.index = free_buff_idx;

    buf_srv->buf_consume(v4l2_buf);

    return 0;
}

int TFlowPlayer::shmQuery()
{
    shm_tbl = (struct shm_entry*)g_malloc(buffs_num * sizeof(struct shm_entry));

    // Allocate shared memory buffer - local equivalent of v4l2 buffer
    shm_fd = shm_open("/tflow-capture-shm", O_CREAT | O_EXCL | O_RDWR, S_IWUSR);
    if (shm_fd == -1) {
        g_warning("Can't open shm - %s (%d)\r\n", strerror(errno), errno);
        return -1;
    }

    /*
     * Get total memory size
     * 
     *   gap       0xCAFE0000
     *      gap1       0xCAFE0001
     *      frame      w x h x pixel_size
     *      gap2       0xCAFE0002
     *      imu        fixed structure
     *      gap3       0xCAFE0003
     * 
     *      gap1       0xCAFE0011
     *      frame      w x h x pixel_size
     *      gap2       0xCAFE0012
     *      imu        fixed structure
     *      gap3       0xCAFE0013
     *         ....
     *   gap       0xCAFEFFFF
     */

    long frame_size = frame_width * frame_height *
        (frame_format == V4L2_PIX_FMT_GREY) ? 1 : 0;

    long total_mem = 0;
    total_mem += frame_size;
    total_mem += 3 * sizeof(uint32_t);  // 3x GAPs per frame
    total_mem += sizeof(MJPEGCapture::imu_data);
    total_mem *= buffs_num;
    total_mem += 2 * sizeof(uint32_t);  // 2x GAPs for leading and trailing gap
    shm_size = total_mem;

    int rc = ftruncate(shm_fd, shm_size);
    if (rc != 0) {
        g_warning("Can't rwsize shm - %s (%d)\r\n", strerror(errno), errno);
        return -1;
    }

    shm_obj = mmap(nullptr, shm_size, PROT_WRITE, MAP_SHARED, shm_fd, 0);

    uint8_t* shm_wr_ptr = (uint8_t*)shm_obj;

    *(uint32_t*)shm_wr_ptr = 0xCAFE0000; shm_wr_ptr += sizeof(uint32_t);
    for (int i = 0; i < buffs_num; i++) {
        *(uint32_t*)shm_wr_ptr = 0xCAFE0001 + i * 0x10; shm_wr_ptr += sizeof(uint32_t);
        shm_tbl[i].data = shm_wr_ptr; shm_wr_ptr += frame_size;
        *(uint32_t*)shm_wr_ptr = 0xCAFE0002 + i * 0x10; shm_wr_ptr += sizeof(uint32_t);
        shm_tbl[i].imu = (MJPEGCapture::imu_data *)shm_wr_ptr; shm_wr_ptr += sizeof(MJPEGCapture::imu_data);
        shm_tbl[i].owner_player = 1;
        *(uint32_t*)shm_wr_ptr = 0xCAFE0003 + i * 0x10; shm_wr_ptr += sizeof(uint32_t);
    }
    *(uint32_t*)shm_wr_ptr = 0xCAFEFFFF;

    return 0;
}

int TFlowPlayer::Init(const char* media_file_name)
{
    int offset_frame = 0;
    int res;

    res = mjpegCapture.open(std::string(media_file_name), offset_frame);
    if (res) return -1;

    frame_width  =  mjpegCapture.width;
    frame_height =  mjpegCapture.height;
    frame_format =  mjpegCapture.format;

    frames_count = mjpegCapture.frames_count - offset_frame;

    /* Width and height must be known here 
     * Obtained by .open()
     */
    
    res = shmQuery();
    if (res) return -1;
    
    res = mjpegCapture.setupJPEGLib(buffs_num);

    for (int i = 0; i < buffs_num; i++) {
        mjpegCapture.setupRowPointers(i, shm_tbl[i].data);
        buf_srv->buf_create(buffs_num);
    }

    return 0;
}


void TFlowPlayer::Deinit()
{
    if (shm_tbl) {
        free(shm_tbl);
        shm_tbl = nullptr;
    }

    if (shm_obj) {
        munmap(shm_obj, shm_size);
        shm_size = 0;
        shm_obj = nullptr;
    }

    shm_unlink("/tflow-capture-shm");
}

TFlowPlayer::~TFlowPlayer()
{
    if (fps_timer_src) {
        g_source_destroy((GSource*)fps_timer_src);
        g_source_unref((GSource*)fps_timer_src);
            fps_timer_src = nullptr;
    }

    if (is_streaming) {
        is_streaming = false;
    }

    Deinit();
}
