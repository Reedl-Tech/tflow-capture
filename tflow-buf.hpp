#pragma once

#include <glib-unix.h>

/* Class shared between client and server */

class TFlowBuf {
public:

    TFlowBuf();
    ~TFlowBuf();
     
    const char* sign = "TFlowBuf";

    /* Parameters passed from server */
    int index;
    struct timeval ts;
    uint32_t sequence;

    /* Parameters obtained from Kernel*/
    void* start;            // Not used on Server side
    size_t length;          // Not used on Server side

    uint32_t owners;        // Bit mask of TFlowBufCli. Bit 0 - means buffer is in user space

    int age();

    /* 
     * Non camera related data 
     * Server's owner may put auxiliary data here, from the onBuf callback
     * This data will be sent to all TFlowBuf clients
     * max data len defined by TFlowBufPck::pck_consume.aux_data (512)
     */

    uint32_t aux_data_len;
    const uint8_t* aux_data;
};

