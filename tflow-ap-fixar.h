#pragma once
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

//#define BETOH_FLOAT(f) (*(uint32_t*)&f = htobe32(*(uint32_t *)&f))
#define BETOH_FLOAT(d,s) (*(uint32_t*)&d = htobe32(*(uint32_t *)&s))

static_assert (sizeof(float) == sizeof(uint32_t));

class AP_FIXAR {
public:
#define AP_FIXAR_MSG_ID_PE  0x6D
#define AP_FIXAR_MSG_ID_CAS 0x6E

#pragma pack(push, 1)
    struct ap_fixar_hdr {
        uint8_t mark;       // 'g' 0x67
        uint8_t crypter;    // 'A' 0x41
        uint8_t dir;
        uint8_t len16;      // len in 16byte blocks
    };
    // Serial parser assumes that header is always 4 bytes long (uint32_t)
    static_assert (sizeof(struct ap_fixar_hdr) == sizeof(uint32_t), "bad ap header");

    struct ap_fixar_generic {
        struct ap_fixar_hdr hdr;
        uint8_t msg_id;
        uint32_t ts;
    };

    struct ap_fixar_pe {
        struct ap_fixar_hdr hdr;
        uint8_t msg_id;
        uint32_t ts;
        int32_t PE_est_pos_x;
        int32_t PE_est_pos_y;
        int32_t PE_est_pos_z;
        int32_t PE_est_vel_x;
        int32_t PE_est_vel_y;
        int32_t PE_est_vel_z;
        int32_t PE_est_eph;
        int32_t PE_est_epv;
        int32_t PE_imu_accNEU_x;
        int32_t PE_imu_accNEU_y;
        int32_t PE_imu_accNEU_z;
        int32_t PE_baro_alt;
        int32_t PE_gps_pos_x;
        int32_t PE_gps_pos_y;
        int32_t PE_gps_pos_z;
        int32_t PE_gps_vel_x;
        int32_t PE_gps_vel_y;
        int32_t PE_gps_vel_z;
        int32_t PE_gps_eph;
        int32_t PE_gps_epv;
        int32_t PE_surf_alt;
        int32_t PE_surf_rel;
        int32_t PE_est_AGL_alt;
        int32_t PE_est_AGL_vel;
        uint8_t PE_est_AGL_qual;
    };

    struct ap_fixar_cas {
        struct ap_fixar_hdr hdr;
        uint8_t msg_id;
        uint32_t ts;
        int32_t CAS_board_rate_roll;
        int32_t CAS_board_rate_yaw;
        int32_t CAS_board_rate_pitch;
        int32_t CAS_board_att_roll;
        int32_t CAS_board_att_yaw;
        int32_t CAS_board_att_pitch;
        int32_t CAS_board_acc_x;
        int32_t CAS_board_acc_y;
        int32_t CAS_board_acc_z;
        int32_t CAS_copter_rate_roll;
        int32_t CAS_copter_rate_yaw;
        int32_t CAS_copter_rate_pitch;
        int32_t CAS_copter_att_roll;
        int32_t CAS_copter_att_yaw;
        int32_t CAS_copter_att_pitch;
        int32_t CAS_copter_acc_x;
        int32_t CAS_copter_acc_y;
        int32_t CAS_copter_acc_z;
    };
#pragma pack(pop)
    union ap_fixar_msg {
        uint32_t hdr_raw;               // Just to avoid casting
        struct ap_fixar_hdr hdr;        // Header only up to lenght
        struct ap_fixar_generic common; // Header + fields common for all messages (id, timestamp)
        struct ap_fixar_pe pe;          // Position Estimation
        struct ap_fixar_cas cas;        // Cuurrent Axis State

        uint8_t raw[256 + 1][16];       // Maximum possible packet 4kiB+4
    } in_bb_msg;

    int get_btr_dst(uint8_t** dst);
    int data_in(ssize_t bytes_read);
    
    //static void ap_fixar_cas_betoh(ap_fixar_cas &cas_dst, ap_fixar_cas& cas_src);
    //static void ap_fixar_pe_betoh(ap_fixar_pe& pe_dst, ap_fixar_pe& pe_src);

private:
    int wr_idx;
    int chcksum_ok();
};