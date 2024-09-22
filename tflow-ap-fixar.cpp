#include <string.h>
#include "tflow-ap-fixar.h"

int AP_FIXAR::get_btr_dst(uint8_t** dst)
{
    *dst = ((uint8_t*)&in_bb_msg.raw) + wr_idx;

    if (wr_idx < sizeof(struct ap_fixar_hdr)) {
        // waiting for header
        return sizeof(struct ap_fixar_hdr) - wr_idx;
    }
    else {
        // header received - waiting for body + checksum
        size_t msg_len_full = sizeof(struct ap_fixar_hdr) + in_bb_msg.hdr.len16 * 16 + 2;   // +2 for chck sum
        return msg_len_full - wr_idx;
    }
}

int AP_FIXAR::data_in(ssize_t bytes_read)
{
    // TODO: ? implement sync and check sum error statistics counters ? 

    wr_idx += bytes_read;

    if (wr_idx < sizeof(struct ap_fixar_hdr)) {
        return 0; // Keep waiting for header (aka syncing)
    }

    if (wr_idx == sizeof(struct ap_fixar_hdr)) {
        // Check header's mark. If out of sync, then shift header by one byte and check again
        while (in_bb_msg.hdr.mark != 'g' && wr_idx){
            in_bb_msg.hdr_raw = in_bb_msg.hdr_raw >> 8;
            wr_idx--;
        }

        if (wr_idx < sizeof(struct ap_fixar_hdr)) return 0;   // Header is not fully received yet

        if (in_bb_msg.hdr.crypter != 'A') wr_idx = 0;         // Bad crypto, restart header reception
        return 0;
    }

    // Header received - check length and chcksum
    size_t msg_len_full = sizeof(struct ap_fixar_hdr) + in_bb_msg.hdr.len16 * 16 + 2;   // +2 for chck sum
    if (wr_idx < msg_len_full) return 0; // Keep waiting for body

    wr_idx = 0;
    
    // Full message received - calculate checksum
    if (!chcksum_ok()) return 0;

    return 1;
}

int AP_FIXAR::chcksum_ok()
{
    uint8_t ck_1 = 0;
    uint8_t ck_2 = 0;
    size_t msg_len = sizeof(struct ap_fixar_hdr) + in_bb_msg.hdr.len16 * 16;

    uint8_t* p = (uint8_t*)&in_bb_msg;
    for (int i = 0; i < msg_len; i++) {
        ck_1 = ck_1 + *p++;
        ck_2 = ck_2 + ck_1;
    }

    if (ck_1 != p[0] || ck_2 != p[1]) return false;

    return true;
}

#if 0
void AP_FIXAR::ap_fixar_pe_betoh(AP_FIXAR::ap_fixar_pe& pe_dst, AP_FIXAR::ap_fixar_pe& pe_src)
{
    BETOH_FLOAT(pe_dst.PE_est_pos_x   , pe_src.PE_est_pos_x   );
    BETOH_FLOAT(pe_dst.PE_est_pos_y   , pe_src.PE_est_pos_y   );
    BETOH_FLOAT(pe_dst.PE_est_pos_z   , pe_src.PE_est_pos_z   );
    BETOH_FLOAT(pe_dst.PE_est_vel_x   , pe_src.PE_est_vel_x   );
    BETOH_FLOAT(pe_dst.PE_est_vel_y   , pe_src.PE_est_vel_y   );
    BETOH_FLOAT(pe_dst.PE_est_vel_z   , pe_src.PE_est_vel_z   );
    BETOH_FLOAT(pe_dst.PE_est_eph     , pe_src.PE_est_eph     );
    BETOH_FLOAT(pe_dst.PE_est_epv     , pe_src.PE_est_epv     );
    BETOH_FLOAT(pe_dst.PE_imu_accNEU_x, pe_src.PE_imu_accNEU_x);
    BETOH_FLOAT(pe_dst.PE_imu_accNEU_y, pe_src.PE_imu_accNEU_y);
    BETOH_FLOAT(pe_dst.PE_imu_accNEU_z, pe_src.PE_imu_accNEU_z);
    BETOH_FLOAT(pe_dst.PE_baro_alt    , pe_src.PE_baro_alt    );
    BETOH_FLOAT(pe_dst.PE_gps_pos_x   , pe_src.PE_gps_pos_x   );
    BETOH_FLOAT(pe_dst.PE_gps_pos_y   , pe_src.PE_gps_pos_y   );
    BETOH_FLOAT(pe_dst.PE_gps_pos_z   , pe_src.PE_gps_pos_z   );
    BETOH_FLOAT(pe_dst.PE_gps_vel_x   , pe_src.PE_gps_vel_x   );
    BETOH_FLOAT(pe_dst.PE_gps_vel_y   , pe_src.PE_gps_vel_y   );
    BETOH_FLOAT(pe_dst.PE_gps_vel_z   , pe_src.PE_gps_vel_z   );
    BETOH_FLOAT(pe_dst.PE_gps_eph     , pe_src.PE_gps_eph     );
    BETOH_FLOAT(pe_dst.PE_gps_epv     , pe_src.PE_gps_epv     );
    BETOH_FLOAT(pe_dst.PE_surf_alt    , pe_src.PE_surf_alt    );
    BETOH_FLOAT(pe_dst.PE_surf_rel    , pe_src.PE_surf_rel    );
    BETOH_FLOAT(pe_dst.PE_est_AGL_alt , pe_src.PE_est_AGL_alt );
    BETOH_FLOAT(pe_dst.PE_est_AGL_vel , pe_src.PE_est_AGL_vel );
}
void AP_FIXAR::ap_fixar_cas_betoh(AP_FIXAR::ap_fixar_cas& cas_dst, AP_FIXAR::ap_fixar_cas& cas_src)
{
    BETOH_FLOAT(cas_dst.CAS_board_rate_yaw   , cas_src.CAS_board_rate_yaw   );
    BETOH_FLOAT(cas_dst.CAS_board_rate_roll  , cas_src.CAS_board_rate_roll  );
    BETOH_FLOAT(cas_dst.CAS_board_rate_pitch , cas_src.CAS_board_rate_pitch );
    BETOH_FLOAT(cas_dst.CAS_board_att_roll   , cas_src.CAS_board_att_roll   );
    BETOH_FLOAT(cas_dst.CAS_board_att_yaw    , cas_src.CAS_board_att_yaw    );
    BETOH_FLOAT(cas_dst.CAS_board_att_pitch  , cas_src.CAS_board_att_pitch  );
    BETOH_FLOAT(cas_dst.CAS_board_acc_x      , cas_src.CAS_board_acc_x      );
    BETOH_FLOAT(cas_dst.CAS_board_acc_y      , cas_src.CAS_board_acc_y      );
    BETOH_FLOAT(cas_dst.CAS_board_acc_z      , cas_src.CAS_board_acc_z      );
    BETOH_FLOAT(cas_dst.CAS_copter_rate_roll , cas_src.CAS_copter_rate_roll );
    BETOH_FLOAT(cas_dst.CAS_copter_rate_yaw  , cas_src.CAS_copter_rate_yaw  );
    BETOH_FLOAT(cas_dst.CAS_copter_rate_pitch, cas_src.CAS_copter_rate_pitch);
    BETOH_FLOAT(cas_dst.CAS_copter_att_roll  , cas_src.CAS_copter_att_roll  );
    BETOH_FLOAT(cas_dst.CAS_copter_att_yaw   , cas_src.CAS_copter_att_yaw   );
    BETOH_FLOAT(cas_dst.CAS_copter_att_pitch , cas_src.CAS_copter_att_pitch );
    BETOH_FLOAT(cas_dst.CAS_copter_acc_x     , cas_src.CAS_copter_acc_x     );
    BETOH_FLOAT(cas_dst.CAS_copter_acc_y     , cas_src.CAS_copter_acc_y     );
    BETOH_FLOAT(cas_dst.CAS_copter_acc_z     , cas_src.CAS_copter_acc_z     );
}
#endif
