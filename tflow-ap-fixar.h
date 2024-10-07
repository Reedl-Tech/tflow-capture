#pragma once
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

//#define BETOH_FLOAT(f) (*(uint32_t*)&f = htobe32(*(uint32_t *)&f))
#define BETOH_FLOAT(d,s) (*(uint32_t*)&d = htobe32(*(uint32_t *)&s))

static_assert (sizeof(float) == sizeof(uint32_t));

class AP_FIXAR {
public:
#define AP_FIXAR_MSG_ID_PE      0x6D
#define AP_FIXAR_MSG_ID_CAS     0x6E
#define AP_FIXAR_MSG_ID_SENSORS 0x64
#define AP_FIXAR_MSG_ID_STATUS  0x30

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

    struct ap_fixar_status {
        struct ap_fixar_hdr hdr;
        uint8_t msg_id;
        uint32_t ts;
        uint32_t flightModeFlags;
        uint32_t stateFlags;
        uint32_t hwHealthStatus;
        uint8_t  failsafePhase;
        uint8_t  receiver_status;
    };

    struct ap_fixar_sensors {
        struct ap_fixar_hdr hdr;
        uint8_t msg_id;
        uint32_t ts;
        int32_t           sens_BaroAlt;         // Off:   5, Altitude Baro,       cm,
        int32_t           sens_AirSpeed;        // Off:   9, Air Speed pitot,     cm/s,
        int16_t           sens_ADC_amperage;    // Off:  13, ADC amperage,
        int32_t           sens_amperage_raw;    // Off:  15, Amperage raw,        centiAmps,
        int16_t           sens_ADC_voltage;     // Off:  19, ADC voltage,
        int16_t           sens_voltage_raw;     // Off:  21, Voltage raw,         V,
        int32_t           sens_power_raw;       // Off:  23, Power draw raw,      0.01W,
        int32_t           sens_mahDrawn;        // Off:  27, Used energy,         mAh,
        int32_t           sens_mwhDrawn_raw;    // Off:  31, Used energy,         0.01Wh,
        int16_t           stmload;              // Off:  35, system load,         %,
        float             gyroADCf0;            // Off:  37, MPU gyro Roll,       deg/s,
        float             gyroADCf1;            // Off:  41, MPU gyro Pitch,      deg/s,
        float             gyroADCf2;            // Off:  45, MPU gyro Yaw,        deg/s,
        float             accSmooth0;           // Off:  49, MPU acc North,       1g,
        float             accSmooth1;           // Off:  53, MPU acc East,        1g,
        float             accSmooth2;           // Off:  57, MPU acc Vertical,    1g,
        int16_t           attitude0_raw;        // Off:  61, attitude roll,       deg,
        int16_t           attitude1_raw;        // Off:  63, attitude pitch,      deg,
        int16_t           attitude2_raw;        // Off:  65, attitude yaw,        deg,
        float             MTI_roll;             // Off:  67, IMU Roll,            deg,
        float             MTI_yaw;              // Off:  71, IMU Yaw,             deg,
        float             MTI_pitch;            // Off:  75, IMU Pitch,           deg,
        float             MPU_roll;             // Off:  79, IMU Stable Roll,     deg,
        float             MPU_yaw;              // Off:  83, IMU Stable Yaw,      deg,
        float             MPU_pitch;            // Off:  87, IMU Stable Pitch,    deg,
        int16_t           rangefinderRaw;       // Off:  91, Range Raw,           cm,
        int16_t           rangefinderLDT;       // Off:  93, Range detect,        cm,
        uint8_t           battPercent;          // Off:  95, Capacity,            %,
        float             MTI_accx;             // Off:  96, MTI accX,            m/s²,
        float             MTI_accy;             // Off: 100, MTI accY,            m/s²,
        float             MTI_accz;             // Off: 104, MTI accZ,            m/s²,
        int32_t           sens_remainCapacity;  // Off: 108, Remain energy,       mAh, 
        float             rf_meas_x;            // Off: 112, rangefinder x,       m,
        float             rf_meas_y;            // Off: 116, rangefinder y,       m,
        float             rf_meas_z;            // Off: 120, rangefinder z,       m,
        float             VN100_Roll;           // Off: 124, VN100 ROLL,          deg,
        float             VN100_Yaw;            // Off: 128, VN100 YAW,           deg,
        float             VN100_Pitch;          // Off: 132, VN100 PITCH,         deg,
        float             VN100_ax;             // Off: 136, VN100 accX,          m/s²,
        float             VN100_ay;             // Off: 140, VN100 accY,          m/s²,
        float             VN100_az;             // Off: 144, VN100 accZ,          m/s²,
        float             VN100_gx;             // Off: 148, VN100 gyroX,         m/s²,
        float             VN100_gy;             // Off: 152, VN100 gyroY,         m/s²,
        float             VN100_gz;             // Off: 156, VN100 gyroZ,         m/s²,
    };

#pragma pack(pop)

    union ap_fixar_msg {
        uint32_t hdr_raw;               // Just to avoid casting
        struct ap_fixar_hdr hdr;        // Header only up to lenght
        struct ap_fixar_generic common; // Header + fields common for all messages (id, timestamp)
        
        struct ap_fixar_pe     pe;      // Position Estimation
        struct ap_fixar_cas    cas;     // Current Axis State
        struct ap_fixar_status status;  // Statuses
        struct ap_fixar_sensors sensors; // Statuses

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


enum flightModeFlags {
    ANGLE_MODE,
    HORIZON_MODE,
    HEADING_MODE,
    NAV_ALTHOLD_MODE,
    NAV_RTH_MODE,
    NAV_POSHOLD_MODE,
    HEADFREE_MODE,
    NAV_LAUNCH_MODE,
    MANUAL_MODE,
    FAILSAFE_MODE,
    AUTO_TUNE,
    NAV_WP_MODE,
    UNUSED_MODE2,
    FLAPERON
};

enum stateFlags {
    GPS_FIX_HOME,
    GPS_FIX,
    CALIBRATE_MAG,
    SMALL_ANGLE,
    FIXED_WING,
    ANTI_WINDUP,
    FLAPERON_AVAILABLE,
    NAV_MOTOR_STOP_OR_IDLE,
    COMPASS_CALIBRATED,
    ACCELEROMETER_CALIBRATED,
    PWM_DRIVER_AVAILABLE,
    HELICOPTER
};

enum HW_SENSOR_STATUS {
    HW_SENSOR_NONE = 0,
    HW_SENSOR_OK = 1,
    HW_SENSOR_UNAVAILABLE = 2,
    HW_SENSOR_UNHEALTHY = 3
};

struct hwHealth {
    enum HW_SENSOR_STATUS gyro;
    enum HW_SENSOR_STATUS axel;
    enum HW_SENSOR_STATUS compas;
    enum HW_SENSOR_STATUS baro;
    enum HW_SENSOR_STATUS gps;
    enum HW_SENSOR_STATUS rangefinder;
    enum HW_SENSOR_STATUS pitot;
    enum HW_SENSOR_STATUS posest;
    enum HW_SENSOR_STATUS gps_yaw;
    enum HW_SENSOR_STATUS imu_mpu;
    enum HW_SENSOR_STATUS imu_mti;
};














































