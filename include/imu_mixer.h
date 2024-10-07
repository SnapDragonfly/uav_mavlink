#ifndef IMU_MIXER_H
#define IMU_MIXER_H

typedef struct {
    uint32_t img_sec;  // Timestamp seconds
    uint32_t img_nsec; // Timestamp nanoseconds
    uint16_t reserved;
    uint16_t imu_num;  // Number of IMU data followed
} mix_head_t;

typedef struct {
    uint32_t imu_sec;  // Timestamp seconds
    uint32_t imu_nsec; // Timestamp nanoseconds
    float xacc;        // Linear acceleration X
    float yacc;        // Linear acceleration Y
    float zacc;        // Linear acceleration Z
    float xgyro;       // Angular velocity X
    float ygyro;       // Angular velocity Y
    float zgyro;       // Angular velocity Z
    float q_w;         // Quaternion W
    float q_x;         // Quaternion X
    float q_y;         // Quaternion Y
    float q_z;         // Quaternion Z
} imu_data_t;

#define MAGIC_IMU_FRAME_NUM      0xFFFF

#define FORWARD_RTP_IMU_NUM      10                  //uav_mixer settings, RPi3B+ 10 for try
#define FORWARD_RTP_IMG_LEN      sizeof(mix_head_t)
#define FORWARD_RTP_IMU_LEN      sizeof(imu_data_t)
#define FORWARD_RTP_MIX_LEN      (FORWARD_RTP_IMG_LEN+ FORWARD_RTP_IMU_NUM*FORWARD_RTP_IMU_LEN)
#define FORWARD_RTP_PREFIX_LEN   (FORWARD_RTP_IMU_LEN*FORWARD_RTP_IMU_NUM)

#endif