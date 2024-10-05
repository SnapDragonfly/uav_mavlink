#ifndef IMU_MIXER_H
#define IMU_MIXER_H

typedef struct {
    uint32_t img_sec;  // Timestamp seconds
    uint32_t img_nsec; // Timestamp nanoseconds
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

#define FORWARD_RTP_IMU_LEN      sizeof(imu_data_t)                 //uav_mixer settings, RPi3B+ ~56 Bytes
#define FORWARD_RTP_IMU_NUM      1                                  //uav_mixer settings, RPi3B+ 1 for try
#define FORWARD_RTP_PREFIX_LEN   (FORWARD_RTP_IMU_LEN*FORWARD_RTP_IMU_NUM)

#define RTP_BUFFER_ADDR(buffer) (buffer+FORWARD_RTP_PREFIX_LEN)
#define RTP_BUFFER_SIZE(buffer) (sizeof(buffer)-FORWARD_RTP_PREFIX_LEN)

#endif