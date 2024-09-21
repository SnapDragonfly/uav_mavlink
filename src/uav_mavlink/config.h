
#define PACKAGE_NAME "uav_bridge"
#define NODE_NAME    "uav_bridge_mavlink"

#define MAVLINK_DEFAULT_COMP_ID   191
#define MAVLINK_DEFAULT_NUM_PFDS  3
#define MAVLINK_DEFAULT_RATE      100

#define MAVLINK_DEFAULT_UART_PATH "/dev/ttyAMA0"
#define MAVLINK_DEFAULT_UART_BAUD 115200
#define MAVLINK_DEFAULT_UDPS_PORT 14580
#define MAVLINK_DEFAULT_UDPC_ADDR "192.168.1.100"
#define MAVLINK_DEFAULT_UDPC_PORT 14570
#define MAVLINK_DEFAULT_IPC_PATH1 "/tmp/uav_bridge/ipc_server"
#define MAVLINK_DEFAULT_IPC_PATH2 "/tmp/uav_bridge/ipc_server2"
#define MAVLINK_DEFAULT_IMU_TOPIC "/tmp/uav_bridge/imu"

#define MAVLINK_DEFAULT_BUF_LEN   512

#define PACKAGE_CONFIG "config_mavlink.yaml"

//#define MAVLINK_CODE_DEBUG
