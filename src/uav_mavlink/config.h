
#ifndef CONFIG_HANDLER_H
#define CONFIG_HANDLER_H

#include <memory>

#include "global.h"
#include "bridge.h"

#define MAVLINK_DEFAULT_RATE          100

#define MAVLINK_DEFAULT_UART_PATH      "/dev/ttyAMA0"
#define MAVLINK_DEFAULT_UART_BAUD      115200
#define MAVLINK_DEFAULT_UDPS_PORT      14580
#define MAVLINK_DEFAULT_UDPC_ADDR      "192.168.1.100"
#define MAVLINK_DEFAULT_UDPC_PORT       14570
#define MAVLINK_DEFAULT_SPLITTER_PORT_S 5400
#define MAVLINK_DEFAULT_SPLITTER_PORT   5600
#define MAVLINK_DEFAULT_SPLITTER_ADDR   "127.0.0.1"
#define MAVLINK_DEFAULT_IPC_PATH1       "/tmp/uav_bridge/ipc_server"
#define MAVLINK_DEFAULT_IPC_PATH2       "/tmp/uav_bridge/ipc_server2"
#define MAVLINK_DEFAULT_IMU_TOPIC       "/tmp/uav_bridge/imu"
#define SPLITTER_CAMERA_CLOCK_HZ        90000
#define SPLITTER_CAMERA_FRAME_HZ        60
#define SPLITTER_CAMERA_SYNC_NUM        5000
#define SPLITTER_CAMERA_THRESHOLD       0.05

class ConfigHandler {
public:
    // Constructor and Destructor
    ConfigHandler();
    ~ConfigHandler(){
    };

    // Public methods
    int config_read(std::unique_ptr<BridgeHandler>& bridge);
    void config_print(std::string title);

public:
    /*
     * Member variables for configuration
     */
    std::string mavlink_activate;
    float mavlink_rate;
    enum ComType {
        COM_UART,
        COM_UDPS,
        COM_UDPC,
        COM_SPLIT
    } com_uart_udp_type;

    // UART
    std::string com_uart_path;
    int com_uart_baud;

    // UDP Server
    int com_udpls_port;

    // splitter
    int com_splitter_port_s;
    int com_splitter_port;
    std::string com_splitter_addr;
    int splitter_camera_clock_hz;
    int splitter_camera_frame_hz;
    int splitter_camera_sync_num;
    float splitter_camera_threshold;

    // UDP Client
    int com_udprs_port;
    std::string com_udprs_addr;

    std::string ipc1_path;
    std::string ipc2_path;
    std::string imu_topic;

    bool debug_enable;
};

#endif /* CONFIG_HANDLER_H */
