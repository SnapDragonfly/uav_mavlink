#include <ros/ros.h>
#include <ros/package.h>

#include <filesystem>  // C++17

#include <yaml-cpp/yaml.h>

#include "config.h"
#include "uart.h"
#include "udpc.h"
#include "udps.h"
#include "splitter.h"

#define TAG_TYPE_IPC     "ipcs"
#define TAG_TYPE_IPC1    "ipc1"
#define TAG_TYPE_IPC2    "ipc2"
#define TAG_TYPE_UART    "uart"
#define TAG_TYPE_UDP     "udp"

#define TAG_TYPE         "type"
#define TAG_PATH         "path"
#define TAG_BAUD         "baud"
#define TAG_ADDR         "addr"
#define TAG_PORT         "port"
#define TAG_PORT_S       "port-s"
#define TAG_UART         "uart"
#define TAG_UDPS         "udps"
#define TAG_UDPC         "udpc"
#define TAG_SPLIT        "split"
#define TAG_CLOCK        "clock"
#define TAG_SYNC_NUM     "sync"
#define TAG_FRAME_HZ     "frame"
#define TAG_THRESHOLD    "threshold"

#define TAG_UAV_COM      "uav_com"
#define TAG_UAV_IMU      "uav_imu"
#define TAG_UAV_IMG      "uav_img"
#define TAG_UAV_ACTIVATE "mavlink_activate"
#define TAG_UAV_RATE     "mavlink_rate"
#define TAG_UAV_DEBUG    "debug"

#define TAG_COM_ACTIVE   "(active)"
#define IS_ACTIVE(A, B)  ((A == B)?TAG_COM_ACTIVE:"")

#define BOOL_STATUS(A)  ((A == true)?"true":"false")


ConfigHandler::ConfigHandler(){
    // Default settings:
    mavlink_activate  = TAG_UART;
    com_uart_udp_type = COM_UART;
    mavlink_rate      = MAVLINK_DEFAULT_RATE;
    com_uart_path     = MAVLINK_DEFAULT_UART_PATH;
    com_uart_baud     = MAVLINK_DEFAULT_UART_BAUD;
    com_udpls_port    = MAVLINK_DEFAULT_UDPS_PORT;
    com_udprs_addr    = MAVLINK_DEFAULT_UDPC_ADDR;
    com_udprs_port    = MAVLINK_DEFAULT_UDPC_PORT;
    com_splitter_addr = MAVLINK_DEFAULT_SPLITTER_ADDR;
    com_splitter_port = MAVLINK_DEFAULT_SPLITTER_PORT;
    ipc1_path         = MAVLINK_DEFAULT_IPC_PATH1;
    ipc2_path         = MAVLINK_DEFAULT_IPC_PATH2;
    imu_topic         = MAVLINK_DEFAULT_IMU_TOPIC;
    img_topic         = MAVLINK_DEFAULT_IMG_TOPIC;
    splitter_camera_clock_hz  = SPLITTER_CAMERA_CLOCK_HZ;
    splitter_camera_frame_hz  = SPLITTER_CAMERA_FRAME_HZ;
    splitter_camera_sync_num  = SPLITTER_CAMERA_SYNC_NUM;
    splitter_camera_threshold = SPLITTER_CAMERA_THRESHOLD;

    debug_enable      = true;
};

void ConfigHandler::config_print(std::string title){
    ROS_INFO("%s ------------->", title.c_str());

    ROS_INFO("uav active: %s", mavlink_activate.c_str());
    ROS_INFO("uav rate: %.2fHz, %.0fus", mavlink_rate, RATIO_SECOND_TO_MICRO_SECOND/mavlink_rate);

    ROS_INFO("     uart%s: %s", IS_ACTIVE(COM_UART, com_uart_udp_type), com_uart_path.c_str());
    ROS_INFO("     rate%s: %d", IS_ACTIVE(COM_UART, com_uart_udp_type), com_uart_baud);
    ROS_INFO("    udpls%s: %s", IS_ACTIVE(COM_UDPS, com_uart_udp_type), "127.0.0.1");
    ROS_INFO("     port%s: %d", IS_ACTIVE(COM_UDPS, com_uart_udp_type), com_udpls_port);
    ROS_INFO("    udprs%s: %s", IS_ACTIVE(COM_UDPC, com_uart_udp_type), com_udprs_addr.c_str());
    ROS_INFO("     port%s: %d", IS_ACTIVE(COM_UDPC, com_uart_udp_type), com_udprs_port);
    ROS_INFO(" splitter%s: %s", IS_ACTIVE(COM_SPLIT, com_uart_udp_type), com_splitter_addr.c_str());
    ROS_INFO("     port%s: %d", IS_ACTIVE(COM_SPLIT, com_uart_udp_type), com_splitter_port);
    ROS_INFO("   port-s%s: %d", IS_ACTIVE(COM_SPLIT, com_uart_udp_type), com_splitter_port_s);
    ROS_INFO("    clock%s: %d", IS_ACTIVE(COM_SPLIT, com_uart_udp_type), splitter_camera_clock_hz);
    ROS_INFO("    frame%s: %d", IS_ACTIVE(COM_SPLIT, com_uart_udp_type), splitter_camera_frame_hz);
    ROS_INFO("     sync%s: %d", IS_ACTIVE(COM_SPLIT, com_uart_udp_type), splitter_camera_sync_num);
    ROS_INFO("threshold%s: %.3f", IS_ACTIVE(COM_SPLIT, com_uart_udp_type), splitter_camera_threshold);


    ROS_INFO("ipc1: %s", ipc1_path.c_str());
    ROS_INFO("ipc2: %s", ipc2_path.c_str());
    ROS_INFO(" imu: %s", imu_topic.c_str());
    ROS_INFO(" img: %s", img_topic.c_str());

    ROS_INFO("debug: %s", debug_enable?"true":"false");

    ROS_INFO("%s <-------------", title.c_str());
}

int ConfigHandler::config_read(ros::NodeHandle& ros_nh, std::unique_ptr<BridgeHandler>& bridge){
    try {
        int ret;
        //config_print("debug");      

        // Get the directory path of a package
        std::filesystem::path package_path = ros::package::getPath(PACKAGE_NAME);
        std::filesystem::path config_path = package_path / PACKAGE_CONFIG;
        ROS_INFO("Config file path: %s", config_path.c_str());

        // Load the YAML file
        YAML::Node config = YAML::LoadFile(config_path.c_str());

        debug_enable = config[TAG_UAV_DEBUG].as<bool>();
        mavlink_rate = config[TAG_UAV_RATE].as<float>();
        imu_topic = config[TAG_UAV_IMU].as<std::string>();
        img_topic = config[TAG_UAV_IMG].as<std::string>();
        
        //ROS_INFO("%s: %s", TAG_UAV_ACTIVATE, mavlink_activate.c_str());
        //ROS_INFO("%s: %d", TAG_UAV_RATE, mavlink_rate);
        //ROS_INFO("%s: %d", TAG_UAV_IMU, imu_topic);

        // Access uav communications
        const YAML::Node &uav_com = config[TAG_UAV_COM];
        for (std::size_t i = 0; i < uav_com.size(); i++) {
            std::string com_type = uav_com[i][TAG_TYPE].as<std::string>();
            if (com_type == TAG_UART) {
                com_uart_path = uav_com[i][TAG_PATH].as<std::string>();
                com_uart_baud = uav_com[i][TAG_BAUD].as<int>();
                //ROS_INFO("%s: %s", TAG_UART, com_path.c_str());
            } else if (com_type == TAG_UDPS) {
                com_udpls_port = uav_com[i][TAG_PORT_S].as<int>();
                //ROS_INFO("%s: %d", TAG_UDP, com_port);
            } else if (com_type == TAG_UDPC) {
                com_udprs_port = uav_com[i][TAG_PORT].as<int>();
                com_udprs_addr = uav_com[i][TAG_ADDR].as<std::string>();
                //ROS_INFO("%s: %d", TAG_UDP, com_port);
            } else if (com_type == TAG_SPLIT) {
                com_splitter_port_s = uav_com[i][TAG_PORT_S].as<int>();
                com_splitter_port   = uav_com[i][TAG_PORT].as<int>();
                com_splitter_addr   = uav_com[i][TAG_ADDR].as<std::string>();
                splitter_camera_clock_hz  = uav_com[i][TAG_CLOCK].as<int>();
                splitter_camera_frame_hz  = uav_com[i][TAG_FRAME_HZ].as<int>();
                splitter_camera_sync_num  = uav_com[i][TAG_SYNC_NUM].as<int>();
                splitter_camera_threshold = uav_com[i][TAG_THRESHOLD].as<float>();
                //ROS_INFO("%s: %d", TAG_SPLIT, com_port);
            }
        }

        // Access array of ipcs
        const YAML::Node &ipcs = config[TAG_TYPE_IPC];
        for (std::size_t i = 0; i < ipcs.size(); i++) {
            std::string ipc_type = ipcs[i][TAG_TYPE].as<std::string>();
            if (ipc_type == TAG_TYPE_IPC1) {
                ipc1_path = ipcs[i][TAG_PATH].as<std::string>();
                //ROS_INFO("%s: %s", TAG_PATH, ipc1_path.c_str());
            } else if (ipc_type == TAG_TYPE_IPC2) {
                ipc2_path = ipcs[i][TAG_PATH].as<std::string>();
                //ROS_INFO("%s: %s", TAG_TYPE_IPC2, ipc2_path.c_str());
            }
        }

        // Access values from the YAML file
        mavlink_activate = config[TAG_UAV_ACTIVATE].as<std::string>();
        if (TAG_UART == mavlink_activate) {
            com_uart_udp_type = COM_UART;
            bridge = std::make_unique<UartHandler>();

            struct UartParam param;
            param.debug = debug_enable;
            bridge->set(&param);

            ret = bridge->init(com_uart_path, com_uart_baud);
            if(0 != ret){
                return ret;
            }
        } else if(TAG_UDPS == mavlink_activate) {
            com_uart_udp_type = COM_UDPS;
            bridge = std::make_unique<UdpsHandler>();

            struct UdpsParam param;
            param.debug = debug_enable;
            bridge->set(&param);

            ret = bridge->init("", com_udpls_port);
            if(0 != ret){
                return ret;
            }
        } else if(TAG_UDPC == mavlink_activate) {
            com_uart_udp_type = COM_UDPC;
            bridge = std::make_unique<UdpcHandler>();

            struct UdpcParam param;
            param.debug = debug_enable;
            bridge->set(&param);

            ret = bridge->init(com_udprs_addr, com_udprs_port);
            if(0 != ret){
                return ret;
            }
        } else {
            com_uart_udp_type = COM_SPLIT;
            bridge = std::make_unique<SplitterHandler>();

            struct SplitterParam param;
            param.camera_clock_hz  = splitter_camera_clock_hz;
            param.camera_frame_hz  = splitter_camera_frame_hz;
            param.camera_sync_num  = splitter_camera_sync_num;
            param.camera_threshold = splitter_camera_threshold;
            param.splitter_port    = com_splitter_port;
            param.splitter_addr    = com_splitter_addr;
            param.imu_topic        = imu_topic;
            param.img_topic        = img_topic;
            param.ros_nh           = ros_nh;
            param.debug            = debug_enable;
            bridge->set(&param);

            ret = bridge->init("", com_splitter_port_s);
            if(0 != ret){
                return ret;
            }
        }
    }
    catch (const std::runtime_error& e) {
        ROS_ERROR("Runtime error: %s", e.what());
        return 1;  // Return a non-zero value to indicate an error occurred
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 2;  // Return a non-zero value to indicate an error occurred
    }
    catch (...) {
        ROS_ERROR("Unknown exception occurred");
        return 3;  // Return a non-zero value to indicate an error occurred
    }
    return 0;
}