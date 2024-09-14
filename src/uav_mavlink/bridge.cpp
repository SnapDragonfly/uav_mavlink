
#include <iostream>
#include <string>
#include <filesystem>  // C++17

#include <yaml-cpp/yaml.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h> // Contains POSIX terminal control definitions
#include <errno.h> // Error integer and strerror() function
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <math.h>
#include <sys/wait.h>
#include <time.h>

#include "bridge.h"

#define RATIO_SECOND_TO_MICRO_SECOND   1000000

#define TAG_TYPE_IPC     "ipcs"
#define TAG_TYPE_IPC1    "ipc1"
#define TAG_TYPE_IPC2    "ipc2"
#define TAG_TYPE_UART    "uart"
#define TAG_TYPE_UDP     "udp"

#define TAG_TYPE         "type"
#define TAG_PATH         "path"
#define TAG_BAUD         "baud"
#define TAG_PORT         "port"
#define TAG_UART         "uart"
#define TAG_UDPS         "udps"
#define TAG_UDPC         "udpc"
#define TAG_ADDR         "addr"

#define TAG_UAV_COM      "uav_com"
#define TAG_UAV_IMU      "uav_imu"
#define TAG_UAV_ACTIVATE "mavlink_activate"
#define TAG_UAV_RATE     "mavlink_rate"
#define TAG_UAV_DEBUG    "debug"

#define TAG_COM_ACTIVE   "(active)"
#define IS_ACTIVE(A, B)  ((A == B)?TAG_COM_ACTIVE:"")

#define BOOL_STATUS(A)  ((A == true)?"true":"false")

void MavlinkHandler::config_print(std::string title){
    ROS_INFO("%s ------------->", title.c_str());

    ROS_INFO("uav active: %s", mavlink_activate.c_str());
    ROS_INFO("uav rate: %.2fHz, %.0fus", mavlink_rate, update_interval);

    ROS_INFO("uart%s: %s", IS_ACTIVE(COM_UART, com_uart_udp_type), com_uart_path.c_str());
    ROS_INFO("uart%s: %d", IS_ACTIVE(COM_UART, com_uart_udp_type), com_uart_baud);
    ROS_INFO("udpls%s: %s", IS_ACTIVE(COM_UDPS, com_uart_udp_type), "127.0.0.1");
    ROS_INFO("udpls%s: %d", IS_ACTIVE(COM_UDPS, com_uart_udp_type), com_udpls_port);
    ROS_INFO("udprs%s: %s", IS_ACTIVE(COM_UDPC, com_uart_udp_type), com_udprs_addr.c_str());
    ROS_INFO("udprs%s: %d", IS_ACTIVE(COM_UDPC, com_uart_udp_type), com_udprs_port);
    
    ROS_INFO("ipc1: %s", ipc1_path.c_str());
    ROS_INFO("ipc2: %s", ipc2_path.c_str());
    ROS_INFO(" imu: %s", imu_topic.c_str());

    ROS_INFO("debug: %s", debug_enable?"true":"false");

    ROS_INFO("%s <-------------", title.c_str());
}

int MavlinkHandler::config_read(){
    try {
        //config_print("debug");

        // Get the current working directory
        std::filesystem::path work_path = std::filesystem::current_path();
        //ROS_INFO("Current working directory: %s", work_path.c_str());

        // Construct the absolute path to the config file
        std::filesystem::path config_path = work_path / "src" / PACKAGE_NAME / PACKAGE_CONFIG;
        //ROS_INFO("Config file path: %s", config_path.c_str());

        // Load the YAML file
        YAML::Node config = YAML::LoadFile(config_path.c_str());

        // Access values from the YAML file
        mavlink_activate = config[TAG_UAV_ACTIVATE].as<std::string>();
        if (TAG_UART == mavlink_activate) {
            com_uart_udp_type = COM_UART;
        } else if(TAG_UDPS == mavlink_activate) {
            com_uart_udp_type = COM_UDPS;
        } else {
            com_uart_udp_type = COM_UDPC;
        }
        mavlink_rate = config[TAG_UAV_RATE].as<float>();
        update_interval = RATIO_SECOND_TO_MICRO_SECOND/mavlink_rate;
        imu_topic = config[TAG_UAV_IMU].as<std::string>();
        
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
                com_udpls_port = uav_com[i][TAG_PORT].as<int>();
                //ROS_INFO("%s: %d", TAG_UDP, com_port);
            } else if (com_type == TAG_UDPC) {
                com_udprs_port = uav_com[i][TAG_PORT].as<int>();
                com_udprs_addr = uav_com[i][TAG_ADDR].as<std::string>();
                //ROS_INFO("%s: %d", TAG_UDP, com_port);
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

        debug_enable = config[TAG_UAV_DEBUG].as<bool>();
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

int MavlinkHandler::uart_create(){
    struct termios tty; // Create new termios struc, we call it 'tty' for convention

    /*
     * UART initialization from FC
     */
    uart_fd = open(com_uart_path.c_str(), O_RDWR);
    if (uart_fd < 0) {
        ROS_ERROR("Can not open serial port %s.", com_uart_path.c_str());
        return 1;
    }

    if(tcgetattr(uart_fd, &tty) != 0) {
        ROS_ERROR("Error %i from tcgetattr: %s", errno, strerror(errno));
        return 2;
    }
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;
    cfsetispeed(&tty, com_uart_baud);
    cfsetospeed(&tty, com_uart_baud);
    
    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {  // Save tty settings, also checking for error
        ROS_ERROR("Error %i from tcsetattr: %s", errno, strerror(errno));
        return 3;
    }

    return 0;
}

int MavlinkHandler::udps_create(){
    struct sockaddr_in server_addr;

    // Create a UDP socket
    udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd < 0) {
        ROS_ERROR("Create socket failed.");
        return 4;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    memset(&udp_addr, 0, sizeof(udp_addr));

    // Server information
    server_addr.sin_family = AF_INET;  // IPv4
    server_addr.sin_addr.s_addr = INADDR_ANY;  // Local IP
    server_addr.sin_port = htons(com_udpls_port);  // Port

    // Bind the socket
    if (bind(udp_fd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ROS_ERROR("Bind local socket failed");
        close(udp_fd);
        return 5;
    }

    ROS_INFO("udps_create ip: %s, port:%d", "127.0.0.1", com_udpls_port);

    return 0;
}

int MavlinkHandler::udpc_create(){

    // Create a UDP socket
    udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd < 0) {
        ROS_ERROR("Create socket failed.");
        return 4;
    }

    memset(&udp_addr, 0, sizeof(udp_addr));

    // Server information
    udp_addr.sin_family = AF_INET;  // IPv4
    //udp_addr.sin_addr.s_addr = inet_addr(com_udprs_addr.c_str());  // remote IP
    if (inet_pton(AF_INET, com_udprs_addr.c_str(), &udp_addr.sin_addr) <= 0) {
        ROS_ERROR("Invalid address/ Address not supported");
        return 5;
    }
    udp_addr.sin_port = htons(com_udprs_port);  // Port

    const char *message = "Hello, Server!"; // just for link establish
    (void)sendto(udp_fd, message, strlen(message), 0, (struct sockaddr *) &udp_addr, sizeof(udp_addr));

    ROS_INFO("udpc_create ip: %s, port:%d", com_udprs_addr.c_str(), com_udprs_port);

    return 0;
}

int MavlinkHandler::ipc1_create(){
    struct sockaddr_un ipc_addr;

    /*
     * IPC initialization for ???
     */
    if ((ipc_fd = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0) {
        ROS_ERROR("Create socket failed.");
        return 4;
    }
    memset(&ipc_addr, 0, sizeof(ipc_addr));
    ipc_addr.sun_family = AF_UNIX;
    strcpy(ipc_addr.sun_path, ipc1_path.c_str());
    unlink(ipc1_path.c_str());
    if (bind(ipc_fd, (const struct sockaddr *)&ipc_addr, sizeof(ipc_addr)) < 0) {
        ROS_ERROR("Bind local ipc path failed");
        close(ipc_fd);
        return 5;
    }

    return 0;
}

int MavlinkHandler::ipc2_create(){
    struct sockaddr_un ipc_addr2;

   /*
     * IPC initialization for ???
     */
    if ((ipc_fd2 = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0) {
        ROS_ERROR("Create socket failed.");
        return 6;
    }
    memset(&ipc_addr2, 0, sizeof(ipc_addr2));
    ipc_addr2.sun_family = AF_UNIX;
    strcpy(ipc_addr2.sun_path, ipc2_path.c_str());
    unlink(ipc2_path.c_str());
    if (bind(ipc_fd2, (const struct sockaddr *)&ipc_addr2, sizeof(ipc_addr2)) < 0) {
        ROS_ERROR("Bind local ipc path failed");
        close(ipc_fd2);
        return 7;
    }

    return 0;
}

int MavlinkHandler::mavlink_init(ros::NodeHandle &ros_nh){

    int ret = config_read();
    if(0 != ret){
        ROS_WARN("Configure warning, using default values!");
    }
    config_print("configuration");

    imu_pub = ros_nh.advertise<sensor_msgs::Imu>(imu_topic.c_str(), 100);

    switch(com_uart_udp_type){          
        case COM_UDPS:
            ret = udps_create();
            pfds[0].fd= udp_fd;
            break;

        case COM_UDPC:
            ret = udpc_create();
            pfds[0].fd= udp_fd;
            break;

        case COM_UART:
        default:
            ret = uart_create();
            pfds[0].fd= uart_fd;
            break;
    }   
    if(0 != ret){
        return ret;
    }
    pfds[0].events = POLLIN;

    ret = ipc1_create();
    if(0 != ret){
        return ret;
    }
    pfds[1].fd= ipc_fd;
    pfds[1].events = POLLIN;

    ret = ipc2_create();
    if(0 != ret){
        return ret;
    }
    pfds[2].fd= ipc_fd2;
    pfds[2].events = POLLIN;

    return 0;
}

int MavlinkHandler::mavlink_handler(mavlink_message_t &msg, mavlink_status_t &status){
    int len;
    struct timeval tv;
    //ROS_INFO("recv msg ID %d, seq %d\n", msg.msgid, msg.seq);

    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);

        if (msg.sysid != mav_sysid) {
            mav_sysid = msg.sysid;
            ROS_INFO("found MAV %d", msg.sysid);
        }

        if (0 == time_offset_us) {
            gettimeofday(&tv, NULL);
            mavlink_msg_timesync_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, &msg, 0, tv.tv_sec*1000000+tv.tv_usec, mav_sysid, 1); //fill timesync with us instead of ns
            len = mavlink_msg_to_send_buffer(buf, &msg);
            cc_send(buf, len);

            mavlink_msg_system_time_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, 0);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            cc_send(buf, len);

            mavlink_msg_set_gps_global_origin_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, &msg, mav_sysid, 247749434, 1210443077, 100000, tv.tv_sec*1000000+tv.tv_usec);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            cc_send(buf, len);

            ROS_INFO("sync time_offset_us = 0");
        }

        if (no_hr_imu) {
            mavlink_msg_command_long_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, &msg, 0, 0, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_HIGHRES_IMU, update_interval, 0, 0, 0, 0, 0);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            cc_send(buf, len);
            ROS_INFO("HIGHRES_IMU set interval %0.2fus", update_interval);
        }

        if (no_att_q) {
            mavlink_msg_command_long_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, &msg, 0, 0, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, update_interval, 0, 0, 0, 0, 0);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            cc_send(buf, len);
            ROS_INFO("ATTITUDE_QUATERNION set interval %0.2fus", update_interval);
        }

        if (hb.custom_mode == COPTER_MODE_GUIDED) {
            if (demo_stage == 0) {
                demo_stage = 1;
                gettimeofday(&tv, NULL);
                mavlink_msg_command_long_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, &msg, mav_sysid, 1, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 1);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                cc_send(buf, len);
            }
        } else {
            demo_stage = 0;
        }

        if (hb.base_mode & 128) {
            if (gnd_alt == 0) {
                gnd_alt = latest_alt;
                start_x = latest_x;
                ROS_INFO("gnd viso alt %f, start x %f\n", gnd_alt, start_x);
            }
        } else {
            gnd_alt = 0;
        }
    } else if (msg.msgid == MAVLINK_MSG_ID_TIMESYNC) {
        mavlink_timesync_t ts;
        mavlink_msg_timesync_decode(&msg, &ts);
        if (ts.tc1 != 0) {
            time_offset_us = ts.ts1 - ts.tc1;
            ROS_INFO("sync time_offset_us=%ld", time_offset_us);
        }
    } else if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
        mavlink_statustext_t txt;
        mavlink_msg_statustext_decode(&msg, &txt);
        ROS_INFO("fc: %s", txt.text);
    } else if (msg.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
        mavlink_highres_imu_t hr_imu;
        mavlink_msg_highres_imu_decode(&msg, &hr_imu); // time_usec is time since boot
        if (time_offset_us > 0 && hr_imu.time_usec > last_us) {
#if 1
            static int64_t effective_counts = 0;
            static int64_t effective_rate = int64_t(mavlink_rate);
            effective_counts++;
            if (0 == (effective_counts % effective_rate)){
                int64_t effective_hz = RATIO_SECOND_TO_MICRO_SECOND/(hr_imu.time_usec - last_us);

                if (effective_hz >= 0.9*mavlink_rate && effective_hz <= 1.5*mavlink_rate){
                    ROS_INFO("MAVLINK_MSG_ID_HIGHRES_IMU frequency = %ldHz, should be %0.2fHz", 
                                effective_hz, 
                                mavlink_rate);
                    effective_rate *= 5; //increase 5 times for checking
                    no_hr_imu = false;
                    no_att_q = false;
                } else {
                    ROS_WARN("MAVLINK_MSG_ID_HIGHRES_IMU frequency = %ldHz, should be %0.2fHz", 
                                effective_hz, 
                                mavlink_rate);
                    effective_rate = int64_t(mavlink_rate);
                    no_hr_imu = true;
                    no_att_q = true;
                }
            }
#endif

            last_us = hr_imu.time_usec;
            sensor_msgs::Imu imu_msg;
#if 0
            imu_msg.header.stamp = ros::Time::now();
#else
            int64_t ts_us = hr_imu.time_usec + time_offset_us;
            imu_msg.header.stamp.sec = ts_us / 1000000;
            imu_msg.header.stamp.nsec = (ts_us % 1000000) * 1000;
#endif
            imu_msg.header.frame_id = "world";
            imu_msg.linear_acceleration.x = hr_imu.xacc;
            imu_msg.linear_acceleration.y = hr_imu.yacc;
            imu_msg.linear_acceleration.z = hr_imu.zacc;
            imu_msg.angular_velocity.x = hr_imu.xgyro;
            imu_msg.angular_velocity.y = hr_imu.ygyro;
            imu_msg.angular_velocity.z = hr_imu.zgyro;
            imu_msg.orientation.w = att_q_w;
            imu_msg.orientation.x = att_q_x;
            imu_msg.orientation.y = att_q_y;
            imu_msg.orientation.z = att_q_z;
            imu_pub.publish(imu_msg);
        }
    } else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE_QUATERNION) {
        mavlink_attitude_quaternion_t att_q;
        mavlink_msg_attitude_quaternion_decode(&msg, &att_q);
        att_q_w = att_q.q1;
        att_q_x = att_q.q2;
        att_q_y = att_q.q3;
        att_q_z = att_q.q4;
    }

    return 0;
}

int MavlinkHandler::uart_poll(){
    unsigned char byte;    

    if (!(pfds[0].revents & POLLIN)) {
        return 1;
    }

    ssize_t len = read(uart_fd, &byte, 1);
    if (len <= 0) {
        return 2;
    }

#if defined(MAVLINK_CODE_DEBUG)
    if (debug_enable) printf("0x%02x ", byte);
#endif /* MAVLINK_CODE_DEBUG */

    if (mavlink_parse_char(0, byte, &msg, &status)){

#if defined(MAVLINK_CODE_DEBUG)
        if (debug_enable)  ROS_INFO("recv msg ID %d, seq %d", msg.msgid, msg.seq);
#endif /* MAVLINK_CODE_DEBUG */

        if (msg.sysid == 255) return 0;

        return mavlink_handler(msg, status);
    }

    return 0;
}

int MavlinkHandler::udps_poll(){

    if (!(pfds[0].revents & POLLIN)) {
        return 1;
    }

    udp_len = sizeof(udp_addr);
    int len = recvfrom(udp_fd, buf, MAVLINK_DEFAULT_BUF_LEN, 0, (struct sockaddr *)&udp_addr, &udp_len);

    for (int i = 0; i < len; i++) {
        if (!mavlink_parse_char(0, buf[i], &msg, &status)) continue;
        if (msg.sysid == 255) continue;
        return mavlink_handler(msg, status);
    }
    
    return 2;
}

int MavlinkHandler::udpc_poll(){

    if (!(pfds[0].revents & POLLIN)) {
        return 1;
    }

    udp_len = sizeof(udp_addr);
    int len = recvfrom(udp_fd, buf, MAVLINK_DEFAULT_BUF_LEN, 0, (struct sockaddr *) &udp_addr, &udp_len);

    for (int i = 0; i < len; i++) {
        if (!mavlink_parse_char(0, buf[i], &msg, &status)) continue;
        if (msg.sysid == 255) continue;
        return mavlink_handler(msg, status);
    }
    
    return 2;
}

void MavlinkHandler::cc_send(unsigned char *buf, int len){
    switch (com_uart_udp_type) {
        case COM_UDPS:
        case COM_UDPC:
            sendto(udp_fd, buf, len, 0, (struct sockaddr *)&udp_addr, udp_len);
            break;

        case COM_UART:
        default:
            write(uart_fd, buf, len);
            break;
    }
}

int MavlinkHandler::ipc1_poll(){
    float pose[10];
    struct timeval tv;
    unsigned int len;
    mavlink_message_t msg;

    if (!(pfds[1].revents & POLLIN)) {
        return 1;
    }

    int ret = recv(ipc_fd, pose, sizeof(pose), 0);
    if (ret <= 0){
        return 2;
    }

    latest_alt = pose[6];
    latest_x = pose[4];
    float covar[21] = {0};
    pose[2]=-pose[2];
    pose[3]=-pose[3];

    gettimeofday(&tv, NULL);
    mavlink_msg_att_pos_mocap_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, 
                                  &msg, tv.tv_sec*1000000+tv.tv_usec, pose, pose[4], -pose[5], -pose[6], covar);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    cc_send(buf, len);
    
    gettimeofday(&tv, NULL);
    mavlink_msg_vision_speed_estimate_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, 
                                          &msg, tv.tv_sec*1000000+tv.tv_usec, pose[7], -pose[8], -pose[9], covar, 0);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    cc_send(buf, len);

    if (demo_stage == 1 && (latest_alt - gnd_alt) > 0.5f) {
        demo_stage = 2;
        gettimeofday(&tv, NULL);
        mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, 
                                                      &msg, tv.tv_sec*1000+tv.tv_usec*0.001, 0, 0, 
                                                       MAV_FRAME_BODY_OFFSET_NED, 0x0DF8, 5, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        cc_send(buf, len);
    } else if ((demo_stage == 2 || demo_stage == 3) && (latest_x - start_x) > 4.5f) {
        demo_stage = 100;
        gettimeofday(&tv, NULL);
        mavlink_msg_set_mode_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, &msg, mav_sysid, 1, 9);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        cc_send(buf, len);
    }

    return 0;
}

int MavlinkHandler::ipc2_poll(){
    int prx_msg;
    struct timeval tv;
    unsigned int len;
    mavlink_message_t msg;

    if (!(pfds[2].revents & POLLIN)) {
        return 1;
    }

    if (recv(ipc_fd2, &prx_msg, sizeof(prx_msg), 0) > 0) {
        if (demo_stage == 2 && prx_msg == 1) {
            demo_stage = 3;
            gettimeofday(&tv, NULL);

            mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, 
                                                          &msg, tv.tv_sec*1000+tv.tv_usec*0.001, 0, 0, 
                                                           MAV_FRAME_BODY_OFFSET_NED, 0x0DF8, 1, 1.5f, -0.5f, 0, 0, 0, 0, 0, 0, 0, 0);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            cc_send(buf, len);

            mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, 
                                                          &msg, tv.tv_sec*1000+tv.tv_usec*0.001, 0, 0, 
                                                           MAV_FRAME_BODY_OFFSET_NED, 0x0DF8 | 4096, 4, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            cc_send(buf, len);
        }
    }

    return 0;
}

int MavlinkHandler::mavlink_poll(){
    /*
     * Polling on fds for events or messages.
     */
    int ret = poll(pfds, MAVLINK_DEFAULT_NUM_PFDS, 5000);
    if (ret > 0) {
        switch(com_uart_udp_type){          
            case COM_UART:
                uart_poll();
                break;

            case COM_UDPS:
                udps_poll();
                break;

            case COM_UDPC:
                udpc_poll();
                break;

            default:
                ROS_ERROR("%s: mavlink_poll error(%d)", PACKAGE_NAME, ret);
                mavlink_exit();
                exit(1);
                break;
        }
        
        ipc1_poll();
        ipc2_poll();
    }
    return 0;
}

int MavlinkHandler::mavlink_exit(){
    switch(com_uart_udp_type){          
        case COM_UART:
            close(uart_fd);
            break;

        case COM_UDPS:
        case COM_UDPC:
        default:
            close(udp_fd);
            break;
    }
    close(ipc_fd);
    close(ipc_fd2);
    return 0;
}

bool MavlinkHandler::mvalink_debug(){
    return debug_enable;
}

MavlinkHandler::MavlinkHandler(){
    // Default values:
    mav_sysid      = 0;
    demo_stage     = 0;

    no_hr_imu      = true;
    no_att_q       = true;

    att_q_x        = 0;
    att_q_y        = 0;
    att_q_z        = 0;
    att_q_w        = 0;

    time_offset_us = 0;
    last_us        = 0;

    latest_alt     = 0;
    gnd_alt        = 0;
    latest_x       = 0;
    start_x        = 0;

    update_interval = 0;

    // Default settings:
    mavlink_activate  = TAG_UART;
    com_uart_udp_type = COM_UART;
    mavlink_rate      = MAVLINK_DEFAULT_RATE;
    update_interval   = RATIO_SECOND_TO_MICRO_SECOND/mavlink_rate;
    com_uart_path     = MAVLINK_DEFAULT_UART_PATH;
    com_uart_baud     = MAVLINK_DEFAULT_UART_BAUD;
    com_udpls_port    = MAVLINK_DEFAULT_UDPS_PORT;
    com_udprs_addr    = MAVLINK_DEFAULT_UDPC_ADDR;
    com_udprs_port    = MAVLINK_DEFAULT_UDPC_PORT;
    ipc1_path         = MAVLINK_DEFAULT_IPC_PATH1;
    ipc2_path         = MAVLINK_DEFAULT_IPC_PATH2;
    imu_topic         = MAVLINK_DEFAULT_IMU_TOPIC;

    debug_enable      = true;
};
