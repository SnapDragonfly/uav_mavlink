#include <iostream>
#include <string>

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

#include "connect.h"
#include "config.h"

ConnectHandler::ConnectHandler(){
    message = new MessageHandler();
    config  = new ConfigHandler();
};

bool ConnectHandler::debug(){
    return config->debug_enable;
}

int ConnectHandler::init(ros::NodeHandle &ros_nh){

    int ret = config->config_read(bridge);
    if(0 != ret){
        ROS_WARN("Configure warning, using default values!");
    }

    config->config_print("configuration");

    //imu_pub = ros_nh.advertise<sensor_msgs::Imu>(imu_topic.c_str(), 100);
    message->mavlink_init(ros_nh, config->imu_topic, config->mavlink_rate);

    if (!bridge) {
        std::cerr << "Bridge pointer is null!" << std::endl;
        return -1;
    }

    pfds[0].fd     = bridge->get();
    pfds[0].events = POLLIN;

#if 0
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
#endif
    return 0;
}

int ConnectHandler::update(){
    /*
     * Polling on fds for events or messages.
     */
    int ret = poll(pfds, MAVLINK_DEFAULT_NUM_PFDS, 5000);
    if (ret > 0) {
        bridge->update(pfds[0], message, bridge);
#if 0
        ipc1_poll();
        ipc2_poll();
#endif
    }
    return 0;
}

int ConnectHandler::deinit(){

    bridge->deinit();
#if 0
    close(ipc_fd);
    close(ipc_fd2);
#endif
    return 0;
}

#if 0
int ConnectHandler::ipc1_create(){
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

int ConnectHandler::ipc2_create(){
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

int ConnectHandler::ipc1_poll(){
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

int ConnectHandler::ipc2_poll(){
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
#endif
