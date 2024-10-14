
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>

#include "splitter.h"
#include "rtp_head.h"
#include "imu_mixer.h"

SplitterHandler::SplitterHandler() {
    camera_param.camera_clock_hz = 90000;
    camera_param.camera_frame_hz = 30;
    camera_param.camera_sync_num = 100;
    camera_param.camera_threshold = 5;

    splitter_fd = 0;

#if (MAVLINK_CODE_DEBUG)
    if (debug()){
        ROS_DEBUG("SplitterHandler empty applied!");
    }
#endif
}

SplitterHandler::~SplitterHandler() {
}

int SplitterHandler::init(std::string path, int param) {
    struct sockaddr_in server_addr;

    // Create a UDP socket
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        ROS_ERROR("Create socket failed.");
        return 4;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    memset(&udp_addr, 0, sizeof(udp_addr));

    // Server information
    server_addr.sin_family = AF_INET;  // IPv4
    server_addr.sin_addr.s_addr = INADDR_ANY;  // Local IP
    server_addr.sin_port = htons(param);  // Port

    // Bind the socket
    if (bind(fd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ROS_ERROR("Bind local socket failed");
        close(fd);
        return 5;
    }

    init_sync_system(&sys, camera_param.camera_clock_hz); // Initialize with a clock frequency of 90*1000 Hz
    
    img_pub = camera_param.ros_nh.advertise<std_msgs::Header>(camera_param.img_topic.c_str(), 100);
    imu_pub = camera_param.ros_nh.advertise<sensor_msgs::Imu>(camera_param.imu_topic.c_str(), 100);

    ROS_INFO("SplitterHandler ip: %s, port: %d", "127.0.0.1", param);

    return 0;
}

bool SplitterHandler::is_valid_rtp_packet(const uint8_t *data, size_t length) {
    if (length < sizeof(rtp_header_t)) {
        return false; // Packet too short to be a valid RTP packet
    }

    const rtp_header_t *header = (const rtp_header_t *)data;

    // Check RTP version (should be 2)
    if ((header->version & 0x03) != 2) { // Only the last 2 bits for version
        return false; // Invalid RTP version
    }

    // Check for valid payload type (0-127 are standard types)
    if (header->payload_type > 127) {
        return false; // Invalid payload type
    }

    // Check if padding is set
    if (header->padding) {
        // Ensure there are enough bytes for the RTP header + padding length
        if (length < sizeof(rtp_header_t)) {
            return false; // Not enough data for the header
        }
        // The padding length is determined by the last byte of the packet
        uint8_t padding_length = data[length - 1]; // Last byte for padding length
        if (length < sizeof(rtp_header_t) + padding_length) {
            return false; // Not enough data for padding
        }
    }

    // Check the CSRC count and ensure it does not exceed the packet length
    size_t csrc_count = header->cc;
    if (length < sizeof(rtp_header_t) + (csrc_count * sizeof(uint32_t))) {
        return false; // Packet too short for CSRC identifiers
    }

    // Additional checks can be added here (e.g., for sequence number, timestamp)

    return true; // Valid RTP packet
}

#define TIMING_STATUS(A, B)  ((A <= B)?"OK":"NG")

int SplitterHandler::update(struct pollfd& pfds, class MessageHandler* message, std::unique_ptr<BridgeHandler>& bridge) {
    static uint32_t image_time_count = 0;
    static uint32_t imu_data_count   = 0;

    if (!(pfds.revents & POLLIN)) {
        return 1;
    }

    udp_len = sizeof(udp_addr);
    int recv_len = recvfrom(fd, buf, RTP_DEFAULT_BUF_LEN, 0, (struct sockaddr *)&udp_addr, &udp_len);
    if (recv_len < 0) {
        return 2;
    }
    mix_head_t* p_mix_head = (mix_head_t*)buf;

    if (MAGIC_IMU_FRAME_NUM == p_mix_head->reserved ) {

        /*
         * Procedure A: IMU data
         *
         * IDLE UPD time for IMU, handling imu sensor data
         * Please try to empty imu ring buffer
         */
#if (MAVLINK_CODE_DEBUG)
        printf("FF sec: %u nsec: %u num: %u\n", p_mix_head->img_sec, p_mix_head->img_nsec, p_mix_head->imu_num);
#endif

        imu_data_t* p_imu_data = (imu_data_t*)(buf + sizeof(mix_head_t));
        for (int i = 0; i < p_mix_head->imu_num; i++) {
#if (MAVLINK_CODE_DEBUG)
            printf("%02d sec: %u nsec: %u\n", i+1, p_imu_data->imu_sec, p_imu_data->imu_nsec);
#endif
            if (!(p_imu_data->imu_sec == 0 && p_imu_data->imu_nsec ==0)) {

                sensor_msgs::Imu imu_msg;

                imu_msg.header.frame_id = "world";

                // Set the timestamp (from imu_sec and imu_nsec)
                imu_msg.header.stamp.sec  = p_imu_data->imu_sec;
                imu_msg.header.stamp.nsec = p_imu_data->imu_nsec;
                imu_msg.header.seq        = imu_data_count;
                
                // Set the linear acceleration
                imu_msg.linear_acceleration.x = p_imu_data->xacc;
                imu_msg.linear_acceleration.y = p_imu_data->yacc;
                imu_msg.linear_acceleration.z = p_imu_data->zacc;

                // Set the angular velocity
                imu_msg.angular_velocity.x = p_imu_data->xgyro;
                imu_msg.angular_velocity.y = p_imu_data->ygyro;
                imu_msg.angular_velocity.z = p_imu_data->zgyro;

                // Set the orientation as quaternion
                imu_msg.orientation.w = p_imu_data->q_w;
                imu_msg.orientation.x = p_imu_data->q_x;
                imu_msg.orientation.y = p_imu_data->q_y;
                imu_msg.orientation.z = p_imu_data->q_z;

                imu_pub.publish(imu_msg);
                imu_data_count++;
            }
            p_imu_data++;
        }

    } else {
        int head_len      = sizeof(mix_head_t) + p_mix_head->imu_num*sizeof(imu_data_t);
        int remaining_len = recv_len - head_len;

#if (MAVLINK_CODE_DEBUG)
        printf("XX sec: %u nsec: %u num: %u\n", p_mix_head->img_sec, p_mix_head->img_nsec, p_mix_head->imu_num);
#endif

#if (MAVLINK_CODE_DEBUG)
        if (debug()){
            ROS_DEBUG("SplitterHandler buf: 0x%hhn, len: %d", buf, recv_len);
        }
#endif

        if(valid()){

            bool valid = is_valid_rtp_packet((const uint8_t *)buf + head_len, remaining_len);
            if (valid) {

                /*
                * Procedure B: IMU + IMG data
                *
                * Handle IMU data, ahead of real RTP packet
                */
                if (!(0 == p_mix_head->timestamp.img_sec && 0 == p_mix_head->timestamp.img_nsec)) {
                    std_msgs::Header img_msg;
                    img_msg.seq        = p_mix_head->timestamp.img_timestamp;
                    img_msg.stamp.sec  = p_mix_head->timestamp.img_sec;
                    img_msg.stamp.nsec = p_mix_head->timestamp.img_nsec;
                    img_pub.publish(img_msg);
                    image_time_count++;

                    //printf("seq(%u) sec: %u nsec: %u\n", img_msg.seq, p_mix_head->img_sec, p_mix_head->img_nsec);
                }
                
                forward(buf + head_len, remaining_len);

                imu_data_t* p_imu_data = (imu_data_t*)(buf + sizeof(mix_head_t));
                for (int i = 0; i < p_mix_head->imu_num; i++) {
#if (MAVLINK_CODE_DEBUG)
                    printf("%02d sec: %u nsec: %u\n", i+1, p_imu_data->imu_sec, p_imu_data->imu_nsec);
#endif
                    if (!(p_imu_data->imu_sec == 0 && p_imu_data->imu_nsec ==0)) {

                        sensor_msgs::Imu imu_msg;
    
                        imu_msg.header.frame_id = "world";

                        // Set the timestamp (from imu_sec and imu_nsec)
                        imu_msg.header.stamp.sec  = p_imu_data->imu_sec;
                        imu_msg.header.stamp.nsec = p_imu_data->imu_nsec;
                        imu_msg.header.seq        = imu_data_count;
                        
                        // Set the linear acceleration
                        imu_msg.linear_acceleration.x = p_imu_data->xacc;
                        imu_msg.linear_acceleration.y = p_imu_data->yacc;
                        imu_msg.linear_acceleration.z = p_imu_data->zacc;

                        // Set the angular velocity
                        imu_msg.angular_velocity.x = p_imu_data->xgyro;
                        imu_msg.angular_velocity.y = p_imu_data->ygyro;
                        imu_msg.angular_velocity.z = p_imu_data->zgyro;

                        // Set the orientation as quaternion
                        imu_msg.orientation.w = p_imu_data->q_w;
                        imu_msg.orientation.x = p_imu_data->q_x;
                        imu_msg.orientation.y = p_imu_data->q_y;
                        imu_msg.orientation.z = p_imu_data->q_z;

                        imu_pub.publish(imu_msg);
                        imu_data_count++;
                    }
                    p_imu_data++;
                }
            } else {

                /*
                * Procedure C: IMU data
                *
                * Not a valid RTP packet, handling IMU data if necessary
                */
                forward(buf + head_len, remaining_len);

                imu_data_t* p_imu_data = (imu_data_t*)(buf + sizeof(mix_head_t));
                for (int i = 0; i < p_mix_head->imu_num; i++) {
#if (MAVLINK_CODE_DEBUG)
                    printf("%02d sec: %u nsec: %u\n", i+1, p_imu_data->imu_sec, p_imu_data->imu_nsec);
#endif
                    if (!(p_imu_data->imu_sec == 0 && p_imu_data->imu_nsec == 0)) {

                        sensor_msgs::Imu imu_msg;
    
                        imu_msg.header.frame_id = "world";

                        // Set the timestamp (from imu_sec and imu_nsec)
                        imu_msg.header.stamp.sec  = p_imu_data->imu_sec;
                        imu_msg.header.stamp.nsec = p_imu_data->imu_nsec;
                        imu_msg.header.seq        = imu_data_count;
                        
                        // Set the linear acceleration
                        imu_msg.linear_acceleration.x = p_imu_data->xacc;
                        imu_msg.linear_acceleration.y = p_imu_data->yacc;
                        imu_msg.linear_acceleration.z = p_imu_data->zacc;

                        // Set the angular velocity
                        imu_msg.angular_velocity.x = p_imu_data->xgyro;
                        imu_msg.angular_velocity.y = p_imu_data->ygyro;
                        imu_msg.angular_velocity.z = p_imu_data->zgyro;

                        // Set the orientation as quaternion
                        imu_msg.orientation.w = p_imu_data->q_w;
                        imu_msg.orientation.x = p_imu_data->q_x;
                        imu_msg.orientation.y = p_imu_data->q_y;
                        imu_msg.orientation.z = p_imu_data->q_z;

                        imu_pub.publish(imu_msg);
                        imu_data_count++;
                    }
                    p_imu_data++;
                }
            }
        } else {
            connect();

        }
    }

    return 2;
}

int SplitterHandler::send(unsigned char* buf, int& len){
    sendto(fd, buf, len, 0, (struct sockaddr *)&udp_addr, udp_len);
    return len;
}

int SplitterHandler::connect() {
    // Create a UDP socket
    splitter_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (splitter_fd < 0) {
        ROS_ERROR("Create socket failed.");
        return 4;
    }

    memset(&(splitter_addr), 0, sizeof(splitter_addr));

    // Server information
    splitter_addr.sin_family = AF_INET;  // IPv4
    //udp_addr.sin_addr.s_addr = inet_addr(path.c_str());  // remote IP
    if (inet_pton(AF_INET, camera_param.splitter_addr.c_str(), &splitter_addr.sin_addr) <= 0) {
        ROS_ERROR("Invalid address/ Address not supported");
        return 5;
    }
    splitter_addr.sin_port = htons(camera_param.splitter_port);  // Port

    const char *message = "Hello, Server!"; // just for link establish
    (void)sendto(splitter_fd, message, strlen(message), 0, (struct sockaddr *) &splitter_addr, sizeof(splitter_addr));

    ROS_INFO("SplitterHandler forward ip: %s, port: %d", camera_param.splitter_addr.c_str(), camera_param.splitter_port);

    return 0;
}

int SplitterHandler::valid(){
    return splitter_fd;
}

int SplitterHandler::forward(unsigned char* buf, int& len){
    sendto(splitter_fd, buf, len, 0, (struct sockaddr *)&splitter_addr, sizeof(splitter_addr));
    return len;
}

int SplitterHandler::deinit() {
    close(fd);
    close(splitter_fd);
    return 0;
}

