
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

#define TIMING_STATUS(A, B)  ((A <= B)?"OK":"NG")

int SplitterHandler::update(struct pollfd& pfds, class MessageHandler* message, std::unique_ptr<BridgeHandler>& bridge) {

    if (!(pfds.revents & POLLIN)) {
        return 1;
    }

    udp_len = sizeof(udp_addr);
    int recv_len = recvfrom(fd, buf, RTP_DEFAULT_BUF_LEN, 0, (struct sockaddr *)&udp_addr, &udp_len);
    int remaining_len = recv_len - FORWARD_RTP_PREFIX_LEN;

#if (MAVLINK_CODE_DEBUG)
    if (debug()){
        ROS_DEBUG("SplitterHandler buf: 0x%hhn, len: %d", buf, recv_len);
    }
#endif

    if(valid()){
        ImuData* p_imu_data = (ImuData*)buf;
#if 0
        //if(p_imu_data->img_sec == p_imu_data->imu_sec && p_imu_data->imu_nsec > p_imu_data->img_nsec) {
            printf("img_sec: %u img_nsec: %u \n", p_imu_data->img_sec, p_imu_data->img_nsec);
            printf("imu_sec: %u imu_nsec: %u \n", p_imu_data->imu_sec, p_imu_data->imu_nsec);
        //}
#endif

        if (!(p_imu_data->img_sec == 0 && p_imu_data->imu_nsec ==0)) {

            sensor_msgs::Imu imu_msg;
            std_msgs::Header img_msg;

            img_msg.stamp.sec  = p_imu_data->img_sec;
            img_msg.stamp.nsec = p_imu_data->img_nsec;

            img_pub.publish(img_msg);

            imu_msg.header.frame_id = "world";

            // Set the timestamp (from imu_sec and imu_nsec)
            imu_msg.header.stamp.sec = p_imu_data->imu_sec;
            imu_msg.header.stamp.nsec = p_imu_data->imu_nsec;
            
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
        }

        forward(RTP_BUFFER_ADDR(buf), remaining_len);

    }

    // Parse RTP header
    struct rtp_header rtp;
    parse_rtp_header(RTP_BUFFER_ADDR(buf), &rtp);

    // Validate RTP header
    if (validate_rtp_first(&rtp, remaining_len)) {
        static unsigned int packet_count = 0;
        static unsigned int packet_error = 0;
        static unsigned int update_count = camera_param.camera_frame_hz/2;
        static unsigned int check_skip = 0;
        static double previous_error = 100;
        static double latest_error   = 100;
        double error;

        if (packet_count == 0){
            synchronize_time(&sys, rtp.timestamp);
            check_skip = 1;

#if (MAVLINK_CODE_DEBUG)
            if (debug()){
                printf("%u sync first\n", rtp.sequence);
            }
#endif
        } else if (packet_count % camera_param.camera_sync_num == 0) {
            error = calculate_error(&sys, rtp.timestamp);

            bool status1, status2, status_trend;
            status_trend = latest_error > previous_error;  // and error's trend is getting large

            status1   = abs(error) > camera_param.camera_threshold && status_trend;      // abs() > error threshold
            status2   = error < 0 && status_trend;                          // error < 0

            if ( status1 || status2 ){
                synchronize_time(&sys, rtp.timestamp);
                check_skip = 1;
                printf("%u sync error %.2f %.2f %.2f\n", rtp.sequence, error, previous_error, latest_error);
            }else{
                printf("%u skip sync %.2f %.2f %.2f\n", rtp.sequence, error, previous_error, latest_error);
            }
        }
        packet_count++;
        update_count++;

        unsigned int calculated_timestamp = calculate_timestamp(&sys);
        if(rtp.timestamp > calculated_timestamp){
            packet_error++;
            previous_error = latest_error;
            latest_error = 100.0*packet_error/packet_count;
#if (MAVLINK_CODE_DEBUG)
            if (debug()){
                printf("%u error timestamp: %u vs %u\n", rtp.sequence, rtp.timestamp, calculated_timestamp);
            }
        }else{
            if (debug()){
                printf("%u good timestamp: %u vs %u\n", rtp.sequence, rtp.timestamp, calculated_timestamp);
            }
#endif
        }

        if (update_count % camera_param.camera_frame_hz == 0){
            if(!valid()){ //check RTP client
                connect();
            }

            if (check_skip == 0){
                double estimated_time = estimate_time(&sys, rtp.timestamp);
                double system_time    = get_system_time_us();
                // Estimate system time for the latest count
                printf("%u time %s %.2f vs %.2f µs\n", 
                        rtp.sequence, TIMING_STATUS(estimated_time, system_time), estimated_time, system_time);
                printf("%u stmp %s %u vs %u counts\n", 
                        rtp.sequence, TIMING_STATUS(rtp.timestamp, calculated_timestamp), rtp.timestamp, calculated_timestamp);
                previous_error = latest_error;
                latest_error = 100.0*packet_error/packet_count;
                printf("%u erro %.2f %% - %d\n", rtp.sequence, latest_error, packet_count);
            }else{ 
                check_skip = 0;
            }
        }
        //print_rtp_header(&rtp);
    }

#if 0  // function to be implemented
    for (int i = 0; i < len; i++) {
        if (message->mavlink_parse(buf[i])){
            return message->mavlink_handler(bridge);
        }
    }
#endif

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

