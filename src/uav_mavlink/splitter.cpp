
#include <ros/ros.h>

#include "splitter.h"
#include "rtp_head.h"

SplitterHandler::SplitterHandler() {
    camera_param.camera_clock_hz = 90000;
    camera_param.camera_frame_hz = 30;
    camera_param.camera_sync_num = 100;
    camera_param.camera_threshold = 5;

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

#if (MAVLINK_CODE_DEBUG)
    if (debug()){
        ROS_DEBUG("SplitterHandler buf: 0x%hhn, len: %d", buf, recv_len);
    }
#endif

    // Parse RTP header
    struct rtp_header rtp;
    parse_rtp_header(buf, &rtp);

    // Validate RTP header
    if (validate_rtp_first(&rtp, recv_len)) {
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

int SplitterHandler::deinit() {
    close(fd);
    return 0;
}

