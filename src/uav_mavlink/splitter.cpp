
#include <ros/ros.h>

#include "splitter.h"
#include "rtp_head.h"

SplitterHandler::SplitterHandler() {
    camera_clcok_hz = 90000;
    camera_frame_hz = 30;
    camera_sync_num = 100;

#if (MAVLINK_CODE_DEBUG)
    ROS_INFO("SplitterHandler empty applied!");
#endif
}

//SplitterHandler::SplitterHandler(std::unique_ptr<BridgeHandler> ptr) : BridgeHandler(std::move(ptr)) {
//    ROS_INFO("SplitterHandler bridge applied!");
//}

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

    init_sync_system(&sys, camera_clcok_hz); // Initialize with a clock frequency of 90*1000 Hz

    ROS_INFO("SplitterHandler ip: %s, port: %d", "127.0.0.1", param);

    return 0;
}

int SplitterHandler::update(struct pollfd& pfds, class MessageHandler* message, std::unique_ptr<BridgeHandler>& bridge) {

    if (!(pfds.revents & POLLIN)) {
        return 1;
    }

    udp_len = sizeof(udp_addr);
    int recv_len = recvfrom(fd, buf, RTP_DEFAULT_BUF_LEN, 0, (struct sockaddr *)&udp_addr, &udp_len);
    //ROS_INFO("SplitterHandler buf: 0x%hhn, len: %d", buf, recv_len);

    // Parse RTP header
    struct rtp_header rtp;
    parse_rtp_header(buf, &rtp);

#if 0
    static unsigned int timestamp = 0;
    if (timestamp != rtp.timestamp){
        printf("timestamp mismatched!\n");
    }
#endif

    // Validate RTP header
    if (validate_rtp_first(&rtp, recv_len)) {
        static unsigned int packet_count = 0;
        static unsigned int update_count = camera_frame_hz/2;
        static unsigned int check_skip = 0;

        if (packet_count % camera_sync_num == 0){
            synchronize_time(&sys, rtp.timestamp);
            check_skip = 1;
        }
        packet_count++;
        update_count++;

        if (update_count % camera_frame_hz == 0){
            if (check_skip == 0){
                // Estimate system time for the latest count
                double estimated_time = estimate_time(&sys, rtp.timestamp);
                //printf("Estimated time for count %u: %.2f µs actually %.2f µs\n", rtp.timestamp, estimated_time, get_system_time_us());
                double percentage = calculate_error(&sys, rtp.timestamp);

                if (percentage < 0 || percentage > camera_threshold){
                    printf("sync time %.2f %.2f\n", percentage, camera_threshold);
                    synchronize_time(&sys, rtp.timestamp);
                    check_skip = 1;
                } 
            }else{
                check_skip = 0;
            }
        }
        //timestamp = rtp.timestamp;
        //print_rtp_header(&rtp);
    }

#if 0
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

