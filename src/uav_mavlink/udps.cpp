#include <ros/ros.h>

#include "udps.h"

UdpsHandler::UdpsHandler() {
#if (MAVLINK_CODE_DEBUG)
    if (debug()){
        ROS_DEBUG("UdpsHandler empty applied!");
    }
#endif
}

UdpsHandler::~UdpsHandler() {
}

int UdpsHandler::init(std::string path, int param) {
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

    ROS_INFO("UdpsHandler ip: %s, port: %d", "127.0.0.1", param);

    return 0;
}

int UdpsHandler::update(struct pollfd& pfds, class MessageHandler* message, std::unique_ptr<BridgeHandler>& bridge) {

    if (!(pfds.revents & POLLIN)) {
        return 1;
    }

    udp_len = sizeof(udp_addr);
    int len = recvfrom(fd, buf, MAVLINK_DEFAULT_BUF_LEN, 0, (struct sockaddr *)&udp_addr, &udp_len);

    for (int i = 0; i < len; i++) {
        if (message->mavlink_parse(buf[i])){
            return message->mavlink_handler(bridge);
        }
    }

    return 2;
}

int UdpsHandler::send(unsigned char* buf, int& len){
    sendto(fd, buf, len, 0, (struct sockaddr *)&udp_addr, udp_len);
    return len;
}

int UdpsHandler::deinit() {
    close(fd);
    return 0;
}
