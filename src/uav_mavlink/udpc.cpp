#include <ros/ros.h>

#include "udpc.h"

UdpcHandler::UdpcHandler() {
#if (MAVLINK_CODE_DEBUG)
    ROS_INFO("UdpcHandler empty applied!");
#endif
}

//UdpcHandler::UdpcHandler(std::unique_ptr<BridgeHandler> ptr) : BridgeHandler(std::move(ptr)) {
//    ROS_INFO("UdpcHandler bridge applied!");
//}

UdpcHandler::~UdpcHandler() {
}

int UdpcHandler::init(std::string path, int param) {
    // Create a UDP socket
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        ROS_ERROR("Create socket failed.");
        return 4;
    }

    memset(&udp_addr, 0, sizeof(udp_addr));

    // Server information
    udp_addr.sin_family = AF_INET;  // IPv4
    //udp_addr.sin_addr.s_addr = inet_addr(path.c_str());  // remote IP
    if (inet_pton(AF_INET, path.c_str(), &udp_addr.sin_addr) <= 0) {
        ROS_ERROR("Invalid address/ Address not supported");
        return 5;
    }
    udp_addr.sin_port = htons(param);  // Port

    const char *message = "Hello, Server!"; // just for link establish
    (void)sendto(fd, message, strlen(message), 0, (struct sockaddr *) &udp_addr, sizeof(udp_addr));

    ROS_INFO("UdpcHandler ip: %s, port: %d", path.c_str(), param);

    return 0;
}

int UdpcHandler::update(struct pollfd& pfds, class MessageHandler* message, std::unique_ptr<BridgeHandler>& bridge) {
    if (!(pfds.revents & POLLIN)) {
        return 1;
    }

    udp_len = sizeof(udp_addr);
    int len = recvfrom(fd, buf, MAVLINK_DEFAULT_BUF_LEN, 0, (struct sockaddr *) &udp_addr, &udp_len);

    for (int i = 0; i < len; i++) {
        if (message->mavlink_parse(buf[i])){
            return message->mavlink_handler(bridge);
        }
    }
    return 2;
}

int UdpcHandler::send(unsigned char* buf, int& len){
    sendto(fd, buf, len, 0, (struct sockaddr *)&udp_addr, udp_len);
    return len;
}

int UdpcHandler::deinit() {
    close(fd);
    return 0;
}
