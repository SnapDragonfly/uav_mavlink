#ifndef UDPC_HANDLER_H
#define UDPC_HANDLER_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <string>

#include "global.h"
#include "bridge.h"
#include "message.h"

class UdpcHandler : public BridgeHandler {
public:
    // Constructor and Destructor
    UdpcHandler();
    //UdpcHandler(std::unique_ptr<BridgeHandler> ptr);
    ~UdpcHandler() override;

    // Implement the pure virtual methods
    int init(std::string path, int param) override;
    int update(struct pollfd& pfds, class MessageHandler* message, std::unique_ptr<BridgeHandler>& bridge) override;
    int send(unsigned char* buf, int& len);
    int deinit() override;

private:
    unsigned char buf[MAVLINK_DEFAULT_BUF_LEN];
    struct sockaddr_in udp_addr;
    socklen_t udp_len;
};

#endif // UDPC_HANDLER_H
