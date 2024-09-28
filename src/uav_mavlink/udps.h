#ifndef UDPS_HANDLER_H
#define UDPS_HANDLER_H

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

struct UdpsParam {
    bool debug;
};

class UdpsHandler : public BridgeHandler {
public:
    // Constructor and Destructor
    UdpsHandler();
    //UdpsHandler(std::unique_ptr<BridgeHandler> ptr);
    ~UdpsHandler() override;

    // Implement the pure virtual methods
    int init(std::string path, int param) override;
    int update(struct pollfd& pfds, class MessageHandler* message, std::unique_ptr<BridgeHandler>& bridge) override;
    int send(unsigned char* buf, int& len);
    int deinit() override;

    bool debug(){
        return udps_param.debug;
    }
    void set(void* param){
        udps_param.debug  = ((struct UdpsParam*)param)->debug;
    }

private:
    unsigned char buf[MAVLINK_DEFAULT_BUF_LEN];
    struct sockaddr_in udp_addr;
    socklen_t udp_len;

    struct UdpsParam udps_param;
};

#endif // UDPS_HANDLER_H