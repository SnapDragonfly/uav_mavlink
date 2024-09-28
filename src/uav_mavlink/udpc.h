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

struct UdpcParam {
    bool debug;
};

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

    bool debug(){
        return udpc_param.debug;
    }
    void set(void* param){
        udpc_param.debug  = ((struct UdpcParam*)param)->debug;
    }

private:
    unsigned char buf[MAVLINK_DEFAULT_BUF_LEN];
    struct sockaddr_in udp_addr;
    socklen_t udp_len;

    struct UdpcParam udpc_param;
};

#endif // UDPC_HANDLER_H
