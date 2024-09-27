#ifndef SPLITTER_HANDLER_H
#define SPLITTER_HANDLER_H

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
#include "time_sync.h"

class SplitterHandler : public BridgeHandler {
public:
    // Constructor and Destructor
    SplitterHandler();
    //UartHandler(std::unique_ptr<BridgeHandler> ptr);
    ~SplitterHandler() override;

    // Implement the pure virtual methods
    int init(std::string path, int param) override;
    int update(struct pollfd& pfds, class MessageHandler* message, std::unique_ptr<BridgeHandler>& bridge) override;
    int send(unsigned char* buf, int& len);
    int deinit() override;

    virtual void set(int clock, int frame, int sync, int threshold){
        camera_clcok_hz  = clock;
        camera_frame_hz  = frame;
        camera_sync_num  = sync;
        camera_threshold = threshold;
    }

private:
    unsigned char buf[RTP_DEFAULT_BUF_LEN];
    struct sockaddr_in udp_addr;
    socklen_t udp_len;

    int camera_clcok_hz;
    int camera_frame_hz;
    int camera_sync_num;
    float camera_threshold;
    SyncSystem sys;
};

#endif /* SPLITTER_HANDLER_H */