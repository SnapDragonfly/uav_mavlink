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

struct SplitterParam {
    int camera_clock_hz;    // Camera clock frequency in Hz
    int camera_frame_hz;    // Camera frame rate in Hz
    int camera_sync_num;    // Number of camera syncs
    float camera_threshold; // Threshold value for the camera
};

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

    virtual void set(void* param){
        camera_param.camera_clock_hz  = ((struct SplitterParam*)param)->camera_clock_hz;
        camera_param.camera_frame_hz  = ((struct SplitterParam*)param)->camera_frame_hz;
        camera_param.camera_sync_num  = ((struct SplitterParam*)param)->camera_sync_num;
        camera_param.camera_threshold = ((struct SplitterParam*)param)->camera_threshold;
    }

private:
    unsigned char buf[RTP_DEFAULT_BUF_LEN];
    struct sockaddr_in udp_addr;
    socklen_t udp_len;

    struct SplitterParam camera_param;

    SyncSystem sys;
};

#endif /* SPLITTER_HANDLER_H */