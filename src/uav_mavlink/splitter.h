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

#define SPLITTER_TEST_FUNCTION      0

struct SplitterParam {
    int camera_clock_hz;    // Camera clock frequency in Hz
    int camera_frame_hz;    // Camera frame rate in Hz
    int camera_sync_num;    // Number of camera syncs
    float camera_threshold; // Threshold value for the camera
    int splitter_port;
    std::string splitter_addr;
    std::string imu_topic;
    std::string img_topic;
    ros::NodeHandle ros_nh;
    bool debug;
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

    bool debug(){
        return camera_param.debug;
    }
    void set(void* param){
        camera_param.camera_clock_hz  = ((struct SplitterParam*)param)->camera_clock_hz;
        camera_param.camera_frame_hz  = ((struct SplitterParam*)param)->camera_frame_hz;
        camera_param.camera_sync_num  = ((struct SplitterParam*)param)->camera_sync_num;
        camera_param.camera_threshold = ((struct SplitterParam*)param)->camera_threshold;
        camera_param.splitter_port    = ((struct SplitterParam*)param)->splitter_port;
        camera_param.splitter_addr    = ((struct SplitterParam*)param)->splitter_addr;
        camera_param.imu_topic        = ((struct SplitterParam*)param)->imu_topic;
        camera_param.img_topic        = ((struct SplitterParam*)param)->img_topic;
        camera_param.ros_nh           = ((struct SplitterParam*)param)->ros_nh;
        camera_param.debug            = ((struct SplitterParam*)param)->debug;
    }

private:

    int connect();
    int valid();
    int forward(unsigned char* buf, int& len);

    unsigned char buf[RTP_DEFAULT_BUF_LEN];
    struct sockaddr_in udp_addr;
    socklen_t udp_len;

    struct SplitterParam camera_param;
    ros::Publisher img_pub;
    ros::Publisher imu_pub;

    struct sockaddr_in splitter_addr;
    socklen_t splitter_len;
    int splitter_fd;

    SyncSystem sys;
};

#endif /* SPLITTER_HANDLER_H */