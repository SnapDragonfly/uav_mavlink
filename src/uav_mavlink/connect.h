#ifndef CONNECT_HANDLER_H
#define CONNECT_HANDLER_H

#include <poll.h>
#include <netinet/in.h>

#include "global.h"
#include "config.h"
#include "bridge.h"
#include "message.h"

#define MAVLINK_DEFAULT_NUM_PFDS  1

class ConnectHandler {
public:
    // Constructor and Destructor
    ConnectHandler();
    ~ConnectHandler(){
        //delete bridge;
        delete message;
        delete config;
    };

    // Public methods
    int init(ros::NodeHandle &ros_nh);
    int update();
    int deinit();

    bool debug();

private:
    /*
     * Private methods
     */ 
#if 0
    int ipc1_create();
    int ipc2_create();

    int ipc1_poll();
    int ipc2_poll();
#endif
    /*
     * Member variables for bridge
     */ 
    struct pollfd pfds[MAVLINK_DEFAULT_NUM_PFDS];
    //int ipc_fd, ipc_fd2;
    //unsigned char buf[MAVLINK_DEFAULT_BUF_LEN];

    /*
     * Member variables for connection and message
     */
    std::unique_ptr<BridgeHandler> bridge;
    class MessageHandler *message;
    class ConfigHandler  *config;
};


#endif /* CONNECT_HANDLER_H */