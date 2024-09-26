#ifndef BRIDGE_HANDLER_H
#define BRIDGE_HANDLER_H

#include <iostream>
#include <string>

#include "global.h"

class BridgeHandler {
public:
    // Constructor and Destructor
    BridgeHandler();
    virtual ~BridgeHandler() {};

    // Public virtual methods
    virtual int init(std::string path, int param) = 0;
    virtual int update(struct pollfd &pfds, class MessageHandler *message, std::unique_ptr<BridgeHandler>& bridge) = 0;
    virtual int send(unsigned char *buf, int &len) = 0;
    virtual int deinit() = 0;

    // Common methods
    int get(){
        return fd;
    }

protected:
    int fd;
};

#endif /* BRIDGE_HANDLER_H */