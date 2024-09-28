#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include <memory>

#include "global.h"
#include "bridge.h"
#include "message.h"

struct UartParam {
    bool debug;
};

class UartHandler : public BridgeHandler {
public:
    // Constructor and Destructor
    UartHandler();
    //UartHandler(std::unique_ptr<BridgeHandler> ptr);
    ~UartHandler() override;

    // Implement the pure virtual methods
    int init(std::string path, int param) override;
    int update(struct pollfd& pfds, class MessageHandler* message, std::unique_ptr<BridgeHandler>& bridge) override;
    int send(unsigned char* buf, int& len);
    int deinit() override;

    bool debug(){
        return uart_param.debug;
    }
    void set(void* param){
        uart_param.debug  = ((struct UartParam*)param)->debug;
    }

private:
    struct UartParam uart_param;
};

#endif // UART_HANDLER_H
