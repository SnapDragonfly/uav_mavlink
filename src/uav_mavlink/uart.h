#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include <memory>

#include "global.h"
#include "bridge.h"
#include "message.h"

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

};

#endif // UART_HANDLER_H
