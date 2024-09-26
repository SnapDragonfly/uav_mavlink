#include <fcntl.h>
#include <poll.h>
#include <termios.h> // Contains POSIX terminal control definitions

#include <ros/ros.h>

#include "uart.h"

UartHandler::UartHandler() {
    //ROS_INFO("UartHandler empty applied!");
}

//UartHandler::UartHandler(std::unique_ptr<BridgeHandler> ptr) : BridgeHandler(std::move(ptr)) {
//    ROS_INFO("UartHandler bridge(ptr) applied!");
//}

UartHandler::~UartHandler() {
}

int UartHandler::init(std::string path, int param) {
    // path: uart dev path
    //param: uart baudrate
    //topic: imu topic
    struct termios tty; // Create new termios struc, we call it 'tty' for convention

    /*
     * UART initialization from FC
     */
    fd = open(path.c_str(), O_RDWR);
    if (fd < 0) {
        ROS_ERROR("Can not open serial port %s.", path.c_str());
        return 1;
    }

    if(tcgetattr(fd, &tty) != 0) {
        ROS_ERROR("Error %i from tcgetattr: %s", errno, strerror(errno));
        return 2;
    }
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;
    //cfsetispeed(&tty, param);
    //cfsetospeed(&tty, param);
    switch(param) {
        case 9600: cfsetispeed(&tty, B9600); cfsetospeed(&tty, B9600); break;
        case 19200: cfsetispeed(&tty, B19200); cfsetospeed(&tty, B19200); break;
        case 115200: cfsetispeed(&tty, B115200); cfsetospeed(&tty, B115200); break;
        case 921600: cfsetispeed(&tty, B921600); cfsetospeed(&tty, B921600); break;
        // Add more cases as needed
        default: ROS_ERROR("Unsupported baud rate: %d", param); return 4;
    }


    if (tcsetattr(fd, TCSANOW, &tty) != 0) {  // Save tty settings, also checking for error
        ROS_ERROR("Error %i from tcsetattr: %s", errno, strerror(errno));
        return 3;
    }

    ROS_INFO("UartHandler path: %s, baud: %d", path.c_str(), param);

    return 0;
}

int UartHandler::update(struct pollfd& pfds, class MessageHandler* message, std::unique_ptr<BridgeHandler>& bridge) {

    unsigned char byte;    

    if (!(pfds.revents & POLLIN)) {
        return 1;
    }

    ssize_t len = read(fd, &byte, 1);
    if (len <= 0) {
        return 2;
    }

#if (MAVLINK_CODE_DEBUG)
    printf("0x%02x ", byte);
    fflush(stdout);
#endif /* MAVLINK_CODE_DEBUG */

    if (message->mavlink_parse(byte)){
        return message->mavlink_handler(bridge);
    }

    return 0;
}

int UartHandler::send(unsigned char* buf, int& len){
    return write(fd, buf, len);
}

int UartHandler::deinit() {
    close(fd);
    return 0;
}
