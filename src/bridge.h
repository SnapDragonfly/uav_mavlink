

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <poll.h>

#include "config.h"

class MavlinkHandler {
public:
    // Constructor and Destructor
    MavlinkHandler(){};
    ~MavlinkHandler(){};

    // Public methods
    int mavlink_init(ros::NodeHandle &ros_nh);
    int mavlink_poll();
    int mavlink_exit();

private:
    // Member variables
    ros::Publisher imu_pub;
    struct pollfd pfds[MAVLINK_DEFAULT_NUM_PFDS];
    int uart_fd;
    int ipc_fd;
    int ipc_fd2;
};
