

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <poll.h>
#include <netinet/in.h>

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
    // Private methods
    int uart_create();
    int udp_create();
    int ipc1_create();
    int ipc2_create();

    int uart_poll();
    int udp_poll();
    int mavlink_handler(unsigned char *buf, int len);
    void cc_send(unsigned char *buf, int len);

    int ipc1_poll();
    int ipc2_poll();

    int config_read();
    void config_print(std::string title);

    // Member variables
    struct pollfd pfds[MAVLINK_DEFAULT_NUM_PFDS];
    unsigned char buf[MAVLINK_DEFAULT_BUF_LEN];

    int uart_fd, udp_fd, ipc_fd, ipc_fd2;
    ros::Publisher imu_pub;

    uint8_t mav_sysid = 0;
    int demo_stage = 0;
    bool no_hr_imu = true;
    bool no_att_q = true;
    float att_q_x =0, att_q_y = 0, att_q_z = 0, att_q_w = 0;
    int64_t time_offset_us = 0;
    uint64_t last_us = 0;
    float latest_alt = 0, gnd_alt = 0, latest_x = 0, start_x = 0;

    std::string mavlink_activate;
    int mavlink_rate;
    bool uart_enable;
    std::string com_path;

    int com_port;
    struct sockaddr_in com_client_addr;
    socklen_t com_client_len;

    std::string ipc1_path;
    std::string ipc2_path;
    std::string imu_topic;
};
