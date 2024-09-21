
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <poll.h>
#include <netinet/in.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "ardupilotmega/mavlink.h"
#pragma GCC diagnostic pop

#include "config.h"

class MavlinkHandler {
public:
    // Constructor and Destructor
    MavlinkHandler();
    ~MavlinkHandler(){};

    // Public methods
    int mavlink_init(ros::NodeHandle &ros_nh);
    int mavlink_poll();
    int mavlink_exit();

    bool mvalink_debug();

private:
    /*
     * Private methods
     */ 
    int uart_create();
    int udps_create();
    int udpc_create();
    int ipc1_create();
    int ipc2_create();

    int uart_poll();
    int udps_poll();
    int udpc_poll();
    int mavlink_handler(mavlink_message_t &msg, mavlink_status_t &status);
    void cc_send(unsigned char *buf, int len);

    int ipc1_poll();
    int ipc2_poll();

    int config_read();
    void config_print(std::string title);

    /*
     * Member variables for bridge
     */ 
    struct pollfd pfds[MAVLINK_DEFAULT_NUM_PFDS];
    int uart_fd, udp_fd, ipc_fd, ipc_fd2;
    unsigned char buf[MAVLINK_DEFAULT_BUF_LEN];

    struct sockaddr_in udp_addr;
    socklen_t udp_len;

    /*
     * Member variables for mavlink
     */
    mavlink_status_t status;
    mavlink_message_t msg;
    
    float update_interval;
    ros::Publisher imu_pub;

    uint8_t mav_sysid;
    int demo_stage;

    bool no_hr_imu, no_att_q;

    float att_q_x, att_q_y, att_q_z, att_q_w;

    int64_t time_offset_us;
    uint64_t last_us;

    float latest_alt, gnd_alt, latest_x, start_x;

    /*
     * Member variables for configuration
     */
    float mavlink_rate;

    std::string mavlink_activate;
    enum ComType {
        COM_UART,
        COM_UDPS,
        COM_UDPC
    } com_uart_udp_type;

    std::string com_uart_path;
    int com_uart_baud;
    int com_udpls_port;
    int com_udprs_port;
    std::string com_udprs_addr;

    std::string ipc1_path;
    std::string ipc2_path;
    std::string imu_topic;

    bool debug_enable;
};
