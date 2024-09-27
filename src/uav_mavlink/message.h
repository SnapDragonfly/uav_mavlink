#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <ros/ros.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "ardupilotmega/mavlink.h"
#pragma GCC diagnostic pop

#include "global.h"
#include "bridge.h"

#define MAVLINK_DEFAULT_COMP_ID   191


class MessageHandler {
public:
    // Constructor and Destructor
    MessageHandler();
    ~MessageHandler(){};

    // Public methods
    int mavlink_init(ros::NodeHandle &ros_nh, std::string &imu_topic, float rate);
    int mavlink_parse(char byte);
    int mavlink_handler(std::unique_ptr<BridgeHandler>& bridge);

private:
    /*
     * Member variables for mavlink
     */   
    unsigned char buf[MAVLINK_DEFAULT_BUF_LEN];

    mavlink_status_t status;
    mavlink_message_t msg;

    float update_interval;
    float mavlink_rate;

    ros::Publisher imu_pub;

    uint8_t mav_sysid;
    int demo_stage;

    bool no_hr_imu, no_att_q;

    float att_q_x, att_q_y, att_q_z, att_q_w;

    int64_t time_offset_us;
    uint64_t last_us;

    float latest_alt, gnd_alt, latest_x, start_x;

    bool debug_enable;
};

#endif /* MESSAGE_HANDLER_H */