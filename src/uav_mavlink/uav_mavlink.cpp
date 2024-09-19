#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "bridge.h"

int main(int argc, char *argv[]) {

    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle ros_nh;
    MavlinkHandler mavlink_bridge;

    ROS_INFO("%s: hello!", NODE_NAME);

    int ret = mavlink_bridge.mavlink_init(ros_nh);
    if (0 != ret){
        ROS_ERROR("%s: init error(%d)", NODE_NAME, ret);
        return ret;
    }
    while (ros::ok()) {
        ret = mavlink_bridge.mavlink_poll();
        if (0 != ret){
            ROS_ERROR("%s loop error(%d)", NODE_NAME, ret);
            return ret;
        }

#if defined(MAVLINK_CODE_DEBUG)
        if (mavlink_bridge.mvalink_debug()){
            ROS_INFO("%s: loop ret=%d", NODE_NAME, ret);
        }
#endif /* MAVLINK_CODE_DEBUG */
    }

    ret = mavlink_bridge.mavlink_exit();
    if (0 != ret){
        ROS_ERROR("%s: exit error(%d)", NODE_NAME, ret);
        return ret;
    }

    ROS_INFO("%s: bye bye", NODE_NAME);

    return 0;
}
