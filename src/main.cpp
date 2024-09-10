#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "bridge.h"

int main(int argc, char *argv[]) {

    ros::init(argc, argv, PACKAGE_NAME);

    ros::NodeHandle ros_nh;
    MavlinkHandler mavlink_bridge;

    ROS_INFO("%s: hello!", PACKAGE_NAME);

    int ret = mavlink_bridge.mavlink_init(ros_nh);
    if (0 != ret){
        ROS_ERROR("%s: init error(%d)", PACKAGE_NAME, ret);
        return ret;
    }
    while (ros::ok()) {
        ret = mavlink_bridge.mavlink_poll();
        if (0 != ret){
            ROS_ERROR("%s loop error(%d)", PACKAGE_NAME, ret);
            return ret;
        }

        if (mavlink_bridge.mvalink_debug()){
            ROS_INFO("%s: loop ret=%d", PACKAGE_NAME, ret);
        }
        
    }

    ret = mavlink_bridge.mavlink_exit();
    if (0 != ret){
        ROS_ERROR("%s: exit error(%d)", PACKAGE_NAME, ret);
        return ret;
    }

    ROS_INFO("%s: bye bye", PACKAGE_NAME);

    return 0;
}
