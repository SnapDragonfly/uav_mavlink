#include <ros/ros.h>

#include "connect.h"

int main(int argc, char *argv[]) {

    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle ros_nh;
    ConnectHandler connect;

    ROS_INFO("%s: hello!", NODE_NAME);

    int ret = connect.init(ros_nh);
    if (0 != ret){
        ROS_ERROR("%s: init error(%d)", NODE_NAME, ret);
        return ret;
    }

    while (ros::ok()) {
        ret = connect.update();

        if (0 != ret){
            ROS_ERROR("%s update error(%d)", NODE_NAME, ret);
            return ret;
        }

#if (MAVLINK_CODE_DEBUG)
        if (connect.debug()){
            ROS_DEBUG("%s: update ret=%d", NODE_NAME, ret);
        }
#endif /* MAVLINK_CODE_DEBUG */
    }

    ret = connect.deinit();
    if (0 != ret){
        ROS_ERROR("%s: deinit error(%d)", NODE_NAME, ret);
        return ret;
    }

    ROS_INFO("%s: bye bye", NODE_NAME);

    return 0;
}