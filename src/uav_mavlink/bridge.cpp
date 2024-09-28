#include <ros/ros.h>

#include "bridge.h"

BridgeHandler::BridgeHandler() {
#if (MAVLINK_CODE_DEBUG)
    if (debug()){
        ROS_DEBUG("BridgeHandler empty applied!");
    }
#endif
}

