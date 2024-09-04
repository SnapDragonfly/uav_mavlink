#include<ros/ros.h>
 
int main(int argc,char **argv){   
 
ros::init(argc,argv,"uav_mavlink");   
 
ros::NodeHandle nh;   
 
ROS_INFO_STREAM("hello,ROS, uav_mavlink!");
 
}

