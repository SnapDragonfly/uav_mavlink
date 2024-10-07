#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

# Global variables to store the timestamps
img_timestamp = None
imu_timestamp = None

img_topic = '/tmp/uav_img'
imu_topic = '/tmp/uav_imu'

def img_callback(data):
    global img_timestamp
    img_timestamp = data.stamp  # Assuming the Imu message has a header

def imu_callback(data):
    global imu_timestamp
    imu_timestamp = data.header.stamp  # Assuming the Header message has a stamp

def time_difference():
    rospy.init_node('time_difference_node', anonymous=True)
    
    rospy.Subscriber(img_topic, Header, img_callback)
    rospy.Subscriber(imu_topic, Imu, imu_callback)

    rate = rospy.Rate(10)  # 30 Hz
    while not rospy.is_shutdown():
        if img_timestamp and imu_timestamp:
            diff = img_timestamp - imu_timestamp
            rospy.loginfo(f'Time difference: {diff.to_sec()} seconds')
        
        rate.sleep()

if __name__ == '__main__':
    try:
        print(f"Waiting for data from image topic: {img_topic} and IMU topic: {imu_topic}")
        time_difference()
    except rospy.ROSInterruptException:
        pass
