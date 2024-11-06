#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, Image
import matplotlib.pyplot as plt


# Global variables for rosbag topics
cam0_timestamp = None
imu0_timestamp = None
cam0_topic = '/cam0/image_raw'
imu0_topic = '/imu0'

# Time difference function for rosbag topics
ok_count_bag = 0
ng_count_bag = 0
max_delay_bag = float('-inf')
min_delay_bag = float('inf')
delays_bag = []  # List to store rosbag delay values (in ms)

def cam0_callback(data):
    global cam0_timestamp
    cam0_timestamp = data.header.stamp

def imu0_callback(data):
    global imu0_timestamp
    imu0_timestamp = data.header.stamp

# Global variables to store the last difference for logging
last_diff_ns_bag = None


def time_difference_bag():
    global ok_count_bag, ng_count_bag, max_delay_bag, min_delay_bag, last_diff_ns_bag

    # Subscribers for rosbag topics
    rospy.Subscriber(cam0_topic, Image, cam0_callback)
    rospy.Subscriber(imu0_topic, Imu, imu0_callback)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        if cam0_timestamp and imu0_timestamp:
            diff = cam0_timestamp - imu0_timestamp
            diff_ns = diff.to_nsec()

            # Only log if there is a new time difference
            if last_diff_ns_bag != diff_ns:
                last_diff_ns_bag = diff_ns  # Update the last logged difference

                # Convert to milliseconds and microseconds
                diff_ms = diff_ns / 1_000_000.0  # Convert to milliseconds

                # Record the delay for distribution analysis
                delays_bag.append(diff_ms)

                # Update min and max delay
                if diff_ms > max_delay_bag:
                    max_delay_bag = diff_ms
                if diff_ms < min_delay_bag:
                    min_delay_bag = diff_ms

                # Determine if the time difference is OK or NG
                if diff_ns <= 0:
                    status = "OK"
                    ok_count_bag += 1
                else:
                    status = "NG"
                    ng_count_bag += 1

                # Convert milliseconds and remaining microseconds for aligned logging
                diff_us = (diff_ns % 1_000_000) // 1_000  # Get remaining microseconds
                diff_ms_int = int(diff_ms)  # Integer part of milliseconds

                # Format output with aligned width (3 digits for ms and 3 digits for us)
                rospy.loginfo(f'Rosbag time difference[{status}]: {diff_ms_int:3d} ms {diff_us:03d} us')

        rate.sleep()

def print_statistics(total_samples, ok_count, ng_count, max_delay, min_delay, delays):
    if total_samples > 0:
        ok_percentage = (ok_count / total_samples) * 100
        ng_percentage = (ng_count / total_samples) * 100
        rospy.loginfo(f"Total samples: {total_samples}")
        rospy.loginfo(f"OK: {ok_count} ({ok_percentage:.2f}%)")
        rospy.loginfo(f"NG: {ng_count} ({ng_percentage:.2f}%)")
        rospy.loginfo(f"Max delay: {max_delay:.2f} ms")
        rospy.loginfo(f"Min delay: {min_delay:.2f} ms")
    else:
        rospy.loginfo("No samples recorded.")

    # Plot delay distribution
    if delays:
        plt.hist(delays, bins=50, color='blue', alpha=0.7)
        plt.title('Delay Distribution (ms)')
        plt.xlabel('Delay (ms)')
        plt.ylabel('Frequency')
        plt.grid(True)
        plt.show()

if __name__ == '__main__':

    try:
        rospy.init_node('time_difference_bag_node', anonymous=True)
        print(f"Waiting for data from rosbag topics: {cam0_topic} and {imu0_topic}")
        time_difference_bag()
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        # Print statistics for rosbag topics
        total_samples_bag = ok_count_bag + ng_count_bag
        print_statistics(total_samples_bag, ok_count_bag, ng_count_bag, max_delay_bag, min_delay_bag, delays_bag)
