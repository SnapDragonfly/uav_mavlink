#include <ros/ros.h>

#include <sensor_msgs/Imu.h>

#include "message.h"

MessageHandler::MessageHandler(bool debug = false) {
    // Default values:
    mav_sysid       = 0;
    demo_stage      = 0;

    no_hr_imu       = true;
    no_att_q        = true;

    att_q_x         = 0;
    att_q_y         = 0;
    att_q_z         = 0;
    att_q_w         = 0;

    time_offset_us  = 0;
    last_us         = 0;

    latest_alt      = 0;
    gnd_alt         = 0;
    latest_x        = 0;
    start_x         = 0;

    update_interval = 0;

    debug_enable    = debug;
}

int MessageHandler::mavlink_init(ros::NodeHandle &ros_nh, std::string &imu_topic, float rate){
    imu_pub = ros_nh.advertise<sensor_msgs::Imu>(imu_topic.c_str(), 100);
    mavlink_rate = rate;
    update_interval = RATIO_SECOND_TO_MICRO_SECOND/mavlink_rate;

#if (MAVLINK_CODE_DEBUG)
    ROS_DEBUG("mavlink topic: %s", imu_topic.c_str());
#endif
    return 0;
}

int MessageHandler::mavlink_parse(char byte){
    if (!mavlink_parse_char(0, byte, &msg, &status)) return 0;

    if (msg.sysid == 255) return 0;

#if (MAVLINK_CODE_DEBUG)
    if(debug_enable) printf("recv msg ID %d, seq %d\n", msg.msgid, msg.seq);
#endif /* MAVLINK_CODE_DEBUG */

    return 1;
}

int MessageHandler::mavlink_handler(std::unique_ptr<BridgeHandler>& bridge){
    int len;
    struct timeval tv;
    //ROS_INFO("recv msg ID %d, seq %d\n", msg.msgid, msg.seq);

    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);

        if (msg.sysid != mav_sysid) {
            mav_sysid = msg.sysid;
            ROS_INFO("found MAV %d", msg.sysid);
        }

        if (0 == time_offset_us) {
            gettimeofday(&tv, NULL);
            mavlink_msg_timesync_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, &msg, 0, tv.tv_sec*1000000+tv.tv_usec, mav_sysid, 1); //fill timesync with us instead of ns
            len = mavlink_msg_to_send_buffer(buf, &msg);
            bridge->send(buf, len);

            mavlink_msg_system_time_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, 0);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            bridge->send(buf, len);

            mavlink_msg_set_gps_global_origin_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, &msg, mav_sysid, 247749434, 1210443077, 100000, tv.tv_sec*1000000+tv.tv_usec);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            bridge->send(buf, len);

            ROS_INFO("sync time_offset_us = 0");
        }

        if (no_hr_imu) {
            mavlink_msg_command_long_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, &msg, 0, 0, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_HIGHRES_IMU, update_interval, 0, 0, 0, 0, 0);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            bridge->send(buf, len);
            ROS_INFO("HIGHRES_IMU set interval %0.2fus", update_interval);
        }

        if (no_att_q) {
            mavlink_msg_command_long_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, &msg, 0, 0, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, update_interval, 0, 0, 0, 0, 0);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            bridge->send(buf, len);
            ROS_INFO("ATTITUDE_QUATERNION set interval %0.2fus", update_interval);
        }

        if (hb.custom_mode == COPTER_MODE_GUIDED) {
            if (demo_stage == 0) {
                demo_stage = 1;
                gettimeofday(&tv, NULL);
                mavlink_msg_command_long_pack(mav_sysid, MAVLINK_DEFAULT_COMP_ID, &msg, mav_sysid, 1, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 1);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                bridge->send(buf, len);
            }
        } else {
            demo_stage = 0;
        }

        if (hb.base_mode & 128) {
            if (gnd_alt == 0) {
                gnd_alt = latest_alt;
                start_x = latest_x;
                ROS_INFO("gnd viso alt %f, start x %f\n", gnd_alt, start_x);
            }
        } else {
            gnd_alt = 0;
        }
    } else if (msg.msgid == MAVLINK_MSG_ID_TIMESYNC) {
        mavlink_timesync_t ts;
        mavlink_msg_timesync_decode(&msg, &ts);
        if (ts.tc1 != 0) {
            time_offset_us = ts.ts1 - ts.tc1;
            ROS_INFO("sync time_offset_us=%ld", time_offset_us);
        }
    } else if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
        mavlink_statustext_t txt;
        mavlink_msg_statustext_decode(&msg, &txt);
        ROS_INFO("fc: %s", txt.text);
    } else if (msg.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
        mavlink_highres_imu_t hr_imu;
        mavlink_msg_highres_imu_decode(&msg, &hr_imu); // time_usec is time since boot
        if (time_offset_us > 0 && hr_imu.time_usec > last_us) {
#if 1
            static int64_t effective_counts = 0;
            static int64_t effective_rate = int64_t(mavlink_rate);
            effective_counts++;
            if (0 == (effective_counts % effective_rate)){
                int64_t effective_hz = RATIO_SECOND_TO_MICRO_SECOND/(hr_imu.time_usec - last_us);

                if (effective_hz >= 0.9*mavlink_rate && effective_hz <= 1.5*mavlink_rate){
                    ROS_INFO("MAVLINK_MSG_ID_HIGHRES_IMU frequency = %ldHz, should be %0.2fHz", 
                                effective_hz, 
                                mavlink_rate);
                    effective_rate *= 5; //increase 5 times for checking
                    no_hr_imu = false;
                    no_att_q = false;
                } else {
                    ROS_WARN("MAVLINK_MSG_ID_HIGHRES_IMU frequency = %ldHz, should be %0.2fHz", 
                                effective_hz, 
                                mavlink_rate);
                    effective_rate = int64_t(mavlink_rate);
                    no_hr_imu = true;
                    no_att_q = true;
                }
            }
#endif

            last_us = hr_imu.time_usec;
            sensor_msgs::Imu imu_msg;
#if 0
            imu_msg.header.stamp = ros::Time::now();
#else
            int64_t ts_us = hr_imu.time_usec + time_offset_us;
            imu_msg.header.stamp.sec = ts_us / 1000000;
            imu_msg.header.stamp.nsec = (ts_us % 1000000) * 1000;
#endif
            imu_msg.header.frame_id = "world";
            imu_msg.linear_acceleration.x = hr_imu.xacc;
            imu_msg.linear_acceleration.y = hr_imu.yacc;
            imu_msg.linear_acceleration.z = hr_imu.zacc;
            imu_msg.angular_velocity.x = hr_imu.xgyro;
            imu_msg.angular_velocity.y = hr_imu.ygyro;
            imu_msg.angular_velocity.z = hr_imu.zgyro;
            imu_msg.orientation.w = att_q_w;
            imu_msg.orientation.x = att_q_x;
            imu_msg.orientation.y = att_q_y;
            imu_msg.orientation.z = att_q_z;
            imu_pub.publish(imu_msg);
        }
    } else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE_QUATERNION) {
        mavlink_attitude_quaternion_t att_q;
        mavlink_msg_attitude_quaternion_decode(&msg, &att_q);
        att_q_w = att_q.q1;
        att_q_x = att_q.q2;
        att_q_y = att_q.q3;
        att_q_z = att_q.q4;
    }

    return 0;
}
