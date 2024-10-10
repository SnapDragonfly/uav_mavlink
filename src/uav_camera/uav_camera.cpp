/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */


#include <iostream>
#include <string>
#include <filesystem>  // C++17

#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include <yaml-cpp/yaml.h>

#include "videoSource.h"
#include "videoOutput.h"

#include "logging.h"
#include "commandLine.h"

#include <signal.h>

#include "config.h"

#define TAG_IMG_LATENCY         "img_latency"
#define TAG_IMG_SOURCE          "img_source"
#define TAG_IMG_TOPIC           "img_topic"
#define TAG_TIM_TOPIC           "tim_topic"
#define TAG_IMG_DEBUG           "debug"
#define TAG_UAV_IMU             "uav_imu"

bool signal_recieved   = false;
videoOutput* output    = NULL;

bool debug_enable      = true;
int  img_latency       = 100;

std::string img_source = CAMERA_DEFAULT_IMAGE_SOURCE;
std::string img_topic  = CAMERA_DEFAULT_IMAGE_TOPIC;
std::string tim_topic  = CAMERA_DEFAULT_TIME_TOPIC;
std::string imu_topic  = MAVLINK_DEFAULT_IMU_TOPIC;

char params[CAMERA_ARGC_LEN][CAMERA_ARGV_LEN];
char* params_ptr[CAMERA_ARGC_LEN];

ros::Time lastest_time    = ros::Time(0);
ros::Time previous_time   = ros::Time(0);
ros::Time local_sys_time  = ros::Time(0);
ros::Time imu_sys_time    = ros::Time(0);

std::deque<ros::Time> time_queue;
const size_t MAX_QUEUE_SIZE = 10;

int popup_state = 0;
uint32_t popup_count = 0;
uint32_t no_popup_count = 0;
uint32_t max_no_popup_count = 1;

void sig_handler(int signo)
{
    if( signo == SIGINT )
    {
        LogInfo("received SIGINT\n");
        signal_recieved = true;
    }
}

std::mutex img_time_mutex;

void imgMsgCallback(const std_msgs::Header::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(img_time_mutex);

    if(time_queue.empty() || local_sys_time != time_queue.back()) {
        if (popup_count < 5) {
            popup_count++;
        }
        previous_time  = lastest_time;
        lastest_time   = msg->stamp;
        local_sys_time = ros::Time::now();
        time_queue.push_back(lastest_time);
    }
}

std::mutex imu_time_mutex;

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    std::lock_guard<std::mutex> lock(imu_time_mutex);
    imu_sys_time = imu_msg->header.stamp;
}

void config_print(const char* title){
    printf("%s ------------->\n", title);

    printf("img latency: %d\n", img_latency);
    printf(" img source: %s\n",img_source.c_str());
    printf("  img topic: %s\n",img_topic.c_str());
    printf("  imu topic: %s\n",imu_topic.c_str());
    printf(" time topic: %s\n",tim_topic.c_str());

    printf("debug: %s\n", debug_enable?"true":"false");

    printf("%s <-------------\n", title);
}

void camera_print() {
    printf(" --------------\n");
    printf("Max no popup count: %u\n", max_no_popup_count);
    printf(" --------------\n");
}

int config_read(int argc, char** argv){
    try {   
        // Get the directory path of a package
        std::filesystem::path package_path = ros::package::getPath(PACKAGE_NAME);
        std::filesystem::path config_path = package_path / PACKAGE_CONFIG;
        printf("Config file path: %s\n", config_path.c_str());

        // Load the YAML file
        YAML::Node config = YAML::LoadFile(config_path.c_str());

        img_latency  = config[TAG_IMG_LATENCY].as<int>();
        img_topic    = config[TAG_IMG_TOPIC].as<std::string>();
        imu_topic    = config[TAG_UAV_IMU].as<std::string>();
        tim_topic    = config[TAG_TIM_TOPIC].as<std::string>();
        img_source   = config[TAG_IMG_SOURCE].as<std::string>();

        debug_enable = config[TAG_IMG_DEBUG].as<bool>();

        config_print("set"); 
        // Assign argc/argv
        memset(params[0], 0, CAMERA_ARGV_LEN);
        strncpy(params[0], argv[0], CAMERA_ARGV_LEN - 1);
        memset(params[1], 0, CAMERA_ARGV_LEN);
        strncpy(params[1], img_source.c_str(), strlen(img_source.c_str()));

        params_ptr[0] = params[0];
        params_ptr[1] = params[1];
    }
    catch (const std::runtime_error& e) {
        ROS_ERROR("Runtime error: %s", e.what());
        return 1;  // Return a non-zero value to indicate an error occurred
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 2;  // Return a non-zero value to indicate an error occurred
    }
    catch (...) {
        ROS_ERROR("Unknown exception occurred");
        return 3;  // Return a non-zero value to indicate an error occurred
    }
    return 0;
}

int usage()
{
    printf("usage: video-viewer [--help] input_URI [output_URI]\n\n");
    printf("View/output a video or image stream.\n");
    printf("See below for additional arguments that may not be shown above.\n\n");
    printf("positional arguments:\n");
    printf("    input_URI       resource URI of input stream  (see videoSource below)\n");
    printf("    output_URI      resource URI of output stream (see videoOutput below)\n\n");

    printf("%s", videoSource::Usage());
    printf("%s", videoOutput::Usage());
    printf("%s", Log::Usage());

    return 0;
}

int main( int argc, char** argv )
{
    // Initialize ROS
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;
    
    int ret = config_read(argc, argv);
    // Parse command line
    commandLine cmdLine(ret?argc:CAMERA_ARGC_LEN, ret?argv:&params_ptr[0]);

    if( ret && cmdLine.GetFlag("help") )
        return usage();

    /*
     * attach signal handler
     */    
    if( signal(SIGINT, sig_handler) == SIG_ERR )
        LogError("can't catch SIGINT\n");


    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>(img_topic.c_str(), 1);
    ros::Subscriber image_sub = nh.subscribe(tim_topic.c_str(), 1, imgMsgCallback);
    ros::Subscriber imu_sub = nh.subscribe(imu_topic.c_str(), 1, imuCallback);
    /*
     * create input video stream
     */
    videoSource* input = videoSource::Create(cmdLine, ARG_POSITION(0));

    if( !input )
    {
        LogError("video-viewer:  failed to create input stream\n");
        return 0;
    }


    if(debug_enable){
        /*
        * create output video stream
        */
        output = videoOutput::Create(cmdLine, ARG_POSITION(1));
        
        if( !output )
        {
            LogError("video-viewer:  failed to create output stream\n");
            return 0;
        }
    }
    

    /*
     * capture/display loop
     */
    uint32_t numFrames = 0;
    while( !signal_recieved )
    {
        uchar3* image = NULL;
        int status = 0;
        
        if( !input->Capture(&image, &status) )
        {
            if( status == videoSource::TIMEOUT )
                continue;
            
            break; // EOS
        }

        if( numFrames % 25 == 0 || numFrames < 15 )
            LogVerbose("video-viewer:  captured %u frames (%ux%u)\n", numFrames, input->GetWidth(), input->GetHeight());
        
        numFrames++;
        
        if(debug_enable){
            if( output != NULL )
            {
                output->Render(image, input->GetWidth(), input->GetHeight());

                // update status bar
                char str[256];
                sprintf(str, "Video Viewer (%ux%u) | %.1f FPS", input->GetWidth(), input->GetHeight(), output->GetFrameRate());
                output->SetStatus(str);    

                // check if the user quit
                if( !output->IsStreaming() )
                    break;
            }
        }

#if 0
        // Convert frame to ROS image message
        cv::Mat cv_image(input->GetHeight(), input->GetWidth(), CV_8UC3, image);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = "camera_link";
        msg->width = input->GetWidth();
        msg->height = input->GetHeight();
        msg->encoding = "bgr8";
        msg->is_bigendian = 0;
        if (msg->data.size() != msg->width * msg->height * 3) {
            ROS_ERROR("Image data size mismatch: expected %d, got %ld", msg->width * msg->height * 3, msg->data.size());
        }
        image_pub.publish(msg);

        rate.sleep();
#else
        // Convert frame to ROS image message
        cv::Mat cv_image(input->GetHeight(), input->GetWidth(), CV_8UC3, image);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
#if 1
        ros::Duration time_diff;
        { // image time mark
            std::lock_guard<std::mutex> lock(img_time_mutex);

            static ros::Duration min_popup_time_diff;
            static ros::Duration max_popup_time_diff;
            static ros::Time lastest_nopopup_time;

            if (!time_queue.empty()) { //popup
                no_popup_count = 1;
                switch(popup_state) {
                    case 0: //not initialized
                        msg->header.stamp = time_queue.front();
                        time_queue.pop_front();

                        lastest_nopopup_time = ros::Time(0);
                        max_popup_time_diff  = ros::Duration(0);
                        min_popup_time_diff  = ros::Duration(1e9);  // 1 billion seconds or another suitably large value

                        if (popup_count > 2) {
                            popup_state = 1;
                        }
                        break;
                    
                    case 1: //previously popup
                        msg->header.stamp = time_queue.front();
                        time_queue.pop_front();

                        time_diff = msg->header.stamp - previous_time;
                        if (time_diff < min_popup_time_diff) {
                            min_popup_time_diff = time_diff;
                        }

                        if (time_diff > max_popup_time_diff) {
                            max_popup_time_diff = time_diff;
                        }
                        break;

                    case 2: //previously no popup
                        msg->header.stamp = time_queue.front();
                        time_queue.pop_front();

                        popup_state = 1;
                        break;

                    default: //can't be here
                        break;
                }
                //ROS_INFO("popup time: %d.%d.%ld", msg->header.stamp.sec, msg->header.stamp.nsec, time_queue.size());
        } else { //no popup
                no_popup_count++;
                if (no_popup_count > max_no_popup_count) {
                    max_no_popup_count = no_popup_count;
                }
                switch(popup_state) {
                    case 0: //not initialized
                        msg->header.stamp = ros::Time(0);
                        lastest_nopopup_time = msg->header.stamp;

                        popup_count = 0;
                        break;
                    
                    case 1: //previously popup

#if 1
                        msg->header.stamp = lastest_time + max_popup_time_diff * (1.0 / max_no_popup_count);
#else
                        time_diff = ros::Time::now() - local_sys_time;
                        if (time_diff > max_popup_time_diff) {
                            msg->header.stamp = lastest_time + min_popup_time_diff;
                        } else {
                            msg->header.stamp = lastest_time + time_diff;
                        }
#endif
                        lastest_nopopup_time = msg->header.stamp;

                        popup_state = 2;
                        if (popup_count < 2){
                            popup_count = 0;
                        }
                        break;

                    case 2: //previously no popup
#if 1
                        msg->header.stamp = lastest_nopopup_time + max_popup_time_diff * (1.0 / max_no_popup_count);
#else
                        time_diff = ros::Time::now() - lastest_nopopup_time;
                        if (time_diff > max_popup_time_diff) {
                            msg->header.stamp = lastest_nopopup_time + min_popup_time_diff;
                        } else {
                            msg->header.stamp = lastest_nopopup_time + time_diff;
                        }
#endif
                        lastest_nopopup_time = msg->header.stamp;

                        if (popup_count < 2){
                            popup_count = 0;
                        }
                        break;

                    default: //can't be here
                        break;
                }
                //ROS_INFO("nopop time: %d.%d.%ld", msg->header.stamp.sec, msg->header.stamp.nsec, time_queue.size());
            }  
        } 
#else
        //msg->header.stamp = ros::Time::now() - ros::Duration(img_latency / 1000.0);
        msg->header.stamp = ros::Time::now();
#endif
        msg->header.frame_id = "world";
        msg->width = cv_image.cols;
        msg->height = cv_image.rows;
        msg->encoding = "bgr8";
        msg->is_bigendian = 0;
        msg->step = msg->width * 3;

        // Check if the size of the image data matches the expected size
        if (msg->data.size() != msg->height * msg->step) {
            ROS_ERROR("Image data size mismatch: expected %d, got %ld", msg->height * msg->step, msg->data.size());
        }

        { // frame validation compared with imu timeline
            static uint64_t last_frame_timestamp = 0;
            uint64_t curr_frame_timestamp = input->GetLastTimestamp();

            std::lock_guard<std::mutex> lock(imu_time_mutex);
            time_diff = imu_sys_time - msg->header.stamp;
            if (curr_frame_timestamp >  last_frame_timestamp  
                && time_diff < ros::Duration(0.1) 
                && time_diff > ros::Duration(0.0)) {
                image_pub.publish(msg);
                last_frame_timestamp = curr_frame_timestamp;
            } else {
                ROS_WARN("frame dropped curr %ld last %ld diff %f sec", curr_frame_timestamp, last_frame_timestamp, time_diff.toSec());
            }
        }
#endif
        ros::spinOnce(); // Handle ROS callbacks
    }


    /*
     * destroy resources
     */
    printf("video-viewer:  shutting down...\n");

    camera_print();
    
    SAFE_DELETE(input);

    if(debug_enable){
        SAFE_DELETE(output);
    }

    printf("video-viewer:  shutdown complete\n");
}