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

#define TAG_IMG_SOURCE          "img_source"
#define TAG_IMG_TOPIC           "img_topic"
#define TAG_UAV_IMU             "imu_topic"
#define TAG_TIM_TOPIC           "tim_topic"
#define TAG_IMG_DEBUG           "enable_debug"
#define TAG_UDP_SPLIT           "enable_split"

bool signal_recieved   = false;
videoOutput* output    = NULL;

bool enable_debug      = true;
bool enable_split      = true;

std::string img_source = CAMERA_DEFAULT_IMAGE_SOURCE;
std::string img_topic  = CAMERA_DEFAULT_IMAGE_TOPIC;
std::string tim_topic  = CAMERA_DEFAULT_TIME_TOPIC;
std::string imu_topic  = MAVLINK_DEFAULT_IMU_TOPIC;

char params[CAMERA_ARGC_LEN][CAMERA_ARGV_LEN];
char* params_ptr[CAMERA_ARGC_LEN];

void sig_handler(int signo)
{
    if( signo == SIGINT )
    {
        LogInfo("received SIGINT\n");
        signal_recieved = true;
    }
}

std::mutex img_time_mutex;
ros::Time img_lastest_time    = ros::Time(0);
ros::Time img_previous_time   = ros::Time(0);
ros::Time img_local_time      = ros::Time(0);

std::deque<std_msgs::Header> time_queue;
const size_t MAX_QUEUE_SIZE = 120;

void imgMsgCallback(const std_msgs::Header::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(img_time_mutex);

    // Check if the queue already contains an entry with the same seq
    for (const auto& header : time_queue) {
        if (header.seq == msg->seq) {
            // If we have the same seq, we can skip adding to the queue
            return;
        }
    }

    img_previous_time = img_lastest_time;     // Store the previous timestamp
    img_lastest_time  = msg->stamp;           // Update the latest timestamp
    img_local_time    = ros::Time::now();     // Get the current system time

    // Add the new header to the queue
    time_queue.push_back(*msg);

    // If the queue exceeds the maximum size, remove the oldest entry
    if (time_queue.size() > MAX_QUEUE_SIZE) {
        time_queue.pop_front();
    }
}

std_msgs::Header popupHeaderBySeq(uint32_t seq)
{
    std::lock_guard<std::mutex> lock(img_time_mutex);

    // If seq is 0, return the oldest header
    if (seq == 0 && !time_queue.empty()) {
        std_msgs::Header header = time_queue.front(); // Get the oldest header
        time_queue.pop_front(); // Remove the oldest header from the queue
        return header; // Return the popped header
    }

    // Otherwise, search for the header by seq
    for (auto it = time_queue.begin(); it != time_queue.end(); ++it) {
        if (it->seq == seq) {
            std_msgs::Header header = *it; // Store the header to return it
            time_queue.erase(it);          // Remove the header from the queue
            return header;                 // Return the popped header
        }
    }

    // Return an empty header if the seq is not found
    return std_msgs::Header();
}

std::mutex imu_time_mutex;
ros::Time imu_latest_time    = ros::Time(0);

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    std::lock_guard<std::mutex> lock(imu_time_mutex);
    imu_latest_time = imu_msg->header.stamp;
}

void config_print(const char* title){
    printf("%s ------------->\n", title);

    printf(" img source: %s\n",img_source.c_str());
    printf("  img topic: %s\n",img_topic.c_str());
    printf("  imu topic: %s\n",imu_topic.c_str());
    printf(" time topic: %s\n",tim_topic.c_str());

    printf("split: %s\n", enable_split?"true":"false");
    printf("debug: %s\n", enable_debug?"true":"false");

    printf("%s <-------------\n", title);
}

int config_read(int argc, char** argv){
    try {   
        // Get the directory path of a package
        std::filesystem::path package_path = ros::package::getPath(PACKAGE_NAME);
        std::filesystem::path config_path = package_path / PACKAGE_CONFIG;
        printf("Config file path: %s\n", config_path.c_str());

        // Load the YAML file
        YAML::Node config = YAML::LoadFile(config_path.c_str());

        img_source   = config[TAG_IMG_SOURCE].as<std::string>();
        img_topic    = config[TAG_IMG_TOPIC].as<std::string>();
        imu_topic    = config[TAG_UAV_IMU].as<std::string>();
        tim_topic    = config[TAG_TIM_TOPIC].as<std::string>();

        enable_split = config[TAG_UDP_SPLIT].as<bool>();
        enable_debug = config[TAG_IMG_DEBUG].as<bool>();


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

bool isHeaderEmpty(const std_msgs::Header& header)
{
    return (header.seq == 0 && header.stamp.isZero() && header.frame_id.empty());
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
    if (enable_split) {
        ros::Subscriber image_sub = nh.subscribe(tim_topic.c_str(), 1, imgMsgCallback);
        ros::Subscriber imu_sub = nh.subscribe(imu_topic.c_str(), 1, imuCallback);
    }
    /*
     * create input video stream
     */
    videoSource* input = videoSource::Create(cmdLine, ARG_POSITION(0));

    if( !input )
    {
        LogError("video-viewer:  failed to create input stream\n");
        return 0;
    }

    if(enable_debug){
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
        
        if (enable_debug) {
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

        // Convert frame to ROS image message
        if (enable_split) {
            cv::Mat cv_image(input->GetHeight(), input->GetWidth(), CV_8UC3, image);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
#if 0
            msg->header.stamp = ros::Time::now();
#else
            std_msgs::Header img_msg = popupHeaderBySeq(0);
            if (isHeaderEmpty(img_msg)) {
                msg->header.stamp = img_lastest_time;
            } else {
                msg->header.stamp = img_msg.stamp;
            }

            if (msg->header.stamp > imu_latest_time) {
                ROS_ERROR("img(%d.%d) comes after imu(%d.%d)", 
                           img_msg.stamp.sec, img_msg.stamp.nsec, imu_latest_time.sec, imu_latest_time.nsec);
            }
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

            image_pub.publish(msg);

        } else {
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
        }

        ros::spinOnce(); // Handle ROS callbacks
    }


    /*
     * destroy resources
     */
    printf("video-viewer:  shutting down...\n");
    
    SAFE_DELETE(input);

    if(enable_debug){
        SAFE_DELETE(output);
    }

    printf("video-viewer:  shutdown complete\n");
}