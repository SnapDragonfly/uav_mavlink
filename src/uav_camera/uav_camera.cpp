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
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
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

bool signal_recieved   = false;
videoOutput* output    = NULL;

bool debug_enable      = true;
int  img_latency       = 100;
std::string img_source = CAMERA_DEFAULT_IMAGE_SOURCE;
std::string img_topic  = CAMERA_DEFAULT_IMAGE_TOPIC;
std::string tim_topic  = CAMERA_DEFAULT_TIME_TOPIC;
char params[CAMERA_ARGC_LEN][CAMERA_ARGV_LEN];
char* params_ptr[CAMERA_ARGC_LEN];
ros::Time current_time = ros::Time(0);
ros::Time last_time = ros::Time(0);

void sig_handler(int signo)
{
    if( signo == SIGINT )
    {
        LogInfo("received SIGINT\n");
        signal_recieved = true;
    }
}

void imgMsgCallback(const std_msgs::Header::ConstPtr& msg)
{
    if(msg->stamp > current_time) {
        last_time = current_time;
        current_time = msg->stamp;
    }

    //printf("Received img_msg:\n");
    //printf("Stamp: %d.%d\n", img_time.sec, img_time.nsec);
}

void config_print(const char* title){
    printf("%s ------------->\n", title);

    printf("img latency: %d\n", img_latency);
    printf(" img source: %s\n",img_source.c_str());
    printf("  img topic: %s\n",img_topic.c_str());
    printf(" time topic: %s\n",tim_topic.c_str());

    printf("debug: %s\n", debug_enable?"true":"false");

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

        img_latency  = config[TAG_IMG_LATENCY].as<int>();
        img_topic    = config[TAG_IMG_TOPIC].as<std::string>();
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
#else
        // Convert frame to ROS image message
        cv::Mat cv_image(input->GetHeight(), input->GetWidth(), CV_8UC3, image);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
#if 1
        msg->header.stamp = last_time;
#else
        msg->header.stamp = ros::Time::now() - ros::Duration(img_latency / 1000.0);
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
#endif
        // Publish the image message
        image_pub.publish(msg);

        ros::spinOnce(); // Handle ROS callbacks
    }


    /*
     * destroy resources
     */
    printf("video-viewer:  shutting down...\n");
    
    SAFE_DELETE(input);

    if(debug_enable){
        SAFE_DELETE(output);
    }

    printf("video-viewer:  shutdown complete\n");
}