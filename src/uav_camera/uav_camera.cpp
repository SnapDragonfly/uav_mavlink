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
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "videoSource.h"
#include "videoOutput.h"

#include "logging.h"
#include "commandLine.h"

#include <signal.h>

#include "config.h"

bool signal_recieved = false;

void sig_handler(int signo)
{
	if( signo == SIGINT )
	{
		LogInfo("received SIGINT\n");
		signal_recieved = true;
	}
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
    ros::init(argc, argv, PACKAGE_NAME);
    ros::NodeHandle nh;
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>(CAMERA_DEFAULT_IMAGE_TOPIC, 1);

    // Parse command line
	commandLine cmdLine(argc, argv);

	if( cmdLine.GetFlag("help") )
		return usage();


	/*
	 * attach signal handler
	 */	
	if( signal(SIGINT, sig_handler) == SIG_ERR )
		LogError("can't catch SIGINT\n");


	/*
	 * create input video stream
	 */
	videoSource* input = videoSource::Create(cmdLine, ARG_POSITION(0));

	if( !input )
	{
		LogError("video-viewer:  failed to create input stream\n");
		return 0;
	}


	/*
	 * create output video stream
	 */
	videoOutput* output = videoOutput::Create(cmdLine, ARG_POSITION(1));
	
	if( !output )
	{
		LogError("video-viewer:  failed to create output stream\n");
		return 0;
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
		
#if defined(CAMERA_CODE_DEBUG)
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
#endif /* CAMERA_CODE_DEBUG */

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
		msg->header.stamp = ros::Time::now();
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
	SAFE_DELETE(output);

	printf("video-viewer:  shutdown complete\n");
}