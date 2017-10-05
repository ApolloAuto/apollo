/******************************************************************************
  * Copyright 2017 The Apollo Authors. All Rights Reserved.
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  * http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
//#include <df_nav_msgs/ImageCompressed.h>
#include "camera_px2.h"

#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "px2_camera_image");
    ros::NodeHandle n;
   
    // px2 camera initialize
    int screen_shot_count = 0;
    int image_width = 1920;
    int image_height = 1208;
    init(image_width, image_height);
    start();
   
    // define image compress topic
    ros::Publisher image_publisher;
    image_publisher = n.advertise<sensor_msgs::Image>("/car_msgs/image", 10);
    ROS_INFO("initialize px2 image topic...");
    //ros::Rate loop_rate(20);
    
    int i = 0;
    while (ros::ok())
    {

        // get image from px2 camera
        int size = image_width * image_height * 4;
        unsigned char* image_data = 0; 
        bool ret = read_frame(&image_data);

        if (!ret){
            ROS_ERROR_STREAM("read frame error");
            continue;
        }
        
       
        //if (i % 4 == 0) {
        // convert to jpg
        //cv::Mat image = cv::Mat(image_height, image_width, CV_8UC4, image_data);

        cv_bridge::CvImage image;
        image.header.stamp = ros::Time::now();
        cv::Mat src = cv::Mat(image_height, image_width, CV_8UC4, image_data);
        cv::resize(src, image.image, cv::Size(960, 604));
        image.encoding = "rgba8";
        image_publisher.publish(image.toImageMsg());

        ++i;
        reset_frame();
        //ros::spinOnce();
        //loop_rate.sleep();
    }


  //  ros::spin();
    stop();
    release();

    return 0;
}
