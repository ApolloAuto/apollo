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
#include <string>
#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h" 
#include "std_msgs/UInt8.h" 
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "camera_px2.h"
#include "ProgramArguments.hpp"

bool is_saving = false;
void callback(const std_msgs::Int16::ConstPtr& msg) {
    is_saving = (msg->data == 2 || msg->data == 4);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Please specify saving folder." << std::endl;
        return 1;
    }
    ros::init(argc, argv, "front_camera");
    ros::NodeHandle n;
    ros::Rate rate(30);
    
    int ori_width = 1920;
    int ori_height = 1208;
    ProgramArguments g_arguments(
    {
        ProgramArguments::Option_t("camera-type", "ar0231"),
        ProgramArguments::Option_t("csi-port", "ab"),
        ProgramArguments::Option_t("offscreen", "0"),
        ProgramArguments::Option_t("write-file", ""),
        ProgramArguments::Option_t("serialize-type", "h264"),
        ProgramArguments::Option_t("serialize-bitrate", "8000000"),
        ProgramArguments::Option_t("serialize-framerate", "30"),
        ProgramArguments::Option_t("slave", "0"),
    });
    init(ori_width, ori_height, g_arguments);
    start();

    ros::Publisher image_publisher;
    image_publisher = n.advertise<sensor_msgs::Image>("/car_msgs/front_image", 10);
    std_msgs::Bool save_st;
    ros::Publisher saving_image_pub = n.advertise<std_msgs::Bool>("/data/saving_front_image", 10); 
    ros::Subscriber saving_sub = n.subscribe("/data/saving", 1, &callback);
    
    while (ros::ok()) {
        // cur frame
        int size = ori_width * ori_height * 4;
        unsigned char* image_data = 0;
        bool ret = read_frame(&image_data);
        if (!ret) {
          ROS_ERROR_STREAM("read frame error");
          continue;
        }
        auto cur_time = ros::Time::now();

        // convert data to image
        cv_bridge::CvImage image;
        image.header.stamp = cur_time;
        cv::Mat src = cv::Mat(ori_height, ori_width, CV_8UC4, image_data);
        cv::resize(src, image.image, cv::Size(960, 604));
        image.encoding = "rgba8";
        image_publisher.publish(image.toImageMsg());
        
        if (is_saving) {
          //is_saving = false;  // reset
          std::string path= argv[1] + std::to_string(cur_time.toSec()) + ".jpg";
          cv::imwrite(path, image.image);
          std::cout << "saved: " << path << std::endl; 
          save_st.data = true;
        } else {
          save_st.data = false;
        } 
        saving_image_pub.publish(save_st);
        
        ros::spinOnce();
        reset_frame();
        rate.sleep();
    }
    
    stop();
    release();
    return 0;
}
