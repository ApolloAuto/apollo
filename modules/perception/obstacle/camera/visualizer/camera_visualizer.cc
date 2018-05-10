/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/log.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/traffic_light/base/image_lights.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

using apollo::perception::PerceptionObstacles;
using apollo::perception::traffic_light::CameraId;
using apollo::perception::traffic_light::Image;

std::map<std::string, cv::Scalar> kColorTable = {
    {std::string("red_light_box"), cv::Scalar(0, 0, 255)},
    {std::string("green_light_box"), cv::Scalar(0, 255, 0)},
    {std::string("yellow_light_box"), cv::Scalar(0, 255, 255)},
    {std::string("black_light_box"), cv::Scalar(255, 90, 199)},
    {std::string("unknown_light_box"), cv::Scalar(0, 76, 153)},
    {std::string("projection_roi"), cv::Scalar(255, 255, 0)},
    {std::string("crop_roi"), cv::Scalar(0, 255, 255)},
    {std::string("debug_roi"), cv::Scalar(255, 169, 255)}};

std::vector<std::shared_ptr<Image>> g_cached_images;
const int kMaxCachedImageNum = 10;

void OnPerception(const PerceptionObstacles &);
void OnImageShort(const sensor_msgs::ImagePtr &);

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_visualizer");
  ros::NodeHandle n;

  ros::Subscriber sub_perception_debug =
      n.subscribe(FLAGS_perception_obstacle_topic, 1000, OnPerception);
  ros::Subscriber sub_tl_image_short =
      n.subscribe(FLAGS_image_short_topic, 1000, OnImageShort);

  ros::spin();
  return 0;
}

void OnPerception(const PerceptionObstacles &obstacles) {
  // TODO(all): add debug into perception debug pb and draw on image.
  if (g_cached_images.empty()) {
    return;
  }
  g_cached_images.back()->GenerateMat();
  cv::Mat img = g_cached_images.back()->mat();

  cv::resize(img, img, cv::Size(960, 540));
  cv::imshow("camera_debug_image", img);
  cv::waitKey(10);
}

void OnImage(CameraId camera_id, const sensor_msgs::ImagePtr &msg) {
  boost::shared_ptr<sensor_msgs::Image> img(new sensor_msgs::Image);
  *img = *msg;
  boost::shared_ptr<const sensor_msgs::Image> img_msg(img);
  std::shared_ptr<Image> image(new Image);
  if (!image->Init(img_msg->header.stamp.toSec(), camera_id, img_msg)) {
    std::cerr << "camera_visualizer load image failed.";
  }
  g_cached_images.push_back(image);

  while (g_cached_images.size() > kMaxCachedImageNum) {
    g_cached_images.erase(g_cached_images.begin());
  }
}

void OnImageShort(const sensor_msgs::ImagePtr &msg) {
  OnImage(CameraId::SHORT_FOCUS, msg);
}
