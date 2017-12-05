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
#include <array>
#include <map>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "gflags/gflags.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "ros/meta_stats.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"

#include "modules/perception/traffic_light_detection.pb.h"

#include "modules/perception/lib/base/perf.h"
#include "modules/perception/lib/base/time_util.h"
#include "modules/perception/onboard/stream_output.h"
#include "modules/perception/traffic_light/base/image_lights.h"

using apollo::perception::traffic_light::Image;
using apollo::perception::traffic_light::CameraId;

static std::map<std::string, cv::Scalar> s_color_table = {
    {std::string("red_light_box"), cv::Scalar(0, 0, 255)},
    {std::string("green_light_box"), cv::Scalar(0, 255, 0)},
    {std::string("yellow_light_box"), cv::Scalar(0, 255, 255)},
    {std::string("black_light_box"), cv::Scalar(255, 90, 199)},
    {std::string("unknown_light_box"), cv::Scalar(0, 76, 153)},
    {std::string("projection_roi"), cv::Scalar(255, 255, 0)},
    {std::string("crop_roi"), cv::Scalar(0, 255, 255)},
    {std::string("debug_roi"), cv::Scalar(255, 169, 255)}};

static std::map<std::string, CameraId> s_camera_id_map = {
    {std::string("222"), CameraId::LONG_FOCUS},
    {std::string("111"), CameraId::SHORT_FOCUS},
    {std::string("444"), CameraId::NARROW_FOCUS},
    {std::string("333"), CameraId::WIDE_FOCUS}};

static std::vector<std::shared_ptr<Image>> s_cached_images;
const int MAX_CACHED_IMAGES_NUM = 100;

static cv::Mat s_img(1080, 1920, CV_8UC3, cv::Scalar(128, 128, 128));
std::unique_ptr<apollo::perception::StreamOutput> g_output_stream;

void tl_debug_callback(const std_msgs::String::ConstPtr &msg);
void tl_image_long_callback(const sensor_msgs::ImageConstPtr &msg);
void tl_image_short_callback(const sensor_msgs::ImageConstPtr &msg);
void tl_image_narrow_callback(const sensor_msgs::ImageConstPtr &msg);
void tl_image_wide_callback(const sensor_msgs::ImageConstPtr &msg);

int main(int argc, char **argv) {
  ros::init(argc, argv, "traffic_light_viz_listener");
  ros::NodeHandle n;

  g_output_stream.reset(new apollo::perception::StreamOutput());
  if (g_output_stream == nullptr ||
      !g_output_stream->register_publisher<sensor_msgs::Image>(
          "sink_type=9&sink_name=/perception/traffic_light_debug")) {
    return -1;
  }

  cv::namedWindow("tl_debug_image", cv::WINDOW_NORMAL);
  cv::resizeWindow("tl_debug_image", 960, 540);
  cv::startWindowThread();

  ros::Subscriber sub_tl_debug =
      n.subscribe("/perception/traffic_light_status", 1000, tl_debug_callback);
  ros::Subscriber sub_tl_image_long = n.subscribe(
      "/sensor/camera/traffic/image_long", 1000, tl_image_long_callback);
  ros::Subscriber sub_tl_image_short = n.subscribe(
      "/sensor/camera/traffic/image_short", 1000, tl_image_short_callback);
  ros::Subscriber sub_tl_image_narrow = n.subscribe(
      "/sensor/camera/obstacle/image_narrow", 1000, tl_image_narrow_callback);
  ros::Subscriber sub_tl_image_wide = n.subscribe(
      "/sensor/camera/obstacle/image_wide", 1000, tl_image_wide_callback);

  ros::spin();
  cv::destroyAllWindows();

  return 0;
}

void tl_debug_callback(const std_msgs::String::ConstPtr &msg) {
  apollo::perception::TrafficLightDetection tl_result;
  tl_result.ParseFromString(msg->data);
  // ROS_INFO("I heard: [%s]", tl_result.ShortDebugString().c_str());

  auto img_ts = tl_result.header().camera_timestamp() / 1000000000.0;
  auto ts_str = std::to_string(tl_result.header().camera_timestamp());
  auto camera_ts_id = ts_str.substr(ts_str.length() - 3);
  auto camera_id = s_camera_id_map[camera_ts_id];

  bool found_image = false;
  for (int i = s_cached_images.size() - 1; i >= 0; --i) {
    if (fabs(s_cached_images[i]->ts() - img_ts) < 0.005 &&
        camera_id == s_cached_images[i]->camera_id()) {
      s_img = s_cached_images[i]->mat().clone();
      found_image = true;
      break;
    }
  }
  if (!found_image) {
    return;
  }

  auto tl_debug_msg = tl_result.traffic_light_debug();
  auto signals_num = tl_debug_msg.signal_num();
  auto box_size = tl_debug_msg.box_size();

  if (signals_num > 0 && tl_debug_msg.has_cropbox()) {
    // crop roi
    auto crop_box = tl_debug_msg.cropbox();
    cv::Rect cv_crop_rect(crop_box.x(), crop_box.y(), crop_box.width(),
                          crop_box.height());
    cv::rectangle(s_img, cv_crop_rect, s_color_table["crop_roi"], 2);

    // debug roi
    for (int box_idx = signals_num * 2; box_idx < box_size; ++box_idx) {
      auto debug_roi_box = tl_debug_msg.box(box_idx);
      cv::Rect cv_debug_roi_box(debug_roi_box.x(), debug_roi_box.y(),
                                debug_roi_box.width(), debug_roi_box.height());
      cv::rectangle(s_img, cv_debug_roi_box, s_color_table["debug_roi"], 2);
    }

    // projection roi
    for (int box_idx = signals_num; box_idx < signals_num * 2; ++box_idx) {
      auto projection_box = tl_debug_msg.box(box_idx);
      cv::Rect cv_projection_box(projection_box.x(), projection_box.y(),
                                 projection_box.width(),
                                 projection_box.height());
      cv::rectangle(s_img, cv_projection_box, s_color_table["projection_roi"],
                    2);
    }

    // rectified roi
    for (int box_idx = 0; box_idx < signals_num; ++box_idx) {
      auto rectified_box = tl_debug_msg.box(box_idx);
      cv::Rect cv_rectified_box(rectified_box.x(), rectified_box.y(),
                                rectified_box.width(), rectified_box.height());
      cv::Scalar color;
      switch (rectified_box.color()) {
        case apollo::perception::TrafficLight::RED:
          color = s_color_table["red_light_box"];
          break;
        case apollo::perception::TrafficLight::GREEN:
          color = s_color_table["green_light_box"];
          break;
        case apollo::perception::TrafficLight::BLACK:
          color = s_color_table["black_light_box"];
          break;
        case apollo::perception::TrafficLight::YELLOW:
          color = s_color_table["yellow_light_box"];
          break;
        default:
          color = s_color_table["unknown_light_box"];
          break;
      }

      cv::rectangle(s_img, cv_rectified_box, color, 2);
    }
  }

  // draw camera timestamp
  int pos_y = 40;
  std::string ts_text = cv::format("img ts=%lf", img_ts);
  cv::putText(s_img, ts_text, cv::Point(30, pos_y), cv::FONT_HERSHEY_PLAIN, 3.0,
              CV_RGB(128, 255, 0), 2);
  // draw distance to stopline
  pos_y += 50;
  double distance = tl_debug_msg.distance_to_stop_line();
  if (signals_num > 0) {
    std::string dis2sl_text = cv::format("dis2sl=%lf", distance);
    cv::putText(s_img, dis2sl_text, cv::Point(30, pos_y),
                cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(128, 255, 0), 2);
  }

  // draw "Signals Num"
  pos_y += 50;
  if (tl_debug_msg.valid_pos()) {
    std::string signal_txt = "Signals Num: " + std::to_string(signals_num);
    cv::putText(s_img, signal_txt, cv::Point(30, pos_y), cv::FONT_HERSHEY_PLAIN,
                3.0, CV_RGB(255, 0, 0), 2);
  }

  // draw "No Pose info."
  pos_y += 50;
  if (!tl_debug_msg.valid_pos()) {
    cv::putText(s_img, "No Valid Pose.", cv::Point(30, pos_y),
                cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(255, 0, 0), 2);

    // if image's timestamp is too early or too old
    // draw timestamp difference between image and pose
    pos_y += 50;
    std::string diff_img_pose_ts_str =
        "ts diff: " + std::to_string(tl_debug_msg.ts_diff_pos());
    cv::putText(s_img, diff_img_pose_ts_str, cv::Point(30, pos_y),
                cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(255, 0, 0), 2);

    pos_y += 50;
    std::string diff_img_sys_ts_str =
        "ts diff sys: " + std::to_string(tl_debug_msg.ts_diff_sys());
    cv::putText(s_img, diff_img_sys_ts_str, cv::Point(30, pos_y),
                cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(255, 0, 0), 2);
  }

  // draw image border size (offset between hdmap-box and detection-box)
  if (camera_ts_id == "222") {
    if (tl_debug_msg.project_error() > 100) {
      std::string img_border_txt =
          "Offset size: " + std::to_string(tl_debug_msg.project_error());
      constexpr int kPosYOffset = 1000;
      cv::putText(s_img, img_border_txt, cv::Point(30, kPosYOffset),
                  cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(255, 0, 0), 2);
    }
  }

  cv::resize(s_img, s_img, cv::Size(960, 540));
  cv::imshow("tl_debug_image", s_img);
  cv::waitKey(1);

  // publish image
  auto img_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", s_img).toImageMsg();
  if (!g_output_stream->publish<sensor_msgs::Image>(img_msg)) {
    std::cerr << "TLOutputSubnode publish debug image message failed. ";
    return;
  }
}

void tl_image_long_callback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    if (msg->encoding.compare("rgb8") == 0) {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } else if (msg->encoding.compare("8UC3") == 0) {
      cv_ptr = cv_bridge::toCvShare(msg, "8UC3");
    } else {
      std::cerr << "tl_visualizer get unknown image format. "
                << "format:" << msg->encoding;
      return;
    }
  } catch (const cv_bridge::Exception &e) {
    std::cerr << "tl_visualizer trans msg to image failed." << e.what();
    return;
  }

  // cv::Mat img = cv_ptr->image;
  // cv::imshow("tl_image_long", img);

  std::shared_ptr<Image> image(new Image);
  if (!image->Init(msg->header.stamp.toSec(), CameraId::LONG_FOCUS,
                   cv_ptr->image)) {
    std::cerr << "tl_visualizer load image failed.";
  }
  s_cached_images.push_back(image);

  while (s_cached_images.size() > MAX_CACHED_IMAGES_NUM) {
    s_cached_images.erase(s_cached_images.begin());
  }
}

void tl_image_short_callback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    if (msg->encoding.compare("rgb8") == 0) {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } else if (msg->encoding.compare("8UC3") == 0) {
      cv_ptr = cv_bridge::toCvShare(msg, "8UC3");
    } else {
      std::cerr << "tl_visualizer get unknown image format. "
                << "format:" << msg->encoding;
      return;
    }
  } catch (const cv_bridge::Exception &e) {
    std::cerr << "tl_visualizer trans msg to image failed." << e.what();
    return;
  }

  std::shared_ptr<Image> image(new Image);
  if (!image->Init(msg->header.stamp.toSec(), CameraId::SHORT_FOCUS,
                   cv_ptr->image)) {
    std::cerr << "tl_visualizer load image failed.";
  }
  s_cached_images.push_back(image);

  while (s_cached_images.size() > MAX_CACHED_IMAGES_NUM) {
    s_cached_images.erase(s_cached_images.begin());
  }
}

void tl_image_narrow_callback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    if (msg->encoding.compare("rgb8") == 0) {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } else if (msg->encoding.compare("8UC3") == 0) {
      cv_ptr = cv_bridge::toCvShare(msg, "8UC3");
    } else {
      std::cerr << "tl_visualizer get unknown image format. "
                << "format:" << msg->encoding;
      return;
    }
  } catch (const cv_bridge::Exception &e) {
    std::cerr << "tl_visualizer trans msg to image failed." << e.what();
    return;
  }

  std::shared_ptr<Image> image(new Image);
  if (!image->Init(msg->header.stamp.toSec(), CameraId::NARROW_FOCUS,
                   cv_ptr->image)) {
    std::cerr << "tl_visualizer load image failed.";
  }
  s_cached_images.push_back(image);

  while (s_cached_images.size() > MAX_CACHED_IMAGES_NUM) {
    s_cached_images.erase(s_cached_images.begin());
  }
}

void tl_image_wide_callback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    if (msg->encoding.compare("rgb8") == 0) {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } else if (msg->encoding.compare("8UC3") == 0) {
      cv_ptr = cv_bridge::toCvShare(msg, "8UC3");
    } else {
      std::cerr << "tl_visualizer get unknown image format. "
                << "format:" << msg->encoding;
      return;
    }
  } catch (const cv_bridge::Exception &e) {
    std::cerr << "tl_visualizer trans msg to image failed." << e.what();
    return;
  }

  std::shared_ptr<Image> image(new Image);
  if (!image->Init(msg->header.stamp.toSec(), CameraId::WIDE_FOCUS,
                   cv_ptr->image)) {
    std::cerr << "tl_visualizer load image failed.";
  }
  s_cached_images.push_back(image);

  while (s_cached_images.size() > MAX_CACHED_IMAGES_NUM) {
    s_cached_images.erase(s_cached_images.begin());
  }
}
