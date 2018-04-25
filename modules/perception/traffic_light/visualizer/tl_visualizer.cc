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
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/log.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/perception/traffic_light/base/image_lights.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

using apollo::perception::traffic_light::Image;
using apollo::perception::traffic_light::CameraId;
using apollo::perception::TrafficLightDetection;

std::unordered_map<std::string, cv::Scalar> kColorTable = {
    {std::string("red_light_box"), cv::Scalar(0, 0, 255)},
    {std::string("green_light_box"), cv::Scalar(0, 255, 0)},
    {std::string("yellow_light_box"), cv::Scalar(0, 255, 255)},
    {std::string("black_light_box"), cv::Scalar(255, 90, 199)},
    {std::string("unknown_light_box"), cv::Scalar(0, 76, 153)},
    {std::string("projection_roi"), cv::Scalar(255, 255, 0)},
    {std::string("crop_roi"), cv::Scalar(0, 255, 255)},
    {std::string("debug_roi"), cv::Scalar(255, 169, 255)}};

std::vector<std::shared_ptr<Image>> cached_images;
const int kMaxCachedImageNum = 100;

void SubDebugCallback(const TrafficLightDetection &);
void SubLongFocusCallback(const sensor_msgs::ImagePtr &);
void SubShortFocusCallback(const sensor_msgs::ImagePtr &);

int main(int argc, char **argv) {
  ros::init(argc, argv, "traffic_light_viz_listener");
  ros::NodeHandle n;

  ros::Subscriber sub_tl_debug =
      n.subscribe(FLAGS_traffic_light_detection_topic, 1000, SubDebugCallback);
  ros::Subscriber sub_tl_image_long =
      n.subscribe(FLAGS_image_long_topic, 1000, SubLongFocusCallback);
  ros::Subscriber sub_tl_image_short =
      n.subscribe(FLAGS_image_short_topic, 1000, SubShortFocusCallback);

  ros::spin();
  return 0;
}

void SubDebugCallback(const TrafficLightDetection &tl_result) {
  auto img_ts = tl_result.header().camera_timestamp() / 1e9;
  auto tl_debug_msg = tl_result.traffic_light_debug();
  auto signals_num = tl_debug_msg.signal_num();
  auto box_size = tl_debug_msg.box_size();
  auto camera_id = tl_debug_msg.camera_id();
  cv::Mat img;
  bool found_image = false;
  for (int i = cached_images.size() - 1; i >= 0; --i) {
    if (fabs(cached_images[i]->ts() - img_ts) < 0.005 &&
        camera_id == cached_images[i]->camera_id()) {
      cached_images[i]->GenerateMat();
      img = cached_images[i]->mat();
      found_image = true;
      break;
    }
  }
  if (!found_image) {
    return;
  }

  if (signals_num > 0 && tl_debug_msg.has_cropbox()) {
    // crop roi
    auto crop_box = tl_debug_msg.cropbox();
    cv::Rect cv_crop_rect(crop_box.x(), crop_box.y(), crop_box.width(),
                          crop_box.height());
    cv::rectangle(img, cv_crop_rect, kColorTable["crop_roi"], 2);

    // debug roi
    for (int box_idx = signals_num * 2; box_idx < box_size; ++box_idx) {
      auto debug_roi_box = tl_debug_msg.box(box_idx);
      cv::Rect cv_debug_roi_box(debug_roi_box.x(), debug_roi_box.y(),
                                debug_roi_box.width(), debug_roi_box.height());
      cv::rectangle(img, cv_debug_roi_box, kColorTable["debug_roi"], 2);
    }

    // projection roi
    for (int box_idx = signals_num; box_idx < signals_num * 2; ++box_idx) {
      auto projection_box = tl_debug_msg.box(box_idx);
      cv::Rect cv_projection_box(projection_box.x(), projection_box.y(),
                                 projection_box.width(),
                                 projection_box.height());
      cv::rectangle(img, cv_projection_box, kColorTable["projection_roi"], 2);
    }

    // rectified roi
    for (int box_idx = 0; box_idx < signals_num; ++box_idx) {
      auto rectified_box = tl_debug_msg.box(box_idx);
      cv::Rect cv_rectified_box(rectified_box.x(), rectified_box.y(),
                                rectified_box.width(), rectified_box.height());
      cv::Scalar color;

      using apollo::perception::TrafficLight;
      switch (rectified_box.color()) {
        case TrafficLight::RED:
          color = kColorTable["red_light_box"];
          break;
        case TrafficLight::GREEN:
          color = kColorTable["green_light_box"];
          break;
        case TrafficLight::BLACK:
          color = kColorTable["black_light_box"];
          break;
        case TrafficLight::YELLOW:
          color = kColorTable["yellow_light_box"];
          break;
        default:
          color = kColorTable["unknown_light_box"];
          break;
      }

      cv::rectangle(img, cv_rectified_box, color, 2);
    }
  }

  // draw camera timestamp
  int pos_y = 40;
  std::string ts_text = cv::format("img ts=%lf", img_ts);
  cv::putText(img, ts_text, cv::Point(30, pos_y), cv::FONT_HERSHEY_PLAIN, 3.0,
              CV_RGB(128, 255, 0), 2);
  // draw distance to stopline
  pos_y += 50;
  double distance = tl_debug_msg.distance_to_stop_line();
  if (signals_num > 0) {
    std::string dis2sl_text = cv::format("dis2sl=%lf", distance);
    cv::putText(img, dis2sl_text, cv::Point(30, pos_y), cv::FONT_HERSHEY_PLAIN,
                3.0, CV_RGB(128, 255, 0), 2);
  }

  // draw "Signals Num"
  pos_y += 50;
  if (tl_debug_msg.valid_pos()) {
    std::string signal_txt = "Signals Num: " + std::to_string(signals_num);
    cv::putText(img, signal_txt, cv::Point(30, pos_y), cv::FONT_HERSHEY_PLAIN,
                3.0, CV_RGB(255, 0, 0), 2);
  }

  // draw "No Pose info."
  pos_y += 50;
  if (!tl_debug_msg.valid_pos()) {
    cv::putText(img, "No Valid Pose.", cv::Point(30, pos_y),
                cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(255, 0, 0), 2);

    // if image's timestamp is too early or too old
    // draw timestamp difference between image and pose
    pos_y += 50;
    std::string diff_img_pose_ts_str =
        "ts diff: " + std::to_string(tl_debug_msg.ts_diff_pos());
    cv::putText(img, diff_img_pose_ts_str, cv::Point(30, pos_y),
                cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(255, 0, 0), 2);

    pos_y += 50;
    std::string diff_img_sys_ts_str =
        "ts diff sys: " + std::to_string(tl_debug_msg.ts_diff_sys());
    cv::putText(img, diff_img_sys_ts_str, cv::Point(30, pos_y),
                cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(255, 0, 0), 2);
  }
  pos_y += 50;
  {
    std::string signal_txt =
        "camera id: " +
        apollo::perception::traffic_light::kCameraIdToStr.at(
            tl_debug_msg.camera_id());
    cv::putText(img, signal_txt, cv::Point(30, pos_y), cv::FONT_HERSHEY_PLAIN,
                3.0, CV_RGB(255, 0, 0), 2);
  }
  // draw image border size (offset between hdmap-box and detection-box)
  if (tl_debug_msg.project_error() > 100) {
    std::string img_border_txt =
        "Offset size: " + std::to_string(tl_debug_msg.project_error());
    constexpr int kPosYOffset = 1000;
    cv::putText(img, img_border_txt, cv::Point(30, kPosYOffset),
                cv::FONT_HERSHEY_PLAIN, 3.0, CV_RGB(255, 0, 0), 2);
  }

  cv::resize(img, img, cv::Size(960, 540));
  cv::imshow("tl_debug_image", img);
  cv::waitKey(10);
}
void SubImage(CameraId camera_id, const sensor_msgs::ImagePtr &msg) {
  boost::shared_ptr<sensor_msgs::Image> img(new sensor_msgs::Image);
  *img = *msg;
  boost::shared_ptr<const sensor_msgs::Image> img_msg(img);
  std::shared_ptr<Image> image(new Image);
  if (!image->Init(img_msg->header.stamp.toSec(), camera_id, img_msg)) {
    std::cerr << "tl_visualizer load image failed.";
  }
  cached_images.push_back(image);

  while (cached_images.size() > kMaxCachedImageNum) {
    cached_images.erase(cached_images.begin());
  }
}
void SubLongFocusCallback(const sensor_msgs::ImagePtr &msg) {
  SubImage(CameraId::LONG_FOCUS, msg);
}

void SubShortFocusCallback(const sensor_msgs::ImagePtr &msg) {
  SubImage(CameraId::SHORT_FOCUS, msg);
}
