/******************************************************************************
* Copyright 2018 The Apollo Authors. All Rights Reserved.
*
* Licensed under the Apache License, Version 2.0 (the License);
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an AS IS BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*****************************************************************************/
#include "modules/perception/camera/tools/offline/visualizer.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

std::vector<cv::Scalar> colorlist = {
    cv::Scalar(0, 0, 255),
    cv::Scalar(0, 100, 255),
    cv::Scalar(0, 200, 255),
    cv::Scalar(100, 255, 255),
    cv::Scalar(200, 255, 255),
    cv::Scalar(255, 100, 255),
    cv::Scalar(255, 0, 255),
    cv::Scalar(255, 255, 100),
    cv::Scalar(255, 255, 0),
    cv::Scalar(255, 0, 100),
    cv::Scalar(255, 0, 0),
    cv::Scalar(0, 255, 0),
    cv::Scalar(100, 255, 100)};

bool Visualizer::Init(const std::vector<std::string> &camera_names,
                      TransformServer *tf_server) {
  tf_server_ = tf_server;
  CHECK(tf_server_ != nullptr);
  last_timestamp_ = 0;
  int width = 1920;
  int height = 1080;
  small_h_ = static_cast<int>(height * 0.7);
  small_w_ = static_cast<int>(width * 0.7);
  world_h_ = 2 * small_h_;
  wide_pixel_ = 800;
  m2pixel_ = 6;

  for (size_t i = 0; i < camera_names.size(); ++i) {
    camera_image_[camera_names[i]] =
        cv::Mat(small_h_, small_w_, CV_8UC3, cv::Scalar(0, 0, 0));
  }
  world_image_ = cv::Mat(world_h_, wide_pixel_, CV_8UC3, cv::Scalar(0, 0, 0));
  return true;
}

void Visualizer::SetDirectory(const std::string &path) {
  std::string command;
  command = "mkdir -p " + path;
  system(command.c_str());
  command = "rm " + path + "/*.jpg";
  system(command.c_str());
  path_ = path;
}

void Visualizer::Draw2Dand3D(const cv::Mat &img, const CameraFrame &frame) {
  cv::Mat image = img.clone();
  Eigen::Affine3d pose;
  if (!tf_server_->QueryPos(frame.timestamp, &pose)) {
    pose.setIdentity();
  }
  Eigen::Affine3d lidar2novatel;
  tf_server_->QueryTransform("velodyne64", "novatel", &lidar2novatel);
  Eigen::Affine3d lidar2world = pose * lidar2novatel;
  Eigen::Affine3d world2lidar = lidar2world.inverse();
  for (const auto &object : frame.tracked_objects) {
    base::RectF rect(object->camera_supplement.box);
    cv::Rect r(static_cast<int>(rect.x),
               static_cast<int>(rect.y),
               static_cast<int>(rect.width),
               static_cast<int>(rect.height));
    cv::rectangle(image, r, colorlist[object->track_id % colorlist.size()], 2);
    cv::putText(image,
                std::to_string(object->track_id),
                cv::Point(static_cast<int>(rect.x), static_cast<int>(rect.y)),
                cv::FONT_HERSHEY_DUPLEX,
                1,
                cv::Scalar(0, 0, 255),
                2);
    Eigen::Vector3d theta;
    theta << cos(object->theta), sin(object->theta), 0;
    theta = world2lidar.linear() * theta;
    float yaw = static_cast<float>(atan2(theta[1], theta[0]));
    Eigen::Matrix2d rotate;
    rotate << cos(yaw), -sin(yaw),
      sin(yaw), cos(yaw);

    Eigen::Vector3d pos;
    pos << object->center[0], object->center[1], object->center[2];
    pos = world2lidar * pos;
    Eigen::Vector2d pos_2d;
    pos_2d << pos[0], pos[1];
    Eigen::Vector3d v;
    v << object->velocity[0], object->velocity[1], object->velocity[2];
    v = world2lidar.linear() * v;
    Eigen::Vector2d v_2d;
    v_2d << v[0] + pos_2d[0], v[1] + pos_2d[1];
    Eigen::Vector2d p1;
    p1 << object->size[0] / 2, object->size[1] / 2;
    p1 = rotate * p1 + pos_2d;
    Eigen::Vector2d p2;
    p2 << -object->size[0] / 2, object->size[1] / 2;
    p2 = rotate * p2 + pos_2d;
    Eigen::Vector2d p3;
    p3 << -object->size[0] / 2, -object->size[1] / 2;
    p3 = rotate * p3 + pos_2d;
    Eigen::Vector2d p4;
    p4 << object->size[0] / 2, -object->size[1] / 2;
    p4 = rotate * p4 + pos_2d;

    cv::line(world_image_, world_point_to_bigimg(p1),
             world_point_to_bigimg(p2),
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(world_image_, world_point_to_bigimg(p2),
             world_point_to_bigimg(p3),
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(world_image_, world_point_to_bigimg(p3),
             world_point_to_bigimg(p4),
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(world_image_, world_point_to_bigimg(p4),
             world_point_to_bigimg(p1),
             colorlist[object->track_id % colorlist.size()], 2);
    cv::line(world_image_, world_point_to_bigimg(pos_2d),
             world_point_to_bigimg(v_2d),
             colorlist[object->track_id % colorlist.size()], 2);
  }
  last_timestamp_ = frame.timestamp;
  camera_image_[frame.data_provider->sensor_name()] = image;
  cv::resize(image, camera_image_[frame.data_provider->sensor_name()],
             cv::Size(small_w_, small_h_));
}

void Visualizer::ShowResult(const cv::Mat &img, const CameraFrame &frame) {
  cv::Mat image = img.clone();
  std::string camera_name = frame.data_provider->sensor_name();

  if (frame.timestamp - last_timestamp_ > 0.02) {
    cv::Mat bigimg(world_h_, small_w_ + wide_pixel_, CV_8UC3);
    camera_image_["front_6mm"].copyTo(bigimg(cv::Rect(0,
                                                            0,
                                                            small_w_,
                                                            small_h_)));
    camera_image_["front_12mm"].copyTo(bigimg(cv::Rect(0,
                                                          small_h_,
                                                          small_w_,
                                                          small_h_)));
    world_image_.copyTo(bigimg(cv::Rect(small_w_, 0, wide_pixel_, world_h_)));
    char path[1000];
    snprintf(path, sizeof(path), "%s/%06d.jpg", path_.c_str(), frame.frame_id);
    AINFO << path;
    cv::imwrite(path, bigimg);
    world_image_ = cv::Mat(world_h_, wide_pixel_, CV_8UC3, cv::Scalar(0, 0, 0));
    draw_range_circle();
  }

  cv::putText(image,
              "timestamp:" + std::to_string(frame.timestamp),
              cv::Point(10, 50),
              cv::FONT_HERSHEY_DUPLEX,
              1.3,
              cv::Scalar(0, 0, 255),
              3);
  cv::putText(image,
              "camera_name: " + camera_name,
              cv::Point(10, 100),
              cv::FONT_HERSHEY_DUPLEX,
              1.3,
              cv::Scalar(0, 0, 255),
              3);
  cv::putText(image,
              "frame id: " + std::to_string(frame.frame_id),
              cv::Point(10, 150),
              cv::FONT_HERSHEY_DUPLEX,
              1.3,
              cv::Scalar(0, 0, 255),
              3);
  Draw2Dand3D(image, frame);
}

void Visualizer::draw_range_circle() {
  cv::Scalar color(255, 100, 0);
  cv::circle(world_image_,
             cv::Point(wide_pixel_ / 2, world_h_),
             1 * m2pixel_,
             color,
             1);
  for (int i = 20; i < 300; i += 20) {
    cv::circle(world_image_,
               cv::Point(wide_pixel_ / 2, world_h_),
               i * m2pixel_,
               color,
               2);
  }
  for (int i = 50; i < 300; i += 50) {
    cv::putText(world_image_,
                std::to_string(i),
                cv::Point(wide_pixel_ / 2, world_h_ - i * m2pixel_),
                cv::FONT_HERSHEY_DUPLEX,
                1,
                cv::Scalar(0, 0, 255),
                2);
  }
}

cv::Point Visualizer::world_point_to_bigimg(const Eigen::Vector2d &p) {
  cv::Point point;
  point.x = static_cast<int>(-p[1] * m2pixel_ + wide_pixel_ / 2);
  point.y = static_cast<int>(world_h_ - p[0] * m2pixel_);
  return point;
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
