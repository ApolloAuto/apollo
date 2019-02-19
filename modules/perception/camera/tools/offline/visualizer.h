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
#pragma once

#include <map>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/tools/offline/transform_server.h"

namespace apollo {
namespace perception {
namespace camera {

class Visualizer {
 public:
  bool Init(const std::vector<std::string> &camera_names,
            TransformServer *tf_server);
  bool Init_all_info_single_camera(
      const std::string &camera_name,
      std::map<std::string, Eigen::Matrix3f> intrinsic_map,
      std::map<std::string, Eigen::Matrix4d> extrinsic_map,
      Eigen::Matrix4d ex_lidar2imu, double pitch_adj, int image_height,
      int image_width);
  void SetDirectory(const std::string &path);
  void ShowResult(const cv::Mat &img, const CameraFrame &frame);
  void Draw2Dand3D(const cv::Mat &img, const CameraFrame &frame);
  void ShowResult_all_info_single_camera(const cv::Mat &img,
                                         const CameraFrame &frame);
  void Draw2Dand3D_all_info_single_camera(const cv::Mat &img,
                                          const CameraFrame &frame,
                                          Eigen::Matrix3d intrinsic,
                                          Eigen::Matrix4d extrinsic);
  cv::Point world_point_to_bigimg(const Eigen::Vector2d &p);
  Eigen::Vector2d image2ground(cv::Point p_img);
  std::string type_to_string(const apollo::perception::base::ObjectType type);
  std::string sub_type_to_string(
      const apollo::perception::base::ObjectSubType type);

  bool write_out_img_ = false;

 private:
  std::map<std::string, cv::Mat> camera_image_;
  cv::Mat world_image_;
  TransformServer *tf_server_;
  std::string path_;
  double last_timestamp_ = 0.0;
  int image_width_ = 1920;
  int image_height_ = 1080;
  int wide_pixel_ = 800;
  double scale_ratio_ = 0.6;
  int small_h_ = 0;
  int small_w_ = 0;
  int world_h_ = 0;
  int m2pixel_ = 6;
  double fov_cut_ratio_ = 0.6;
  cv::Point p_fov_1_;
  cv::Point p_fov_2_;
  cv::Point p_fov_3_;
  cv::Point p_fov_4_;

  void draw_range_circle();

  // map for store params
  std::map<std::string, Eigen::Matrix3f> intrinsic_map_;
  std::map<std::string, Eigen::Matrix4d> extrinsic_map_;

  // homograph between image and ground plane
  Eigen::Matrix3d homography_im2ground_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
