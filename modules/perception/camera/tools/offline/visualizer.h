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
#include <opencv2/opencv.hpp>
#include <map>
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
  void SetDirectory(const std::string &path);
  void ShowResult(const cv::Mat &img, const CameraFrame &frame);
  void Draw2Dand3D(const cv::Mat &img, const CameraFrame &frame);
 private:
  std::map<std::string, cv::Mat> camera_image_;
  cv::Mat world_image_;
  TransformServer *tf_server_;
  std::string path_;
  double last_timestamp_ = 0.0;
  int wide_pixel_ = 0;
  int small_h_ = 0;
  int small_w_ = 0;
  int world_h_ = 0;
  int m2pixel_ = 0;
  void draw_range_circle();
  cv::Point world_point_to_bigimg(const Eigen::Vector2d &p);
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
