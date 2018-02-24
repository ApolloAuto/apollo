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

// Convert 2D detections into 3D objects with physical position in camera space

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_CONVERTER_GEOMETRY_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_CONVERTER_GEOMETRY_H_

#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/camera/common/camera.h"
#include "modules/perception/obstacle/camera/common/visual_object.h"
#include "modules/perception/obstacle/camera/interface/base_camera_converter.h"

namespace apollo {
namespace perception {

class GeometryCameraConverter : public BaseCameraConverter {
 public:
  GeometryCameraConverter() : BaseCameraConverter() {}

  virtual ~GeometryCameraConverter() {}

  bool Init() override;

  // @brief: Convert 2D detected objects into physical 3D objects
  // @param [in/out] objects : detected object lists, added 3D position and
  // orientation
  bool Convert(std::vector<VisualObjectPtr> *objects) override;

  void SetDebug(bool flag);

  std::string Name() const override;

 private:
  bool LoadCameraIntrinsics(const std::string &file_path);

  bool ConvertSingle(const float &h, const float &w, const float &l,
                     const float &alpha_deg, const Eigen::Vector2f &upper_left,
                     const Eigen::Vector2f &lower_right, float *distance_w,
                     float *distance_h, Eigen::Vector2f *mass_center_pixel);

  void Rotate(const float &alpha_deg,
              std::vector<Eigen::Vector3f> *corners) const;

  float SearchDistance(const int &pixel_length, const bool &use_width,
                       const Eigen::Matrix<float, 3, 1> &mass_center_v) const;

  void SearchCenterDirection(
      const Eigen::Matrix<float, 2, 1> &box_center_pixel, const float &curr_d,
      Eigen::Matrix<float, 3, 1> *mass_center_v,
      Eigen::Matrix<float, 2, 1> *mass_center_pixel) const;

  Eigen::Matrix<float, 3, 1> MakeUnit(
      const Eigen::Matrix<float, 3, 1> &v) const;

  Camera<float> camera_model_;
  std::vector<Eigen::Vector3f> corners_;
  static const int kMaxDistanceSearchDepth_ = 20;
  static const int kMaxCenterDirectionSearchDepth_ = 10;
  bool debug_ = false;

  DISALLOW_COPY_AND_ASSIGN(GeometryCameraConverter);
};

// Register plugin
REGISTER_CAMERA_CONVERTER(GeometryCameraConverter);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_CONVERTER_GEOMETRY_H_
