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

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"

#include "modules/common/log.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/base/types.h"
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
  bool Convert(std::vector<std::shared_ptr<VisualObject>> *objects) override;

  std::string Name() const override;

 private:
  bool LoadCameraIntrinsics(const std::string &file_path);

  bool ConvertSingle(const float &h, const float &w, const float &l,
                     const float &alpha_deg, const Eigen::Vector2f &upper_left,
                     const Eigen::Vector2f &lower_right, bool use_width,
                     float *distance, Eigen::Vector2f *mass_center_pixel);

  void Rotate(const float &alpha_deg,
              std::vector<Eigen::Vector3f> *corners) const;

  float SearchDistance(const int &pixel_length, const bool &use_width,
                       const Eigen::Matrix<float, 3, 1> &mass_center_v,
                       float close_d, float far_d);

  void SearchCenterDirection(
      const Eigen::Matrix<float, 2, 1> &box_center_pixel, const float &curr_d,
      Eigen::Matrix<float, 3, 1> *mass_center_v,
      Eigen::Matrix<float, 2, 1> *mass_center_pixel) const;

  Eigen::Matrix<float, 3, 1> MakeUnit(
      const Eigen::Matrix<float, 3, 1> &v) const;

  // Physical Size sanity check based on type
  void CheckSizeSanity(std::shared_ptr<VisualObject> obj) const;

  // Check truncation based on 2D box position
  void CheckTruncation(std::shared_ptr<VisualObject> obj,
                       Eigen::Matrix<float, 2, 1> *trunc_center_pixel) const;

  // Choose distance based on 2D box width or height
  float DecideDistance(const float &distance_h, const float &distance_w,
                       std::shared_ptr<VisualObject> obj) const;

  void DecideAngle(const Eigen::Vector3f &camera_ray,
                   std::shared_ptr<VisualObject> obj) const;

  void SetBoxProjection(std::shared_ptr<VisualObject> obj) const;

  CameraDistort<float> camera_model_;
  std::vector<Eigen::Vector3f> corners_;
  std::vector<Eigen::Vector2f> pixel_corners_;
  static const int kMaxDistanceSearchDepth_ = 10;
  static const int kMaxCenterDirectionSearchDepth_ = 5;

  DISALLOW_COPY_AND_ASSIGN(GeometryCameraConverter);
};

// Register plugin
REGISTER_CAMERA_CONVERTER(GeometryCameraConverter);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_CONVERTER_GEOMETRY_H_
