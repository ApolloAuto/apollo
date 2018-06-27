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

/* Transform objects in 3D camera space into 3D ego-car space
 *
 * Two assumptions are used for this module
 *
 *  1. The ego-car space is on flat ground. 3D objects touches the ground
 * (Z direction of this local space is always vertical to ground plane,
 * regardless of ego car pose)
 *
 * 2. The input 3D distances for objects, from camera origin to object center,
 *    is accurate and in unit meter
 */

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_TRANSFORMER_FLAT_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_TRANSFORMER_FLAT_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"

#include "modules/common/log.h"
#include "modules/perception/obstacle/camera/common/camera.h"
#include "modules/perception/obstacle/camera/common/visual_object.h"
#include "modules/perception/obstacle/camera/interface/base_camera_transformer.h"

namespace apollo {
namespace perception {

class FlatCameraTransformer : public BaseCameraTransformer {
 public:
  FlatCameraTransformer() : BaseCameraTransformer() {}

  virtual ~FlatCameraTransformer() = default;

  bool Init() override;

  bool Transform(std::vector<std::shared_ptr<VisualObject>> *objects) override;

  // @brief Set static extrinsic matrix for camera space to car space
  bool SetExtrinsics(const Eigen::Matrix<double, 4, 4> &extrinsics) override;

  // @brief Get Adjusted camera-to-car extrinsics
  // Return false if no valid objects in scene
  bool GetAdjustedExtrinsics(Eigen::Matrix<double, 4, 4>* extrinsics) override;

  std::string Name() const override;

 private:
  // Static Extrinsics for transforming camera space to ego car space
  Eigen::Matrix<float, 4, 4> camera2car_;
  Eigen::Matrix<float, 3, 1> camera2car_flat_offset_;

  // Pitch-angle Adjusted Extrinsics
  bool adjust_pitch_ = false;
  Eigen::Matrix<float, 4, 4> camera2car_adj_;
  float pitch_diff_ = 0.0f;  // degree

  Eigen::Matrix<float, 3, 1> MakeUnit(
      const Eigen::Matrix<float, 3, 1> &v) const;

  bool HaveHighConfidence(std::shared_ptr<VisualObject> obj_ptr);

  // Real-time (Per-frame) Pitch angle adjustment between camera and ground
  // Assume objects are on ground, and ground is a simple flat plane
  void GetDynamicExtrinsics(
    const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &centers);

  DISALLOW_COPY_AND_ASSIGN(FlatCameraTransformer);
};

// Register plugin
REGISTER_CAMERA_TRANSFORMER(FlatCameraTransformer);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_TRANSFORMER_FLAT_H_
