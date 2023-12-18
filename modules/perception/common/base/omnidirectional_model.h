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
#pragma once

#include <memory>
#include <string>

#include "Eigen/Core"

#include "modules/perception/common/base/camera.h"
#include "modules/perception/common/base/distortion_model.h"
#include "modules/perception/common/base/polynomial.h"

namespace apollo {
namespace perception {
namespace base {

class OmnidirectionalCameraDistortionModel : public BaseCameraDistortionModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  OmnidirectionalCameraDistortionModel() = default;
  ~OmnidirectionalCameraDistortionModel() = default;

  Eigen::Vector2f Project(const Eigen::Vector3f& point3d) override;

  std::shared_ptr<BaseCameraModel> get_camera_model() override;

  std::string name() const override {
    return "OmnidirectionalCameraDistortionModel";
  }

  bool set_params(size_t width, size_t height,
                  const Eigen::VectorXf& params) override;

 protected:
  Eigen::Matrix3f intrinsic_params_;
  Polynomial cam2world_;
  Polynomial world2cam_;
  float center_[2];  // x, y
  float affine_[3];  // c, d, e
};

}  // namespace base
}  // namespace perception
}  // namespace apollo
