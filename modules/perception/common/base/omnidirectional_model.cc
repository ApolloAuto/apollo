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
#include "modules/perception/common/base/omnidirectional_model.h"

#include <limits>

#include "cyber/common/log.h"

#include "modules/perception/common/base/camera.h"
#include "modules/perception/common/base/polynomial.h"

namespace apollo {
namespace perception {
namespace base {

Eigen::Vector2f OmnidirectionalCameraDistortionModel::Project(
    const Eigen::Vector3f& point3d) {
  if (std::isgreater(point3d[2], 0.f)) {
    AERROR << "The input point (" << point3d
           << ") should be in front of the camera";
  }

  // rotate:
  // [0 1 0;
  //  1 0 0;
  //  0 0 -1];
  double x[3] = {point3d(1), point3d(0), -point3d(2)};
  const double norm = sqrt(x[0] * x[0] + x[1] * x[1]);

  Eigen::Vector2f projection;
  if (norm < std::numeric_limits<double>::epsilon()) {
    projection(0) = center_[1];
    projection(1) = center_[0];
    return projection;
  }

  const double theta = atan(x[2] / norm);
  const double rho = world2cam_(theta);

  const float u = static_cast<float>(x[0] / norm * rho);
  const float v = static_cast<float>(x[1] / norm * rho);
  projection(1) = affine_[0] * u + affine_[1] * v + center_[0];
  projection(0) = affine_[2] * u + v + center_[1];
  return projection;
}

std::shared_ptr<BaseCameraModel>
OmnidirectionalCameraDistortionModel::get_camera_model() {
  std::shared_ptr<PinholeCameraModel> camera_model(new PinholeCameraModel());
  camera_model->set_width(width_);
  camera_model->set_height(height_);
  camera_model->set_intrinsic_params(intrinsic_params_);

  return std::dynamic_pointer_cast<BaseCameraModel>(camera_model);
}

bool OmnidirectionalCameraDistortionModel::set_params(
    size_t width, size_t height, const Eigen::VectorXf& params) {
  if (params.size() < 9) {
    AINFO << "Missing cam2world and world2cam model.";
    return false;
  }

  uint32_t cam2world_order = uint32_t(params(8));
  AINFO << "cam2world order: " << cam2world_order << ", size: " << params.size()
        << std::endl;

  if (params.size() < 9 + cam2world_order + 1) {
    AINFO << "Incomplete cam2world model or missing world2cam model.";
    return false;
  }

  uint32_t world2cam_order = uint32_t(params(9 + cam2world_order));
  AINFO << "world2cam order: " << world2cam_order << ", size: " << params.size()
        << std::endl;

  if (params.size() < 9 + cam2world_order + 1 + world2cam_order) {
    AINFO << "Incomplete world2cam model.";
    return false;
  }

  width_ = width;
  height_ = height;

  center_[0] = params(0);
  center_[1] = params(1);

  affine_[0] = params(2);
  affine_[1] = params(3);
  affine_[2] = params(4);

  intrinsic_params_ = Eigen::Matrix3f::Identity();
  intrinsic_params_(0, 0) = params(5);
  intrinsic_params_(1, 1) = params(5);
  intrinsic_params_(0, 2) = params(6);
  intrinsic_params_(1, 2) = params(7);

  for (size_t i = 0; i < cam2world_order; ++i) {
    cam2world_[static_cast<uint32_t>(i)] = static_cast<double>(params(9 + i));
  }

  for (size_t i = 0; i < world2cam_order; ++i) {
    world2cam_[static_cast<uint32_t>(i)] =
        static_cast<double>(params(10 + cam2world_order + i));
  }

  return true;
}

}  // namespace base
}  // namespace perception
}  // namespace apollo
