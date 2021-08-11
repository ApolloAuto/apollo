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

namespace apollo {
namespace perception {
namespace base {

class BaseCameraModel {
 public:
  virtual ~BaseCameraModel() = default;

  virtual Eigen::Vector2f Project(const Eigen::Vector3f& point3d) = 0;
  virtual Eigen::Vector3f UnProject(const Eigen::Vector2f& point2d) = 0;
  virtual std::string name() const = 0;

  inline void set_width(size_t width) { image_width_ = width; }
  inline void set_height(size_t height) { image_height_ = height; }

  inline size_t get_width() const { return image_width_; }
  inline size_t get_height() const { return image_height_; }

 protected:
  size_t image_width_ = 0;
  size_t image_height_ = 0;
};

class PinholeCameraModel : public BaseCameraModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  ~PinholeCameraModel() = default;

  Eigen::Vector2f Project(const Eigen::Vector3f& point3d) override;
  Eigen::Vector3f UnProject(const Eigen::Vector2f& point2d) override;
  std::string name() const override { return "PinholeCameraModel"; }

  inline void set_intrinsic_params(const Eigen::Matrix3f& params) {
    intrinsic_params_ = params;
  }

  inline Eigen::Matrix3f get_intrinsic_params() const {
    return intrinsic_params_;
  }

 protected:
  /*     fx  0   cx
         0   fy  cy
         0    0  1
  */
  Eigen::Matrix3f intrinsic_params_;
};

// TODO(all) remove later
typedef std::shared_ptr<BaseCameraModel> BaseCameraModelPtr;
typedef std::shared_ptr<const BaseCameraModel> BaseCameraModelConstPtr;
typedef std::shared_ptr<PinholeCameraModel> PinholeCameraModelPtr;
typedef std::shared_ptr<const PinholeCameraModel> PinholeCameraModelConstPtr;

}  // namespace base
}  // namespace perception
}  // namespace apollo
