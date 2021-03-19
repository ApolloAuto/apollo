/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "Eigen/Core"

namespace apollo {
namespace perception {
namespace benchmark {

struct alignas(16) LidarSupplement {
  LidarSupplement();
  ~LidarSupplement();
  LidarSupplement(const LidarSupplement& rhs);
  LidarSupplement& operator=(const LidarSupplement& rhs);
  void clone(const LidarSupplement& rhs);
};

typedef std::shared_ptr<LidarSupplement> LidarSupplementPtr;
typedef std::shared_ptr<const LidarSupplement> LidarSupplementConstPtr;

struct alignas(16) RadarSupplement {
  RadarSupplement();
  ~RadarSupplement();
  RadarSupplement(const RadarSupplement& rhs);
  RadarSupplement& operator=(const RadarSupplement& rhs);
  void clone(const RadarSupplement& rhs);

  // distance
  float range = 0.0f;
  // x -> forward, y -> left
  float angle = 0.0f;
  float relative_radial_velocity = 0.0f;
  float relative_tangential_velocity = 0.0f;
};

typedef std::shared_ptr<RadarSupplement> RadarSupplementPtr;
typedef std::shared_ptr<const RadarSupplement> RadarSupplementConstPtr;

struct alignas(16) CameraSupplement {
  CameraSupplement();
  ~CameraSupplement();
  CameraSupplement(const CameraSupplement& rhs);
  CameraSupplement& operator=(const CameraSupplement& rhs);
  void clone(const CameraSupplement& rhs);

  // upper-left corner: x1, y1
  Eigen::Vector2d upper_left;
  // lower-right corner: x2, y2
  Eigen::Vector2d lower_right;
};

typedef std::shared_ptr<CameraSupplement> CameraSupplementPtr;
typedef std::shared_ptr<const CameraSupplement> CameraSupplementConstPtr;

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
