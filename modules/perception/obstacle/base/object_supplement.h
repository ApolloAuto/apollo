/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#ifndef MODULES_PERCEPTION_OBSTACLE_BASE_OBJECT_SUPPLEMENT_H_
#define MODULES_PERCEPTION_OBSTACLE_BASE_OBJECT_SUPPLEMENT_H_

#include <Eigen/Core>
#include <memory>
#include "modules/perception/obstacle/base/types.h"

namespace apollo {
namespace perception {

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
  float radial_velocity = 0.0f;
};
typedef std::shared_ptr<RadarSupplement> RadarSupplementPtr;
typedef std::shared_ptr<const RadarSupplement> RadarSupplementConstPtr;

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_BASE_OBJECT_SUPPLEMENT_H_
