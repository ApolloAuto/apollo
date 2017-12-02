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
#include "modules/perception/traffic_light/base/pose.h"

namespace apollo {
namespace perception {
namespace traffic_light {

bool CarPose::set_pose(const Eigen::Matrix4d &pose) {
  _pose = pose;
  return true;
}

const Eigen::Matrix4d CarPose::pose() const {
  return _pose;
}

std::ostream &operator<<(std::ostream &os, const CarPose &pose) {
  os << pose._pose;
  return os;
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
