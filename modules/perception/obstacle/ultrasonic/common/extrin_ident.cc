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

#include "modules/perception/obstacle/ultrasonic/common/extrin_ident.h"

namespace apollo {
namespace perception {

static const float kMaxRange = 4.0;

ExtrinIdent::ExtrinIdent(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    const int ultrasonic_id,
    const float cone_fov) :
  ultrasonic_id_(ultrasonic_id),
  cone_fov_(cone_fov),
  position_(position),
  orientation_(orientation) {
  transform_ = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
  transform_.prerotate(orientation);
  transform_.pretranslate(position);

  for (float angle = -(cone_fov_ / 2.0); angle < (cone_fov_ / 2.0);
       angle += 10.0) {
    float angle_arc = angle * M_PI / 180.0;
    Eigen::Vector3d upoint(cos(angle_arc), sin(angle_arc), 0.0);
    upoint_arc_.push_back(upoint);
  }

  for (size_t i = 0; i < upoint_arc_.size(); ++i) {
    Eigen::Vector3d localpoint = kMaxRange * upoint_arc_.at(i);
    Eigen::Vector3d fov_margin_point = transform_ * localpoint;
    fov_margin_.push_back(fov_margin_point);
  }
}

bool ExtrinIdent::operator < (const ExtrinIdent& second) {
  return ultrasonic_id_ < second.ultrasonic_id_;
}

void ExtrinIdent::operator = (const ExtrinIdent& second) {
  transform_ = second.transform_;
  position_ = second.position_;
  orientation_ = second.orientation_;
  ultrasonic_id_ = second.ultrasonic_id_;
  cone_fov_ = second.cone_fov_;
  upoint_arc_ = second.upoint_arc_;
  fov_margin_ = second.fov_margin_;
}

}  // namespace perception
}  // namespace apollo
