/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/camera/lib/obstacle/tracker/common/half_circle_angle.h"

#include "modules/perception/camera/common/util.h"

namespace apollo {
namespace perception {
namespace camera {
void HalfCircleAngle::SetDirection(float theta) {
  if (theta_ > M_PI) {
    theta_ = theta;
  }
  if (std::abs(theta_ - theta) > M_PI_2) {
    if (theta_ > 0) {
      theta_ -= static_cast<float>(M_PI);
    } else {
      theta_ += static_cast<float>(M_PI);
    }
  }
}

HalfCircleAngle &HalfCircleAngle::operator=(const float &theta) {
  theta_ = theta;
  return *this;
}
float HalfCircleAngle::operator+(const float &theta) const {
  return theta_ + theta;
}
float HalfCircleAngle::operator*(const float &scale) const {
  return theta_ * scale;
}
HalfCircleAngle &HalfCircleAngle::operator=(const HalfCircleAngle &theta) {
  this->theta_ = theta.value();
  return *this;
}
bool HalfCircleAngle::operator==(const HalfCircleAngle &theta) const {
  return Equal(theta_, theta.value(), 0.01f);
}
bool HalfCircleAngle::operator==(const float &theta) const {
  return Equal(theta_, theta, 0.01f);
}
float HalfCircleAngle::value() const { return theta_; }
}  // namespace camera
}  // namespace perception
}  // namespace apollo
