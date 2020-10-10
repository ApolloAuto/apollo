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
#pragma once
namespace apollo {
namespace perception {
namespace camera {

// @description an angle in [-pi/2, pi/2)
class HalfCircleAngle {
 public:
  HalfCircleAngle() : theta_(10000) {}
  void SetDirection(float theta);
  float operator+(const float &theta) const;
  float operator*(const float &scale) const;
  HalfCircleAngle &operator=(const float &theta);
  HalfCircleAngle &operator=(const HalfCircleAngle &theta);
  bool operator==(const HalfCircleAngle &theta) const;
  bool operator==(const float &theta) const;
  float value() const;

 private:
  float theta_;
};
}  // namespace camera
}  // namespace perception
}  // namespace apollo
