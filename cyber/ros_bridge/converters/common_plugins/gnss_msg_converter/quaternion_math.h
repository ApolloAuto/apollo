/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include <cmath>

namespace apollo {
namespace cyber {

inline void QuaternionToEuler(const double quaternion[4], double att[3]) {
  double dcm21 =
      2 * (quaternion[2] * quaternion[3] + quaternion[0] * quaternion[1]);
  double dcm20 =
      2 * (quaternion[1] * quaternion[3] - quaternion[0] * quaternion[2]);
  double dcm22 = quaternion[0] * quaternion[0] - quaternion[1] * quaternion[1] -
                 quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3];
  double dcm01 =
      2 * (quaternion[1] * quaternion[2] - quaternion[0] * quaternion[3]);
  double dcm11 = quaternion[0] * quaternion[0] - quaternion[1] * quaternion[1] +
                 quaternion[2] * quaternion[2] - quaternion[3] * quaternion[3];

  att[0] = std::asin(dcm21);           // the angle rotate respect to X
  att[1] = std::atan2(-dcm20, dcm22);  // the angle rotate respect to Y
  att[2] = std::atan2(dcm01, dcm11);   // the angle rotate respect to Z

  return;
}

inline double QuaternionToHeading(double yaw) {
  double a = std::fmod(yaw + M_PI_2 + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

}  // namespace cyber
}  // namespace apollo
