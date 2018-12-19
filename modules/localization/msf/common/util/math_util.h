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

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {
namespace msf {

struct math {
  static void EulerToQuaternion(const double x_euler, const double y_euler,
                                const double z_euler, double qbn[4]) {
    qbn[0] = sin(x_euler / 2.0) * sin(y_euler / 2.0) * sin(z_euler / 2.0) +
             cos(x_euler / 2.0) * cos(y_euler / 2.0) * cos(z_euler / 2.0);
    qbn[1] = cos(y_euler / 2.0) * cos(z_euler / 2.0) * sin(x_euler / 2.0) +
             cos(x_euler / 2.0) * sin(y_euler / 2.0) * sin(z_euler / 2.0);
    qbn[2] = cos(x_euler / 2.0) * cos(z_euler / 2.0) * sin(y_euler / 2.0) -
             cos(y_euler / 2.0) * sin(x_euler / 2.0) * sin(z_euler / 2.0);
    qbn[3] = cos(z_euler / 2.0) * sin(x_euler / 2.0) * sin(y_euler / 2.0) -
             cos(x_euler / 2.0) * cos(y_euler / 2.0) * sin(z_euler / 2.0);

    return;
  }

  static void QuaternionToDcm(const double *quaternion, double dcm[3][3]) {
    dcm[0][0] = quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] -
                quaternion[2] * quaternion[2] - quaternion[3] * quaternion[3];
    dcm[0][1] =
        2 * (quaternion[1] * quaternion[2] - quaternion[0] * quaternion[3]);
    dcm[0][2] =
        2 * (quaternion[1] * quaternion[3] + quaternion[0] * quaternion[2]);

    dcm[1][0] =
        2 * (quaternion[1] * quaternion[2] + quaternion[0] * quaternion[3]);
    dcm[1][1] = quaternion[0] * quaternion[0] - quaternion[1] * quaternion[1] +
                quaternion[2] * quaternion[2] - quaternion[3] * quaternion[3];
    dcm[1][2] =
        2 * (quaternion[2] * quaternion[3] - quaternion[0] * quaternion[1]);

    dcm[2][0] =
        2 * (quaternion[1] * quaternion[3] - quaternion[0] * quaternion[2]);
    dcm[2][1] =
        2 * (quaternion[2] * quaternion[3] + quaternion[0] * quaternion[1]);
    dcm[2][2] = quaternion[0] * quaternion[0] - quaternion[1] * quaternion[1] -
                quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3];
    return;
  }
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
