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

/**
 * @file
 * @brief Defines the EulerAnglesZXY class.
 */

#ifndef MODULES_COMMON_MATH_EULER_ANGLES_ZXY_H_
#define MODULES_COMMON_MATH_EULER_ANGLES_ZXY_H_

// TODO(all): should use Angle class internally.

#include <cmath>

#include "Eigen/Geometry"

#include "modules/common/math/math_utils.h"

/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace apollo {
namespace common {
namespace math {

/**
 * @class EulerAnglesZXY
 *
 * Any orientation of a rigid body on a 3-D space can be achieved by
 * composing three rotations about the axes of an orthogonal coordinate system.
 * These rotations are said to be extrinsic if the axes are assumed to be
 * motionless, and intrinsic otherwise. Here, we use an intrinsic referential,
 * which is relative to the car's orientation.
 * Our vehicle reference frame follows NovAtel's convention:
 * Right/Forward/Up (RFU) respectively for the axes x/y/z.
 * In particular, we describe the orientation of the car by three angles:
 * 1) the pitch, in (-pi/2, pi/2), corresponds to a rotation around the x-axis;
 * 2) the roll, in [-pi, pi), corresponds to a rotation around the y-axis;
 * 3) the yaw, in [-pi, pi), corresponds to a rotation around the z-axis.
 * The pitch is zero when the car is level and positive when the nose is up.
 * The roll is zero when the car is level and positive when the left part is up.
 * The yaw is zero when the car is facing North, and positive when facing West.
 * In turn, in the world frame, the x/y/z axes point to East/North/Up (ENU).
 * These angles represent the rotation from the world to the vehicle frames.
 *
 * @brief Implements a class of Euler angles (actually, Tait-Bryan angles),
 * with intrinsic sequence ZXY.
 *
 * @param T Number type: double or float
 */
template <typename T>
class EulerAnglesZXY {
 public:
  /**
   * @brief Constructs an identity rotation.
   */
  EulerAnglesZXY() : roll_(0), pitch_(0), yaw_(0) {}

  /**
    * @brief Constructs a rotation using only yaw (i.e., around the z-axis).
    *
    * @param yaw The yaw of the car
    * @param pitch The pitch of the car
    * @param roll The roll of the car
    */
  explicit EulerAnglesZXY(T yaw) : roll_(0), pitch_(0), yaw_(yaw) {}

  /**
   * @brief Constructs a rotation using arbitrary roll, pitch, and yaw.
   *
   * @param roll The roll of the car
   * @param pitch The pitch of the car
   * @param yaw The yaw of the car
  */
  EulerAnglesZXY(T roll, T pitch, T yaw)
      : roll_(roll), pitch_(pitch), yaw_(yaw) {}

  /**
   * @brief Constructs a rotation using components of a quaternion.
   *
   * @param qw Quaternion w-coordinate
   * @param qx Quaternion x-coordinate
   * @param qy Quaternion y-coordinate
   * @param qz Quaternion z-coordinate
   */
  EulerAnglesZXY(T qw, T qx, T qy, T qz)
      : roll_(std::atan2(2.0 * (qw * qy - qx * qz),
                         2.0 * (Square<T>(qw) + Square<T>(qz)) - 1.0)),
        pitch_(std::asin(2.0 * (qw * qx + qy * qz))),
        yaw_(std::atan2(2.0 * (qw * qz - qx * qy),
                        2.0 * (Square<T>(qw) + Square<T>(qy)) - 1.0)) {}

  /**
   * @brief Constructs a rotation from quaternion.
   * @param q Quaternion
   */
  explicit EulerAnglesZXY(const Eigen::Quaternion<T> &q)
      : EulerAnglesZXY(q.w(), q.x(), q.y(), q.z()) {}

  /**
   * @brief Getter for roll_
   * @return The roll of the car
   */
  T roll() const { return roll_; }

  /**
   * @brief Getter for pitch_
   * @return The pitch of the car
   */
  T pitch() const { return pitch_; }

  /**
   * @brief Getter for yaw_
   * @return The yaw of the car
   */
  T yaw() const { return yaw_; }

  /**
   * @brief Normalizes roll_, pitch_, and yaw_ to [-PI, PI).
   */
  void Normalize() {
    roll_ = NormalizeAngle(roll_);
    pitch_ = NormalizeAngle(pitch_);
    yaw_ = NormalizeAngle(yaw_);
  }

  /**
   * @brief Verifies the validity of the specified rotation.
   * @return True iff -PI/2 < pitch < PI/2
   */
  bool IsValid() {
    Normalize();
    return pitch_ < M_PI_2 && pitch_ > -M_PI_2;
  }

  /**
   * @brief Converts to a quaternion with a non-negative scalar part
   * @return Quaternion encoding this rotation.
   */
  Eigen::Quaternion<T> ToQuaternion() const {
    T r = roll_ * 0.5;
    T p = pitch_ * 0.5;
    T y = yaw_ * 0.5;

    T sr = std::sin(r);
    T sp = std::sin(p);
    T sy = std::sin(y);

    T cr = std::cos(r);
    T cp = std::cos(p);
    T cy = std::cos(y);

    T qw = cr * cp * cy - sr * sp * sy;
    T qx = cr * sp * cy - sr * cp * sy;
    T qy = cr * sp * sy + sr * cp * cy;
    T qz = cr * cp * sy + sr * sp * cy;
    if (qw < 0.0) return {-qw, -qx, -qy, -qz};
    return {qw, qx, qy, qz};
  }

 private:
  T roll_;
  T pitch_;
  T yaw_;
};

using EulerAnglesZXYf = EulerAnglesZXY<float>;
using EulerAnglesZXYd = EulerAnglesZXY<double>;

}  // namespace math
}  // namespace common
}  // namespace apollo

#endif /* MODULES_COMMON_MATH_EULER_ANGLES_ZXY_H_ */
