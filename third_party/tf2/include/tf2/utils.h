// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TF2_UTILS_H
#define TF2_UTILS_H

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>

namespace tf2 {

/** Return the yaw, pitch, roll of anything that can be converted to a tf2::Quaternion
 * The conventions are the usual ROS ones defined in tf2/LineMath/Matrix3x3.h
 * \param a the object to get data from (it represents a rotation/quaternion)
 * \param yaw yaw
 * \param pitch pitch
 * \param roll roll
 */
template <class A>
  void getEulerYPR(const A& a, double& yaw, double& pitch, double& roll)
  {
    tf2::Quaternion q = impl::toQuaternion(a);
    impl::getEulerYPR(q, yaw, pitch, roll);
  }

/** Return the yaw of anything that can be converted to a tf2::Quaternion
 * The conventions are the usual ROS ones defined in tf2/LineMath/Matrix3x3.h
 * This function is a specialization of getEulerYPR and is useful for its
 * wide-spread use in navigation
 * \param a the object to get data from (it represents a rotation/quaternion)
 * \param yaw yaw
 */
template <class A>
  double getYaw(const A& a)
  {
    tf2::Quaternion q = impl::toQuaternion(a);
    return impl::getYaw(q);
  }

/** Return the identity for any type that can be converted to a tf2::Transform
 * \return an object of class A that is an identity transform
 */
template <class A>
  A getTransformIdentity()
  {
    tf2::Transform t;
    t.setIdentity();
    A a;
    convert(t, a);
    return a;
  }

}

#endif //TF2_UTILS_H
