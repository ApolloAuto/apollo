
#ifndef CYBERTRON_TF2_UTILS_H_
#define CYBERTRON_TF2_UTILS_H_

#include <cybertron/tf2/LinearMath/Transform.h>
#include <cybertron/tf2/LinearMath/Quaternion.h>
#include <cybertron/tf2/impl/utils.h>

namespace apollo {
namespace cybertron {
namespace tf2 {
/** Return the yaw, pitch, roll of anything that can be converted to a
 * tf2::Quaternion
 * The conventions are the usual ROS ones defined in tf2/LineMath/Matrix3x3.h
 * \param a the object to get data from (it represents a rotation/quaternion)
 * \param yaw yaw
 * \param pitch pitch
 * \param roll roll
 */
template <typename A>
void getEulerYPR(const A& a, double& yaw, double& pitch, double& roll) {
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
template <typename A>
double getYaw(const A& a) {
  tf2::Quaternion q = impl::toQuaternion(a);
  return impl::getYaw(q);
}

/** Return the identity for any type that can be converted to a tf2::Transform
 * \return an object of class A that is an identity transform
 */
template <typename A>
A getTransformIdentity() {
  tf2::Transform t;
  t.setIdentity();
  A a;
  convert(t, a);
  return a;
}
}
}
}

#endif  // INCLUDE_CYBERTRON_TF2_UTILS_H_
