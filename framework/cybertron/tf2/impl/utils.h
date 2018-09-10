
#ifndef CYBERTRON_TF2_IMPL_UTILS_H_
#define CYBERTRON_TF2_IMPL_UTILS_H_

#include <cybertron/tf2/transform_datatypes.h>
#include <cybertron/tf2/LinearMath/Quaternion.h>

#include "cybertron/proto/common_geomery.pb.h"

namespace apollo {
namespace cybertron {
namespace tf2 {
namespace impl {

/** Function needed for the generalization of toQuaternion
 * \param q a tf2::Quaternion
 * \return a copy of the same quaternion
 */
inline tf2::Quaternion toQuaternion(const tf2::Quaternion& q) { return q; }

/** Function needed for the generalization of toQuaternion
 * \param q a cybertron::proto::Quaternion
 * \return a copy of the same quaternion as a tf2::Quaternion
 */
inline tf2::Quaternion toQuaternion(const ::adu::common::Quaternion& q) {
  tf2::Quaternion res;
  fromMsg(q, res);
  return res;
}

/** Function needed for the generalization of toQuaternion
 * \param q a ::adu::common::TransformStamped
 * \return a copy of the same quaternion as a tf2::Quaternion
 */
inline tf2::Quaternion toQuaternion(const ::adu::common::TransformStamped& q) {
  tf2::Quaternion res;
  fromMsg(q.quaternion, res);
  return res;
}

/** Function needed for the generalization of toQuaternion
 * \param t some tf2::Stamped object
 * \return a copy of the same quaternion as a tf2::Quaternion
 */
template <typename T>
tf2::Quaternion toQuaternion(const tf2::Stamped<T>& t) {
  ::adu::common::TransformStamped q =
      toMsg<tf2::Stamped<T>, ::adu::common::TransformStamped>(t);
  return toQuaternion(q);
}

/** Generic version of toQuaternion. It tries to convert the argument
 * to a cybertron::proto::Quaternion
 * \param t some object
 * \return a copy of the same quaternion as a tf2::Quaternion
 */
template <typename T>
tf2::Quaternion toQuaternion(const T& t) {
  ::adu::common::Quaternion q = toMsg<T, ::adu::common::TransformStamped>(t);
  return toQuaternion(q);
}

/** The code below is blantantly copied from urdfdom_headers
 * only the normalization has been added.
 * It computes the Euler roll, pitch yaw from a tf2::Quaternion
 * It is equivalent to tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
 * \param q a tf2::Quaternion
 * \param yaw the computed yaw
 * \param pitch the computed pitch
 * \param roll the computed roll
 */
inline void getEulerYPR(const tf2::Quaternion& q, double& yaw, double& pitch,
                        double& roll) {
  double sqw;
  double sqx;
  double sqy;
  double sqz;

  sqx = q.x() * q.x();
  sqy = q.y() * q.y();
  sqz = q.z() * q.z();
  sqw = q.w() * q.w();

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  double sarg =
      -2 * (q.x() * q.z() - q.w() * q.y()) /
      (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */
  if (sarg <= -0.99999) {
    pitch = -0.5 * M_PI;
    roll = 0;
    yaw = -2 * atan2(q.y(), q.x());
  } else if (sarg >= 0.99999) {
    pitch = 0.5 * M_PI;
    roll = 0;
    yaw = 2 * atan2(q.y(), q.x());
  } else {
    pitch = asin(sarg);
    roll = atan2(2 * (q.y() * q.z() + q.w() * q.x()), sqw - sqx - sqy + sqz);
    yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), sqw + sqx - sqy - sqz);
  }
};

/** The code below is a simplified version of getEulerRPY that only
 * returns the yaw. It is mostly useful in navigation where only yaw
 * matters
 * \param q a tf2::Quaternion
 * \return the computed yaw
 */
inline double getYaw(const tf2::Quaternion& q) {
  double yaw;

  double sqw;
  double sqx;
  double sqy;
  double sqz;

  sqx = q.x() * q.x();
  sqy = q.y() * q.y();
  sqz = q.z() * q.z();
  sqw = q.w() * q.w();

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  double sarg =
      -2 * (q.x() * q.z() - q.w() * q.y()) /
      (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */

  if (sarg <= -0.99999) {
    yaw = -2 * atan2(q.y(), q.x());
  } else if (sarg >= 0.99999) {
    yaw = 2 * atan2(q.y(), q.x());
  } else {
    yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), sqw + sqx - sqy - sqz);
  }
  return yaw;
};
}
}
}
}

#endif  // INCLUDE_CYBERTRON_TF2_IMPL_UTILS_H_
