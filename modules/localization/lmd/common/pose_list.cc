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

#include "modules/localization/lmd/common/pose_list.h"

#include <cmath>

#include "modules/common/log.h"
#include "modules/common/math/quaternion.h"

namespace apollo {
namespace localization {

using apollo::common::math::QuaternionToHeading;

namespace {
template <class T>
T InterpolateXYZ(const T &p1, const T &p2, double frac1) {
  T p;
  double frac2 = 1.0 - frac1;
  if (p1.has_x() && !std::isnan(p1.x()) && p2.has_x() && !std::isnan(p2.x())) {
    p.set_x(p1.x() * frac2 + p2.x() * frac1);
  }
  if (p1.has_y() && !std::isnan(p1.y()) && p2.has_y() && !std::isnan(p2.y())) {
    p.set_y(p1.y() * frac2 + p2.y() * frac1);
  }
  if (p1.has_z() && !std::isnan(p1.z()) && p2.has_z() && !std::isnan(p2.z())) {
    p.set_z(p1.z() * frac2 + p2.z() * frac1);
  }
  return p;
}

template <class T>
T InterpolateQuaternion(const T &p1, const T &p2, double frac1) {
  T p;
  T p21 = p2;
  auto cosa = p1.qw() * p2.qw() + p1.qx() * p2.qx() + p1.qy() * p2.qy() +
              p1.qz() * p2.qz();
  if (cosa < 0.0) {
    p21.set_qw(-p2.qw());
    p21.set_qx(-p2.qx());
    p21.set_qy(-p2.qy());
    p21.set_qz(-p2.qz());
    cosa = -cosa;
  }

  double frac2;
  constexpr double kCriticalCosa = 0.9995;
  if (cosa > kCriticalCosa) {
    // if cosa near 1.0, user linear interpolation
    frac2 = 1.0 - frac1;
  } else {
    auto sina = std::sqrt(1.0 - cosa * cosa);
    auto a = std::atan2(sina, cosa);
    frac2 = std::sin((1.0 - frac1) * a) / sina;
    frac1 = std::sin(frac1 * a) / sina;
  }

  p.set_qw(p1.qw() * frac2 + p21.qw() * frac1);
  p.set_qx(p1.qx() * frac2 + p21.qx() * frac1);
  p.set_qy(p1.qy() * frac2 + p21.qy() * frac1);
  p.set_qz(p1.qz() * frac2 + p21.qz() * frac1);
  return p;
}
}  // namespace

bool PoseList::FindMatchingPose(double timestamp_sec, Pose *pose) const {
  CHECK_NOTNULL(pose);

  auto p = RangeOf(timestamp_sec);
  if (p.first == end() || p.second == end()) {
    return false;
  }

  InterpolatePose(p.first->first, p.first->second, p.second->first,
                  p.second->second, timestamp_sec, pose);
  return true;
}

bool PoseList::FindNearestPose(double timestamp_sec, Pose *pose) const {
  CHECK_NOTNULL(pose);

  if (!FindMatchingPose(timestamp_sec, pose)) {
    auto it = Nearest(timestamp_sec);
    if (it == end()) {
      return false;
    }
    pose->CopyFrom(it->second);
  }

  return true;
}

void PoseList::InterpolatePose(double timestamp_sec1, const Pose &pose1,
                               double timestamp_sec2, const Pose &pose2,
                               double timestamp_sec, Pose *pose) {
  CHECK_GE(timestamp_sec, timestamp_sec1);
  CHECK_GE(timestamp_sec2, timestamp_sec);
  CHECK_NOTNULL(pose);

  auto time_diff = timestamp_sec2 - timestamp_sec1;
  if (fabs(time_diff) > 0.0) {
    auto frac1 = (timestamp_sec - timestamp_sec1) / time_diff;

    if (pose1.has_position() && pose2.has_position()) {
      auto val = InterpolateXYZ(pose1.position(), pose2.position(), frac1);
      pose->mutable_position()->CopyFrom(val);
    }

    if (pose1.has_orientation() && pose2.has_orientation() &&
        pose1.orientation().has_qw() && pose1.orientation().has_qx() &&
        pose1.orientation().has_qy() && pose1.orientation().has_qz() &&
        pose2.orientation().has_qw() && pose2.orientation().has_qx() &&
        pose2.orientation().has_qy() && pose2.orientation().has_qz()) {
      auto val = InterpolateQuaternion(pose1.orientation(), pose2.orientation(),
                                       frac1);
      pose->mutable_orientation()->CopyFrom(val);
      pose->set_heading(QuaternionToHeading(
          pose->orientation().qw(), pose->orientation().qx(),
          pose->orientation().qy(), pose->orientation().qz()));
    }

    if (pose1.has_linear_velocity() && pose2.has_linear_velocity()) {
      auto val = InterpolateXYZ(pose1.linear_velocity(),
                                pose2.linear_velocity(), frac1);
      pose->mutable_linear_velocity()->CopyFrom(val);
    }

    if (pose1.has_angular_velocity() && pose2.has_angular_velocity()) {
      auto val = InterpolateXYZ(pose1.angular_velocity(),
                                pose2.angular_velocity(), frac1);
      pose->mutable_angular_velocity()->CopyFrom(val);
    }

    if (pose1.has_linear_acceleration() && pose2.has_linear_acceleration()) {
      auto val = InterpolateXYZ(pose1.linear_acceleration(),
                                pose2.linear_acceleration(), frac1);
      pose->mutable_linear_acceleration()->CopyFrom(val);
    }

    if (pose1.has_euler_angles() && pose2.has_euler_angles()) {
      auto val =
          InterpolateXYZ(pose1.euler_angles(), pose2.euler_angles(), frac1);
      pose->mutable_euler_angles()->CopyFrom(val);
    }
  } else {
    pose->CopyFrom(pose1);
  }
}

}  // namespace localization
}  // namespace apollo
