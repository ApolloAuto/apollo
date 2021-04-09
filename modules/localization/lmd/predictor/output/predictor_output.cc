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

#include "modules/localization/lmd/predictor/output/predictor_output.h"

#include <cmath>

#include "modules/common/proto/geometry.pb.h"

#include "modules/common/log.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using apollo::common::Point3D;
using apollo::common::PointENU;
using apollo::common::Quaternion;
using apollo::common::Status;
using apollo::common::math::EulerAnglesZXYd;
using apollo::common::math::QuaternionRotate;
using apollo::common::math::QuaternionToHeading;

namespace {
template <class T>
T QuaternionRotateXYZ(const T v, const Quaternion& orientation) {
  auto vec =
      QuaternionRotate(orientation, Eigen::Vector3d(v.x(), v.y(), v.z()));
  T ret;
  ret.set_x(vec[0]);
  ret.set_y(vec[1]);
  ret.set_z(vec[2]);
  return ret;
}

void FillPoseFromImu(const Pose& imu_pose, Pose* pose) {
  // linear acceleration
  if (imu_pose.has_linear_acceleration()) {
    if (FLAGS_enable_map_reference_unify) {
      if (pose->has_orientation()) {
        pose->mutable_linear_acceleration()->CopyFrom(QuaternionRotateXYZ(
            imu_pose.linear_acceleration(), pose->orientation()));
        pose->mutable_linear_acceleration_vrf()->CopyFrom(
            imu_pose.linear_acceleration());
      } else {
        AERROR << "Fail to convert linear_acceleration";
      }
    } else {
      pose->mutable_linear_acceleration()->CopyFrom(
          imu_pose.linear_acceleration());
    }
  }

  // angular velocity
  if (imu_pose.has_angular_velocity()) {
    if (FLAGS_enable_map_reference_unify) {
      if (FLAGS_enable_map_reference_unify) {
        if (pose->has_orientation()) {
          pose->mutable_angular_velocity()->CopyFrom(QuaternionRotateXYZ(
              imu_pose.angular_velocity(), pose->orientation()));
          pose->mutable_angular_velocity_vrf()->CopyFrom(
              imu_pose.angular_velocity());
        } else {
          AERROR << "Fail to convert angular_velocity";
        }
      } else {
        pose->mutable_angular_velocity()->CopyFrom(imu_pose.angular_velocity());
      }
    } else {
      pose->mutable_angular_velocity()->CopyFrom(imu_pose.angular_velocity());
    }
  }

  // euler angle
  if (imu_pose.has_euler_angles())
    pose->mutable_euler_angles()->CopyFrom(imu_pose.euler_angles());
}
}  // namespace

PredictorOutput::PredictorOutput(
    double memory_cycle_sec,
    const std::function<Status(double, const Pose&)>& publish_loc_func)
    : Predictor(memory_cycle_sec), publish_loc_func_(publish_loc_func) {
  name_ = kPredictorOutputName;
  dep_predicteds_.emplace(kPredictorGpsName, PoseList(memory_cycle_sec));
  dep_predicteds_.emplace(kPredictorImuName, PoseList(memory_cycle_sec));
  on_adapter_thread_ = true;
}

PredictorOutput::~PredictorOutput() {}

bool PredictorOutput::Updateable() const {
  const auto& imu = dep_predicteds_.find(kPredictorImuName)->second;
  if (predicted_.empty()) {
    const auto& gps = dep_predicteds_.find(kPredictorGpsName)->second;
    return !gps.empty() && !imu.empty();
  } else {
    return !imu.empty() && predicted_.Older(imu);
  }
}

Status PredictorOutput::Update() {
  if (predicted_.empty()) {
    const auto& gps = dep_predicteds_[kPredictorGpsName];
    auto gps_latest = gps.Latest();
    auto timestamp_sec = gps_latest->first;
    auto pose = gps_latest->second;

    if (!pose.has_heading() && pose.has_orientation()) {
      pose.set_heading(QuaternionToHeading(
          pose.orientation().qw(), pose.orientation().qx(),
          pose.orientation().qy(), pose.orientation().qz()));
    }

    const auto& imu = dep_predicteds_[kPredictorImuName];
    Pose imu_pose;
    imu.FindNearestPose(timestamp_sec, &imu_pose);

    // fill pose from imu
    FillPoseFromImu(imu_pose, &pose);

    // push pose to list
    predicted_.Push(timestamp_sec, pose);

    // publish
    return publish_loc_func_(timestamp_sec, pose);
  } else {
    // get timestamp from imu
    const auto& imu = dep_predicteds_[kPredictorImuName];
    auto timestamp_sec = imu.Latest()->first;

    // base pose for prediction
    auto base_timestamp_sec = predicted_.Latest()->first;
    auto base_pose = predicted_.Latest()->second;

    // predict
    Pose pose;
    PredictByImu(base_timestamp_sec, base_pose, timestamp_sec, &pose);

    // push pose to list
    predicted_.Push(timestamp_sec, pose);

    // publish
    return publish_loc_func_(timestamp_sec, pose);
  }
}

bool PredictorOutput::PredictByImu(double old_timestamp_sec,
                                   const Pose& old_pose,
                                   double new_timestamp_sec, Pose* new_pose) {
  if (!old_pose.has_position() || !old_pose.has_orientation() ||
      !old_pose.has_linear_velocity()) {
    AERROR << "Pose has no some fields";
    return false;
  }

  const auto& imu = dep_predicteds_[kPredictorImuName];
  auto p = imu.RangeOf(old_timestamp_sec);
  auto it = p.first;
  if (it == imu.end()) {
    if (p.second != imu.end()) {
      it = p.second;
    } else {
      AERROR << std::setprecision(15)
             << "Cannot get the lower of range from imu with timestamp["
             << old_timestamp_sec << "]";
      return false;
    }
  }

  auto timestamp_sec = old_timestamp_sec;
  new_pose->CopyFrom(old_pose);
  bool finished = false;
  while (!finished) {
    Pose imu_pose;
    double timestamp_sec_1;
    Pose imu_pose_1;
    auto it_1 = it;
    it_1++;
    if (it_1 != imu.end() && new_timestamp_sec >= it_1->first) {
      PoseList::InterpolatePose(it->first, it->second, it_1->first,
                                it_1->second, timestamp_sec, &imu_pose);
      timestamp_sec_1 = it_1->first;
      imu_pose_1 = it_1->second;
    } else {
      timestamp_sec_1 = new_timestamp_sec;
      imu_pose = it->second;
      imu_pose_1 = imu_pose;
    }

    if (new_timestamp_sec <= timestamp_sec_1) {
      finished = true;
    }

    if (!finished && timestamp_sec_1 <= timestamp_sec) {
      it = it_1;
      imu_pose = imu_pose_1;
      continue;
    }

    if (!imu_pose.has_linear_acceleration() ||
        !imu_pose_1.has_linear_acceleration() ||
        !imu_pose.has_angular_velocity() ||
        !imu_pose_1.has_angular_velocity()) {
      AERROR << "Imu_pose or imu_pose_1 has no some fields";
      return false;
    }

    auto dt = timestamp_sec_1 - timestamp_sec;
    auto orientation = new_pose->orientation();

    Point3D angular_velocity;
    if (FLAGS_enable_map_reference_unify) {
      angular_velocity.CopyFrom(
          QuaternionRotateXYZ(imu_pose.angular_velocity(), orientation));
    } else {
      angular_velocity.CopyFrom(imu_pose.angular_velocity());
    }
    Point3D angular_velocity_1;
    if (FLAGS_enable_map_reference_unify) {
      angular_velocity_1.CopyFrom(
          QuaternionRotateXYZ(imu_pose_1.angular_velocity(), orientation));
    } else {
      angular_velocity_1.CopyFrom(imu_pose_1.angular_velocity());
    }

    Point3D angular_vel;
    angular_vel.set_x((angular_velocity.x() + angular_velocity_1.x()) / 2.0);
    angular_vel.set_y((angular_velocity.y() + angular_velocity_1.y()) / 2.0);
    angular_vel.set_z((angular_velocity.z() + angular_velocity_1.z()) / 2.0);
    EulerAnglesZXYd euler_a(orientation.qw(), orientation.qx(),
                            orientation.qy(), orientation.qz());
    auto derivation_roll =
        angular_vel.x() +
        std::sin(euler_a.roll()) * std::tan(euler_a.pitch()) * angular_vel.y() +
        std::cos(euler_a.roll()) * std::tan(euler_a.pitch()) * angular_vel.z();
    auto derivation_pitch = std::cos(euler_a.roll()) * angular_vel.y() -
                            std::sin(euler_a.roll()) * angular_vel.z();
    auto derivation_yaw =
        std::sin(euler_a.roll()) / std::cos(euler_a.pitch()) * angular_vel.y() +
        std::cos(euler_a.roll()) / std::cos(euler_a.pitch()) * angular_vel.z();
    EulerAnglesZXYd euler_b(euler_a.roll() + derivation_roll * dt,
                            euler_a.pitch() + derivation_pitch * dt,
                            euler_a.yaw() + derivation_yaw * dt);
    auto q = euler_b.ToQuaternion();
    Quaternion orientation_1;
    orientation_1.set_qw(q.w());
    orientation_1.set_qx(q.x());
    orientation_1.set_qy(q.y());
    orientation_1.set_qz(q.z());

    Point3D linear_acceleration;
    if (FLAGS_enable_map_reference_unify) {
      linear_acceleration.CopyFrom(
          QuaternionRotateXYZ(imu_pose.linear_acceleration(), orientation));
    } else {
      linear_acceleration.CopyFrom(imu_pose.linear_acceleration());
    }
    Point3D linear_acceleration_1;
    if (FLAGS_enable_map_reference_unify) {
      linear_acceleration_1.CopyFrom(
          QuaternionRotateXYZ(imu_pose_1.linear_acceleration(), orientation_1));
    } else {
      linear_acceleration_1.CopyFrom(imu_pose_1.linear_acceleration());
    }

    auto linear_velocity = new_pose->linear_velocity();
    Point3D linear_velocity_1;
    linear_velocity_1.set_x(
        (linear_acceleration.x() + linear_acceleration_1.x()) / 2.0 * dt +
        linear_velocity.x());
    linear_velocity_1.set_y(
        (linear_acceleration.y() + linear_acceleration_1.y()) / 2.0 * dt +
        linear_velocity.y());
    linear_velocity_1.set_z(
        (linear_acceleration.z() + linear_acceleration_1.z()) / 2.0 * dt +
        linear_velocity.z());

    auto position = new_pose->position();
    PointENU position_1;
    position_1.set_x(
        (linear_acceleration.x() / 3.0 + linear_acceleration_1.x() / 6.0) * dt *
            dt +
        linear_velocity.x() * dt + position.x());
    position_1.set_y(
        (linear_acceleration.y() / 3.0 + linear_acceleration_1.y() / 6.0) * dt *
            dt +
        linear_velocity.y() * dt + position.y());
    position_1.set_z(
        (linear_acceleration.z() / 3.0 + linear_acceleration_1.z() / 6.0) * dt *
            dt +
        linear_velocity.z() * dt + position.z());

    new_pose->mutable_position()->CopyFrom(position_1);
    new_pose->mutable_orientation()->CopyFrom(orientation_1);
    new_pose->set_heading(QuaternionToHeading(
        new_pose->orientation().qw(), new_pose->orientation().qx(),
        new_pose->orientation().qy(), new_pose->orientation().qz()));
    new_pose->mutable_linear_velocity()->CopyFrom(linear_velocity_1);
    FillPoseFromImu(imu_pose_1, new_pose);

    if (!finished) {
      timestamp_sec = timestamp_sec_1;
      it = it_1;
      imu_pose = imu_pose_1;
    }
  }

  return true;
}

}  // namespace localization
}  // namespace apollo
