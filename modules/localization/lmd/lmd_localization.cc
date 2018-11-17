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

#include "modules/localization/lmd/lmd_localization.h"

#include <algorithm>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using apollo::canbus::Chassis;
using apollo::common::Point3D;
using apollo::common::PointENU;
using apollo::common::Quaternion;
using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::ChassisAdapter;
using apollo::common::adapter::GpsAdapter;
using apollo::common::adapter::ImuAdapter;
using apollo::common::math::HeadingToQuaternion;
using apollo::common::math::QuaternionRotate;
using apollo::common::math::QuaternionToHeading;
using apollo::common::math::RotateAxis;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::time::Clock;
using apollo::common::time::ToSecond;
using apollo::perception::PerceptionObstacles;

template <class T>
static T InterpolateXYZ(const T &p1, const T &p2, const double frac1) {
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
static T InterpolateXYZW(const T &p1, const T &p2, const double frac1) {
  T p;
  double frac2 = 1.0 - frac1;
  if (p1.has_qx() && !std::isnan(p1.qx()) && p2.has_qx() &&
      !std::isnan(p2.qx())) {
    p.set_qx(p1.qx() * frac2 + p2.qx() * frac1);
  }
  if (p1.has_qy() && !std::isnan(p1.qy()) && p2.has_qy() &&
      !std::isnan(p2.qy())) {
    p.set_qy(p1.qy() * frac2 + p2.qy() * frac1);
  }
  if (p1.has_qz() && !std::isnan(p1.qz()) && p2.has_qz() &&
      !std::isnan(p2.qz())) {
    p.set_qz(p1.qz() * frac2 + p2.qz() * frac1);
  }
  if (p1.has_qw() && !std::isnan(p1.qw()) && p2.has_qw() &&
      !std::isnan(p2.qw())) {
    p.set_qw(p1.qw() * frac2 + p2.qw() * frac1);
  }
  return p;
}

static bool InterpolateGPS(const Gps &gps1, const Gps &gps2,
                           const double timestamp_sec, Gps *gps_msg) {
  if (!(gps1.has_header() && gps1.header().has_timestamp_sec() &&
        gps2.has_header() && gps2.header().has_timestamp_sec() &&
        gps1.has_localization() && gps2.has_localization())) {
    AERROR << "gps1 and gps2 has no header or no some fields";
    return false;
  }
  if (timestamp_sec - gps1.header().timestamp_sec() <
      FLAGS_timestamp_sec_tolerance) {
    AERROR << "[InterpolateGPS]: the given time stamp[" << timestamp_sec
           << "] is older than the 1st message["
           << gps1.header().timestamp_sec() << "]";
    *gps_msg = gps1;
  } else if (timestamp_sec - gps2.header().timestamp_sec() >
             FLAGS_timestamp_sec_tolerance) {
    AERROR << "[InterpolateGPS]: the given time stamp[" << timestamp_sec
           << "] is newer than the 2nd message["
           << gps2.header().timestamp_sec() << "]";
    *gps_msg = gps1;
  } else {
    *gps_msg = gps1;
    gps_msg->mutable_header()->set_timestamp_sec(timestamp_sec);

    double time_diff =
        gps2.header().timestamp_sec() - gps1.header().timestamp_sec();
    if (fabs(time_diff) >= 0.001) {
      double frac1 =
          (timestamp_sec - gps1.header().timestamp_sec()) / time_diff;
      const auto &gps1_pose = gps1.localization();
      const auto &gps2_pose = gps2.localization();

      if (gps1_pose.has_position() && gps2_pose.has_position()) {
        auto val =
            InterpolateXYZ(gps1_pose.position(), gps2_pose.position(), frac1);
        gps_msg->mutable_localization()->mutable_position()->CopyFrom(val);
      }

      if (gps1_pose.has_orientation() && gps2_pose.has_orientation()) {
        auto val = InterpolateXYZW(gps1_pose.orientation(),
                                   gps2_pose.orientation(), frac1);
        gps_msg->mutable_localization()->mutable_orientation()->CopyFrom(val);
      }

      if (gps1_pose.has_linear_velocity() && gps2_pose.has_linear_velocity()) {
        auto val = InterpolateXYZ(gps1_pose.linear_velocity(),
                                  gps2_pose.linear_velocity(), frac1);
        gps_msg->mutable_localization()->mutable_linear_velocity()->CopyFrom(
            val);
      }
    }
  }
  return true;
}

static bool InterpolateIMU(const CorrectedImu &imu1, const CorrectedImu &imu2,
                           const double timestamp_sec, CorrectedImu *imu_msg) {
  if (!(imu1.has_header() && imu1.header().has_timestamp_sec() &&
        imu2.has_header() && imu2.header().has_timestamp_sec())) {
    AERROR << "imu1 and imu2 has no header or no timestamp_sec in header";
    return false;
  }
  if (timestamp_sec - imu1.header().timestamp_sec() <
      FLAGS_timestamp_sec_tolerance) {
    AERROR << "[InterpolateIMU]: the given time stamp[" << timestamp_sec
           << "] is older than the 1st message["
           << imu1.header().timestamp_sec() << "]";
    *imu_msg = imu1;
  } else if (timestamp_sec - imu2.header().timestamp_sec() >
             FLAGS_timestamp_sec_tolerance) {
    AERROR << "[InterpolateIMU]: the given time stamp[" << timestamp_sec
           << "] is newer than the 2nd message["
           << imu2.header().timestamp_sec() << "]";
    *imu_msg = imu1;
  } else {
    *imu_msg = imu1;
    imu_msg->mutable_header()->set_timestamp_sec(timestamp_sec);

    double time_diff =
        imu2.header().timestamp_sec() - imu1.header().timestamp_sec();
    if (fabs(time_diff) >= 0.001) {
      double frac1 =
          (timestamp_sec - imu1.header().timestamp_sec()) / time_diff;

      if (imu1.has_imu() && imu1.imu().has_angular_velocity() &&
          imu2.has_imu() && imu2.imu().has_angular_velocity()) {
        auto val = InterpolateXYZ(imu1.imu().angular_velocity(),
                                  imu2.imu().angular_velocity(), frac1);
        imu_msg->mutable_imu()->mutable_angular_velocity()->CopyFrom(val);
      }

      if (imu1.has_imu() && imu1.imu().has_linear_acceleration() &&
          imu2.has_imu() && imu2.imu().has_linear_acceleration()) {
        auto val = InterpolateXYZ(imu1.imu().linear_acceleration(),
                                  imu2.imu().linear_acceleration(), frac1);
        imu_msg->mutable_imu()->mutable_linear_acceleration()->CopyFrom(val);
      }

      if (imu1.has_imu() && imu1.imu().has_euler_angles() && imu2.has_imu() &&
          imu2.imu().has_euler_angles()) {
        auto val = InterpolateXYZ(imu1.imu().euler_angles(),
                                  imu2.imu().euler_angles(), frac1);
        imu_msg->mutable_imu()->mutable_euler_angles()->CopyFrom(val);
      }
    }
  }
  return true;
}

static bool InterpolateChassis(const Chassis &chassis1, const Chassis &chassis2,
                               const double timestamp_sec,
                               Chassis *chassis_msg) {
  if (!(chassis1.has_header() && chassis1.header().has_timestamp_sec() &&
        chassis2.has_header() && chassis2.header().has_timestamp_sec())) {
    AERROR
        << "chassis1 and chassis2 has no header or no timestamp_sec in header";
    return false;
  }
  if (timestamp_sec - chassis1.header().timestamp_sec() <
      FLAGS_timestamp_sec_tolerance) {
    AERROR << "[InterpolateChassis]: the given time stamp[" << timestamp_sec
           << "] is older than the 1st message["
           << chassis1.header().timestamp_sec() << "]";
    *chassis_msg = chassis1;
  } else if (timestamp_sec - chassis2.header().timestamp_sec() >
             FLAGS_timestamp_sec_tolerance) {
    AERROR << "[InterpolateChassis]: the given time stamp[" << timestamp_sec
           << "] is newer than the 2nd message["
           << chassis2.header().timestamp_sec() << "]";
    *chassis_msg = chassis1;
  } else {
    *chassis_msg = chassis1;
    chassis_msg->mutable_header()->set_timestamp_sec(timestamp_sec);

    double time_diff =
        chassis2.header().timestamp_sec() - chassis1.header().timestamp_sec();
    if (fabs(time_diff) >= 0.001) {
      double frac1 =
          (timestamp_sec - chassis1.header().timestamp_sec()) / time_diff;

      double frac2 = 1.0 - frac1;
      if (chassis1.has_speed_mps()) {
        float val = 0.0;
        if (!std::isnan(chassis1.speed_mps()) &&
            !std::isnan(chassis2.speed_mps())) {
          val = chassis1.speed_mps() * frac2 + chassis2.speed_mps() * frac1;
        }
        chassis_msg->set_speed_mps(val);
      }

      if (chassis1.has_odometer_m()) {
        float val = 0.0;
        if (!std::isnan(chassis1.odometer_m()) &&
            !std::isnan(chassis2.odometer_m())) {
          val = chassis1.odometer_m() * frac2 + chassis2.odometer_m() * frac1;
        }
        chassis_msg->set_odometer_m(val);
      }
    }
  }
  return true;
}

static void FillPoseFromImu(const Pose &imu, Pose *pose) {
  // linear acceleration
  if (imu.has_linear_acceleration()) {
    if (FLAGS_enable_map_reference_unify) {
      if (pose->has_orientation()) {
        // linear_acceleration:
        // convert from vehicle reference to map reference
        Eigen::Vector3d orig(imu.linear_acceleration().x(),
                             imu.linear_acceleration().y(),
                             imu.linear_acceleration().z());
        auto vec = QuaternionRotate(pose->orientation(), orig);
        pose->mutable_linear_acceleration()->set_x(vec[0]);
        pose->mutable_linear_acceleration()->set_y(vec[1]);
        pose->mutable_linear_acceleration()->set_z(vec[2]);

        // linear_acceleration_vrf
        pose->mutable_linear_acceleration_vrf()->CopyFrom(
            imu.linear_acceleration());
      } else {
        AERROR << "[PrepareLocalizationMsg]: "
               << "fail to convert linear_acceleration";
      }
    } else {
      pose->mutable_linear_acceleration()->CopyFrom(imu.linear_acceleration());
    }
  }

  // angular velocity
  if (imu.has_angular_velocity()) {
    if (FLAGS_enable_map_reference_unify) {
      if (pose->has_orientation()) {
        // angular_velocity:
        // convert from vehicle reference to map reference
        Eigen::Vector3d orig(imu.angular_velocity().x(),
                             imu.angular_velocity().y(),
                             imu.angular_velocity().z());
        auto vec = QuaternionRotate(pose->orientation(), orig);
        pose->mutable_angular_velocity()->set_x(vec[0]);
        pose->mutable_angular_velocity()->set_y(vec[1]);
        pose->mutable_angular_velocity()->set_z(vec[2]);

        // angular_velocity_vrf
        pose->mutable_angular_velocity_vrf()->CopyFrom(imu.angular_velocity());
      } else {
        AERROR << "[PrepareLocalizationMsg]: "
               << "fail to convert angular_velocity";
      }
    } else {
      pose->mutable_angular_velocity()->CopyFrom(imu.angular_velocity());
    }
  }

  // euler angle
  if (imu.has_euler_angles())
    pose->mutable_euler_angles()->CopyFrom(imu.euler_angles());
}

LMDLocalization::LMDLocalization()
    : monitor_logger_(MonitorMessageItem::LOCALIZATION),
      map_offset_{FLAGS_map_offset_x, FLAGS_map_offset_y, FLAGS_map_offset_z} {}

LMDLocalization::~LMDLocalization() {}

Status LMDLocalization::Start() {
  AdapterManager::Init(FLAGS_lmd_adapter_config_file);

  // GPS
  AdapterManager::AddGpsCallback(&LMDLocalization::OnGps, this);

  // Perception Obstacles
  AdapterManager::AddPerceptionObstaclesCallback(
      &LMDLocalization::OnPerceptionObstacles, this);

  // start ROS timer, one-shot = false, auto-start = true
  const double duration = 1.0 / FLAGS_localization_publish_freq;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &LMDLocalization::OnTimer, this);

  tf2_broadcaster_.reset(new tf2_ros::TransformBroadcaster);

  return Status::OK();
}

Status LMDLocalization::Stop() {
  timer_.stop();
  return Status::OK();
}

void LMDLocalization::OnGps(const Gps &gps) {
  // take a snapshot of the current received messages
  AdapterManager::Observe();

  // TODO(all): only for init
  if (has_last_pose_) return;

  Pose new_pose;
  double new_timestamp_sec;
  if (!GetGpsPose(gps, &new_pose, &new_timestamp_sec)) return;

  if (!has_last_pose_ || last_pose_timestamp_sec_ < new_timestamp_sec) {
    if (!has_last_pose_) AINFO << "initialize pose";

    has_last_pose_ = true;
    last_pose_.CopyFrom(new_pose);
    last_pose_timestamp_sec_ = new_timestamp_sec;
  }
}

void LMDLocalization::OnPerceptionObstacles(
    const PerceptionObstacles &obstacles) {
  if (has_last_pose_ && obstacles.has_lane_marker()) {
    if (!obstacles.has_header() || !obstacles.header().has_timestamp_sec()) {
      AERROR << "obstacles has no header or no some fields";
      return;
    }

    // take a snapshot of the current received messages
    AdapterManager::Observe();

    auto timestamp_sec = obstacles.header().timestamp_sec();
    if (last_pose_timestamp_sec_ > timestamp_sec) return;

    // predict pose
    Pose new_pose;
    if (!PredictPose(last_pose_, last_pose_timestamp_sec_, timestamp_sec,
                     &new_pose)) {
      AERROR << "predict pose failed";
      return;
    }
    const auto &position_estimated = new_pose.position();
    auto heading_estimated = new_pose.heading();

    // TODO(all): add registration with perception
    auto position = position_estimated;
    auto heading = heading_estimated;

    // position
    // world frame -> map frame
    new_pose.mutable_position()->set_x(position.x() - map_offset_[0]);
    new_pose.mutable_position()->set_y(position.y() - map_offset_[1]);
    new_pose.mutable_position()->set_z(position.z() - map_offset_[2]);

    // TODO(all): heading to orientation
    // orientation
    new_pose.set_heading(heading);
    auto orientation = HeadingToQuaternion(heading);
    new_pose.mutable_orientation()->set_qx(orientation.x());
    new_pose.mutable_orientation()->set_qy(orientation.y());
    new_pose.mutable_orientation()->set_qz(orientation.z());
    new_pose.mutable_orientation()->set_qw(orientation.w());

    ADEBUG << "before pc registration";
    PrintPoseError(last_pose_, last_pose_timestamp_sec_);

    last_pose_.CopyFrom(new_pose);
    last_pose_timestamp_sec_ = timestamp_sec;

    ADEBUG << "after pc registration";
    PrintPoseError(last_pose_, last_pose_timestamp_sec_);
  }
}

void LMDLocalization::OnTimer(const ros::TimerEvent &event) {
  if (!has_last_pose_) return;

  // take a snapshot of the current received messages
  AdapterManager::Observe();

  //  Prepare localization message.
  LocalizationEstimate localization;
  PrepareLocalizationMsg(&localization);

  // publish localization messages
  AdapterManager::PublishLocalization(localization);
  PublishPoseBroadcastTF(localization);
  ADEBUG << "[OnTimer]: Localization message publish success!";

  // watch dog
  RunWatchDog();
}

void LMDLocalization::PrepareLocalizationMsg(
    LocalizationEstimate *localization) {
  localization->Clear();

  // header
  AdapterManager::FillLocalizationHeader(FLAGS_localization_module_name,
                                         localization);

  // set timestamp
  auto *imu_adapter = AdapterManager::GetImu();
  if (imu_adapter->Empty()) {
    AERROR << "IMU message Queue is empty!";
    return;
  }
  auto imu_msg = imu_adapter->GetLatestObserved();
  if (!imu_msg.has_header() || !imu_msg.header().has_timestamp_sec()) {
    AERROR << "imu has no header or no timestamp_sec in header";
    return;
  }
  auto timestamp_sec =
      std::max(imu_msg.header().timestamp_sec(), last_pose_timestamp_sec_);

  // predict pose
  Pose new_pose;
  if (!PredictPose(last_pose_, last_pose_timestamp_sec_, timestamp_sec,
                   &new_pose)) {
    AERROR << "predict pose failed";
    return;
  }
  last_pose_ = new_pose;
  last_pose_timestamp_sec_ = timestamp_sec;

  // measurement_time
  localization->set_measurement_time(last_pose_timestamp_sec_);

  // pose
  localization->mutable_pose()->CopyFrom(last_pose_);
}

bool LMDLocalization::GetGpsPose(const Gps &gps, Pose *pose,
                                 double *timestamp_sec) {
  if (!gps.has_header() || !gps.header().has_timestamp_sec() ||
      !gps.has_localization() || !gps.localization().has_position()) {
    AERROR << "gps has no header or no some fields";
    return false;
  }

  *timestamp_sec = gps.header().timestamp_sec();
  const auto &gps_pose = gps.localization();

  // position
  // world frame -> map frame
  pose->mutable_position()->set_x(gps_pose.position().x() - map_offset_[0]);
  pose->mutable_position()->set_y(gps_pose.position().y() - map_offset_[1]);
  pose->mutable_position()->set_z(gps_pose.position().z() - map_offset_[2]);

  // orientation
  if (gps_pose.has_orientation()) {
    pose->mutable_orientation()->CopyFrom(gps_pose.orientation());
    auto heading = QuaternionToHeading(
        gps_pose.orientation().qw(), gps_pose.orientation().qx(),
        gps_pose.orientation().qy(), gps_pose.orientation().qz());
    pose->set_heading(heading);
  }

  // linear velocity
  if (gps_pose.has_linear_velocity())
    pose->mutable_linear_velocity()->CopyFrom(gps_pose.linear_velocity());

  // IMU
  CorrectedImu imu_msg;
  if (!FindMatchingIMU(*timestamp_sec, &imu_msg)) return false;
  CHECK(imu_msg.has_imu());
  const auto &imu = imu_msg.imu();
  FillPoseFromImu(imu, pose);

  return true;
}

bool LMDLocalization::PredictPose(const Pose &old_pose,
                                  double old_timestamp_sec,
                                  double new_timestamp_sec, Pose *new_pose) {
  return PredictByLinearIntergrate(old_pose, old_timestamp_sec,
                                   new_timestamp_sec, new_pose);
}

bool LMDLocalization::FindMatchingGPS(double timestamp_sec, Gps *gps_msg) {
  auto *gps_adapter = AdapterManager::GetGps();
  if (gps_adapter->Empty()) {
    AERROR << "Cannot find Matching GPS. "
           << "GPS message Queue is empty! timestamp[" << timestamp_sec << "]";
    return false;
  }

  // scan gps buffer, find first gps message that is newer than the given
  // timestamp
  GpsAdapter::Iterator gps_it = gps_adapter->begin();
  for (; gps_it != gps_adapter->end(); ++gps_it) {
    if ((*gps_it)->header().timestamp_sec() - timestamp_sec >
        FLAGS_timestamp_sec_tolerance) {
      break;
    }
  }

  if (gps_it != gps_adapter->end()) {  // found one
    if (gps_it == gps_adapter->begin()) {
      AERROR << "Gps queue too short or request too old. "
             << "Oldest timestamp["
             << gps_adapter->GetOldestObserved().header().timestamp_sec()
             << "], Newest timestamp["
             << gps_adapter->GetLatestObserved().header().timestamp_sec()
             << "], timestamp[" << timestamp_sec << "]";
      *gps_msg = gps_adapter->GetOldestObserved();  // the oldest gps
    } else {
      // here is the normal case
      auto gps_it_1 = gps_it;
      gps_it_1--;
      if (!(*gps_it)->has_header() || !(*gps_it_1)->has_header()) {
        AERROR << "gps1 and gps_it_1 must both have header.";
        return false;
      }
      if (!InterpolateGPS(**gps_it_1, **gps_it, timestamp_sec, gps_msg)) {
        AERROR << "failed to interpolate GPS";
        return false;
      }
    }
  } else {
    // give the newest gps, without extrapolation
    *gps_msg = gps_adapter->GetLatestObserved();
    if (gps_msg == nullptr) {
      AERROR << "Fail to get latest observed gps_msg.";
      return false;
    }

    if (!gps_msg->has_header()) {
      AERROR << "gps_msg must have header.";
      return false;
    }
  }
  return true;
}  // namespace localization

bool LMDLocalization::FindMatchingIMU(double timestamp_sec,
                                      CorrectedImu *imu_msg) {
  auto *imu_adapter = AdapterManager::GetImu();
  if (imu_adapter->Empty()) {
    AERROR << "Cannot find Matching IMU. "
           << "IMU message Queue is empty! timestamp[" << timestamp_sec << "]";
    return false;
  }

  // scan imu buffer, find first imu message that is newer than the given
  // timestamp
  ImuAdapter::Iterator imu_it = imu_adapter->begin();
  for (; imu_it != imu_adapter->end(); ++imu_it) {
    if ((*imu_it)->header().timestamp_sec() - timestamp_sec >
        FLAGS_timestamp_sec_tolerance) {
      break;
    }
  }

  if (imu_it != imu_adapter->end()) {  // found one
    if (imu_it == imu_adapter->begin()) {
      AERROR << "IMU queue too short or request too old. "
             << "Oldest timestamp["
             << imu_adapter->GetOldestObserved().header().timestamp_sec()
             << "], Newest timestamp["
             << imu_adapter->GetLatestObserved().header().timestamp_sec()
             << "], timestamp[" << timestamp_sec << "]";
      *imu_msg = imu_adapter->GetOldestObserved();  // the oldest imu
    } else {
      // here is the normal case
      auto imu_it_1 = imu_it;
      imu_it_1--;
      if (!(*imu_it)->has_header() || !(*imu_it_1)->has_header()) {
        AERROR << "imu1 and imu_it_1 must both have header.";
        return false;
      }
      if (!InterpolateIMU(**imu_it_1, **imu_it, timestamp_sec, imu_msg)) {
        AERROR << "failed to interpolate IMU";
        return false;
      }
    }
  } else {
    // give the newest imu, without extrapolation
    *imu_msg = imu_adapter->GetLatestObserved();
    if (imu_msg == nullptr) {
      AERROR << "Fail to get latest observed imu_msg.";
      return false;
    }

    if (!imu_msg->has_header()) {
      AERROR << "imu_msg must have header.";
      return false;
    }
  }
  return true;
}

bool LMDLocalization::FindMatchingChassis(double timestamp_sec,
                                          Chassis *chassis_msg) {
  auto *chassis_adapter = AdapterManager::GetChassis();

  if (chassis_adapter->Empty()) {
    AERROR << "Cannot find Matching Chassis. "
           << "Chassis message Queue is empty! timestamp[" << timestamp_sec
           << "]";
    return false;
  }

  // scan chassis buffer, find first chassis message that is newer than the
  // given timestamp
  ChassisAdapter::Iterator chassis_it = chassis_adapter->begin();
  for (; chassis_it != chassis_adapter->end(); ++chassis_it) {
    if ((*chassis_it)->header().timestamp_sec() - timestamp_sec >
        FLAGS_timestamp_sec_tolerance) {
      break;
    }
  }

  if (chassis_it != chassis_adapter->end()) {  // found one
    if (chassis_it == chassis_adapter->begin()) {
      AERROR << "Chassis queue too short or request too old. "
             << "Oldest timestamp["
             << chassis_adapter->GetOldestObserved().header().timestamp_sec()
             << "], Newest timestamp["
             << chassis_adapter->GetLatestObserved().header().timestamp_sec()
             << "], timestamp[" << timestamp_sec << "]";
      *chassis_msg =
          chassis_adapter->GetOldestObserved();  // the oldest chassis
    } else {
      // here is the normal case
      auto chassis_it_1 = chassis_it;
      chassis_it_1--;
      if (!(*chassis_it)->has_header() || !(*chassis_it_1)->has_header()) {
        AERROR << "chassis_it and chassis_it_1 must both have header.";
        return false;
      }
      if (!InterpolateChassis(**chassis_it_1, **chassis_it, timestamp_sec,
                              chassis_msg)) {
        AERROR << "failed to interpolate IMU";
        return false;
      }
    }
  } else {
    // give the newest imu, without extrapolation
    *chassis_msg = chassis_adapter->GetLatestObserved();
    if (chassis_msg == nullptr) {
      AERROR << "Fail to get latest observed chassis_msg.";
      return false;
    }

    if (!chassis_msg->has_header()) {
      AERROR << "chassis_msg must have header.";
      return false;
    }
  }
  return true;
}

bool LMDLocalization::PredictByLinearIntergrate(const Pose &old_pose,
                                                double old_timestamp_sec,
                                                double new_timestamp_sec,
                                                Pose *new_pose) {
  if (!old_pose.has_position() || !old_pose.has_orientation() ||
      !old_pose.has_linear_velocity()) {
    AERROR << "old_pose has no some fields";
    return false;
  }

  auto *imu_adapter = AdapterManager::GetImu();
  if (imu_adapter->Empty()) {
    AERROR << "Cannot find Matching IMU. "
           << "IMU message Queue is empty!";
    return false;
  }

  auto imu_it = imu_adapter->end();
  do {
    imu_it--;
    if ((*imu_it)->header().timestamp_sec() - old_timestamp_sec >
        FLAGS_timestamp_sec_tolerance)
      break;
  } while (imu_it != imu_adapter->begin());
  imu_it++;

  if (imu_it == imu_adapter->end()) {
    AERROR << std::setprecision(15)
           << "IMU queue too short or request too old. "
           << "Oldest timestamp["
           << imu_adapter->GetOldestObserved().header().timestamp_sec()
           << "], Newest timestamp["
           << imu_adapter->GetLatestObserved().header().timestamp_sec()
           << "], timestamp[" << old_timestamp_sec << "]";
    return false;
  }

  CorrectedImu imu;
  if (imu_it != imu_adapter->begin()) {
    auto imu_it_1 = imu_it;
    imu_it_1--;
    if (!(*imu_it)->has_header() || !(*imu_it_1)->has_header() ||
        !(*imu_it)->has_imu() || !(*imu_it_1)->has_imu()) {
      AERROR << "imu_it and imu_it_1 must both have header or some fields.";
      return false;
    }
    if (!InterpolateIMU(**imu_it, **imu_it_1, old_timestamp_sec, &imu)) {
      AERROR << "failed to interpolate IMU";
      return false;
    }
  } else {
    imu.CopyFrom(**imu_it);
  }

  new_pose->CopyFrom(old_pose);
  auto timestamp_sec = old_timestamp_sec;
  bool finished = false;
  while (!finished) {
    CorrectedImu imu_1;
    double timestamp_sec_1;

    auto imu_it_1 = imu_it;
    if (imu_it_1 != imu_adapter->begin()) {
      imu_it_1--;

      if (!(*imu_it_1)->has_header() ||
          !(*imu_it_1)->header().has_timestamp_sec()) {
        AERROR << "imu_it_1 must have header and timestamp_sec.";
        return false;
      }

      timestamp_sec_1 = (*imu_it_1)->header().timestamp_sec();
      if (timestamp_sec_1 < new_timestamp_sec) {
        imu_1.CopyFrom(**imu_it_1);
      } else {
        timestamp_sec_1 = new_timestamp_sec;
        if (!InterpolateIMU(**imu_it, **imu_it_1, timestamp_sec_1, &imu_1)) {
          AERROR << "failed to interpolate IMU";
          return false;
        }
        finished = true;
      }
    } else {
      timestamp_sec_1 = new_timestamp_sec;
      imu_1.CopyFrom(imu);
      finished = true;
    }

    if (!imu.has_imu() || !imu_1.has_imu() ||
        !imu.imu().has_linear_acceleration() ||
        !imu_1.imu().has_linear_acceleration() ||
        !imu.imu().has_angular_velocity() ||
        !imu_1.imu().has_angular_velocity()) {
      AERROR << "imu or imu1 has no some fields";
      return false;
    }

    // auto angular_velocity = imu.imu().angular_velocity();
    // auto angular_velocity1 = imu_1.imu().angular_velocity();
    auto orientation = new_pose->orientation();
    auto orientation_1 = orientation;

    Point3D linear_acceleration;
    if (FLAGS_enable_map_reference_unify) {
      Eigen::Vector3d orig(imu.imu().linear_acceleration().x(),
                           imu.imu().linear_acceleration().y(),
                           imu.imu().linear_acceleration().z());
      auto vec = QuaternionRotate(orientation, orig);
      linear_acceleration.set_x(vec[0]);
      linear_acceleration.set_y(vec[1]);
      linear_acceleration.set_z(vec[2]);
    } else {
      linear_acceleration.CopyFrom(imu.imu().linear_acceleration());
    }

    Point3D linear_acceleration_1;
    if (FLAGS_enable_map_reference_unify) {
      Eigen::Vector3d orig(imu_1.imu().linear_acceleration().x(),
                           imu_1.imu().linear_acceleration().y(),
                           imu_1.imu().linear_acceleration().z());
      auto vec = QuaternionRotate(orientation_1, orig);
      linear_acceleration_1.set_x(vec[0]);
      linear_acceleration_1.set_y(vec[1]);
      linear_acceleration_1.set_z(vec[2]);
    } else {
      linear_acceleration_1.CopyFrom(imu_1.imu().linear_acceleration());
    }

    auto linear_velocity = new_pose->linear_velocity();
    Point3D linear_velocity_1;
    auto dt = timestamp_sec_1 - timestamp_sec;
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
    FillPoseFromImu(imu_1.imu(), new_pose);

    if (!finished) {
      timestamp_sec = timestamp_sec_1;
      imu_it = imu_it_1;
      imu.CopyFrom(imu_1);
    }
  }
  return true;
}

void LMDLocalization::PrintPoseError(const Pose &pose, double timestamp_sec) {
  Gps gps;
  if (!FindMatchingGPS(timestamp_sec, &gps)) return;

  Pose gps_pose;
  double t;
  if (!GetGpsPose(gps, &gps_pose, &t)) return;

  if (!pose.has_position() || !gps_pose.has_position() ||
      !pose.has_orientation() || !gps_pose.has_orientation() ||
      !pose.has_linear_velocity() || !gps_pose.has_linear_velocity())
    return;

  auto heading =
      QuaternionToHeading(pose.orientation().qw(), pose.orientation().qx(),
                          pose.orientation().qy(), pose.orientation().qz());
  auto gps_heading = QuaternionToHeading(
      gps_pose.orientation().qw(), gps_pose.orientation().qx(),
      gps_pose.orientation().qy(), gps_pose.orientation().qz());

  double flu_vx, flu_vy;
  RotateAxis(gps_heading, gps_pose.linear_velocity().x(),
             gps_pose.linear_velocity().y(), &flu_vx, &flu_vy);

  double flu_dx, flu_dy;
  RotateAxis(gps_heading, pose.position().x() - gps_pose.position().x(),
             pose.position().y() - gps_pose.position().y(), &flu_dx, &flu_dy);

  double flu_dvx, flu_dvy;
  RotateAxis(gps_heading, pose.linear_velocity().x(),
             pose.linear_velocity().y(), &flu_dvx, &flu_dvy);
  flu_dvx -= flu_vx;
  flu_dvy -= flu_vy;

  ADEBUG << "timestamp[" << timestamp_sec << "]";
  ADEBUG << "true heading[" << gps_heading << "]";
  ADEBUG << "heading error[" << heading - gps_heading << "]";
  ADEBUG << "true position, x[" << gps_pose.position().x() << "], y["
         << gps_pose.position().y() << "], z[" << gps_pose.position().z()
         << "]";
  ADEBUG << "position error, station[" << flu_dx << "], lateral[" << flu_dy
         << "]";
  ADEBUG << "true velocity, station[" << flu_vx << "], lateral[" << flu_vy
         << "]";
  ADEBUG << "velocity error, station[" << flu_dvx << "], lateral[" << flu_dvy
         << "]";
}

void LMDLocalization::RunWatchDog() {
  if (!FLAGS_enable_watchdog) return;

  // Add code to implement watch dog
}

}  // namespace localization
}  // namespace apollo
