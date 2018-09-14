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

#include "modules/localization/rtk/rtk_localization_component.h"

#include <limits>

#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"

#include "modules/localization/proto/rtk_config.pb.h"

namespace apollo {
namespace localization {

// using apollo::common::monitor::MonitorMessageItem;
using apollo::common::time::Clock;
using ::Eigen::Vector3d;

RTKLocalizationComponent::RTKLocalizationComponent()
    : map_offset_{0.0, 0.0, 0.0} {}
// monitor_logger_(MonitorMessageItem::LOCALIZATION) {}

bool RTKLocalizationComponent::Init() {
  if (InitConfig() != true) {
    AERROR << "Init Config falseed.";
    return false;
  }

  if (InitIO() != true) {
    AERROR << "Init Interval falseed.";
    return false;
  }

  return true;
}

bool RTKLocalizationComponent::InitConfig() {
  rtk_config::Config rtk_config;
  if (!apollo::cybertron::common::GetProtoFromFile(config_file_path_,
                                                   &rtk_config)) {
    return false;
  }
  AINFO << "Rtk localization config: " << rtk_config.DebugString();

  localization_topic_ = rtk_config.localization_topic();
  imu_topic_ = rtk_config.imu_topic();
  gps_topic_ = rtk_config.gps_topic();
  broadcast_tf_frame_id_ = rtk_config.broadcast_tf_frame_id();
  broadcast_tf_child_frame_id_ = rtk_config.broadcast_tf_child_frame_id();
  imu_list_max_size_ = rtk_config.imu_list_max_size();
  gps_imu_time_diff_threshold_ = rtk_config.gps_imu_time_diff_threshold();
  map_offset_[0] = rtk_config.map_offset_x();
  map_offset_[1] = rtk_config.map_offset_y();
  map_offset_[2] = rtk_config.map_offset_z();

  return true;
}

bool RTKLocalizationComponent::InitIO() {
  std::function<void(const std::shared_ptr<localization::CorrectedImu>&)>
      imu_register_call = std::bind(&RTKLocalizationComponent::ImuCallback,
                                    this, std::placeholders::_1);
  corrected_imu_listener_ =
      this->node_->CreateReader<localization::CorrectedImu>(imu_topic_,
                                                            imu_register_call);

  localization_talker_ =
      this->node_->CreateWriter<LocalizationEstimate>(localization_topic_);

  return true;
}

bool RTKLocalizationComponent::Proc(
    const std::shared_ptr<localization::Gps>& gps_msg) {
  // check imu list
  // double time_delay =
  //     common::time::ToSecond(Clock::Now()) - last_received_timestamp_sec_;
  // common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  // if (enable_gps_timestamp_ && time_delay > gps_time_delay_tolerance_) {
  //   buffer.ERROR() << "GPS message time delay: " << time_delay;
  //   buffer.PrintLog();
  // }

  // {
  //   std::unique_lock<std::mutex> lock(imu_list_mutex_);

  //   if (imu_list_.empty()) {
  //     AERROR << "IMU message buffer is empty.";
  //     if (service_started_) {
  //       buffer.ERROR("IMU message buffer is empty.");
  //     }
  //   }
  // }

  // publish localization messages
  PublishLocalization(*gps_msg);
  service_started_ = true;

  // watch dog
  RunWatchDog(gps_msg->header().timestamp_sec());

  last_received_timestamp_sec_ = common::time::ToSecond(Clock::Now());
  return true;
}

void RTKLocalizationComponent::ImuCallback(
    const std::shared_ptr<localization::CorrectedImu>& imu_msg) {
  std::unique_lock<std::mutex> lock(imu_list_mutex_);
  if (imu_list_.size() < imu_list_max_size_) {
    imu_list_.push_back(*imu_msg);
  } else {
    imu_list_.pop_front();
    imu_list_.push_back(*imu_msg);
  }
  return;
}

void RTKLocalizationComponent::PublishLocalization(
    const localization::Gps& gps_msg) {
  LocalizationEstimate localization;
  PrepareLocalizationMsg(gps_msg, &localization);

  // publish localization messages
  PublishPoseBroadcastTopic(localization);
  PublishPoseBroadcastTF(localization);
  ADEBUG << "[OnTimer]: Localization message publish success!";
  return;
}

void RTKLocalizationComponent::RunWatchDog(double gps_timestamp) {
  if (!enable_watch_dog_) {
    return;
  }

  // common::monitor::MonitorLogBuffer buffer(&monitor_logger_);

  // // check GPS time stamp against ROS timer
  // double gps_delay_sec = common::time::ToSecond(Clock::Now()) -
  // gps_timestamp; int64_t gps_delay_cycle_cnt =
  //     static_cast<int64_t>(gps_delay_sec * localization_publish_freq_);

  // bool msg_lost = false;
  // if (enable_gps_timestamp_ &&
  //     (gps_delay_cycle_cnt > report_threshold_err_num_)) {
  //   msg_lost = true;

  //   buffer.ERROR() << "Raw GPS Message Lost. GPS message is "
  //                  << gps_delay_cycle_cnt << " cycle " << gps_delay_sec
  //                  << " sec behind current time.";
  //   buffer.PrintLog();
  // }

  // // to prevent it from beeping continuously
  // if (msg_lost && (last_reported_timestamp_sec_ < 1. ||
  //                  common::time::ToSecond(Clock::Now()) >
  //                      last_reported_timestamp_sec_ + 1.)) {
  //   AERROR << "gps/imu frame lost!";
  //   last_reported_timestamp_sec_ = common::time::ToSecond(Clock::Now());
  // }

  return;
}

void RTKLocalizationComponent::PrepareLocalizationMsg(
    const localization::Gps& gps_msg, LocalizationEstimate* localization) {
  bool imu_valid = true;
  CorrectedImu imu_msg;

  // find the matching gps and imu message
  double gps_time_stamp = gps_msg.header().timestamp_sec();
  if (!FindMatchingIMU(gps_time_stamp, &imu_msg)) {
    imu_valid = false;
  }

  ComposeLocalizationMsg(gps_msg, imu_msg, localization);
  return;
}

void RTKLocalizationComponent::FillLocalizationMsgHeader(
    LocalizationEstimate* localization) {
  DCHECK_NOTNULL(localization);

  auto* header = localization->mutable_header();
  double timestamp = apollo::common::time::Clock::NowInSeconds();
  header->set_module_name(module_name_);
  header->set_timestamp_sec(timestamp);
  header->set_sequence_num(++localization_seq_num_);

  return;
}

void RTKLocalizationComponent::ComposeLocalizationMsg(
    const localization::Gps& gps_msg, const localization::CorrectedImu& imu_msg,
    LocalizationEstimate* localization) {
  localization->Clear();

  FillLocalizationMsgHeader(localization);

  localization->set_measurement_time(gps_msg.header().timestamp_sec());

  // combine gps and imu
  auto mutable_pose = localization->mutable_pose();
  if (gps_msg.has_localization()) {
    const auto& pose = gps_msg.localization();

    if (pose.has_position()) {
      // position
      // world frame -> map frame
      mutable_pose->mutable_position()->set_x(pose.position().x() -
                                              map_offset_[0]);
      mutable_pose->mutable_position()->set_y(pose.position().y() -
                                              map_offset_[1]);
      mutable_pose->mutable_position()->set_z(pose.position().z() -
                                              map_offset_[2]);
    }

    // orientation
    if (pose.has_orientation()) {
      mutable_pose->mutable_orientation()->CopyFrom(pose.orientation());
      double heading = common::math::QuaternionToHeading(
          pose.orientation().qw(), pose.orientation().qx(),
          pose.orientation().qy(), pose.orientation().qz());
      mutable_pose->set_heading(heading);
    }
    // linear velocity
    if (pose.has_linear_velocity()) {
      mutable_pose->mutable_linear_velocity()->CopyFrom(pose.linear_velocity());
    }
  }

  if (imu_msg.has_imu()) {
    const auto& imu = imu_msg.imu();
    // linear acceleration
    if (imu.has_linear_acceleration()) {
      if (localization->pose().has_orientation()) {
        // linear_acceleration:
        // convert from vehicle reference to map reference
        Vector3d orig(imu.linear_acceleration().x(),
                      imu.linear_acceleration().y(),
                      imu.linear_acceleration().z());
        Vector3d vec = common::math::QuaternionRotate(
            localization->pose().orientation(), orig);
        mutable_pose->mutable_linear_acceleration()->set_x(vec[0]);
        mutable_pose->mutable_linear_acceleration()->set_y(vec[1]);
        mutable_pose->mutable_linear_acceleration()->set_z(vec[2]);

        // linear_acceleration_vfr
        mutable_pose->mutable_linear_acceleration_vrf()->CopyFrom(
            imu.linear_acceleration());
      } else {
        AERROR << "[PrepareLocalizationMsg]: "
               << "fail to convert linear_acceleration";
      }
    }

    // angular velocity
    if (imu.has_angular_velocity()) {
      if (localization->pose().has_orientation()) {
        // angular_velocity:
        // convert from vehicle reference to map reference
        Vector3d orig(imu.angular_velocity().x(), imu.angular_velocity().y(),
                      imu.angular_velocity().z());
        Vector3d vec = common::math::QuaternionRotate(
            localization->pose().orientation(), orig);
        mutable_pose->mutable_angular_velocity()->set_x(vec[0]);
        mutable_pose->mutable_angular_velocity()->set_y(vec[1]);
        mutable_pose->mutable_angular_velocity()->set_z(vec[2]);

        // angular_velocity_vf
        mutable_pose->mutable_angular_velocity_vrf()->CopyFrom(
            imu.angular_velocity());
      } else {
        AERROR << "[PrepareLocalizationMsg]: "
               << "fail to convert angular_velocity";
      }
    }

    // euler angle
    if (imu.has_euler_angles()) {
      mutable_pose->mutable_euler_angles()->CopyFrom(imu.euler_angles());
    }
  }
  return;
}

bool RTKLocalizationComponent::FindMatchingIMU(const double gps_timestamp_sec,
                                               CorrectedImu* imu_msg) {
  if (imu_msg == nullptr) {
    AERROR << "imu_msg should NOT be nullptr.";
    return false;
  }

  std::unique_lock<std::mutex> lock(imu_list_mutex_);
  auto imu_list = imu_list_;
  lock.unlock();

  if (imu_list.empty()) {
    AERROR << "Cannot find Matching IMU. "
           << "IMU message Queue is empty! GPS timestamp[" << gps_timestamp_sec
           << "]";
    return false;
  }

  // scan imu buffer, find first imu message that is newer than the given
  // timestamp
  auto imu_it = imu_list.begin();
  for (; imu_it != imu_list.end(); ++imu_it) {
    if ((*imu_it).header().timestamp_sec() - gps_timestamp_sec >
        std::numeric_limits<double>::min()) {
      break;
    }
  }

  if (imu_it != imu_list.end()) {  // found one
    if (imu_it == imu_list.begin()) {
      AERROR << "IMU queue too short or request too old. "
             << "Oldest timestamp[" << imu_list.front().header().timestamp_sec()
             << "], Newest timestamp["
             << imu_list.back().header().timestamp_sec() << "], GPS timestamp["
             << gps_timestamp_sec << "]";
      *imu_msg = imu_list.front();  // the oldest imu
    } else {
      // here is the normal case
      auto imu_it_1 = imu_it;
      imu_it_1--;
      if (!(*imu_it).has_header() || !(*imu_it_1).has_header()) {
        AERROR << "imu1 and imu_it_1 must both have header.";
        return false;
      }
      if (!InterpolateIMU(*imu_it_1, *imu_it, gps_timestamp_sec, imu_msg)) {
        AERROR << "failed to interpolate IMU";
        return false;
      }
    }
  } else {
    // give the newest imu, without extrapolation
    *imu_msg = imu_list.back();
    if (imu_msg == nullptr) {
      AERROR << "Fail to get latest observed imu_msg.";
      return false;
    }

    if (!imu_msg->has_header()) {
      AERROR << "imu_msg must have header.";
      return false;
    }

    if (std::fabs(imu_msg->header().timestamp_sec() - gps_timestamp_sec) >
        gps_imu_time_diff_threshold_) {
      // 20ms threshold to report error
      AERROR << "Cannot find Matching IMU. "
             << "IMU messages too old"
             << "Newest timestamp[" << imu_list.back().header().timestamp_sec()
             << "], GPS timestamp[" << gps_timestamp_sec << "]";
    }
  }

  return true;
}

bool RTKLocalizationComponent::InterpolateIMU(const CorrectedImu& imu1,
                                              const CorrectedImu& imu2,
                                              const double timestamp_sec,
                                              CorrectedImu* imu_msg) {
  DCHECK_NOTNULL(imu_msg);
  if (!(imu1.has_header() && imu1.header().has_timestamp_sec() &&
        imu2.has_header() && imu2.header().has_timestamp_sec())) {
    AERROR << "imu1 and imu2 has no header or no timestamp_sec in header";
    return false;
  }
  if (timestamp_sec - imu1.header().timestamp_sec() <
      std::numeric_limits<double>::min()) {
    AERROR << "[InterpolateIMU]: the given time stamp[" << timestamp_sec
           << "] is older than the 1st message["
           << imu1.header().timestamp_sec() << "]";
    *imu_msg = imu1;
  } else if (timestamp_sec - imu2.header().timestamp_sec() >
             std::numeric_limits<double>::min()) {
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

template <class T>
T RTKLocalizationComponent::InterpolateXYZ(const T& p1, const T& p2,
                                           const double frac1) {
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

void RTKLocalizationComponent::PublishPoseBroadcastTF(
    const LocalizationEstimate& localization) {
  // broadcast tf message
  adu::common::TransformStamped tf2_msg;

  auto mutable_head = tf2_msg.mutable_header();
  mutable_head->set_stamp(
      cybertron::Time(localization.measurement_time()).ToNanosecond());
  mutable_head->set_frame_id(broadcast_tf_frame_id_);
  tf2_msg.set_child_frame_id(broadcast_tf_child_frame_id_);

  auto mutable_translation = tf2_msg.mutable_transform()->mutable_translation();
  mutable_translation->set_x(localization.pose().position().x());
  mutable_translation->set_y(localization.pose().position().y());
  mutable_translation->set_z(localization.pose().position().z());

  auto mutable_rotation = tf2_msg.mutable_transform()->mutable_rotation();
  mutable_rotation->set_qx(localization.pose().orientation().qx());
  mutable_rotation->set_qy(localization.pose().orientation().qy());
  mutable_rotation->set_qz(localization.pose().orientation().qz());
  mutable_rotation->set_qw(localization.pose().orientation().qw());

  tf2_broadcaster_.sendTransform(tf2_msg);
  return;
}

void RTKLocalizationComponent::PublishPoseBroadcastTopic(
    const LocalizationEstimate& localization) {
  localization_talker_->Write(localization);
  return;
}

}  // namespace localization
}  // namespace apollo
