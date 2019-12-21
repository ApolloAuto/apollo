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

#include "modules/localization/rtk/rtk_localization.h"

#include <limits>

#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"

namespace apollo {
namespace localization {

using apollo::common::time::Clock;
using ::Eigen::Vector3d;

RTKLocalization::RTKLocalization()
    : map_offset_{0.0, 0.0, 0.0},
      monitor_logger_(
          apollo::common::monitor::MonitorMessageItem::LOCALIZATION) {}

void RTKLocalization::InitConfig(const rtk_config::Config &config) {
  imu_list_max_size_ = config.imu_list_max_size();
  gps_imu_time_diff_threshold_ = config.gps_imu_time_diff_threshold();
  map_offset_[0] = config.map_offset_x();
  map_offset_[1] = config.map_offset_y();
  map_offset_[2] = config.map_offset_z();
}

void RTKLocalization::GpsCallback(
    const std::shared_ptr<localization::Gps> &gps_msg) {
  double time_delay =
      last_received_timestamp_sec_
          ? common::time::Clock::NowInSeconds() - last_received_timestamp_sec_
          : last_received_timestamp_sec_;
  if (time_delay > gps_time_delay_tolerance_) {
    std::stringstream ss;
    ss << "GPS message time interval: " << time_delay;
    monitor_logger_.WARN(ss.str());
  }

  {
    std::unique_lock<std::mutex> lock(imu_list_mutex_);

    if (imu_list_.empty()) {
      AERROR << "IMU message buffer is empty.";
      if (service_started_) {
        monitor_logger_.ERROR("IMU message buffer is empty.");
      }
      return;
    }
  }

  {
    std::unique_lock<std::mutex> lock(gps_status_list_mutex_);

    if (gps_status_list_.empty()) {
      AERROR << "Gps status message buffer is empty.";
      if (service_started_) {
        monitor_logger_.ERROR("Gps status message buffer is empty.");
      }
      return;
    }
  }

  // publish localization messages
  PrepareLocalizationMsg(*gps_msg, &last_localization_result_,
                         &last_localization_status_result_);
  service_started_ = true;
  if (service_started_time == 0.0) {
    service_started_time = common::time::Clock::NowInSeconds();
  }

  // watch dog
  RunWatchDog(gps_msg->header().timestamp_sec());

  last_received_timestamp_sec_ = common::time::Clock::NowInSeconds();
}

void RTKLocalization::GpsStatusCallback(
    const std::shared_ptr<drivers::gnss::InsStat> &status_msg) {
  std::unique_lock<std::mutex> lock(gps_status_list_mutex_);
  if (gps_status_list_.size() < gps_status_list_max_size_) {
    gps_status_list_.push_back(*status_msg);
  } else {
    gps_status_list_.pop_front();
    gps_status_list_.push_back(*status_msg);
  }
}

void RTKLocalization::ImuCallback(
    const std::shared_ptr<localization::CorrectedImu> &imu_msg) {
  std::unique_lock<std::mutex> lock(imu_list_mutex_);
  if (imu_list_.size() < imu_list_max_size_) {
    imu_list_.push_back(*imu_msg);
  } else {
    imu_list_.pop_front();
    imu_list_.push_back(*imu_msg);
  }
}

bool RTKLocalization::IsServiceStarted() { return service_started_; }

void RTKLocalization::GetLocalization(LocalizationEstimate *localization) {
  *localization = last_localization_result_;
}

void RTKLocalization::GetLocalizationStatus(
    LocalizationStatus *localization_status) {
  *localization_status = last_localization_status_result_;
}

void RTKLocalization::RunWatchDog(double gps_timestamp) {
  if (!enable_watch_dog_) {
    return;
  }

  // check GPS time stamp against system time
  double gps_delay_sec = common::time::Clock::NowInSeconds() - gps_timestamp;
  double gps_service_delay =
      common::time::Clock::NowInSeconds() - service_started_time;
  int64_t gps_delay_cycle_cnt =
      static_cast<int64_t>(gps_delay_sec * localization_publish_freq_);

  bool msg_delay = false;
  if (gps_delay_cycle_cnt > report_threshold_err_num_ &&
      static_cast<int>(gps_service_delay) > service_delay_threshold) {
    msg_delay = true;
    std::stringstream ss;
    ss << "Raw GPS Message Delay. GPS message is " << gps_delay_cycle_cnt
       << " cycle " << gps_delay_sec << " sec behind current time.";
    monitor_logger_.ERROR(ss.str());
  }

  // check IMU time stamp against system time
  std::unique_lock<std::mutex> lock(imu_list_mutex_);
  auto imu_msg = imu_list_.back();
  lock.unlock();
  double imu_delay_sec =
      common::time::Clock::NowInSeconds() - imu_msg.header().timestamp_sec();
  int64_t imu_delay_cycle_cnt =
      static_cast<int64_t>(imu_delay_sec * localization_publish_freq_);
  if (imu_delay_cycle_cnt > report_threshold_err_num_ &&
      static_cast<int>(gps_service_delay) > service_delay_threshold) {
    msg_delay = true;
    std::stringstream ss;
    ss << "Raw IMU Message Delay. IMU message is " << imu_delay_cycle_cnt
       << " cycle " << imu_delay_sec << " sec behind current time.";
    monitor_logger_.ERROR(ss.str());
  }

  // to prevent it from beeping continuously
  if (msg_delay && (last_reported_timestamp_sec_ < 1. ||
                    common::time::Clock::NowInSeconds() >
                        last_reported_timestamp_sec_ + 1.)) {
    AERROR << "gps/imu frame Delay!";
    last_reported_timestamp_sec_ = common::time::Clock::NowInSeconds();
  }
}

void RTKLocalization::PrepareLocalizationMsg(
    const localization::Gps &gps_msg, LocalizationEstimate *localization,
    LocalizationStatus *localization_status) {
  // find the matching gps and imu message
  double gps_time_stamp = gps_msg.header().timestamp_sec();
  CorrectedImu imu_msg;
  FindMatchingIMU(gps_time_stamp, &imu_msg);
  ComposeLocalizationMsg(gps_msg, imu_msg, localization);

  drivers::gnss::InsStat gps_status;
  FindNearestGpsStatus(gps_time_stamp, &gps_status);
  FillLocalizationStatusMsg(gps_status, localization_status);
}

void RTKLocalization::FillLocalizationMsgHeader(
    LocalizationEstimate *localization) {
  auto *header = localization->mutable_header();
  double timestamp = apollo::common::time::Clock::NowInSeconds();
  header->set_module_name(module_name_);
  header->set_timestamp_sec(timestamp);
  header->set_sequence_num(static_cast<unsigned int>(++localization_seq_num_));
}

void RTKLocalization::FillLocalizationStatusMsg(
    const drivers::gnss::InsStat &status,
    LocalizationStatus *localization_status) {
  apollo::common::Header *header = localization_status->mutable_header();
  double timestamp = apollo::common::time::Clock::NowInSeconds();
  header->set_timestamp_sec(timestamp);
  localization_status->set_measurement_time(status.header().timestamp_sec());

  if (!status.has_pos_type()) {
    localization_status->set_fusion_status(MeasureState::ERROR);
    localization_status->set_state_message(
        "Error: Current Localization Status Is Missing.");
    return;
  }

  auto pos_type = static_cast<drivers::gnss::SolutionType>(status.pos_type());
  switch (pos_type) {
    case drivers::gnss::SolutionType::INS_RTKFIXED:
      localization_status->set_fusion_status(MeasureState::OK);
      localization_status->set_state_message("");
      break;
    case drivers::gnss::SolutionType::INS_RTKFLOAT:
      localization_status->set_fusion_status(MeasureState::WARNNING);
      localization_status->set_state_message(
          "Warning: Current Localization Is Unstable.");
      break;
    default:
      localization_status->set_fusion_status(MeasureState::ERROR);
      localization_status->set_state_message(
          "Error: Current Localization Is Very Unstable.");
      break;
  }
}

void RTKLocalization::ComposeLocalizationMsg(
    const localization::Gps &gps_msg, const localization::CorrectedImu &imu_msg,
    LocalizationEstimate *localization) {
  localization->Clear();

  FillLocalizationMsgHeader(localization);

  localization->set_measurement_time(gps_msg.header().timestamp_sec());

  // combine gps and imu
  auto mutable_pose = localization->mutable_pose();
  if (gps_msg.has_localization()) {
    const auto &pose = gps_msg.localization();

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
    const auto &imu = imu_msg.imu();
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
        AERROR << "[PrepareLocalizationMsg]: fail to convert angular_velocity";
      }
    }

    // euler angle
    if (imu.has_euler_angles()) {
      mutable_pose->mutable_euler_angles()->CopyFrom(imu.euler_angles());
    }
  }
}

bool RTKLocalization::FindMatchingIMU(const double gps_timestamp_sec,
                                      CorrectedImu *imu_msg) {
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
      AERROR << "Cannot find Matching IMU. IMU messages too old. "
             << "Newest timestamp[" << imu_list.back().header().timestamp_sec()
             << "], GPS timestamp[" << gps_timestamp_sec << "]";
    }
  }

  return true;
}

bool RTKLocalization::InterpolateIMU(const CorrectedImu &imu1,
                                     const CorrectedImu &imu2,
                                     const double timestamp_sec,
                                     CorrectedImu *imu_msg) {
  if (!(imu1.header().has_timestamp_sec() &&
        imu2.header().has_timestamp_sec())) {
    AERROR << "imu1 and imu2 has no header or no timestamp_sec in header";
    return false;
  }
  if (timestamp_sec - imu1.header().timestamp_sec() <
      std::numeric_limits<double>::min()) {
    AERROR << "[InterpolateIMU1]: the given time stamp[" << timestamp_sec
           << "] is older than the 1st message["
           << imu1.header().timestamp_sec() << "]";
    *imu_msg = imu1;
  } else if (timestamp_sec - imu2.header().timestamp_sec() >
             std::numeric_limits<double>::min()) {
    AERROR << "[InterpolateIMU2]: the given time stamp[" << timestamp_sec
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

      if (imu1.imu().has_angular_velocity() &&
          imu2.imu().has_angular_velocity()) {
        auto val = InterpolateXYZ(imu1.imu().angular_velocity(),
                                  imu2.imu().angular_velocity(), frac1);
        imu_msg->mutable_imu()->mutable_angular_velocity()->CopyFrom(val);
      }

      if (imu1.imu().has_linear_acceleration() &&
          imu2.imu().has_linear_acceleration()) {
        auto val = InterpolateXYZ(imu1.imu().linear_acceleration(),
                                  imu2.imu().linear_acceleration(), frac1);
        imu_msg->mutable_imu()->mutable_linear_acceleration()->CopyFrom(val);
      }

      if (imu1.imu().has_euler_angles() && imu2.imu().has_euler_angles()) {
        auto val = InterpolateXYZ(imu1.imu().euler_angles(),
                                  imu2.imu().euler_angles(), frac1);
        imu_msg->mutable_imu()->mutable_euler_angles()->CopyFrom(val);
      }
    }
  }
  return true;
}

template <class T>
T RTKLocalization::InterpolateXYZ(const T &p1, const T &p2,
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

bool RTKLocalization::FindNearestGpsStatus(const double gps_timestamp_sec,
                                           drivers::gnss::InsStat *status) {
  CHECK_NOTNULL(status);

  std::unique_lock<std::mutex> lock(gps_status_list_mutex_);
  auto gps_status_list = gps_status_list_;
  lock.unlock();

  double timestamp_diff_sec = 1e8;
  auto nearest_itr = gps_status_list.end();
  for (auto itr = gps_status_list.begin(); itr != gps_status_list.end();
       ++itr) {
    double diff = std::abs(itr->header().timestamp_sec() - gps_timestamp_sec);
    if (diff < timestamp_diff_sec) {
      timestamp_diff_sec = diff;
      nearest_itr = itr;
    }
  }

  if (nearest_itr == gps_status_list.end()) {
    return false;
  }

  if (timestamp_diff_sec > gps_status_time_diff_threshold_) {
    return false;
  }

  *status = *nearest_itr;
  return true;
}

}  // namespace localization
}  // namespace apollo
