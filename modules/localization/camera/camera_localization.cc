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

#include "modules/localization/camera/camera_localization.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using ::Eigen::Vector3d;
using ::apollo::common::adapter::AdapterManager;
using ::apollo::common::adapter::ImuAdapter;
using ::apollo::common::monitor::MonitorMessageItem;
using ::apollo::common::monitor::MonitorBuffer;
using ::apollo::common::Status;
using ::apollo::common::time::Clock;

CameraLocalization::CameraLocalization()
    : monitor_(MonitorMessageItem::LOCALIZATION) {}

Status CameraLocalization::Start() {
  MonitorBuffer buffer(&monitor_);
  if (!apollo::common::util::GetProtoFromFile(
          FLAGS_camera_parameter_config_file, &camera_parameter_)) {
    buffer.ERROR() << "Camera parameter is not initialized. Check "
                   << FLAGS_camera_parameter_config_file;
    buffer.PrintLog();
    return Status(apollo::common::LOCALIZATION_ERROR,
                  "failed to load camera parameter");
  }

  AdapterManager::Init(FLAGS_camera_adapter_config_file);
  // start ROS timer, one-shot = false, auto-start = true
  const double duration = 1.0 / FLAGS_localization_publish_freq;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &CameraLocalization::OnTimer, this);
  if (!AdapterManager::GetGps()) {
    buffer.ERROR() << "GPS input not initialized. Check "
                   << FLAGS_camera_adapter_config_file;
    buffer.PrintLog();
    return Status(apollo::common::LOCALIZATION_ERROR, "no GPS adapter");
  }
  if (!AdapterManager::GetCamera()) {
    buffer.ERROR() << "Camera input not initialized. Check "
                   << FLAGS_camera_adapter_config_file;
    buffer.PrintLog();
    return Status(apollo::common::LOCALIZATION_ERROR, "no Camera adapter");
  }
  // IMU is optional
  if (!AdapterManager::GetImu()) {
    buffer.INFO("IMU input not initialized. Check your adapter.conf file!");
    buffer.PrintLog();
    use_imu_ = false;
  } else {
    use_imu_ = true;
  }
  return Status::OK();
}

Status CameraLocalization::Stop() {
  timer_.stop();
  return Status::OK();
}

void CameraLocalization::OnTimer(const ros::TimerEvent &event) {
  double time_delay = apollo::common::time::ToSecond(Clock::Now()) -
                      last_received_timestamp_sec_;
  MonitorBuffer buffer(&monitor_);
  if (FLAGS_enable_gps_timestamp &&
      time_delay > FLAGS_gps_time_delay_tolerance) {
    buffer.ERROR() << "GPS message time delay: " << time_delay;
    buffer.PrintLog();
  }
  if (FLAGS_enable_camera_timestamp &&
      time_delay > FLAGS_camera_time_delay_tolerance) {
    AERROR << "Camera message time delay: " << time_delay;
    buffer.ERROR() << "Camera message time delay: " << time_delay;
    buffer.PrintLog();
  }

  // Take a snapshot of the current received messages.
  AdapterManager::Observe();

  if (AdapterManager::GetGps()->Empty()) {
    AERROR << "GPS message buffer is empty.";
    if (service_started_) {
      buffer.ERROR("GPS message buffer is empty.");
    }
    return;
  }
  if (AdapterManager::GetCamera()->Empty()) {
    AERROR << "Camera message buffer is empty.";
    if (service_started_) {
      buffer.ERROR("Camera message buffer is empty.");
    }
    return;
  }
  if (use_imu_ && AdapterManager::GetImu()->Empty()) {
    AERROR << "Imu message buffer is empty.";
    if (service_started_) {
      buffer.ERROR("Imu message buffer is empty.");
    }
    return;
  }

  // publish localization messages
  if (!PublishLocalization()) {
    buffer.ERROR("Publish localization failed");
    buffer.PrintLog();
    return;
  }
  service_started_ = true;

  // watch dog
  RunWatchDog();

  last_received_timestamp_sec_ = apollo::common::time::ToSecond(Clock::Now());
}

// TODO(Dong): Implement this function for camera based high accuracy
// localization.
bool CameraLocalization::CreateLocalizationMsg(
    const ::apollo::localization::Gps &gps_msg,
    const ::apollo::localization::Camera &camera_msg,
    LocalizationEstimate *localization) {
  localization->Clear();

  // header
  AdapterManager::FillLocalizationHeader(FLAGS_localization_module_name,
                                         localization->mutable_header());
  if (FLAGS_enable_gps_timestamp) {
    // copy time stamp, do NOT use Clock::Now()
    localization->mutable_header()->set_timestamp_sec(
        gps_msg.header().timestamp_sec());
  }

  if (!gps_msg.has_localization()) {
    AERROR << "GPS has no localization field";
    return false;
  }

  if (!camera_msg.has_image() || !camera_msg.has_width() ||
      !camera_msg.has_height()) {
    AERROR << "camera data is invalid";
    return false;
  }

  // TODO(Dong): Get high accuracy position information from gps and camera
  // image.
  // const auto &gps_location = gps_msg.localization();
  // const auto &image = camera_msg.image();
  // const auto image_width = camera_msg.width();
  // const auto image_height = camera_msg.height();
  return true;
}

bool CameraLocalization::PublishLocalization() {
  LocalizationEstimate localization;
  const auto &gps_msg = AdapterManager::GetGps()->GetLatestObserved();
  const auto &camera_msg = AdapterManager::GetCamera()->GetLatestObserved();
  // TODO(Dong): Implement how to apply camera parameter to camera data.
  //
  // TODO(Dong): Find the timestamp difference in GPS message and Camera
  // message, do time alignment if necessary.
  //
  // TODO(Dong): Produce localization message based on GPS and Camera Data.
  if (!CreateLocalizationMsg(gps_msg, camera_msg, &localization)) {
    AERROR << "Failed to compose localization message";
    return false;
  }

  // publish localization messages
  AdapterManager::PublishLocalization(localization);
  AINFO << "[OnTimer]: Localization message publish success!";
  return true;
}

void CameraLocalization::RunWatchDog() {
  if (!FLAGS_enable_watchdog) {
    return;
  }

  bool msg_lost = false;

  MonitorBuffer buffer(&monitor_);

  // check GPS time stamp against ROS timer
  double gps_delay_sec =
      apollo::common::time::ToSecond(Clock::Now()) -
      AdapterManager::GetGps()->GetLatestObserved().header().timestamp_sec();
  int64_t gps_delay_cycle_cnt =
      static_cast<int64_t>(gps_delay_sec * FLAGS_localization_publish_freq);
  if (FLAGS_enable_gps_timestamp &&
      (gps_delay_cycle_cnt > FLAGS_report_threshold_err_num)) {
    msg_lost = true;

    buffer.ERROR() << "Raw GPS Message Lost. GPS message is "
                   << gps_delay_cycle_cnt << " cycle " << gps_delay_sec
                   << " sec behind current time.";
    buffer.PrintLog();
  }

  // check Camera time stamp against ROS timer
  double camera_delay_sec =
      apollo::common::time::ToSecond(Clock::Now()) -
      AdapterManager::GetCamera()->GetLatestObserved().header().timestamp_sec();
  int64_t camera_delay_cycle_cnt =
      static_cast<int64_t>(camera_delay_sec * FLAGS_localization_publish_freq);
  if (FLAGS_enable_gps_timestamp &&
      camera_delay_cycle_cnt > FLAGS_report_threshold_err_num) {
    msg_lost = true;

    buffer.ERROR() << "Raw Camera Message Lost. IMU message is "
                   << camera_delay_cycle_cnt << " cycle " << camera_delay_sec
                   << " sec behind current time.";
    buffer.PrintLog();
  }

  // to prevent it from beeping continuously
  if (msg_lost && (last_reported_timestamp_sec_ < 1. ||
                   apollo::common::time::ToSecond(Clock::Now()) >
                       last_reported_timestamp_sec_ + 1.)) {
    AERROR << "gps/camera frame lost!";
    last_reported_timestamp_sec_ = apollo::common::time::ToSecond(Clock::Now());
  }
}

}  // namespace localization
}  // namespace apollo
