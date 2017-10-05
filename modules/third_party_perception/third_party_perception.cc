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
#include "modules/third_party_perception/third_party_perception.h"

#include <cmath>

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/third_party_perception/conversion.h"
#include "modules/third_party_perception/fusion.h"
#include "modules/third_party_perception/common/third_party_perception_gflags.h"
#include "modules/third_party_perception/common/third_party_perception_util.h"
#include "ros/include/ros/ros.h"

namespace apollo {
namespace third_party_perception {

using apollo::common::adapter::AdapterManager;
using apollo::common::Status;
using apollo::common::ErrorCode;
using apollo::drivers::Mobileye;
using apollo::drivers::DelphiESR;
using apollo::drivers::Esr_track01_500;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacles;
using apollo::perception::PerceptionObstacle;
using apollo::perception::Point;

std::string ThirdPartyPerception::Name() const { return FLAGS_hmi_name; }

Status ThirdPartyPerception::Init() {
  AdapterManager::Init(FLAGS_adapter_config_filename);

  CHECK(AdapterManager::GetMobileye()) << "Mobileye is not initialized.";
  AdapterManager::AddMobileyeCallback(&ThirdPartyPerception::OnMobileye, this);
  CHECK(AdapterManager::GetDelphiESR()) << "DelphiESR is not initialized.";
  AdapterManager::AddDelphiESRCallback(&ThirdPartyPerception::OnDelphiESR, this);
  CHECK(AdapterManager::GetLocalization())
      << "Localization is not initialized.";
  AdapterManager::AddLocalizationCallback(&ThirdPartyPerception::OnLocalization, this);

  return Status::OK();
}

Status ThirdPartyPerception::Start() {
  const double duration = 1.0 / FLAGS_third_party_perception_freq;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &ThirdPartyPerception::OnTimer, this);

  return Status::OK();
}

void ThirdPartyPerception::Stop() { timer_.stop(); }

void ThirdPartyPerception::OnMobileye(const Mobileye& message) {
  AINFO << "receive Mobileye callback";
  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);
  mobileye_obstacles_ =
      conversion::MobileyeToPerceptionObstacles(message, localization_);
}

void ThirdPartyPerception::OnDelphiESR(const DelphiESR& message) {
  AINFO << "receive DelphiESR callback";
  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);
  last_radar_obstacles_.CopyFrom(current_radar_obstacles_);
  current_radar_obstacles_.Clear();
  current_radar_obstacles_ = conversion::DelphiToRadarObstacles(
      message, localization_, last_radar_obstacles_);
}

void ThirdPartyPerception::OnLocalization(const LocalizationEstimate& message) {
  AINFO << "receive Localization callback";
  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);
  localization_.CopyFrom(message);
}

bool IsPreserved(const RadarObstacle& radar_obstacle) {
  /*
  if (sqrt(perception_obstacle.velocity().x() *
               perception_obstacle.velocity().x() +
           perception_obstacle.velocity().y() *
               perception_obstacle.velocity().y()) < 5.0) {
    return false;
  }
  if (esr_track01_500.can_tx_track_range() > 40.0) {
    return false;
  }
  if (esr_track01_500.can_tx_track_angle() > 25.0 ||
      esr_track01_500.can_tx_track_angle() < -25.0) {
    return false;
  }
  if (esr_track01_500.can_tx_track_status() ==
  ::apollo::drivers::Esr_track01_500::CAN_TX_TRACK_STATUS_NO_TARGET ||
    esr_track01_500.can_tx_track_status() ==
  ::apollo::drivers::Esr_track01_500::CAN_TX_TRACK_STATUS_COASTED_TARGET ) {
    return false;
  }
  if (radar_obstacle.rcs() < -1.0) {
    return false;
  }
*/
  if (!radar_obstacle.movable()) {
    return false;
  }
  // TODO(rongqiqiu): keep those with stable forward velocity
  double nearest_l =
      GetLateralDistanceToNearestLane(radar_obstacle.absolute_position());
  if (std::abs(nearest_l) > FLAGS_filter_y_distance) {
    return false;
  }
  // TODO(rongqiqiu): create a new gflag for this
  if (radar_obstacle.count() < FLAGS_keep_delphi_esr_frames) {
    return false;
  }
  return true;
}

RadarObstacles ThirdPartyPerception::FilterRadarObstacles(
    const RadarObstacles& radar_obstacles) {
  RadarObstacles filtered_radar_obstacles;
  for (const auto& iter : radar_obstacles.radar_obstacle()) {
    if (IsPreserved(iter.second)) {
      (*filtered_radar_obstacles.mutable_radar_obstacle())[iter.first] =
          iter.second;
    }
  }
  filtered_radar_obstacles.mutable_header()->CopyFrom(radar_obstacles.header());
  return filtered_radar_obstacles;
}

void ThirdPartyPerception::OnTimer(const ros::TimerEvent&) {
  AINFO << "publish PerceptionObstacles";

  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);

  RadarObstacles filtered_radar_obstacles =
      FilterRadarObstacles(current_radar_obstacles_);
  PerceptionObstacles filtered_delphi_esr_obstacles =
      conversion::RadarObstaclesToPerceptionObstacles(filtered_radar_obstacles);

  PerceptionObstacles obstacles = fusion::MobileyeRadarFusion(
      mobileye_obstacles_, filtered_delphi_esr_obstacles);

  AdapterManager::FillPerceptionObstaclesHeader(FLAGS_node_name, &obstacles);
  AdapterManager::PublishPerceptionObstacles(obstacles);

  mobileye_obstacles_.Clear();
}

}  // namespace third_party_perception
}  // namespace apollo
