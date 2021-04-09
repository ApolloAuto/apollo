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
#include "modules/third_party_perception/common/third_party_perception_gflags.h"
#include "modules/third_party_perception/common/third_party_perception_util.h"
#include "modules/third_party_perception/conversion.h"
#include "modules/third_party_perception/filter.h"
#include "modules/third_party_perception/fusion.h"
#include "ros/include/ros/ros.h"

namespace apollo {
namespace third_party_perception {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::drivers::ContiRadar;
using apollo::drivers::DelphiESR;
using apollo::drivers::Esr_track01_500;
using apollo::drivers::Mobileye;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using apollo::perception::Point;

std::string ThirdPartyPerception::Name() const { return FLAGS_module_name; }

Status ThirdPartyPerception::Init() {
  AdapterManager::Init(FLAGS_adapter_config_filename);

  CHECK(AdapterManager::GetMobileye()) << "Mobileye is not initialized.";
  AdapterManager::AddMobileyeCallback(&ThirdPartyPerception::OnMobileye, this);
  CHECK(AdapterManager::GetLocalization())
      << "Localization is not initialized.";
  AdapterManager::AddLocalizationCallback(&ThirdPartyPerception::OnLocalization,
                                          this);
  CHECK(AdapterManager::GetChassis()) << "Chassis is not initialized.";
  AdapterManager::AddChassisCallback(&ThirdPartyPerception::OnChassis, this);

  // TODO(all) : need to merge the delphi/conti_radar before adaptor manager
  // level. This is just temperary change.
  if (!FLAGS_use_conti_radar) {
    CHECK(AdapterManager::GetDelphiESR()) << "DelphiESR is not initialized.";
    AdapterManager::AddDelphiESRCallback(&ThirdPartyPerception::OnDelphiESR,
                                         this);
  } else {
    CHECK(AdapterManager::GetContiRadar())
        << "GetContiRadar is not initialized.";
    AdapterManager::AddContiRadarCallback(&ThirdPartyPerception::OnContiRadar,
                                          this);
  }
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
  ADEBUG << "Received mobileye data: run mobileye callback.";
  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);
  if (FLAGS_enable_mobileye) {
    mobileye_obstacles_ = conversion::MobileyeToPerceptionObstacles(
        message, localization_, chassis_);
  }
}

void ThirdPartyPerception::OnChassis(const Chassis& message) {
  ADEBUG << "Received chassis data: run chassis callback.";
  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);
  chassis_.CopyFrom(message);
}

void ThirdPartyPerception::OnDelphiESR(const DelphiESR& message) {
  ADEBUG << "Received delphi esr data: run delphi esr callback.";
  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);
  last_radar_obstacles_.CopyFrom(current_radar_obstacles_);
  current_radar_obstacles_ = conversion::DelphiToRadarObstacles(
      message, localization_, last_radar_obstacles_);
  RadarObstacles filtered_radar_obstacles =
      filter::FilterRadarObstacles(current_radar_obstacles_);
  if (FLAGS_enable_radar) {
    radar_obstacles_ = conversion::RadarObstaclesToPerceptionObstacles(
        filtered_radar_obstacles);
  }
}

void ThirdPartyPerception::OnContiRadar(const ContiRadar& message) {
  ADEBUG << "Received delphi esr data: run continental radar callback.";
  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);
  last_radar_obstacles_.CopyFrom(current_radar_obstacles_);
  current_radar_obstacles_ = conversion::ContiToRadarObstacles(
      message, localization_, last_radar_obstacles_, chassis_);
  RadarObstacles filtered_radar_obstacles =
      filter::FilterRadarObstacles(current_radar_obstacles_);
  if (FLAGS_enable_radar) {
    radar_obstacles_ = conversion::RadarObstaclesToPerceptionObstacles(
        filtered_radar_obstacles);
  }
}

void ThirdPartyPerception::OnLocalization(const LocalizationEstimate& message) {
  ADEBUG << "Received localization data: run localization callback.";
  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);
  localization_.CopyFrom(message);
}

void ThirdPartyPerception::OnTimer(const ros::TimerEvent&) {
  ADEBUG << "Timer is triggered: publish PerceptionObstacles";

  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);

  PerceptionObstacles obstacles =
      fusion::MobileyeRadarFusion(mobileye_obstacles_, radar_obstacles_);

  AdapterManager::FillPerceptionObstaclesHeader(FLAGS_node_name, &obstacles);
  AdapterManager::PublishPerceptionObstacles(obstacles);

  mobileye_obstacles_.Clear();
  radar_obstacles_.Clear();
}

}  // namespace third_party_perception
}  // namespace apollo
