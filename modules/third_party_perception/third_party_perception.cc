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

#include "modules/common/util/message_util.h"
#include "modules/third_party_perception/common/third_party_perception_gflags.h"
#include "modules/third_party_perception/conversion.h"
#include "modules/third_party_perception/filter.h"
#include "modules/third_party_perception/fusion.h"

namespace apollo {
namespace third_party_perception {

using apollo::canbus::Chassis;
using apollo::common::Status;
using apollo::drivers::ContiRadar;
using apollo::drivers::DelphiESR;
using apollo::drivers::Mobileye;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacles;

std::string ThirdPartyPerception::Name() const {
  return FLAGS_third_party_perception_node_name;
}

Status ThirdPartyPerception::Init() { return Status::OK(); }

Status ThirdPartyPerception::Start() { return Status::OK(); }

void ThirdPartyPerception::Stop() {}

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

bool ThirdPartyPerception::Process(PerceptionObstacles* const response) {
  ADEBUG << "Timer is triggered: publish PerceptionObstacles";
  CHECK_NOTNULL(response);

  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);

  *response =
      fusion::MobileyeRadarFusion(mobileye_obstacles_, radar_obstacles_);

  common::util::FillHeader(FLAGS_third_party_perception_node_name, response);

  mobileye_obstacles_.Clear();
  radar_obstacles_.Clear();
  return true;
}

}  // namespace third_party_perception
}  // namespace apollo
