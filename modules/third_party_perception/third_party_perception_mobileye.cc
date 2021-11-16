/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/third_party_perception/third_party_perception_mobileye.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/third_party_perception/common/third_party_perception_gflags.h"
#include "modules/third_party_perception/tools/conversion_mobileye.h"
#include "modules/third_party_perception/tools/conversion_radar.h"
#include "modules/third_party_perception/tools/filter.h"
#include "modules/third_party_perception/tools/fusion.h"

namespace apollo {
namespace third_party_perception {

using apollo::drivers::ContiRadar;
using apollo::drivers::DelphiESR;
using apollo::drivers::Mobileye;
using apollo::perception::PerceptionObstacles;

ThirdPartyPerceptionMobileye::ThirdPartyPerceptionMobileye(
    apollo::cyber::Node* const node)
    : ThirdPartyPerception(node) {
  mobileye_reader_ = node_->CreateReader<apollo::drivers::Mobileye>(
      FLAGS_mobileye_topic,
      [this](const std::shared_ptr<apollo::drivers::Mobileye>& message) {
        OnMobileye(*message.get());
      });
  delphi_esr_reader_ = node_->CreateReader<apollo::drivers::DelphiESR>(
      FLAGS_delphi_esr_topic,
      [this](const std::shared_ptr<apollo::drivers::DelphiESR>& message) {
        OnDelphiESR(*message.get());
      });

  conti_radar_reader_ = node_->CreateReader<apollo::drivers::ContiRadar>(
      FLAGS_conti_radar_topic,
      [this](const std::shared_ptr<apollo::drivers::ContiRadar>& message) {
        OnContiRadar(*message.get());
      });
}

void ThirdPartyPerceptionMobileye::OnMobileye(const Mobileye& message) {
  ADEBUG << "Received mobileye data: run mobileye callback.";
  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);
  eye_obstacles_ = conversion_mobileye::MobileyeToPerceptionObstacles(
      message, localization_, chassis_);
}

void ThirdPartyPerceptionMobileye::OnDelphiESR(const DelphiESR& message) {
  ADEBUG << "Received delphi esr data: run delphi esr callback.";
  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);
  last_radar_obstacles_.CopyFrom(current_radar_obstacles_);
  current_radar_obstacles_ = conversion_radar::DelphiToRadarObstacles(
      message, localization_, last_radar_obstacles_);
  RadarObstacles filtered_radar_obstacles =
      filter::FilterRadarObstacles(current_radar_obstacles_);
  if (FLAGS_enable_radar) {
    radar_obstacles_ = conversion_radar::RadarObstaclesToPerceptionObstacles(
        filtered_radar_obstacles);
  }
}

void ThirdPartyPerceptionMobileye::OnContiRadar(const ContiRadar& message) {
  ADEBUG << "Received delphi esr data: run continental radar callback.";
  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);
  last_radar_obstacles_.CopyFrom(current_radar_obstacles_);
  current_radar_obstacles_ = conversion_radar::ContiToRadarObstacles(
      message, localization_, last_radar_obstacles_, chassis_);
  RadarObstacles filtered_radar_obstacles =
      filter::FilterRadarObstacles(current_radar_obstacles_);
  if (FLAGS_enable_radar) {
    radar_obstacles_ = conversion_radar::RadarObstaclesToPerceptionObstacles(
        filtered_radar_obstacles);
  }
}

bool ThirdPartyPerceptionMobileye::Process(
    PerceptionObstacles* const response) {
  ADEBUG << "Timer is triggered: publish PerceptionObstacles";
  CHECK_NOTNULL(response);

  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);

  *response = fusion::EyeRadarFusion(eye_obstacles_, radar_obstacles_);

  common::util::FillHeader(FLAGS_third_party_perception_node_name, response);

  eye_obstacles_.Clear();
  radar_obstacles_.Clear();
  return true;
}

}  // namespace third_party_perception
}  // namespace apollo
