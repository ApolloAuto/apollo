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

#include "modules/third_party_perception/third_party_perception_component.h"

#include "modules/common/adapters/adapter_gflags.h"

DECLARE_string(flagfile);

namespace apollo {
namespace third_party_perception {

bool ThirdPartyPerceptionComponent::Init() {
  if (!perception_.Init().ok()) {
    return false;
  }

  writer_ = node_->CreateWriter<apollo::perception::PerceptionObstacles>(
      FLAGS_perception_obstacle_topic);

  std::shared_ptr<Reader<apollo::drivers::Mobileye>> mobileye_reader_ = nullptr;
  std::shared_ptr<Reader<apollo::drivers::DelphiESR>> delphi_esr_reader_ =
      nullptr;
  std::shared_ptr<Reader<apollo::drivers::ContiRadar>> conti_radar_reader_ =
      nullptr;

  mobileye_reader_ = node_->CreateReader<apollo::drivers::Mobileye>(
      FLAGS_mobileye_topic,
      [this](const std::shared_ptr<apollo::drivers::Mobileye> &message) {
        perception_.OnMobileye(*message.get());
      });

  delphi_esr_reader_ = node_->CreateReader<apollo::drivers::DelphiESR>(
      FLAGS_delphi_esr_topic,
      [this](const std::shared_ptr<apollo::drivers::DelphiESR> &message) {
        perception_.OnDelphiESR(*message.get());
      });

  conti_radar_reader_ = node_->CreateReader<apollo::drivers::ContiRadar>(
      FLAGS_conti_radar_topic,
      [this](const std::shared_ptr<apollo::drivers::ContiRadar> &message) {
        perception_.OnContiRadar(*message.get());
      });

  localization_reader_ =
      node_->CreateReader<apollo::localization::LocalizationEstimate>(
          FLAGS_localization_topic,
          [this](
              const std::shared_ptr<apollo::localization::LocalizationEstimate>
                  &localization) {
            perception_.OnLocalization(*localization.get());
          });

  chassis_reader_ = node_->CreateReader<apollo::canbus::Chassis>(
      FLAGS_chassis_topic,
      [this](const std::shared_ptr<apollo::canbus::Chassis> &chassis) {
        perception_.OnChassis(*chassis.get());
      });

  return perception_.Start().ok();
}

bool ThirdPartyPerceptionComponent::Proc() {
  auto response = std::make_shared<apollo::perception::PerceptionObstacles>();
  if (!perception_.Process(response.get())) {
    return false;
  }
  writer_->Write(response);
  return true;
}

}  // namespace third_party_perception
}  // namespace apollo
