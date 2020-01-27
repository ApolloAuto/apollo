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

#include "modules/planning/pipeline/feature_generator.h"

#include <string>

#include "cyber/common/file.h"
#include "cyber/record/record_reader.h"
#include "modules/common/adapters/adapter_gflags.h"


namespace apollo {
namespace planning {

using apollo::canbus::Chassis;
using apollo::cyber::record::RecordMessage;
using apollo::cyber::record::RecordReader;
using apollo::localization::LocalizationEstimate;

void FeatureGenerator::Init() {
}

void FeatureGenerator::Close() {
}

void FeatureGenerator::OnLocalization(
  const apollo::localization::LocalizationEstimate& le) {
    auto features = instance_.mutable_localization_feature();
    const auto& pose = le.pose();
    features->mutable_position()->CopyFrom(pose.position());
    features->set_heading(pose.heading());
    features->mutable_linear_velocity()->CopyFrom(pose.linear_velocity());
    features->mutable_linear_acceleration()->CopyFrom(
        pose.linear_acceleration());
    features->mutable_angular_velocity()->CopyFrom(pose.angular_velocity());
}

void FeatureGenerator::OnChassis(const apollo::canbus::Chassis& chassis) {
  auto features = instance_.mutable_chassis_feature();
  features->set_speed_mps(chassis.speed_mps());
  features->set_throttle_percentage(chassis.throttle_percentage());
  features->set_brake_percentage(chassis.brake_percentage());
  features->set_steering_percentage(chassis.steering_percentage());
  features->set_gear_location(chassis.gear_location());
}

void FeatureGenerator::ProcessOfflineData(const std::string& record_filename) {
  RecordReader reader(record_filename);
  if (!reader.IsValid()) {
    AERROR << "Fail to open " << record_filename;
    return;
  }

  RecordMessage message;
  while (reader.ReadMessage(&message)) {
    if (message.channel_name == FLAGS_localization_topic) {
      LocalizationEstimate localization;
      if (localization.ParseFromString(message.content)) {
        OnLocalization(localization);
      }
    } else if (message.channel_name == FLAGS_chassis_topic) {
      Chassis chassis;
      if (chassis.ParseFromString(message.content)) {
        OnChassis(chassis);
      }
    }
  }
}

}  // namespace planning
}  // namespace apollo

