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

}

void FeatureGenerator::OnChassis(const apollo::canbus::Chassis& chassis) {

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

