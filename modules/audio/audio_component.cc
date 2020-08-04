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

#include "modules/audio/audio_component.h"
#include "modules/audio/inference/direction_detection.h"
#include "modules/audio/proto/audio_conf.pb.h"

namespace apollo {
namespace audio {

using apollo::drivers::microphone::config::AudioData;

AudioComponent::~AudioComponent() {}

std::string AudioComponent::Name() const {
  // TODO(all) implement
  return "";
}

bool AudioComponent::Init() {
  AudioConf audio_conf;
  if (!ComponentBase::GetProtoConfig(&audio_conf)) {
    AERROR << "Unable to load audio conf file: "
           << ComponentBase::ConfigFilePath();
    return false;
  }
  localization_reader_ =
      node_->CreateReader<localization::LocalizationEstimate>(
          audio_conf.topic_conf().localization_topic_name(), nullptr);
  audio_writer_ = node_->CreateWriter<AudioDetection>(
      audio_conf.topic_conf().audio_detection_topic_name());
  return true;
}

bool AudioComponent::Proc(const std::shared_ptr<AudioData>& audio_data) {
  audio_info_.Insert(audio_data);
  AINFO << "Current direction is: "
        << get_direction(
               audio_info_.GetSignals(audio_data->microphone_config().chunk()),
               audio_data->microphone_config().sample_rate(),
               audio_data->microphone_config().mic_distance());
  return true;
}

}  // namespace audio
}  // namespace apollo
