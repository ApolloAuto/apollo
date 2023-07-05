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

#include "modules/audio/common/message_process.h"

namespace apollo {
namespace audio {

using apollo::drivers::microphone::config::AudioData;

void MessageProcess::OnMicrophone(
    const AudioData& audio_data,
    const std::string& respeaker_extrinsics_file,
    AudioInfo* audio_info,
    DirectionDetection* direction_detection,
    MovingDetection* moving_detection,
    SirenDetection* siren_detection,
    AudioDetection* audio_detection) {
  audio_info->Insert(audio_data);
  auto direction_result =
      direction_detection->EstimateSoundSource(
          audio_info->GetSignals(audio_data.microphone_config().chunk()),
          respeaker_extrinsics_file,
          audio_data.microphone_config().sample_rate(),
          audio_data.microphone_config().mic_distance());
  *(audio_detection->mutable_position()) = direction_result.first;
  audio_detection->set_source_degree(direction_result.second);

  bool is_siren = siren_detection->Evaluate(audio_info->GetSignals(72000));
  audio_detection->set_is_siren(is_siren);
  auto signals =
      audio_info->GetSignals(audio_data.microphone_config().chunk());
  MovingResult moving_result = moving_detection->Detect(signals);
  audio_detection->set_moving_result(moving_result);
}

}  // namespace audio
}  // namespace apollo
