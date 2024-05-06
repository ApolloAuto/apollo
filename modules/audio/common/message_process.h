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

#pragma once

#include <string>

#include "modules/audio/common/audio_info.h"
#include "modules/audio/inference/direction_detection.h"
#include "modules/audio/inference/moving_detection.h"
#include "modules/audio/inference/siren_detection.h"
#include "modules/common_msgs/audio_msgs/audio.pb.h"
#include "modules/drivers/microphone/proto/audio.pb.h"

namespace apollo {
namespace audio {

class MessageProcess {
 public:
  MessageProcess() = delete;

  static void OnMicrophone(
      const apollo::drivers::microphone::config::AudioData& audio_data,
      const std::string& respeaker_extrinsics_file,
      AudioInfo* audio_info,
      DirectionDetection* direction_detection,
      MovingDetection* moving_detection,
      SirenDetection* siren_detection,
      AudioDetection* audio_detection);
};

}  // namespace audio
}  // namespace apollo
