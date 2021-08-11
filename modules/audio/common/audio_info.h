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

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "modules/drivers/microphone/proto/audio.pb.h"
#include "modules/drivers/microphone/proto/microphone_config.pb.h"

namespace apollo {
namespace audio {

class AudioInfo {
 public:
  AudioInfo() = default;

  void Insert(
      const apollo::drivers::microphone::config::AudioData&);

  std::vector<std::vector<double>> GetSignals(const int signal_length);

 private:
  void InsertChannelData(
      const std::size_t index,
      const apollo::drivers::microphone::config::ChannelData& channel_data,
      const apollo::drivers::microphone::config::MicrophoneConfig&
          microphone_config);

  std::vector<std::deque<double>> signals_;
};

}  // namespace audio
}  // namespace apollo
