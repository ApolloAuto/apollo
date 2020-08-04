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

#include "modules/audio/common/audio_info.h"

#include "modules/audio/common/audio_gflags.h"

namespace apollo {
namespace audio {

using apollo::drivers::microphone::config::AudioData;
using apollo::drivers::microphone::config::ChannelData;
using apollo::drivers::microphone::config::ChannelType;
using apollo::drivers::microphone::config::MicrophoneConfig;

void AudioInfo::Init(const MicrophoneConfig& microphone_config) {
  microphone_config_ = microphone_config;
}

void AudioInfo::Insert(const std::shared_ptr<AudioData>& audio_data) {
  std::size_t index = 0;
  for (const auto& channel_data : audio_data->channel_data()) {
    if (channel_data.channel_type() == ChannelType::RAW) {
      InsertChannelData(index, channel_data);
      ++index;
    }
  }
}

void AudioInfo::InsertChannelData(
    const std::size_t index, const ChannelData& channel_data) {
  while (index >= signals_.size()) {
    signals_.push_back(std::deque<float>());
  }
  int width = microphone_config_.sample_width();
  const std::string& data = channel_data.data();
  for (int i = 0; i + 1 < channel_data.size(); i += width) {
    uint16_t signal = ((data[i] - '0') << 8) + (data[i + 1] - '0');
    signals_[index].push_back(static_cast<float>(signal));
  }
  std::size_t max_signal_length = static_cast<std::size_t>(
      FLAGS_cache_signal_time * microphone_config_.sample_rate());
  while (signals_[index].size() > max_signal_length) {
    signals_[index].pop_front();
  }
}

std::vector<std::vector<float>> AudioInfo::GetSignals(const int signal_length) {
  std::vector<std::vector<float>> signals;
  // TODO(all): implement
  return signals;
}

}  // namespace audio
}  // namespace apollo
