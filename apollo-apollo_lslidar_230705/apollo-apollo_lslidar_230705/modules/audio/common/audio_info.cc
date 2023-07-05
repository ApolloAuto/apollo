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

#include <algorithm>

#include "modules/audio/common/audio_gflags.h"

namespace apollo {
namespace audio {

using apollo::drivers::microphone::config::AudioData;
using apollo::drivers::microphone::config::ChannelData;
using apollo::drivers::microphone::config::ChannelType;
using apollo::drivers::microphone::config::MicrophoneConfig;

void AudioInfo::Insert(const AudioData& audio_data) {
  std::size_t index = 0;
  for (const auto& channel_data : audio_data.channel_data()) {
    if (channel_data.channel_type() == ChannelType::RAW) {
      InsertChannelData(index, channel_data, audio_data.microphone_config());
      ++index;
    }
  }
}

void AudioInfo::InsertChannelData(const std::size_t index,
                                  const ChannelData& channel_data,
                                  const MicrophoneConfig& microphone_config) {
  while (index >= signals_.size()) {
    signals_.push_back(std::deque<double>());
  }
  int width = microphone_config.sample_width();
  const std::string& data = channel_data.data();
  for (std::size_t i = 0; i < data.length(); i += width) {
    int16_t signal = ((int16_t(data[i + 1])) << 8) | (0x00ff & data[i]);
    signals_[index].push_back(static_cast<double>(signal));
  }
  std::size_t max_signal_length = static_cast<std::size_t>(
      FLAGS_cache_signal_time * microphone_config.sample_rate());
  while (signals_[index].size() > max_signal_length) {
    signals_[index].pop_front();
  }
}

std::vector<std::vector<double>> AudioInfo::GetSignals(
    const int signal_length) {
  std::vector<std::vector<double>> signals;
  for (std::size_t i = 0; i < signals_.size(); ++i) {
    int start_index = static_cast<int>(signals_[i].size()) - signal_length;
    start_index = std::max(0, start_index);
    std::deque<double>::iterator iter = signals_[i].begin();
    iter += start_index;
    std::vector<double> signal(iter, signals_[i].end());
    signals.push_back(signal);
  }
  return signals;
}

}  // namespace audio
}  // namespace apollo
