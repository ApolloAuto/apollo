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

#include <atomic>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "modules/common/util/message_util.h"
#include "modules/drivers/audio/proto/audio.pb.h"
#include "modules/drivers/audio/proto/speaker_config.pb.h"
#include "modules/drivers/audio/respeaker.h"

namespace apollo {
namespace drivers {
namespace audio {

using apollo::common::util::FillHeader;
using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::audio::config::AudioData;
using apollo::drivers::audio::config::ChannelData;
using apollo::drivers::audio::config::SpeakerConfig;

class SpeakerComponent : public Component<> {
 public:
  bool Init() override;
  ~SpeakerComponent();

 private:
  void run();

  // Configuration
  int n_chunks_, n_channels_, chunk_, chunk_size_, buffer_size_, total_frames_;

  // class data
  std::shared_ptr<AudioData> audio_data_ptr_;
  std::shared_ptr<Writer<AudioData>> writer_ptr_;
  std::unique_ptr<Respeaker> speaker_device_ptr_;
  std::vector<std::string *> channel_data_ptrs_;
  std::shared_ptr<SpeakerConfig> speaker_config_ptr_;
  char *buffer_;

  std::future<void> async_result_;
  std::atomic<bool> running_ = {false};
};

CYBER_REGISTER_COMPONENT(SpeakerComponent)
}  // namespace audio
}  // namespace drivers
}  // namespace apollo
