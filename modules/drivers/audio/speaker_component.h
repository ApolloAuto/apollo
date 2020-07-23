/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/audio/proto/speaker_config.pb.h"
#include "modules/drivers/audio/proto/audio.pb.h"
#include "modules/drivers/audio/respeaker.h"
#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace audio {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::audio::proto::AudioData;  // check if it is package
                                                 // name(proto) / lib name:
                                                 // respeaker_config
using apollo::drivers::audio::proto::ChannelData;
using apollo::drivers::audio::proto::SpeakerConfig;

class SpeakerComponent : public Component<> {
 public:
  bool Init() override;
  ~SpeakerComponent();

 private:
  void run();

  // Configuration
  int sample_rate_, n_channels_, sample_width_, record_seconds_, chunk_;

  // class data
  std::shared_ptr<Writer<AudioData>> writer_;
  std::unique_ptr<Respeaker> speaker_device_;
  std::shared_ptr<SpeakerConfig> speaker_config_;
  std::shared_ptr<AudioData> audio_data_;
  std::vector<std::unique_ptr<ChannelData>>* channels_;
  char *buffer_;
  int n_chunks_, chunk_size_, buffer_size_;
};

CYBER_REGISTER_COMPONENT(SpeakerComponent)
}  // namespace audio
}  // namespace drivers
}  // namespace apollo
