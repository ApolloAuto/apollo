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

#include "modules/drivers/audio/speaker_component.h"

namespace apollo {
namespace drivers {
namespace audio {

bool SpeakerComponent::Init() {
  speaker_config_ = std::make_shared<Config>();
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               camera_config_.get())) {
    return false;
  }
  AINFO << "Speaker config: " << speaker_config_->DebugString();

  // dump config
  n_channels_ = speaker_config_->channel_config_size();
  sample_rate_ = speaker_config_->sample_rate;
  sample_width_ = speaker_config_->sample_width;
  record_seconds_ = speaker_config_->record_seconds;
  chunk_ = speaker_config_->chunk;

  // new speaker device
  speaker_device_.reset(new Respeaker());
  speaker_device_->init(sample_rate_, sample_width_, n_channels_);

  // calculate size and reserve buffer for audio
  chunk_size_ = speaker_device_->stream->get_chunk_size(chunk_);
  n_chunks_ = sample_rate_ * 1.0 / chunk_size_ * record_seconds_;
  buffer_size_ = n_chunks_ * chunk_size_;
  buffer_ = reinterpret_cast<char *> malloc(buffer_size);
  if (buffer_ == nullptr) {
    AERROR << "System calloc memory error, size:" << num_bytes;
    return false;
  }

  // std::unique_ptr<ChannelData> channel_data = nullptr;
  ChannelData *channel_data = nullptr;
  for (int i = 0; i < n_channels_; ++i) {
    channel_data = audio_data_->add_channel_data();
    channel_data->set_channel_type(speaker_config_->channel_type(i));
    channel_data->set_size(sample_width_);
    audio_data->mutable_data()->resize(buffer_size_ / n_channels_);
  }

  writer_ = node_->CreateWriter<AudioData>(speaker_config_->channel_name());

  writer_ = node_->CreateWriter<Image>(camera_config_->channel_name());
  async_result_ = cyber::Async(&CameraComponent::run, this);
  return true;
}

void CameraComponent::run() {
  char *pos = nullptr;
  while (!cyber::IsShutdown()) {  // TODO: or apollo::cyber::OK() ?
    AINFO << "Start recording";
    pos = buffer_
    for (int i = 0; i < n_chunks; ++i) {
      speaker_device_->stream->read_stream(chunk_size_, pos);
      pos += chunk_size_;
    }
    AINFO << "End recording";
    for (int buff_i = 0; buff_i < buffer_size_; ++buff_i)
      for (int channel_i = 0; channel_i < n_channels_; ++channel_i) {
        audio_data_->channel_data(channel_i).data[i] = buffer_[buff_i++];
        audio_data_->channel_data(channel_i).data[i] = buffer_[buff_i++];
      }
  }
  writer_->Write(audio_data_);
}

}  // namespace audio
}  // namespace drivers
}  // namespace apollo
