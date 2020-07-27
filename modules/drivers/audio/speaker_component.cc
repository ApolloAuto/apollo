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
  speaker_config_ptr_ = std::make_shared<SpeakerConfig>();
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               speaker_config_ptr_.get())) {
    return false;
  }
  AINFO << "Speaker config: " << speaker_config_ptr_->DebugString();

  // new speaker device
  speaker_device_ptr_.reset(new Respeaker());
  speaker_device_ptr_->init(speaker_config_ptr_);

  // dump config, calculate size and reserve buffer for audio
  // chunk_: number of frames per chunk; chunk_size_: number of bytes per chunk
  n_channels_ = speaker_config_ptr_->channel_type_size();
  chunk_ = speaker_config_ptr_->chunk();
  total_frames_ = speaker_config_ptr_->sample_rate() *
                  speaker_config_ptr_->record_seconds();
  chunk_size_ = speaker_device_ptr_->stream_ptr_->get_chunk_size(chunk_);

  buffer_size_ = speaker_device_ptr_->stream_ptr_->get_chunk_size(total_frames_);
  buffer_ = reinterpret_cast<char *>(malloc(buffer_size_));
  if (buffer_ == nullptr) {
    AERROR << "System calloc memory error, size:" << buffer_size_;
    return false;
  }

  // assemble AudioData -- fill speaker_config, allocate memory for channel_data
  audio_data_ptr_.reset(new AudioData());
  std::string config;
  speaker_config_ptr_->SerializeToString(&config);
  audio_data_ptr_->mutable_speaker_config()->ParseFromString(config);
  audio_data_ptr_->mutable_header()->set_frame_id(
      speaker_config_ptr_->frame_id());

  ChannelData *channel_data = nullptr;
  int channel_size = buffer_size_ / n_channels_;
  for (int i = 0; i < n_channels_; ++i) {
    channel_data = audio_data_ptr_->add_channel_data();
    channel_data->set_channel_type(speaker_config_ptr_->channel_type(i));
    channel_data->set_size(channel_size);
    channel_data->mutable_data()->resize(channel_size);
    channel_data_ptrs_.push_back(channel_data->mutable_data());
  }

  writer_ptr_ =
      node_->CreateWriter<AudioData>(speaker_config_ptr_->channel_name());
  async_result_ = cyber::Async(&SpeakerComponent::run, this);
  return true;
}

void SpeakerComponent::run() {
  char *pos = nullptr;
  int i, seq = 0;
  while (!cyber::IsShutdown()) {
    AINFO << "Start recording";
    pos = buffer_;
    for (i = 1; chunk_ * i <= total_frames_; ++i) {
      speaker_device_ptr_->stream_ptr_->read_stream(chunk_, pos);
      pos += chunk_size_;
    }
    if (chunk_ * --i < total_frames_)
      speaker_device_ptr_->stream_ptr_->read_stream(total_frames_ - chunk_ * i,
                                                pos);
    AINFO << "End recording";

    for (int buff_i = 0, i = 0; buff_i < buffer_size_; i += 2) {
      for (int channel_i = 0; channel_i < n_channels_; ++channel_i) {
        (*channel_data_ptrs_[channel_i])[i] = buffer_[buff_i++];
        (*channel_data_ptrs_[channel_i])[i + 1] = buffer_[buff_i++];
      }
    }
    audio_data_ptr_->mutable_header()->set_sequence_num(seq++);
    audio_data_ptr_->mutable_header()->set_timestamp_sec(
        cyber::Time().Now().ToSecond());
    writer_ptr_->Write(audio_data_ptr_);
  }
}

SpeakerComponent::~SpeakerComponent() {
  free(buffer_);
  if (running_.load()) {
    running_.exchange(false);
    async_result_.wait();
  }
}

}  // namespace audio
}  // namespace drivers
}  // namespace apollo
