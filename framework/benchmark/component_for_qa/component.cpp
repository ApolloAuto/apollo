/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include <chrono>

#include "component.h"

SenderComponent::SenderComponent() {
}

bool SenderComponent::Init() {
  return ComponentTest::Init(node_, config_file_path_);
}

bool SenderComponent::Proc() {
  if (_seq < _num_messages) {
    for (const auto& channel : _out_channel_names) {
      WhisperCommon::ChannelPtr msg = _whispers[channel]->next();
      _writers[channel]->Write(msg);
    }   

    WhisperCommon::sleep_for(_usleep_time, _usleep_interval, _usleep_step);
  } else if (_seq == _num_messages) {
    AINFO << _name << " finished writing.";
  }
  _seq += 1;
  return true;
}

bool OneChannelReceiverComponent::Init() {
  return ComponentTest::Init(node_, config_file_path_); 
}

bool OneChannelReceiverComponent::Proc(const std::shared_ptr<CybertronWhisper>& whisper) {
  AINFO << "\n-----------------------------\n"
        << _whispers.find(whisper->channel_name())->second->show(whisper) << "\n"
        << "+++++++++++++++++++++++++++++";
  WhisperCommon::sleep_for(_usleep_time, _usleep_interval, _usleep_step);

  for (const auto& channel : _out_channel_names) {
    WhisperCommon::ChannelPtr msg =
        _whispers.find(channel)->second->next();
    _writers[channel]->Write(msg);
  }
  return true;
}

bool TwoChannelReceiverComponent::Init() {
  return ComponentTest::Init(node_, config_file_path_); 
}

bool TwoChannelReceiverComponent::Proc(const WhisperCommon::ChannelPtr& whisper1,
                                       const WhisperCommon::ChannelPtr& whisper2) {
  AINFO << "\n-----------------------------\n"
        << _whispers.find(whisper1->channel_name())->second->show(whisper1) << "\n"
        << _whispers.find(whisper2->channel_name())->second->show(whisper2) << "\n"
        << "+++++++++++++++++++++++++++++";
  WhisperCommon::sleep_for(_usleep_time, _usleep_interval, _usleep_step);

  for (const auto& channel : _out_channel_names) {
    WhisperCommon::ChannelPtr msg =
        _whispers.find(channel)->second->next();
    _writers[channel]->Write(msg);
  }
  return true;
}

bool ThreeChannelReceiverComponent::Init() {
  return ComponentTest::Init(node_, config_file_path_); 
}

bool ThreeChannelReceiverComponent::Proc(const WhisperCommon::ChannelPtr& whisper1,
                                         const WhisperCommon::ChannelPtr& whisper2,
                                         const WhisperCommon::ChannelPtr& whisper3) {
  AINFO << "\n-----------------------------\n"
        << _whispers.find(whisper1->channel_name())->second->show(whisper1) << "\n"
        << _whispers.find(whisper2->channel_name())->second->show(whisper2) << "\n"
        << _whispers.find(whisper3->channel_name())->second->show(whisper3) << "\n"
        << "+++++++++++++++++++++++++++++";
  WhisperCommon::sleep_for(_usleep_time, _usleep_interval, _usleep_step);

  for (const auto& channel : _out_channel_names) {
    WhisperCommon::ChannelPtr msg =
        _whispers.find(channel)->second->next();
    _writers[channel]->Write(msg);
  }
  return true;
}
