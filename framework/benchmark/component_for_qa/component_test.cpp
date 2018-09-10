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
#include "cybertron/cybertron.h"
#include "component_test.h"

using apollo::cybertron::proto::CybertronWhisperConfig;
using apollo::cybertron::RoleAttributes;

bool ComponentTest::Init(const std::shared_ptr<Node>& node,
                         const std::string& path) {
  CybertronWhisperConfig config;
  _configfile = path;
  _node = node;
  if (!apollo::cybertron::common::GetProtoFromFile(_configfile, &config)) {
    return false;
  }
  _seq = 0;
  _length = config.length();
  _length_interval = config.length_interval();
  _length_step = config.length_step();
  _start_wait_sec = config.start_wait_sec();
  _end_wait_sec = config.end_wait_sec();
  _usleep_time = config.usleep_time();
  _usleep_interval = config.usleep_interval();
  _usleep_step = config.usleep_step();
  _num_messages = config.num_messages();
  _name = config.name();
  _data_pool.reset(new DataPool(_length, _length_interval, _length_step));
  for (int i = 0; i < config.in_channel_names().size(); i++) {
    const std::string& channel = config.out_channel_names(i);
    _in_channel_names.push_back(config.in_channel_names(i));
    _whispers[channel].reset(
        new WhisperCommon(_name, channel, _name, _data_pool));
  }
  for (int i = 0; i < config.out_channel_names().size(); i++) {
    const std::string& channel = config.out_channel_names(i);
    RoleAttributes attr;
    attr.set_channel_name(channel);
    _writers[channel] = _node->CreateWriter<CybertronWhisper>(attr);
    if (_writers[channel] == nullptr) {
      AERROR << "Sender create writer failed.";
      return false;
    }
    _whispers[channel].reset(
        new WhisperCommon(_name, channel, _name, _data_pool));
    _out_channel_names.push_back(channel);
  }

  std::stringstream sstr;
  sstr << "length: " << _length << ", length_interval: " << _length_interval
       << ", length_step: " << _length_step
       << ", start_wait_usec: " << _start_wait_sec
       << ", end_wait_usec: " << _end_wait_sec
       << ", usleep_time: " << _usleep_time
       << ", usleep_interval: " << _usleep_interval
       << ", usleep_step: " << _usleep_step
       << ", num_messages: " << _num_messages << ", in_channel_names: ";
  for (auto channel_name : _in_channel_names) {
    sstr << "|" << channel_name;
  }
  sstr << ", out_channel_names: ";
  for (auto channel_name : _out_channel_names) {
    sstr << "|" << channel_name;
  }
  AINFO << sstr.str();

  std::this_thread::sleep_for(std::chrono::seconds(_start_wait_sec));
  return true;
}

