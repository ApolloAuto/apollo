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

#include <sstream>
#include <glog/logging.h>
#include "whisper_common.h"

int32_t WhisperCommon::sleep_for(int32_t value, int32_t interval,
                                    int32_t step) {
  if (interval > 0) {
    value = value + (interval - rand() % interval) / step * step;
    if (value < 0) {
      value = 0;
    }
  }
  if (value > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(value));
  }   
}

WhisperCommon::WhisperCommon(const std::string& me,
                             const std::string& channel_name,
                             const std::string& frame_id,
                             const std::shared_ptr<DataPool>& data_pool)
    : _me(me),
      _frame_id(frame_id),
      _seq(0),
      _position(0),
      _data_pool(data_pool),
      _last_receive_time(0),
      _channel_name(channel_name) {
  _allocating = _data_pool->allocate();
  AINFO << "WhisperCommon: " << me << ", " << channel_name;
}

WhisperCommon::~WhisperCommon() {
  if (_allocating) {
    delete _allocating;
    _allocating = nullptr;
  }
}

WhisperCommon::ChannelPtr WhisperCommon::next() {
  auto start = apollo::cybertron::Time::Now().ToNanosecond();
  AINFO << _me << " start to generate [" << _seq << "] message";
  std::shared_ptr<ChannelType> whisper(new ChannelType());
  // std::lock_guard<std::mutex> lck (_mutex);
  // set body
  whisper->set_seq(_seq);
  whisper->set_telling_name(_me);
  whisper->set_channel_name(_channel_name);
  // set content
  int32_t length = _data_pool->allocate(_allocating, _position);
  whisper->set_content(_allocating);
  whisper->set_length(length);
  whisper->set_position(_position);
  // set header, so the interval will not include the contents generation
  adu::common::header::Header* hdr = whisper->mutable_cyber_header();
  hdr->set_timestamp_sec(apollo::cybertron::Time::Now().ToNanosecond());
  hdr->set_stamp(apollo::cybertron::Time::Now().ToNanosecond());
  hdr->set_sequence_num(_seq);
  hdr->set_frame_id(_frame_id);
  AINFO << _me << " end to generate [" << _seq << "] message";

  AINFO << "\n-----------------------------\n"
        << "me: [" << _me << "], whisper: [" << _channel_name << "] -- "
        << ", content length: " << length
        << ", content positoin: " << _position << ", seq: " << _seq << ", calling: " << start
        << ", meta stamp: " << hdr->timestamp_sec()
        << ", stamp: " << hdr->stamp() << ", role: writer"
        << "\n+++++++++++++++++++++++++++++" << std::endl;

  auto end = apollo::cybertron::Time::Now().ToNanosecond();
  AINFO << "********* " << _me << " generate message [" << _seq << "] use "
        << (end - start) / 1000 << " us " << end - start;

  // plus after log
  _seq++;
  _position++;
  return whisper;
}

std::string WhisperCommon::show(const WhisperCommon::ConstChannelPtr& whisper) {
  auto timestamp = apollo::cybertron::Time::Now().ToNanosecond();
  auto start = timestamp;
  // compare
  uint64_t now = apollo::cybertron::Time::Now().ToNanosecond();
  int64_t time_channel = now - whisper->cyber_header().stamp();
  int64_t time_total = now - whisper->cyber_header().timestamp_sec();
  int position = whisper->position();
  size_t length = whisper->content().size();
  bool length_equal = whisper->length() == length;
  bool content_equal = 
      _data_pool->compare(whisper->content().c_str(), length, position);
  int64_t receive_interval = 0;
  if (_last_receive_time != 0) {
    receive_interval = now - _last_receive_time;
  }
  _last_receive_time = now;
  // output
  std::stringstream ss;
  ss << "me: [" << _me << "], whisper: [" << whisper->channel_name() << "] -- "
     << ", content length: " << whisper->content().size()
     << ", content position: " << whisper->position()
     << ", length equal: " << length_equal
     << ", content equal: " << content_equal << ", time total: " << time_total
     << ", time channel: " << time_channel << ", seq: " << whisper->seq()
     << ", meta_stamp: " << whisper->cyber_header().timestamp_sec()
     << ", telling: " << whisper->telling_name() << ", timestamp: " << timestamp
     << ", role: reader"
     << ", receive_interval: " << receive_interval;
  auto end = apollo::cybertron::Time::Now().ToNanosecond();
  AINFO << "********* " << _me << " compare message [" << whisper->seq()
        << "] use " << (end - start) / 1000 << " us " << end - start;
  return ss.str();
}

