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

#include "cyber/tools/cyber_recorder/player/play_task_producer.h"

#include <iostream>

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "cyber/message/protobuf_factory.h"
#include "cyber/record/record_viewer.h"

namespace apollo {
namespace cyber {
namespace record {

const uint32_t PlayTaskProducer::kMinTaskBufferSize = 500;
const uint32_t PlayTaskProducer::kPreloadTimeSec = 3;
const uint64_t PlayTaskProducer::kSleepIntervalNanoSec = 1000000;
const uint64_t PlayTaskProducer::kPreloadTimeNanoSec = kPreloadTimeSec * 1e9;

PlayTaskProducer::PlayTaskProducer(const TaskBufferPtr& task_buffer,
                                   const PlayParam& play_param)
    : play_param_(play_param),
      task_buffer_(task_buffer),
      produce_th_(nullptr),
      is_initialized_(false),
      is_stopped_(true),
      node_(nullptr),
      earliest_begin_time_(UINT64_MAX),
      latest_end_time_(0),
      total_msg_num_(0) {}

PlayTaskProducer::~PlayTaskProducer() { Stop(); }

bool PlayTaskProducer::Init() {
  if (is_initialized_.exchange(true)) {
    AERROR << "producer has been initialized.";
    return false;
  }

  if (!ReadRecordInfo()) {
    is_initialized_.exchange(false);
    return false;
  }

  if (!UpdatePlayParam()) {
    is_initialized_.exchange(false);
    return false;
  }

  if (!CreateWriters()) {
    is_initialized_.exchange(false);
    return false;
  }

  return true;
}

void PlayTaskProducer::Start() {
  if (!is_initialized_.load()) {
    AERROR << "please call Init firstly.";
    return;
  }

  if (!is_stopped_.exchange(false)) {
    AERROR << "producer has been started.";
    return;
  }

  produce_th_.reset(new std::thread(&PlayTaskProducer::ThreadFunc, this));

  auto preload_sec = kPreloadTimeSec;
  while (preload_sec > 0 && !is_stopped_.load()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    --preload_sec;
  }
}

void PlayTaskProducer::Stop() {
  if (!is_stopped_.exchange(true)) {
    return;
  }
  if (produce_th_ != nullptr && produce_th_->joinable()) {
    produce_th_->join();
    produce_th_ = nullptr;
  }
}

bool PlayTaskProducer::ReadRecordInfo() {
  if (play_param_.files_to_play.empty()) {
    AINFO << "no file to play.";
    return false;
  }

  auto pb_factory = message::ProtobufFactory::Instance();

  // loop each file
  for (auto& file : play_param_.files_to_play) {
    RecordInfo record_info;
    record_info.record_reader = std::make_shared<RecordReader>(file);

    auto& channel_info = record_info.record_reader->channel_info();
    // loop each channel info
    for (auto& item : channel_info) {
      auto& channel_name = item.first;
      auto& msg_type = item.second.message_type();
      msg_types_[channel_name] = msg_type;

      if (!play_param_.is_play_all_channels &&
          play_param_.channels_to_play.count(channel_name) > 0) {
        total_msg_num_ += item.second.message_number();
      }

      auto& proto_desc = item.second.proto_desc();
      pb_factory->RegisterMessage(proto_desc);
    }

    auto& header = record_info.record_reader->header();
    record_info.begin_time_ns = header.begin_time();
    record_info.end_time_ns = header.end_time();

    if (play_param_.is_play_all_channels) {
      total_msg_num_ += header.message_number();
    }

    if (header.begin_time() < earliest_begin_time_) {
      earliest_begin_time_ = header.begin_time();
    }
    if (header.end_time() > latest_end_time_) {
      latest_end_time_ = header.end_time();
    }

    record_infos_[file] = record_info;

    std::cout << "file: " << file << ", chunk_number: " << header.chunk_number()
              << ", begin_time: " << header.begin_time()
              << ", end_time: " << header.end_time()
              << ", message_number: " << header.message_number() << std::endl;
  }

  std::cout << "earliest_begin_time: " << earliest_begin_time_
            << ", latest_end_time: " << latest_end_time_
            << ", total_msg_num: " << total_msg_num_ << std::endl;

  return true;
}

bool PlayTaskProducer::UpdatePlayParam() {
  if (play_param_.begin_time_ns < earliest_begin_time_) {
    play_param_.begin_time_ns = earliest_begin_time_;
  }
  if (play_param_.start_time_s > 0) {
    play_param_.begin_time_ns += play_param_.start_time_s * 1e9;
  }
  if (play_param_.end_time_ns > latest_end_time_) {
    play_param_.end_time_ns = latest_end_time_;
  }
  if (play_param_.begin_time_ns >= play_param_.end_time_ns) {
    AERROR << "begin time are equal or larger than end time"
           << ", begin_time_ns=" << play_param_.begin_time_ns
           << ", end_time_ns=" << play_param_.end_time_ns;
    return false;
  }
  return true;
}

bool PlayTaskProducer::CreateWriters() {
  std::string node_name = "cyber_recorder_play_" + std::to_string(getpid());
  node_ = apollo::cyber::CreateNode(node_name);
  if (node_ == nullptr) {
    AERROR << "create node failed.";
    return false;
  }

  for (auto& item : msg_types_) {
    auto& channel_name = item.first;
    auto& msg_type = item.second;

    if (play_param_.is_play_all_channels ||
        play_param_.channels_to_play.count(channel_name) > 0) {
      proto::RoleAttributes attr;
      attr.set_channel_name(channel_name);
      attr.set_message_type(msg_type);
      auto writer = node_->CreateWriter<message::RawMessage>(attr);
      if (writer == nullptr) {
        AERROR << "create wirter failed. channel name: " << channel_name
               << ", message type: " << msg_type;
        return false;
      }
      writers_[channel_name] = writer;
    }
  }

  return true;
}

void PlayTaskProducer::ThreadFunc() {
  const uint64_t loop_time_ns =
      play_param_.end_time_ns - play_param_.begin_time_ns;
  uint64_t avg_interval_time_ns = kSleepIntervalNanoSec;
  if (total_msg_num_ > 0) {
    avg_interval_time_ns = loop_time_ns / total_msg_num_;
  }

  double avg_freq_hz = total_msg_num_ / (loop_time_ns * 1e-9);
  uint32_t preload_size = (uint32_t)avg_freq_hz * kPreloadTimeSec;
  if (preload_size < kMinTaskBufferSize) {
    preload_size = kMinTaskBufferSize;
  }

  uint32_t loop_num = 0;
  while (!is_stopped_.load()) {
    uint64_t curr_begin_time_ns = play_param_.begin_time_ns;
    uint64_t plus_time_ns = loop_num * loop_time_ns;

    while (curr_begin_time_ns < play_param_.end_time_ns &&
           !is_stopped_.load()) {
      uint64_t curr_end_time_ns = curr_begin_time_ns + kPreloadTimeNanoSec;

      while (!is_stopped_.load() && task_buffer_->Size() > preload_size) {
        std::this_thread::sleep_for(
            std::chrono::nanoseconds(avg_interval_time_ns));
      }

      CreateTask(curr_begin_time_ns, curr_end_time_ns, plus_time_ns);

      curr_begin_time_ns += kPreloadTimeNanoSec;
    }

    if (!play_param_.is_loop_playback) {
      is_stopped_.exchange(true);
      break;
    }
    ++loop_num;
  }
}

void PlayTaskProducer::CreateTask(uint64_t begin_time_ns, uint64_t end_time_ns,
                                  uint64_t plus_time_ns) {
  // loop each record file
  for (auto& item : record_infos_) {
    auto& record_info = item.second;
    uint64_t this_begin_time_ns = begin_time_ns;
    uint64_t this_end_time_ns = end_time_ns;
    if (this_begin_time_ns < record_info.begin_time_ns) {
      this_begin_time_ns = record_info.begin_time_ns;
    }
    if (this_end_time_ns > record_info.end_time_ns) {
      this_end_time_ns = record_info.end_time_ns;
    }
    if (this_begin_time_ns > this_end_time_ns) {
      continue;
    }

    auto record_viewer = std::make_shared<RecordViewer>(
        record_info.record_reader, this_begin_time_ns, this_end_time_ns);
    auto itr_end = record_viewer->end();

    for (auto itr = record_viewer->begin(); itr != itr_end; ++itr) {
      auto search = writers_.find(itr->channel_name);
      if (search == writers_.end()) {
        continue;
      }

      auto raw_msg = std::make_shared<message::RawMessage>(itr->content);
      auto task = std::make_shared<PlayTask>(raw_msg, search->second, itr->time,
                                             itr->time + plus_time_ns);
      task_buffer_->Push(task);
    }
  }
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
