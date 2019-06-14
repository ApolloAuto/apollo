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
#include "cyber/common/time_conversion.h"
#include "cyber/cyber.h"
#include "cyber/message/protobuf_factory.h"
#include "cyber/record/record_viewer.h"

namespace apollo {
namespace cyber {
namespace record {

const uint32_t PlayTaskProducer::kMinTaskBufferSize = 500;
const uint32_t PlayTaskProducer::kPreloadTimeSec = 3;
const uint64_t PlayTaskProducer::kSleepIntervalNanoSec = 1000000;

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

  if (!ReadRecordInfo() || !UpdatePlayParam() || !CreateWriters()) {
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
    auto record_reader = std::make_shared<RecordReader>(file);
    if (!record_reader->IsValid()) {
      continue;
    }
    if (!record_reader->GetHeader().is_complete()) {
      std::cout << "file: " << file << " is not complete." << std::endl;
      continue;
    }

    record_readers_.emplace_back(record_reader);

    auto channel_list = record_reader->GetChannelList();
    // loop each channel info
    for (auto& channel_name : channel_list) {
      if (play_param_.black_channels.find(channel_name) !=
          play_param_.black_channels.end()) {
        // minus the black message number from record file header
        total_msg_num_ -= record_reader->GetMessageNumber(channel_name);
        continue;
      }

      auto& msg_type = record_reader->GetMessageType(channel_name);
      msg_types_[channel_name] = msg_type;

      if (!play_param_.is_play_all_channels &&
          play_param_.channels_to_play.count(channel_name) > 0) {
        total_msg_num_ += record_reader->GetMessageNumber(channel_name);
      }

      auto& proto_desc = record_reader->GetProtoDesc(channel_name);
      pb_factory->RegisterMessage(proto_desc);
    }

    auto& header = record_reader->GetHeader();
    if (play_param_.is_play_all_channels) {
      total_msg_num_ += header.message_number();
    }

    if (header.begin_time() < earliest_begin_time_) {
      earliest_begin_time_ = header.begin_time();
    }
    if (header.end_time() > latest_end_time_) {
      latest_end_time_ = header.end_time();
    }

    auto begin_time_s = static_cast<double>(header.begin_time()) / 1e9;
    auto end_time_s = static_cast<double>(header.end_time()) / 1e9;
    auto begin_time_str =
        common::UnixSecondsToString(static_cast<int>(begin_time_s));
    auto end_time_str =
        common::UnixSecondsToString(static_cast<int>(end_time_s));

    std::cout << "file: " << file << ", chunk_number: " << header.chunk_number()
              << ", begin_time: " << header.begin_time() << " ("
              << begin_time_str << ")"
              << ", end_time: " << header.end_time() << " (" << end_time_str
              << ")"
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
    play_param_.begin_time_ns += static_cast<uint64_t>(
        static_cast<double>(play_param_.start_time_s) * 1e9);
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
  if (play_param_.preload_time_s == 0) {
    AINFO << "preload time is zero, we will use defalut value: "
          << kPreloadTimeSec << " seconds.";
    play_param_.preload_time_s = kPreloadTimeSec;
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
      if (play_param_.black_channels.find(channel_name) !=
          play_param_.black_channels.end()) {
        continue;
      }
      proto::RoleAttributes attr;
      attr.set_channel_name(channel_name);
      attr.set_message_type(msg_type);
      auto writer = node_->CreateWriter<message::RawMessage>(attr);
      if (writer == nullptr) {
        AERROR << "create writer failed. channel name: " << channel_name
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

  double avg_freq_hz = static_cast<double>(total_msg_num_) /
                       (static_cast<double>(loop_time_ns) * 1e-9);
  uint32_t preload_size = (uint32_t)avg_freq_hz * play_param_.preload_time_s;
  AINFO << "preload_size: " << preload_size;
  if (preload_size < kMinTaskBufferSize) {
    preload_size = kMinTaskBufferSize;
  }

  auto record_viewer = std::make_shared<RecordViewer>(
      record_readers_, play_param_.begin_time_ns, play_param_.end_time_ns,
      play_param_.channels_to_play);

  uint32_t loop_num = 0;
  while (!is_stopped_.load()) {
    uint64_t plus_time_ns = loop_num * loop_time_ns;
    auto itr = record_viewer->begin();
    auto itr_end = record_viewer->end();

    while (itr != itr_end && !is_stopped_.load()) {
      while (!is_stopped_.load() && task_buffer_->Size() > preload_size) {
        std::this_thread::sleep_for(
            std::chrono::nanoseconds(avg_interval_time_ns));
      }
      for (; itr != itr_end && !is_stopped_.load(); ++itr) {
        if (task_buffer_->Size() > preload_size) {
          break;
        }

        auto search = writers_.find(itr->channel_name);
        if (search == writers_.end()) {
          continue;
        }

        auto raw_msg = std::make_shared<message::RawMessage>(itr->content);
        auto task = std::make_shared<PlayTask>(
            raw_msg, search->second, itr->time, itr->time + plus_time_ns);
        task_buffer_->Push(task);
      }
    }

    if (!play_param_.is_loop_playback) {
      is_stopped_.exchange(true);
      break;
    }
    ++loop_num;
  }
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
