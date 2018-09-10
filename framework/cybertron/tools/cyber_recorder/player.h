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

#ifndef CYBERTRON_TOOLS_CYBER_RECORDER_PLAYER_H_
#define CYBERTRON_TOOLS_CYBER_RECORDER_PLAYER_H_

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <iomanip>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>
#include "cybertron/message/message_traits.h"
#include "cybertron/message/protobuf_factory.h"
#include "cybertron/common/time_conversion.h"
#include "cybertron/cybertron.h"
#include "cybertron/proto/record.pb.h"
#include "cybertron/record/record_file.h"

using ::apollo::cybertron::message::RawMessage;
using ::apollo::cybertron::message::ProtobufFactory;
using ::apollo::cybertron::proto::SingleMessage;
using ::apollo::cybertron::proto::Header;
using ::apollo::cybertron::proto::ChunkHeader;
using ::apollo::cybertron::proto::ChunkBody;
using ::apollo::cybertron::proto::RoleAttributes;
using ::apollo::cybertron::proto::SectionType;
using ::apollo::cybertron::Time;
using ::apollo::cybertron::Node;
using ::apollo::cybertron::Writer;
using ::apollo::cybertron::record::RecordFileReader;

namespace apollo {
namespace cybertron {
namespace record {

class TimedPlayer {
 public:
  TimedPlayer(std::shared_ptr<Writer<RawMessage>>& writer,
              const std::shared_ptr<RawMessage>& raw_message,
              uint64_t real_time, uint64_t play_time);
  virtual ~TimedPlayer();
  void Proc();
  uint64_t real_time_;
  uint64_t play_time_;
  static uint64_t played_message_number_;

 private:
  std::shared_ptr<Writer<RawMessage>>& writer_;
  std::shared_ptr<RawMessage> raw_message_;
};

class Player {
 public:
  Player(const std::string& file, bool all_channels,
         const std::vector<std::string>& channel_vec,
         bool looped_playback = false, float rate = 1.0,
         uint64_t begin_time = 0, uint64_t end_time = UINT64_MAX,
         uint64_t start_seconds = 0, uint64_t delay_seconds = 0);
  virtual ~Player();
  bool Init();
  bool Start();
  bool Stop();

 private:
  bool is_initialized_ = false;
  bool is_started_ = false;
  bool is_paused_ = false;
  bool is_stopped_ = false;
  bool all_channels_ = false;
  bool looped_playback_ = false;
  bool load_finished_ = false;
  std::string file_;
  float rate_;
  std::queue<std::shared_ptr<TimedPlayer>> task_queue_;
  std::vector<std::string> channel_vec_;
  std::unordered_map<std::string, std::string> channel_message_type_map_;
  std::unordered_map<std::string, std::shared_ptr<Writer<RawMessage>>>
      channel_writer_map_;
  Header header_;
  Index index_;
  std::shared_ptr<Node> node_;
  RecordFileReader infileopt_;
  std::shared_ptr<std::thread> loader_thread_;
  std::mutex mutex_;
  uint64_t total_message_number_ = 0;
  uint64_t base_time_ = 0;
  uint64_t last_played_time_ = 0;
  uint64_t begin_time_ = 0;         // expect begin time in this playing
  uint64_t end_time_ = UINT64_MAX;  // expect end time in this playing
  uint64_t task_count_ = 0;         // message count which will be played back
  uint64_t start_seconds_ = 0;      // start seconds into the record
  uint64_t delay_seconds_ = 0;      // delay seconds time before play
  uint32_t min_queue_size_ = 2000;
  uint32_t preload_sec_ = 5;
  bool InitInfileopt();
  bool InitWriters();
  bool InitLoader();
  bool LoadChunk(uint64_t from_time, uint64_t to_time);
  bool GenerateTask(const SingleMessage& msg, const uint64_t& plus_time);

  static const int MIN_SLEEP_INTERVAL = 10 * 1e6;  // 10 ms

  int maxfd_;
  termios orig_flags_;
  fd_set stdin_fdset_;
  void ReadCharFromStdin();
};

}  // namespace record
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TOOLS_CYBER_RECORDER_PLAYER_H_
