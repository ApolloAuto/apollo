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

#ifndef CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_PRODUCER_H_
#define CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_PRODUCER_H_

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "cyber/message/raw_message.h"
#include "cyber/node/node.h"
#include "cyber/node/writer.h"
#include "cyber/record/record_reader.h"
#include "cyber/tools/cyber_recorder/player/play_param.h"
#include "cyber/tools/cyber_recorder/player/play_task_buffer.h"
#include "cyber/proto/record.pb.h"
#include "cyber/record/record_viewer.h"

namespace apollo {
namespace cyber {
namespace record {

using ::apollo::cyber::proto::RecordInfo;

class PlayTaskProducer {
 public:
  using NodePtr = std::shared_ptr<Node>;
  using ThreadPtr = std::unique_ptr<std::thread>;
  using TaskBufferPtr = std::shared_ptr<PlayTaskBuffer>;
  using RecordReaderPtr = std::shared_ptr<RecordReader>;
  using WriterPtr = std::shared_ptr<Writer<message::RawMessage>>;
  using WriterMap = std::unordered_map<std::string, WriterPtr>;
  using MessageTypeMap = std::unordered_map<std::string, std::string>;
  using RecordViewerPtr = std::shared_ptr<RecordViewer>;

  PlayTaskProducer(const TaskBufferPtr& task_buffer,
                   const PlayParam& play_param,
                   const NodePtr& node = nullptr,
                   const bool preload_fill_buffer_mode = false);
  virtual ~PlayTaskProducer();

  bool Init();
  void Start();
  void Stop();

  const PlayParam& play_param() const { return play_param_; }
  bool is_stopped() const { return is_stopped_.load(); }
  bool is_initialized() const { return is_initialized_.load(); }
  void set_stopped() { is_stopped_.exchange(true); }
  void WriteRecordProgress(const double& curr_time_s,
                                           const double& total_time_s);
   /**
   * @brief Preload the player,producer fill play_task_buffer before
   * playing.
   */
  void FillPlayTaskBuffer();
  /**
   * @brief Reset player producer for dv will repeatedly use it.
   * reset the start time when dv reset play record progress.
   */
  void Reset(const double& progress_time_s);

 private:
  bool ReadRecordInfo();
  bool UpdatePlayParam();
  bool CreateWriters();
  bool CreatePlayTaskWriter(const std::string& channel_name,
                            const std::string& msg_type);
  void ThreadFunc();
  void ThreadFuncUnderPreloadMode();

  PlayParam play_param_;
  TaskBufferPtr task_buffer_;
  ThreadPtr produce_th_;

  std::atomic<bool> is_initialized_;
  std::atomic<bool> is_stopped_;

  NodePtr node_;
  WriterMap writers_;
  MessageTypeMap msg_types_;
  std::vector<RecordReaderPtr> record_readers_;
  RecordViewerPtr record_viewer_ptr_;

  uint64_t earliest_begin_time_;
  uint64_t latest_end_time_;
  uint64_t total_msg_num_;

  // This parameter indicates whether the producer needs to preload the buffer
  // When this value is true, it means that we preload the buffer before playing
  // we use it when dv play record under nohup process,all record player related
  // to the same node,so when this value is true,we pass parameter node to
  // assign value to node_
  bool preload_fill_buffer_mode_;

  static const uint32_t kMinTaskBufferSize;
  static const uint32_t kPreloadTimeSec;
  static const uint64_t kSleepIntervalNanoSec;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_PRODUCER_H_
