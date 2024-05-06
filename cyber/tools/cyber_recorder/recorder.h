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

#ifndef CYBER_TOOLS_CYBER_RECORDER_RECORDER_H_
#define CYBER_TOOLS_CYBER_RECORDER_RECORDER_H_

#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <unordered_map>
#include <vector>

#include "cyber/proto/record.pb.h"
#include "cyber/proto/topology_change.pb.h"

#include "cyber/base/signal.h"
#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"
#include "cyber/record/record_writer.h"

using apollo::cyber::Node;
using apollo::cyber::ReaderBase;
using apollo::cyber::base::Connection;
using apollo::cyber::message::RawMessage;
using apollo::cyber::proto::ChangeMsg;
using apollo::cyber::proto::RoleAttributes;
using apollo::cyber::proto::RoleType;
using apollo::cyber::service_discovery::ChannelManager;
using apollo::cyber::service_discovery::TopologyManager;

namespace apollo {
namespace cyber {
namespace record {

class Recorder : public std::enable_shared_from_this<Recorder> {
 public:
  Recorder(const std::string& output, bool all_channels,
           const std::vector<std::string>& white_channels,
           const std::vector<std::string>& black_channels);
  Recorder(const std::string& output, bool all_channels,
           const std::vector<std::string>& white_channels,
           const std::vector<std::string>& black_channels,
           const proto::Header& header);
  ~Recorder();
  bool Start();
  bool Stop();

 private:
  bool is_started_ = false;
  bool is_stopping_ = false;
  std::shared_ptr<Node> node_ = nullptr;
  std::shared_ptr<RecordWriter> writer_ = nullptr;
  std::shared_ptr<std::thread> display_thread_ = nullptr;
  Connection<const ChangeMsg&> change_conn_;
  std::string output_;
  bool all_channels_ = true;
  std::vector<std::string> white_channels_;
  std::vector<std::regex> white_channel_patterns_;
  std::vector<std::string> black_channels_;
  std::vector<std::regex> black_channel_patterns_;
  proto::Header header_;
  std::unordered_map<std::string, std::shared_ptr<ReaderBase>>
      channel_reader_map_;
  uint64_t message_count_;
  uint64_t message_time_;

  bool InitReadersImpl();

  bool FreeReadersImpl();

  bool InitReaderImpl(const std::string& channel_name,
                      const std::string& message_type);

  void TopologyCallback(const ChangeMsg& msg);

  void ReaderCallback(const std::shared_ptr<RawMessage>& message,
                      const std::string& channel_name);

  void FindNewChannel(const RoleAttributes& role_attr);

  void ShowProgress();
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_RECORDER_H_
