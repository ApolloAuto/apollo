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

#ifndef CYBERTRON_TOOLS_CYBER_RECORDER_RECORDER_H_
#define CYBERTRON_TOOLS_CYBER_RECORDER_RECORDER_H_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include "cybertron/message/raw_message.h"
#include "cybertron/base/signal.h"
#include "cybertron/cybertron.h"
#include "cybertron/proto/record.pb.h"
#include "cybertron/proto/topology_change.pb.h"
#include "cybertron/record/record_writer.h"

using apollo::cybertron::topology::Topology;
using apollo::cybertron::topology::ChannelManager;
using apollo::cybertron::proto::ChangeMsg;
using apollo::cybertron::proto::RoleType;
using apollo::cybertron::proto::RoleAttributes;
using apollo::cybertron::message::RawMessage;
using apollo::cybertron::base::Connection;
using apollo::cybertron::ReaderBase;
using apollo::cybertron::Node;

namespace apollo {
namespace cybertron {
namespace record {

class Recorder : public std::enable_shared_from_this<Recorder> {
 public:
  Recorder(const std::string& output, bool all_channels,
           const std::vector<std::string>& channel_vec);
  ~Recorder();
  bool Start();
  bool Stop();

 private:
  bool is_started_;
  bool is_stopping_;
  std::shared_ptr<Node> node_;
  std::shared_ptr<RecordWriter> writer_;
  Connection<const ChangeMsg&> change_conn_;
  std::vector<std::string> channel_vec_;
  std::string output_;
  bool all_channels_;
  std::unordered_map<std::string, std::shared_ptr<ReaderBase>>
      channel_reader_map_;

  bool InitReadersImpl();

  bool FreeReadersImpl();

  bool InitReaderImpl(const std::string& channel_name,
                      const std::string& message_type);

  void TopologyCallback(const ChangeMsg& msg);

  void FindNewChannel(const RoleAttributes& role_attr);

  template <typename T>
  void callback_(const std::shared_ptr<const T>& message,
                 const std::string& channel_name);
};

template <typename T>
void Recorder::callback_(const std::shared_ptr<const T>& message,
                         const std::string& channel_name) {
  if (!is_started_ || is_stopping_) {
    AERROR << "record procedure is not started or stopping.";
    return;
  }

  if (message == nullptr) {
    AERROR << "message is nullptr, channel: " << channel_name;
    return;
  }

  if (!writer_->WriteMessage(channel_name, message)) {
    AERROR << "write data fail, channel: " << channel_name;
    return;
  }
}

}  // namespace record
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TOOLS_CYBER_RECORDER_RECORDER_H_
