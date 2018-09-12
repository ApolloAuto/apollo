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

#include "cybertron/tools/cyber_recorder/recorder.h"

namespace apollo {
namespace cybertron {
namespace record {

Recorder::Recorder(const std::string& output, bool all_channels,
                   const std::vector<std::string>& channel_vec)
    : is_started_(false),
      is_stopping_(false),
      output_(output),
      all_channels_(all_channels),
      channel_vec_(channel_vec),
      writer_(nullptr),
      node_(nullptr) {}

Recorder::~Recorder() { Stop(); }

bool Recorder::Start() {
  writer_.reset(new RecordWriter());
  if (!writer_->Open(output_)) {
    AERROR << "Datafile open file error.";
    return false;
  }
  std::string node_name = "cyber_recorder_record_" + std::to_string(getpid());
  node_ = ::apollo::cybertron::CreateNode(node_name);
  if (node_ == nullptr) {
    AERROR << "create node failed, node: " << node_name;
    return false;
  }
  if (!InitReadersImpl()) {
    AERROR << " _init_readers error.";
    return false;
  }
  is_started_ = true;
  return true;
}

bool Recorder::Stop() {
  if (!is_started_ || is_stopping_) {
    return false;
  }
  is_stopping_ = true;
  if (!FreeReadersImpl()) {
    AERROR << " _free_readers error.";
    return false;
  }
  writer_->Close();
  node_.reset();
  return true;
}

void Recorder::TopologyCallback(const ChangeMsg& change_message) {
  ADEBUG << "ChangeMsg in Topology Callback:" << std::endl
         << change_message.ShortDebugString();
  if (change_message.role_type() != apollo::cybertron::proto::ROLE_WRITER) {
    ADEBUG << "Change message role type is not ROLE_WRITER.";
    return;
  }
  FindNewChannel(change_message.role_attr());
}

void Recorder::FindNewChannel(const RoleAttributes& role_attr) {
  if (!role_attr.has_channel_name() || role_attr.channel_name().empty()) {
    AWARN << "change message not has a channel name or has an empty one.";
    return;
  }
  if (!role_attr.has_message_type() || role_attr.message_type().empty()) {
    AWARN << "Change message not has a message type or has an empty one.";
    return;
  }
  if (!role_attr.has_proto_desc() || role_attr.proto_desc().empty()) {
    AWARN << "Change message not has a proto desc or has an empty one.";
    return;
  }
  if (!all_channels_ &&
      std::find(channel_vec_.begin(), channel_vec_.end(),
                role_attr.channel_name()) == channel_vec_.end()) {
    ADEBUG << "New channel was found, but not in record list.";
    return;
  }
  if (channel_reader_map_.find(role_attr.channel_name()) ==
      channel_reader_map_.end()) {
    writer_->WriteChannel(role_attr.channel_name(), role_attr.message_type(),
                          role_attr.proto_desc());
    InitReaderImpl(role_attr.channel_name(), role_attr.message_type());
  }
}

bool Recorder::InitReadersImpl() {
  std::shared_ptr<ChannelManager> channel_manager =
      TopologyManager::Instance()->channel_manager();

  // get historical writers
  std::vector<proto::RoleAttributes> role_attr_vec;
  channel_manager->GetWriters(&role_attr_vec);
  for (auto role_attr : role_attr_vec) {
    FindNewChannel(role_attr);
  }

  // listen new writers in future
  change_conn_ = channel_manager->AddChangeListener(
      std::bind(&Recorder::TopologyCallback, this, std::placeholders::_1));
  if (!change_conn_.IsConnected()) {
    AERROR << "change connection is not connected";
    return false;
  }
  return true;
}

bool Recorder::FreeReadersImpl() {
  std::shared_ptr<ChannelManager> channel_manager =
      TopologyManager::Instance()->channel_manager();

  channel_manager->RemoveChangeListener(change_conn_);

  return true;
}

bool Recorder::InitReaderImpl(const std::string& channel_name,
                              const std::string& message_type) {
  try {
    std::weak_ptr<Recorder> weak_this = shared_from_this();
    std::shared_ptr<ReaderBase> reader = nullptr;
    auto callback = [weak_this, channel_name](
                        const std::shared_ptr<RawMessage>& raw_message) {
      auto share_this = weak_this.lock();
      if (!share_this) {
        return;
      }
      share_this->template callback_<RawMessage>(raw_message, channel_name);
      share_this->writer_->ShowProgress();
    };
    reader = node_->CreateReader<RawMessage>(channel_name, callback);
    if (reader == nullptr) {
      AERROR << "Create reader failed.";
      return false;
    }
    channel_reader_map_[channel_name] = reader;
    return true;
  } catch (const std::bad_weak_ptr& e) {
    AERROR << e.what();
    return false;
  }
}

}  // namespace record
}  // namespace cybertron
}  // namespace apollo
