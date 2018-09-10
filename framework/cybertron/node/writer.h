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

#ifndef CYBERTRON_NODE_WRITER_H_
#define CYBERTRON_NODE_WRITER_H_

#include <memory>
#include <string>
#include <vector>

#include "cybertron/common/log.h"
#include "cybertron/node/writer_base.h"
#include "cybertron/topology/topology.h"
#include "cybertron/transport/transport.h"

namespace apollo {
namespace cybertron {

template <typename MessageT>
class Writer : public WriterBase {
 public:
  using UpperReachPtr = std::shared_ptr<transport::UpperReach<MessageT>>;
  using ChangeConnection = typename topology::Manager::ChangeConnection;

  explicit Writer(const proto::RoleAttributes& role_attr);
  virtual ~Writer();

  bool Init() override;
  void Shutdown() override;

  bool Write(const std::shared_ptr<MessageT>& message);

 protected:
  void JoinTheTopology();
  void LeaveTheTopology();
  void OnChannelChange(const proto::ChangeMsg& change_msg);

  UpperReachPtr upper_reach_;

  ChangeConnection change_conn_;
  topology::ChannelManagerPtr channel_manager_;
};

template <typename MessageT>
Writer<MessageT>::Writer(const proto::RoleAttributes& role_attr)
    : WriterBase(role_attr), upper_reach_(nullptr), channel_manager_(nullptr) {}

template <typename MessageT>
Writer<MessageT>::~Writer() {
  Shutdown();
}

template <typename MessageT>
bool Writer<MessageT>::Init() {
  if (init_.exchange(true)) {
    return true;
  }
  // TODO: singleton by channel name
  upper_reach_ = transport::Transport::CreateUpperReach<MessageT>(role_attr_);
  RETURN_VAL_IF_NULL(upper_reach_, false);
  this->role_attr_.set_id(upper_reach_->id().HashValue());

  channel_manager_ = topology::Topology::Instance()->channel_manager();
  JoinTheTopology();

  // TODO more check
  return true;
}

template <typename MessageT>
void Writer<MessageT>::Shutdown() {
  if (!init_.exchange(false)) {
    return;
  }

  LeaveTheTopology();
  upper_reach_ = nullptr;
  channel_manager_ = nullptr;
}

template <typename MessageT>
bool Writer<MessageT>::Write(const std::shared_ptr<MessageT>& message) {
  RETURN_VAL_IF(!init_.load(), false);
  return upper_reach_->Transmit(message);
}

template <typename MessageT>
void Writer<MessageT>::JoinTheTopology() {
  // add listener
  change_conn_ = channel_manager_->AddChangeListener(std::bind(
      &Writer<MessageT>::OnChannelChange, this, std::placeholders::_1));

  // get peer readers
  const std::string& channel_name = this->role_attr_.channel_name();
  std::vector<proto::RoleAttributes> readers;
  channel_manager_->GetReadersOfChannel(channel_name, &readers);
  for (auto& reader : readers) {
    upper_reach_->Enable(reader);
  }

  channel_manager_->Join(this->role_attr_, RoleType::ROLE_WRITER);
}

template <typename MessageT>
void Writer<MessageT>::LeaveTheTopology() {
  channel_manager_->RemoveChangeListener(change_conn_);
  channel_manager_->Leave(this->role_attr_, RoleType::ROLE_WRITER);
}

template <typename MessageT>
void Writer<MessageT>::OnChannelChange(const proto::ChangeMsg& change_msg) {
  if (change_msg.role_type() != proto::RoleType::ROLE_READER) {
    return;
  }

  auto& reader_attr = change_msg.role_attr();
  if (reader_attr.channel_name() != this->role_attr_.channel_name()) {
    return;
  }

  auto operate_type = change_msg.operate_type();
  if (operate_type == proto::OperateType::OPT_JOIN) {
    upper_reach_->Enable(reader_attr);
  } else {
    upper_reach_->Disable(reader_attr);
  }
}

}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_NODE_WRITER_H_
