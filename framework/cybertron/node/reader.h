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

#ifndef CYBERTRON_NODE_READER_H_
#define CYBERTRON_NODE_READER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cybertron/croutine/routine_factory.h"
#include "cybertron/data/data_visitor.h"
#include "cybertron/scheduler/scheduler.h"
#include "cybertron/time/time.h"
#include "cybertron/service_discovery/topology_manager.h"
#include "cybertron/transport/transport.h"

#include "cybertron/common/global_data.h"
#include "cybertron/node/reader_base.h"

namespace apollo {
namespace cybertron {

template <typename M0>
using CallbackFunc = std::function<void(const std::shared_ptr<M0>&)>;

using proto::RoleType;

template <typename MessageT>
class Reader : public ReaderBase {
 public:
  using LowerReachPtr = std::shared_ptr<transport::LowerReach<MessageT>>;
  using ChangeConnection = typename service_discovery::Manager::ChangeConnection;

  Reader(const proto::RoleAttributes& role_attr,
         const CallbackFunc<MessageT>& reader_func = nullptr);
  virtual ~Reader();

  bool Init() override;
  void Shutdown() override;

 protected:
  void JoinTheTopology();
  void LeaveTheTopology();
  void OnChannelChange(const proto::ChangeMsg& change_msg);

  CallbackFunc<MessageT> reader_func_;
  LowerReachPtr lower_reach_;
  std::string croutine_name_;

  ChangeConnection change_conn_;
  service_discovery::ChannelManagerPtr channel_manager_;
};

template <typename MessageT>
Reader<MessageT>::Reader(const proto::RoleAttributes& role_attr,
                         const CallbackFunc<MessageT>& reader_func)
    : ReaderBase(role_attr),
      reader_func_(reader_func),
      lower_reach_(nullptr),
      croutine_name_(""),
      channel_manager_(nullptr) {}

template <typename MessageT>
Reader<MessageT>::~Reader() {
  Shutdown();
}

template <typename MessageT>
bool Reader<MessageT>::Init() {
  if (init_.exchange(true)) {
    return true;
  }
  if (reader_func_ != nullptr) {
    auto sched = scheduler::Scheduler::Instance();
    croutine_name_ = role_attr_.node_name() + "_" + role_attr_.channel_name();

    auto dv = std::make_shared<data::DataVisitor<MessageT>>(
        role_attr_.channel_id(), role_attr_.qos_profile().depth());
    // Using factory to wrap templates.
    croutine::RoutineFactory factory =
        croutine::CreateRoutineFactory<MessageT>(reader_func_, dv);
    if (!sched->CreateTask(factory, croutine_name_)) {
      init_.exchange(false);
      return false;
    }
  }

  lower_reach_ = ReaderManager<MessageT>::Instance()->GetReader(role_attr_);
  this->role_attr_.set_id(lower_reach_->id().HashValue());

  channel_manager_ = service_discovery::TopologyManager::Instance()->channel_manager();
  JoinTheTopology();

  // TODO more check
  return true;
}

template <typename MessageT>
void Reader<MessageT>::Shutdown() {
  // TODO: delete task and reset transport reader
  if (!init_.exchange(false)) {
    return;
  }
  LeaveTheTopology();
  lower_reach_ = nullptr;
  channel_manager_ = nullptr;

  if (!croutine_name_.empty()) {
    scheduler::Scheduler::Instance()->RemoveTask(croutine_name_);
  }
}

template <typename MessageT>
void Reader<MessageT>::JoinTheTopology() {
  // add listener
  change_conn_ = channel_manager_->AddChangeListener(std::bind(
      &Reader<MessageT>::OnChannelChange, this, std::placeholders::_1));

  // get peer writers
  const std::string& channel_name = this->role_attr_.channel_name();
  std::vector<proto::RoleAttributes> writers;
  channel_manager_->GetWritersOfChannel(channel_name, &writers);
  for (auto& writer : writers) {
    lower_reach_->Enable(writer);
  }
  channel_manager_->Join(this->role_attr_, RoleType::ROLE_READER);
}

template <typename MessageT>
void Reader<MessageT>::LeaveTheTopology() {
  channel_manager_->RemoveChangeListener(change_conn_);
  channel_manager_->Leave(this->role_attr_, RoleType::ROLE_READER);
}

template <typename MessageT>
void Reader<MessageT>::OnChannelChange(const proto::ChangeMsg& change_msg) {
  if (change_msg.role_type() != proto::RoleType::ROLE_WRITER) {
    return;
  }

  auto& writer_attr = change_msg.role_attr();
  if (writer_attr.channel_name() != this->role_attr_.channel_name()) {
    return;
  }

  auto operate_type = change_msg.operate_type();
  if (operate_type == proto::OperateType::OPT_JOIN) {
    lower_reach_->Enable(writer_attr);
  } else {
    lower_reach_->Disable(writer_attr);
  }
}

}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_NODE_READER_H_
