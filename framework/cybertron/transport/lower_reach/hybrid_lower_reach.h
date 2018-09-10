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

#ifndef CYBERTRON_TRANSPORT_LOWER_REACH_HYBRID_LOWER_REACH_H_
#define CYBERTRON_TRANSPORT_LOWER_REACH_HYBRID_LOWER_REACH_H_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cybertron/common/global_data.h"
#include "cybertron/common/log.h"
#include "cybertron/common/types.h"
#include "cybertron/proto/role_attributes.pb.h"
#include "cybertron/time/time.h"
#include "cybertron/timer/timer_manager.h"
#include "cybertron/topology/role/role.h"
#include "cybertron/transport/lower_reach/intra_lower_reach.h"
#include "cybertron/transport/lower_reach/rtps_lower_reach.h"
#include "cybertron/transport/lower_reach/shm_lower_reach.h"
#include "cybertron/transport/rtps/participant.h"

namespace apollo {
namespace cybertron {
namespace transport {

using apollo::cybertron::proto::OptionalMode;
using apollo::cybertron::proto::QosDurabilityPolicy;
using apollo::cybertron::proto::RoleAttributes;

template <typename MessageT>
class HybridLowerReach : public LowerReach<MessageT> {
 public:
  using HistoryPtr = std::shared_ptr<History<MessageT>>;
  using LowerReachPtr = std::shared_ptr<LowerReach<MessageT>>;
  using LowerReachContainer =
      std::unordered_map<OptionalMode, LowerReachPtr, std::hash<int>>;
  using UpperReachContainer =
      std::unordered_map<OptionalMode,
                         std::unordered_map<uint64_t, RoleAttributes>,
                         std::hash<int>>;
  using CommunicationModePtr = std::shared_ptr<proto::CommunicationMode>;
  using MappingTable =
      std::unordered_map<Relation, OptionalMode, std::hash<int>>;

  HybridLowerReach(
      const RoleAttributes& attr,
      const typename LowerReach<MessageT>::MessageListener& msg_listener,
      const ParticipantPtr& participant);
  virtual ~HybridLowerReach();

  void Enable() override;
  void Disable() override;

  void Enable(const RoleAttributes& opposite_attr) override;
  void Disable(const RoleAttributes& opposite_attr) override;

 private:
  void InitMode();
  void ObtainConfig();
  void InitHistory();
  void InitLowerReaches();
  void ClearLowerReaches();
  void InitUpperReaches();
  void ClearUpperReaches();
  void ReceiveHistoryMsg(const RoleAttributes& opposite_attr);
  void ThreadFunc(const RoleAttributes& opposite_attr);
  Relation GetRelation(const RoleAttributes& opposite_attr);

  HistoryPtr history_;
  LowerReachContainer lower_reaches_;
  UpperReachContainer upper_reaches_;
  std::mutex mutex_;

  CommunicationModePtr mode_;
  MappingTable mapping_table_;

  ParticipantPtr participant_;
};

template <typename MessageT>
HybridLowerReach<MessageT>::HybridLowerReach(
    const RoleAttributes& attr,
    const typename LowerReach<MessageT>::MessageListener& msg_listener,
    const ParticipantPtr& participant)
    : LowerReach<MessageT>(attr, msg_listener),
      history_(nullptr),
      participant_(participant) {
  InitMode();
  ObtainConfig();
  InitHistory();
  InitLowerReaches();
  InitUpperReaches();
}

template <typename MessageT>
HybridLowerReach<MessageT>::~HybridLowerReach() {
  ClearUpperReaches();
  ClearLowerReaches();
}

template <typename MessageT>
void HybridLowerReach<MessageT>::Enable() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& item : lower_reaches_) {
    item.second->Enable();
  }
}

template <typename MessageT>
void HybridLowerReach<MessageT>::Disable() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& item : lower_reaches_) {
    item.second->Disable();
  }
}

template <typename MessageT>
void HybridLowerReach<MessageT>::Enable(const RoleAttributes& opposite_attr) {
  auto relation = GetRelation(opposite_attr);
  RETURN_IF(relation == NO_RELATION);

  uint64_t id = opposite_attr.id();
  std::lock_guard<std::mutex> lock(mutex_);
  if (upper_reaches_[mapping_table_[relation]].count(id) == 0) {
    upper_reaches_[mapping_table_[relation]].insert(
        std::make_pair(id, opposite_attr));
    lower_reaches_[mapping_table_[relation]]->Enable(opposite_attr);
    ReceiveHistoryMsg(opposite_attr);
  }
}

template <typename MessageT>
void HybridLowerReach<MessageT>::Disable(const RoleAttributes& opposite_attr) {
  auto relation = GetRelation(opposite_attr);
  RETURN_IF(relation == NO_RELATION);

  uint64_t id = opposite_attr.id();
  std::lock_guard<std::mutex> lock(mutex_);
  if (upper_reaches_[mapping_table_[relation]].count(id) > 0) {
    upper_reaches_[mapping_table_[relation]].erase(id);
    lower_reaches_[mapping_table_[relation]]->Disable(opposite_attr);
  }
}

template <typename MessageT>
void HybridLowerReach<MessageT>::InitMode() {
  mode_ = std::make_shared<proto::CommunicationMode>();
  mapping_table_[SAME_PROC] = mode_->same_proc();
  mapping_table_[DIFF_PROC] = mode_->diff_proc();
  mapping_table_[DIFF_HOST] = mode_->diff_host();
}

template <typename MessageT>
void HybridLowerReach<MessageT>::ObtainConfig() {
  auto global_conf = common::GlobalData::Instance()->Config();
  RETURN_IF(!global_conf.has_transport_conf());
  RETURN_IF(!global_conf.transport_conf().has_communication_mode());
  mode_->CopyFrom(global_conf.transport_conf().communication_mode());

  mapping_table_[SAME_PROC] = mode_->same_proc();
  mapping_table_[DIFF_PROC] = mode_->diff_proc();
  mapping_table_[DIFF_HOST] = mode_->diff_host();
}

template <typename MessageT>
void HybridLowerReach<MessageT>::InitHistory() {
  HistoryAttributes history_attr(this->attr_.qos_profile().history(),
                                 this->attr_.qos_profile().depth());
  history_ = std::make_shared<History<MessageT>>(history_attr);
  if (this->attr_.qos_profile().durability() ==
      QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL) {
    history_->Enable();
  }
}

template <typename MessageT>
void HybridLowerReach<MessageT>::InitLowerReaches() {
  std::set<OptionalMode> modes;
  modes.insert(mode_->same_proc());
  modes.insert(mode_->diff_proc());
  modes.insert(mode_->diff_host());
  auto listener = std::bind(&HybridLowerReach<MessageT>::OnNewMessage, this,
                            std::placeholders::_1, std::placeholders::_2);
  for (auto& mode : modes) {
    switch (mode) {
      case OptionalMode::INTRA:
        lower_reaches_[mode] =
            std::make_shared<IntraLowerReach<MessageT>>(this->attr_, listener);
        break;
      case OptionalMode::SHM:
        lower_reaches_[mode] =
            std::make_shared<ShmLowerReach<MessageT>>(this->attr_, listener);
        break;
      default:
        lower_reaches_[mode] =
            std::make_shared<RtpsLowerReach<MessageT>>(this->attr_, listener);
        break;
    }
  }
}

template <typename MessageT>
void HybridLowerReach<MessageT>::ClearLowerReaches() {
  lower_reaches_.clear();
}

template <typename MessageT>
void HybridLowerReach<MessageT>::InitUpperReaches() {
  std::unordered_map<uint64_t, RoleAttributes> empty;
  for (auto& item : lower_reaches_) {
    upper_reaches_[item.first] = empty;
  }
}

template <typename MessageT>
void HybridLowerReach<MessageT>::ClearUpperReaches() {
  for (auto& item : upper_reaches_) {
    for (auto& upper_reach : item.second) {
      lower_reaches_[item.first]->Disable(upper_reach.second);
    }
  }
  upper_reaches_.clear();
}

template <typename MessageT>
void HybridLowerReach<MessageT>::ReceiveHistoryMsg(
    const RoleAttributes& opposite_attr) {
  // check qos
  if (this->attr_.qos_profile().durability() !=
          QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL ||
      opposite_attr.qos_profile().durability() !=
          QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL) {
    return;
  }

  auto recv_th =
      std::thread(&HybridLowerReach<MessageT>::ThreadFunc, this, opposite_attr);
  recv_th.detach();
}

template <typename MessageT>
void HybridLowerReach<MessageT>::ThreadFunc(
    const RoleAttributes& opposite_attr) {
  std::string channel_name =
      std::to_string(opposite_attr.id()) + std::to_string(this->attr_.id());
  uint64_t channel_id = common::GlobalData::RegisterChannel(channel_name);
  RoleAttributes attr(this->attr_);
  attr.set_channel_name(channel_name);
  attr.set_channel_id(channel_id);
  bool no_more_msg = false;
  auto task = [&no_more_msg]() { no_more_msg = true; };
  uint64_t timer_id = TimerManager::Instance()->Add(1000, task, true);
  auto listener = [&](const std::shared_ptr<MessageT>& msg,
                      const MessageInfo& msg_info, const RoleAttributes& attr) {
    TimerManager::Instance()->Remove(timer_id);
    this->OnNewMessage(msg, msg_info);
    timer_id = TimerManager::Instance()->Add(1000, task, true);
  };
  auto lower_reach = std::make_shared<RtpsLowerReach<MessageT>>(attr, listener);
  lower_reach->Enable();
  while (!no_more_msg) {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  lower_reach->Disable();
  ADEBUG << "recv threadfunc exit.";
}

template <typename MessageT>
Relation HybridLowerReach<MessageT>::GetRelation(
    const RoleAttributes& opposite_attr) {
  if (opposite_attr.channel_name() != this->attr_.channel_name()) {
    return NO_RELATION;
  }
  if (opposite_attr.host_name() != this->attr_.host_name()) {
    return DIFF_HOST;
  }
  if (opposite_attr.process_id() != this->attr_.process_id()) {
    return DIFF_PROC;
  }

  return SAME_PROC;
}

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TRANSPORT_LOWER_REACH_HYBRID_LOWER_REACH_H_
