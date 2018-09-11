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

#ifndef CYBERTRON_TRANSPORT_UPPER_REACH_HYBRID_UPPER_REACH_H_
#define CYBERTRON_TRANSPORT_UPPER_REACH_HYBRID_UPPER_REACH_H_

#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "cybertron/common/global_data.h"
#include "cybertron/common/log.h"
#include "cybertron/common/types.h"
#include "cybertron/proto/role_attributes.pb.h"
#include "cybertron/proto/transport_conf.pb.h"
#include "cybertron/transport/message/history.h"
#include "cybertron/transport/rtps/participant.h"
#include "cybertron/transport/upper_reach/intra_upper_reach.h"
#include "cybertron/transport/upper_reach/rtps_upper_reach.h"
#include "cybertron/transport/upper_reach/shm_upper_reach.h"
#include "cybertron/transport/upper_reach/upper_reach.h"

namespace apollo {
namespace cybertron {
namespace transport {

using apollo::cybertron::proto::OptionalMode;
using apollo::cybertron::proto::QosDurabilityPolicy;
using apollo::cybertron::proto::RoleAttributes;

template <typename MessageT>
class HybridUpperReach : public UpperReach<MessageT> {
 public:
  using MessagePtr = std::shared_ptr<MessageT>;
  using HistoryPtr = std::shared_ptr<History<MessageT>>;
  using UpperReachPtr = std::shared_ptr<UpperReach<MessageT>>;
  using UpperReachContainer =
      std::unordered_map<OptionalMode, UpperReachPtr, std::hash<int>>;
  using LowerReachContainer =
      std::unordered_map<OptionalMode, std::set<uint64_t>, std::hash<int>>;
  using CommunicationModePtr = std::shared_ptr<proto::CommunicationMode>;
  using MappingTable =
      std::unordered_map<Relation, OptionalMode, std::hash<int>>;

  HybridUpperReach(const RoleAttributes& attr,
                   const ParticipantPtr& participant);
  virtual ~HybridUpperReach();

  void Enable() override;
  void Disable() override;
  void Enable(const RoleAttributes& opposite_attr) override;
  void Disable(const RoleAttributes& opposite_attr) override;

  bool Transmit(const MessagePtr& msg, const MessageInfo& msg_info) override;

 private:
  void InitMode();
  void ObtainConfig();
  void InitHistory();
  void InitUpperReaches();
  void ClearUpperReaches();
  void InitLowerReaches();
  void ClearLowerReaches();
  void TransmitHistoryMsg(const RoleAttributes& opposite_attr);
  void ThreadFunc(const RoleAttributes& opposite_attr);
  Relation GetRelation(const RoleAttributes& opposite_attr);

  HistoryPtr history_;
  UpperReachContainer upper_reaches_;
  LowerReachContainer lower_reaches_;
  std::mutex mutex_;

  CommunicationModePtr mode_;
  MappingTable mapping_table_;

  ParticipantPtr participant_;
};

template <typename MessageT>
HybridUpperReach<MessageT>::HybridUpperReach(const RoleAttributes& attr,
                                             const ParticipantPtr& participant)
    : UpperReach<MessageT>(attr),
      history_(nullptr),
      mode_(nullptr),
      participant_(participant) {
  InitMode();
  ObtainConfig();
  InitHistory();
  InitUpperReaches();
  InitLowerReaches();
}

template <typename MessageT>
HybridUpperReach<MessageT>::~HybridUpperReach() {
  ClearLowerReaches();
  ClearUpperReaches();
}

template <typename MessageT>
void HybridUpperReach<MessageT>::Enable() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& item : upper_reaches_) {
    item.second->Enable();
  }
}

template <typename MessageT>
void HybridUpperReach<MessageT>::Disable() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& item : upper_reaches_) {
    item.second->Disable();
  }
}

template <typename MessageT>
void HybridUpperReach<MessageT>::Enable(const RoleAttributes& opposite_attr) {
  auto relation = GetRelation(opposite_attr);
  if (relation == NO_RELATION) {
    return;
  }
  uint64_t id = opposite_attr.id();
  std::lock_guard<std::mutex> lock(mutex_);
  lower_reaches_[mapping_table_[relation]].insert(id);
  upper_reaches_[mapping_table_[relation]]->Enable();
  TransmitHistoryMsg(opposite_attr);
}

template <typename MessageT>
void HybridUpperReach<MessageT>::Disable(const RoleAttributes& opposite_attr) {
  auto relation = GetRelation(opposite_attr);
  if (relation == NO_RELATION) {
    return;
  }
  uint64_t id = opposite_attr.id();
  std::lock_guard<std::mutex> lock(mutex_);
  lower_reaches_[mapping_table_[relation]].erase(id);
  if (lower_reaches_[mapping_table_[relation]].empty()) {
    upper_reaches_[mapping_table_[relation]]->Disable();
  }
}

template <typename MessageT>
bool HybridUpperReach<MessageT>::Transmit(const MessagePtr& msg,
                                          const MessageInfo& msg_info) {
  std::lock_guard<std::mutex> lock(mutex_);
  history_->Add(msg, msg_info);
  for (auto& item : upper_reaches_) {
    item.second->Transmit(msg, msg_info);
  }
  return true;
}

template <typename MessageT>
void HybridUpperReach<MessageT>::InitMode() {
  mode_ = std::make_shared<proto::CommunicationMode>();
  mapping_table_[SAME_PROC] = mode_->same_proc();
  mapping_table_[DIFF_PROC] = mode_->diff_proc();
  mapping_table_[DIFF_HOST] = mode_->diff_host();
}

template <typename MessageT>
void HybridUpperReach<MessageT>::ObtainConfig() {
  auto global_conf = common::GlobalData::Instance()->Config();
  RETURN_IF(!global_conf.has_transport_conf());
  RETURN_IF(!global_conf.transport_conf().has_communication_mode());

  mode_->CopyFrom(global_conf.transport_conf().communication_mode());

  mapping_table_[SAME_PROC] = mode_->same_proc();
  mapping_table_[DIFF_PROC] = mode_->diff_proc();
  mapping_table_[DIFF_HOST] = mode_->diff_host();
}

template <typename MessageT>
void HybridUpperReach<MessageT>::InitHistory() {
  HistoryAttributes history_attr(this->attr_.qos_profile().history(),
                                 this->attr_.qos_profile().depth());
  history_ = std::make_shared<History<MessageT>>(history_attr);
  if (this->attr_.qos_profile().durability() ==
      QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL) {
    history_->Enable();
  }
}

template <typename MessageT>
void HybridUpperReach<MessageT>::InitUpperReaches() {
  std::set<OptionalMode> modes;
  modes.insert(mode_->same_proc());
  modes.insert(mode_->diff_proc());
  modes.insert(mode_->diff_host());
  for (auto& mode : modes) {
    switch (mode) {
      case OptionalMode::INTRA:
        upper_reaches_[mode] =
            std::make_shared<IntraUpperReach<MessageT>>(this->attr_);
        break;
      case OptionalMode::SHM:
        upper_reaches_[mode] =
            std::make_shared<ShmUpperReach<MessageT>>(this->attr_);
        break;
      default:
        upper_reaches_[mode] = std::make_shared<RtpsUpperReach<MessageT>>(
            this->attr_, participant_);
        break;
    }
  }
}

template <typename MessageT>
void HybridUpperReach<MessageT>::ClearUpperReaches() {
  for (auto& item : upper_reaches_) {
    item.second->Disable();
  }
  upper_reaches_.clear();
}

template <typename MessageT>
void HybridUpperReach<MessageT>::InitLowerReaches() {
  std::set<uint64_t> empty;
  for (auto& item : upper_reaches_) {
    lower_reaches_[item.first] = empty;
  }
}

template <typename MessageT>
void HybridUpperReach<MessageT>::ClearLowerReaches() {
  lower_reaches_.clear();
}

template <typename MessageT>
void HybridUpperReach<MessageT>::TransmitHistoryMsg(
    const RoleAttributes& opposite_attr) {
  // check qos
  if (this->attr_.qos_profile().durability() !=
          QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL ||
      opposite_attr.qos_profile().durability() !=
          QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL) {
    return;
  }

  auto trans_th =
      std::thread(&HybridUpperReach<MessageT>::ThreadFunc, this, opposite_attr);
  trans_th.detach();
}

template <typename MessageT>
void HybridUpperReach<MessageT>::ThreadFunc(
    const RoleAttributes& opposite_attr) {
  // get unsent messages
  std::vector<typename History<MessageT>::CachedMessage> unsent_msgs;
  history_->GetCachedMessage(&unsent_msgs);
  if (unsent_msgs.empty()) {
    return;
  }
  // create upper_reach to transmit msgs
  RoleAttributes new_attr;
  new_attr.CopyFrom(this->attr_);
  std::string new_channel_name =
      std::to_string(this->attr_.id()) + std::to_string(opposite_attr.id());
  uint64_t channel_id = common::GlobalData::RegisterChannel(new_channel_name);
  new_attr.set_channel_name(new_channel_name);
  new_attr.set_channel_id(channel_id);
  auto new_upper_reach =
      std::make_shared<RtpsUpperReach<MessageT>>(new_attr, participant_);
  new_upper_reach->Enable();

  for (auto& item : unsent_msgs) {
    new_upper_reach->Transmit(item.msg, item.msg_info);
  }
  new_upper_reach->Disable();
  ADEBUG << "trans threadfunc exit.";
}

template <typename MessageT>
Relation HybridUpperReach<MessageT>::GetRelation(
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

#endif  // CYBERTRON_TRANSPORT_UPPER_REACH_HYBRID_UPPER_REACH_H_
