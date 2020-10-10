/*****************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include "modules/planning/common/history.h"

#include <utility>

#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

////////////////////////////////////////////////
// HistoryObjectDecision

void HistoryObjectDecision::Init(const ObjectDecision& object_decisions) {
  id_ = object_decisions.id();
  object_decision_.clear();
  for (int i = 0; i < object_decisions.object_decision_size(); i++) {
    object_decision_.push_back(object_decisions.object_decision(i));
  }
}

void HistoryObjectDecision::Init(
    const std::string& id,
    const std::vector<ObjectDecisionType>& object_decision) {
  id_ = id;
  object_decision_.clear();
  for (const auto decision_type : object_decision) {
    object_decision_.push_back(decision_type);
  }
}

std::vector<const ObjectDecisionType*>
HistoryObjectDecision::GetObjectDecision() const {
  std::vector<const ObjectDecisionType*> result;
  for (size_t i = 0; i < object_decision_.size(); i++) {
    result.push_back(&(object_decision_[i]));
  }

  return result;
}

////////////////////////////////////////////////
// HistoryFrame

void HistoryFrame::Init(const ADCTrajectory& adc_trajactory) {
  adc_trajactory_.CopyFrom(adc_trajactory);

  seq_num_ = adc_trajactory.header().sequence_num();
  const auto& object_decisions = adc_trajactory.decision().object_decision();
  for (int i = 0; i < object_decisions.decision_size(); i++) {
    const std::string id = object_decisions.decision(i).id();
    HistoryObjectDecision object_decision;
    object_decision.Init(object_decisions.decision(i));
    object_decisions_map_[id] = object_decision;

    object_decisions_.push_back(object_decision);
  }
}

std::vector<const HistoryObjectDecision*> HistoryFrame::GetObjectDecisions()
    const {
  std::vector<const HistoryObjectDecision*> result;
  for (size_t i = 0; i < object_decisions_.size(); i++) {
    result.push_back(&(object_decisions_[i]));
  }

  return result;
}

std::vector<const HistoryObjectDecision*> HistoryFrame::GetStopObjectDecisions()
    const {
  std::vector<const HistoryObjectDecision*> result;
  for (size_t i = 0; i < object_decisions_.size(); i++) {
    auto obj_decision = object_decisions_[i].GetObjectDecision();
    for (const ObjectDecisionType* decision_type : obj_decision) {
      if (decision_type->has_stop()) {
        std::vector<ObjectDecisionType> object_decision;
        object_decision.push_back(*decision_type);

        HistoryObjectDecision* decision = new HistoryObjectDecision();
        decision->Init(object_decisions_[i].id(), object_decision);
        result.push_back(decision);
      }
    }
  }

  // sort
  std::sort(
      result.begin(), result.end(),
      [](const HistoryObjectDecision* lhs, const HistoryObjectDecision* rhs) {
        return lhs->id() < rhs->id();
      });

  return result;
}

const HistoryObjectDecision* HistoryFrame::GetObjectDecisionsById(
    const std::string& id) const {
  if (object_decisions_map_.find(id) == object_decisions_map_.end()) {
    return nullptr;
  }
  return &(object_decisions_map_.at(id));
}

////////////////////////////////////////////////
// HistoryObjectStatus

void HistoryObjectStatus::Init(const std::string& id,
                               const ObjectStatus& object_status) {
  id_ = id;
  object_status_ = object_status;
}

////////////////////////////////////////////////
// HistoryStatus

void HistoryStatus::SetObjectStatus(const std::string& id,
                                    const ObjectStatus& object_status) {
  object_id_to_status_[id] = object_status;
}

bool HistoryStatus::GetObjectStatus(const std::string& id,
                                    ObjectStatus* const object_status) {
  if (object_id_to_status_.count(id) == 0) {
    return false;
  }

  *object_status = object_id_to_status_[id];
  return true;
}

////////////////////////////////////////////////
// History

const HistoryFrame* History::GetLastFrame() const {
  if (history_frames_.empty()) {
    return nullptr;
  } else {
    return &(history_frames_.back());
  }
}
void History::Clear() { history_frames_.clear(); }

int History::Add(const ADCTrajectory& adc_trajectory_pb) {
  if (history_frames_.size() >=
      static_cast<size_t>(FLAGS_history_max_record_num)) {
    history_frames_.pop_front();
  }

  HistoryFrame history_frame;
  history_frame.Init(adc_trajectory_pb);
  history_frames_.emplace_back(std::move(history_frame));

  return 0;
}

size_t History::Size() const { return history_frames_.size(); }

}  // namespace planning
}  // namespace apollo
