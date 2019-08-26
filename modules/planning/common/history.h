/******************************************************************************
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

#pragma once

#include <list>
#include <string>
#include <unordered_map>
#include <vector>

#include "cyber/common/macros.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {

class HistoryObjectDecision {
 public:
  HistoryObjectDecision() = default;

  void Init(const ObjectDecision& object_decisions);

  const std::string& id() const { return id_; }
  const std::vector<const ObjectDecisionType*> GetObjectDecision() const;

 private:
  std::string id_;
  std::vector<ObjectDecisionType> object_decision_;
};

class HistoryFrame {
 public:
  HistoryFrame() = default;

  void Init(const ADCTrajectory& adc_trajactory);
  int seq_num() const { return seq_num_; }
  const std::vector<const HistoryObjectDecision*> GetObjectDecisions() const;
  const HistoryObjectDecision* GetObjectDecisionsById(
      const std::string& id) const;

 private:
  int seq_num_;
  ADCTrajectory adc_trajactory_;
  std::unordered_map<std::string, HistoryObjectDecision> object_decisions_map_;
  std::vector<HistoryObjectDecision> object_decisions_;
};

class History {
 public:
  const HistoryFrame* GetLastFrame() const;
  int Add(const ADCTrajectory& adc_trajectory_pb);
  void Clear();
  size_t Size() const;

 private:
  std::list<HistoryFrame> history_frames_;

  // this is a singleton class
  DECLARE_SINGLETON(History)
};

}  // namespace planning
}  // namespace apollo
