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
#include "modules/common_msgs/planning_msgs/planning.pb.h"

namespace apollo {
namespace planning {

class HistoryObjectDecision {
 public:
  HistoryObjectDecision() = default;

  void Init(const ObjectDecision& object_decisions);
  void Init(const std::string& id,
            const std::vector<ObjectDecisionType>& object_decisions);

  const std::string& id() const { return id_; }
  std::vector<const ObjectDecisionType*> GetObjectDecision() const;

 private:
  std::string id_;
  std::vector<ObjectDecisionType> object_decision_;
};

class HistoryFrame {
 public:
  HistoryFrame() = default;

  void Init(const ADCTrajectory& adc_trajactory);

  int seq_num() const { return seq_num_; }

  std::vector<const HistoryObjectDecision*> GetObjectDecisions() const;
  std::vector<const HistoryObjectDecision*> GetStopObjectDecisions() const;

  const HistoryObjectDecision* GetObjectDecisionsById(
      const std::string& id) const;

 private:
  int seq_num_;
  ADCTrajectory adc_trajactory_;
  std::unordered_map<std::string, HistoryObjectDecision> object_decisions_map_;
  std::vector<HistoryObjectDecision> object_decisions_;
};

class HistoryObjectStatus {
 public:
  HistoryObjectStatus() = default;

  void Init(const std::string& id, const ObjectStatus& object_status);

  const std::string& id() const { return id_; }
  const ObjectStatus GetObjectStatus() const { return object_status_; }

 private:
  std::string id_;
  ObjectStatus object_status_;
};

class HistoryStatus {
 public:
  HistoryStatus() = default;

  void SetObjectStatus(const std::string& id,
                       const ObjectStatus& object_status);

  bool GetObjectStatus(const std::string& id,
                       ObjectStatus* const object_status);

 private:
  std::unordered_map<std::string, ObjectStatus> object_id_to_status_;
};

class History {
 public:
  History() = default;
  const HistoryFrame* GetLastFrame() const;
  int Add(const ADCTrajectory& adc_trajectory_pb);
  void Clear();
  size_t Size() const;
  HistoryStatus* mutable_history_status() { return &history_status_; }

 private:
  std::list<HistoryFrame> history_frames_;
  HistoryStatus history_status_;
};

}  // namespace planning
}  // namespace apollo
