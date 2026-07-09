/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "modules/perception/lidar_detection/detector/center_point_detection/proto/model_param.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

struct Proposal {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float length = 0.0f;
  float width = 0.0f;
  float height = 0.0f;
  float yaw = 0.0f;
  float score = 0.0f;
  int class_id = 0;
};

struct ProposalDelta {
  float delta_x = 0.0f;
  float delta_y = 0.0f;
  float delta_z = 0.0f;
  float delta_length = 0.0f;
  float delta_width = 0.0f;
  float delta_height = 0.0f;
  float delta_yaw = 0.0f;
  float delta_score = 0.0f;
};

struct ProposalRefineStats {
  size_t proposal_count = 0;
  size_t refined_proposal_count = 0;
  float delta_min = 0.0f;
  float delta_max = 0.0f;
  double latency_ms = 0.0;
};

class ProposalRefineModule {
 public:
  ProposalRefineModule() = default;
  ~ProposalRefineModule() = default;
  ProposalRefineModule(const ProposalRefineModule&) = delete;
  ProposalRefineModule& operator=(const ProposalRefineModule&) = delete;

  bool Init(const centerpoint::ProposalRefineParam& config);

  bool enabled() const { return enabled_; }
  bool debug_log() const { return debug_log_; }
  bool strict_refine() const { return strict_refine_; }
  const std::string& mode() const { return mode_; }
  int refine_call_count() const { return refine_call_count_; }

  bool Refine(std::vector<Proposal>* proposals, ProposalRefineStats* stats);

 private:
  std::vector<size_t> SelectProposalIndices(
      const std::vector<Proposal>& proposals) const;

  bool enabled_ = false;
  std::string mode_ = "mock_zero";
  int top_k_ = -1;
  float score_threshold_ = 0.0f;
  float delta_scale_ = 1.0f;
  bool debug_log_ = false;
  bool strict_refine_ = true;
  int refine_call_count_ = 0;
};

ProposalDelta GetMockDelta();
void ApplyProposalDelta(const ProposalDelta& delta, float delta_scale,
                        Proposal* proposal);

bool MaybeRefineProposals(ProposalRefineModule* module,
                          std::vector<Proposal>* proposals,
                          ProposalRefineStats* stats);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
