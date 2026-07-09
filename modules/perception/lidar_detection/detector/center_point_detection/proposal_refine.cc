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

#include "modules/perception/lidar_detection/detector/center_point_detection/proposal_refine.h"

#include <algorithm>
#include <chrono>
#include <limits>

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace perception {
namespace lidar {
namespace {

constexpr char kMockZeroMode[] = "mock_zero";
constexpr char kMockDeltaMode[] = "mock_delta";
constexpr float kMinBoxDimension = 1.0e-3f;

}  // namespace

bool ProposalRefineModule::Init(
    const centerpoint::ProposalRefineParam& config) {
  enabled_ = config.enable_proposal_refine();
  mode_ = config.proposal_refine_mode();
  if (mode_.empty()) {
    mode_ = kMockZeroMode;
  }
  top_k_ = config.proposal_refine_top_k();
  score_threshold_ = config.proposal_refine_score_threshold();
  delta_scale_ = config.proposal_refine_delta_scale();
  debug_log_ = config.proposal_refine_debug_log();
  strict_refine_ = config.strict_refine();
  refine_call_count_ = 0;

  if (!enabled_) {
    return true;
  }
  if (top_k_ < -1) {
    AERROR << "proposal_refine_top_k must be -1 or non-negative.";
    return false;
  }
  if (mode_ != kMockZeroMode && mode_ != kMockDeltaMode) {
    AERROR << "Unsupported proposal refine mode: " << mode_;
    return false;
  }
  return true;
}

bool ProposalRefineModule::Refine(std::vector<Proposal>* proposals,
                                  ProposalRefineStats* stats) {
  if (proposals == nullptr || stats == nullptr) {
    return false;
  }
  ++refine_call_count_;

  const auto start = std::chrono::steady_clock::now();
  stats->proposal_count = proposals->size();
  const std::vector<size_t> selected_indices =
      SelectProposalIndices(*proposals);
  stats->refined_proposal_count = selected_indices.size();
  stats->delta_min = 0.0f;
  stats->delta_max = 0.0f;

  if (mode_ == kMockDeltaMode) {
    if (!selected_indices.empty()) {
      const ProposalDelta delta = GetMockDelta();
      stats->delta_min = std::numeric_limits<float>::max();
      stats->delta_max = std::numeric_limits<float>::lowest();
      const float raw_deltas[] = {
          delta.delta_x,      delta.delta_y,     delta.delta_z,
          delta.delta_length, delta.delta_width, delta.delta_height,
          delta.delta_yaw,    delta.delta_score,
      };
      for (const float value : raw_deltas) {
        stats->delta_min = std::min(stats->delta_min, value);
        stats->delta_max = std::max(stats->delta_max, value);
      }
      for (const size_t index : selected_indices) {
        ApplyProposalDelta(delta, delta_scale_, &proposals->at(index));
      }
    }
  } else if (mode_ != kMockZeroMode) {
    return false;
  }

  const auto end = std::chrono::steady_clock::now();
  stats->latency_ms =
      std::chrono::duration<double, std::milli>(end - start).count();
  return true;
}

std::vector<size_t> ProposalRefineModule::SelectProposalIndices(
    const std::vector<Proposal>& proposals) const {
  std::vector<size_t> selected_indices;
  selected_indices.reserve(proposals.size());
  for (size_t i = 0; i < proposals.size(); ++i) {
    if (proposals[i].score >= score_threshold_) {
      selected_indices.push_back(i);
    }
  }

  std::stable_sort(selected_indices.begin(), selected_indices.end(),
                   [&proposals](size_t lhs, size_t rhs) {
                     return proposals[lhs].score > proposals[rhs].score;
                   });

  if (top_k_ >= 0 && selected_indices.size() > static_cast<size_t>(top_k_)) {
    selected_indices.resize(static_cast<size_t>(top_k_));
  }
  return selected_indices;
}

ProposalDelta GetMockDelta() {
  ProposalDelta delta;
  delta.delta_x = 0.1f;
  delta.delta_yaw = 0.05f;
  delta.delta_score = 0.01f;
  return delta;
}

void ApplyProposalDelta(const ProposalDelta& delta, float delta_scale,
                        Proposal* proposal) {
  if (proposal == nullptr) {
    return;
  }
  proposal->x += delta_scale * delta.delta_x;
  proposal->y += delta_scale * delta.delta_y;
  proposal->z += delta_scale * delta.delta_z;
  proposal->length = std::max(
      kMinBoxDimension, proposal->length + delta_scale * delta.delta_length);
  proposal->width = std::max(kMinBoxDimension,
                             proposal->width + delta_scale * delta.delta_width);
  proposal->height = std::max(
      kMinBoxDimension, proposal->height + delta_scale * delta.delta_height);
  proposal->yaw = static_cast<float>(apollo::common::math::NormalizeAngle(
      proposal->yaw + delta_scale * delta.delta_yaw));
  proposal->score = apollo::common::math::Clamp(
      proposal->score + delta_scale * delta.delta_score, 0.0f, 1.0f);
}

bool MaybeRefineProposals(ProposalRefineModule* module,
                          std::vector<Proposal>* proposals,
                          ProposalRefineStats* stats) {
  if (module == nullptr) {
    return false;
  }
  if (!module->enabled()) {
    if (stats != nullptr) {
      *stats = ProposalRefineStats();
    }
    return true;
  }
  return module->Refine(proposals, stats);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
