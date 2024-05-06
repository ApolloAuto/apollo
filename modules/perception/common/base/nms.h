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
#pragma once

#include <vector>
#include <algorithm>
#include <limits>
#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace base {


void GetMaxScoreIndex(const std::vector<float> &scores, const float threshold,
                      const int top_k,
                      std::vector<std::pair<float, int>> *score_index_vec) {
  ACHECK(score_index_vec != nullptr);

  for (size_t i = 0; i < scores.size(); ++i) {
    if (scores[i] > threshold) {
      score_index_vec->emplace_back(scores[i], i);
    }
  }

  std::sort(score_index_vec->begin(), score_index_vec->end(),
            [](const std::pair<float, int> &a, const std::pair<float, int> &b) {
              return a.first > b.first;
            });

  if (top_k > 0 && top_k < static_cast<int>(score_index_vec->size())) {
    score_index_vec->resize(top_k);
  }
}

template <typename BoxType>
void Nms(const std::vector<BoxType> &bboxes, const std::vector<float> &scores,
         const float score_threshold, const float nms_threshold,
         const float eta, const int top_k, std::vector<int> *indices,
         float (*ComputeOverlap)(const BoxType &, const BoxType &),
         int limit = std::numeric_limits<int>::max) {
  ACHECK(bboxes.size() == scores.size());
  ACHECK(indices != nullptr);

  std::vector<std::pair<float, int>> score_index_vec;
  GetMaxScoreIndex(scores, score_threshold, top_k, &score_index_vec);

  float adaptive_threshold = nms_threshold;
  indices->clear();
  for (const std::pair<float, int> &score_index : score_index_vec) {
    const int idx = score_index.second;
    bool keep = true;
    for (const int &kept_idx : *indices) {
      float overlap = ComputeOverlap(bboxes[idx], bboxes[kept_idx]);
      if (overlap > adaptive_threshold) {
        keep = false;
        break;
      }
    }

    if (keep) {
      indices->push_back(idx);
      if (static_cast<int>(indices->size()) >= limit) {
        break;
      }
    }

    if (keep && eta < 1 && adaptive_threshold > 0.5) {
      adaptive_threshold *= eta;
    }
  }
}

}  // namespace base
}  // namespace perception
}  // namespace apollo
