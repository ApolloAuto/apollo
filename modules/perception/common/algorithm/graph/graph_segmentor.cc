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

#include "modules/perception/common/algorithm/graph/graph_segmentor.h"

#include <limits>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace algorithm {

namespace {
float GetThreshold(const size_t sz, const float c) {
  return c / static_cast<float>(sz);
}
}  // namespace

void GraphSegmentor::Init(const float initial_threshold) {
  initial_threshold_ = initial_threshold;
  thresholds_.reserve(kMaxVerticesNum);

  thresholds_table_.resize(kMaxThresholdsNum);
  thresholds_table_[0] = std::numeric_limits<float>::max();
  for (size_t i = 1; i < kMaxThresholdsNum; ++i) {
    thresholds_table_[i] = GetThreshold(i, initial_threshold_);
  }
}

void GraphSegmentor::SegmentGraph(const int num_vertices, const int num_edges,
                                  Edge* edges, bool need_sort) {
  if (edges == nullptr) {
    AERROR << "Input Null Edges.";
    return;
  }

  if (need_sort) {
    std::sort(edges, edges + num_edges);
  }

  universe_.Reset(num_vertices);

  thresholds_.assign(num_vertices, initial_threshold_);
  for (int i = 0; i < num_edges; ++i) {
    Edge& edge = edges[i];
    int a = universe_.Find(edge.a);
    int b = universe_.Find(edge.b);
    if (a == b) {
      continue;
    }
    if (edge.w <= thresholds_[a] && edge.w <= thresholds_[b]) {
      universe_.Join(a, b);
      a = universe_.Find(a);
      int size_a = universe_.GetSize(a);
      thresholds_[a] =
          edge.w + (size_a < static_cast<int>(kMaxThresholdsNum)
                        ? thresholds_table_[size_a]
                        : GetThreshold(size_a, initial_threshold_));
    }
  }
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
