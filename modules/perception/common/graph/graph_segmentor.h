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

#include <algorithm>
#include <vector>

#include "modules/perception/common/graph/disjoint_set.h"

namespace apollo {
namespace perception {
namespace common {

// @brief: graph edge definition
struct Edge {
  float w = 0.0f;
  int a = 0;
  int b = 0;
  // @brief: edge comparison
  bool operator<(const Edge& other) const { return this->w < other.w; }
};

class GraphSegmentor {
 public:
  GraphSegmentor() = default;
  ~GraphSegmentor() = default;

  // @brief: initialize thresholds
  void Init(const float initial_threshold);

  // @brief: segment a graph, generating a disjoint-set forest
  //         representing the segmentation.
  // @params[IN] num_vertices: number of vertices in graph.
  // @params[IN] num_edges: number of edges in graph.
  // @params[IN] edges: array of Edges.
  // @params[OUT] need_sort: whether input edges needs to be sorted
  void SegmentGraph(const int num_vertices, const int num_edges, Edge* edges,
                    bool need_sort = true);

  // @brief: return the disjoint-set forest as the segmentation result.
  Universe* mutable_universe() { return &universe_; }

  const Universe& universe() { return universe_; }

 private:
  static const size_t kMaxVerticesNum = 10000;
  static const size_t kMaxThresholdsNum = 50000;
  float initial_threshold_ = 0.f;
  std::vector<float> thresholds_;
  std::vector<float> thresholds_table_;
  Universe universe_;
};  // class GraphSegmentor

}  // namespace common
}  // namespace perception
}  // namespace apollo
