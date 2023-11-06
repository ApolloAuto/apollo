/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/radar4d_detection/lib/tracker/association/multi_hm_bipartite_graph_matcher.h"

#include "cyber/common/log.h"
#include "modules/perception/common/algorithm/graph/gated_hungarian_bigraph_matcher.h"

namespace apollo {
namespace perception {
namespace radar4d {

MultiHmBipartiteGraphMatcher::MultiHmBipartiteGraphMatcher() {
  cost_matrix_ = optimizer_.mutable_global_costs();
}

MultiHmBipartiteGraphMatcher::~MultiHmBipartiteGraphMatcher() {
  cost_matrix_ = nullptr;
}

void MultiHmBipartiteGraphMatcher::Match(
    const BipartiteGraphMatcherOptions &options,
    std::vector<NodeNodePair> *assignments,
    std::vector<size_t> *unassigned_rows,
    std::vector<size_t> *unassigned_cols) {
  algorithm::GatedHungarianMatcher<float>::OptimizeFlag opt_flag =
      algorithm::GatedHungarianMatcher<float>::OptimizeFlag::OPTMIN;
  optimizer_.Match(options.cost_thresh, options.bound_value, opt_flag,
                   assignments, unassigned_rows, unassigned_cols);
}

PERCEPTION_REGISTER_BIPARTITEGRAPHMATCHER(MultiHmBipartiteGraphMatcher);

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
