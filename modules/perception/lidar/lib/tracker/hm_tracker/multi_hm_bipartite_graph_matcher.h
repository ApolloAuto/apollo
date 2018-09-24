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
#ifndef PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_MULTI_HM_BIPARTITE_GRAPH_MATCHER_H_
#define PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_MULTI_HM_BIPARTITE_GRAPH_MATCHER_H_

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>
#include "modules/perception/common/graph/gated_hungarian_bigraph_matcher.h"
#include "modules/perception/common/graph/secure_matrix.h"
#include "modules/perception/lidar/lib/interface/base_bipartite_graph_matcher.h"

namespace apollo {
namespace perception {
namespace lidar {

class MultiHmBipartiteGraphMatcher : public BaseBipartiteGraphMatcher {
 public:
  MultiHmBipartiteGraphMatcher();
  ~MultiHmBipartiteGraphMatcher();

  // @params[IN] options: params
  // @params[OUT] assignments: matched pair of objects & tracks
  // @params[OUT] unassigned_rows: unmatched rows
  // @params[OUT] unassigned_cols: unmatched cols
  // @return nothing
  void Match(const BipartiteGraphMatcherOptions &options,
             std::vector<NodeNodePair> *assignments,
             std::vector<size_t> *unassigned_rows,
             std::vector<size_t> *unassigned_cols);
  std::string Name() const { return "MultiHmBipartiteGraphMatcher"; }

 protected:
  common::GatedHungarianMatcher<float> optimizer_;
};  // class MultiHmObjectMatcher

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_MULTI_HM_BIPARTITE_GRAPH_MATCHER_H_
