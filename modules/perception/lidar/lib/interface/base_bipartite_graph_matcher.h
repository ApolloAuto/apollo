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
#ifndef PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_BASE_BIPARTITE_GRAPH_MATCHER_H_
#define PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_BASE_BIPARTITE_GRAPH_MATCHER_H_

#include <Eigen/Core>
#include <string>
#include <utility>
#include <vector>

#include "modules/perception/common/graph/secure_matrix.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/lidar/common/lidar_log.h"

namespace apollo {
namespace perception {
namespace lidar {

struct BipartiteGraphMatcherInitOptions {};

struct BipartiteGraphMatcherOptions {
  float cost_thresh = 4.0;
  float bound_value = 100.0;
};

class BaseBipartiteGraphMatcher {
 public:
  typedef std::pair<size_t, size_t> NodeNodePair;
  BaseBipartiteGraphMatcher() = default;
  virtual ~BaseBipartiteGraphMatcher() = default;

  // @params[OUT] assignments: matched pair of objects & tracks
  // @params[OUT] unassigned_rows: unmatched rows
  // @params[OUT] unassigned_cols: unmatched cols
  // @return nothing
  virtual void Match(const BipartiteGraphMatcherOptions &options,
                     std::vector<NodeNodePair> *assignments,
                     std::vector<size_t> *unassigned_rows,
                     std::vector<size_t> *unassigned_cols) = 0;
  virtual std::string Name() const = 0;

  virtual common::SecureMat<float> *cost_matrix() { return cost_matrix_; }

  BaseBipartiteGraphMatcher(const BaseBipartiteGraphMatcher &) = delete;
  BaseBipartiteGraphMatcher &operator=(const BaseBipartiteGraphMatcher &) =
      delete;

 protected:
  common::SecureMat<float> *cost_matrix_ = nullptr;
  float max_match_distance_ = 0.0f;
};  // class BaseBipartiteGraphMatcher

PERCEPTION_REGISTER_REGISTERER(BaseBipartiteGraphMatcher);
#define PERCEPTION_REGISTER_BIPARTITEGRAPHMATCHER(name) \
  PERCEPTION_REGISTER_CLASS(BaseBipartiteGraphMatcher, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_BASE_BIPARTITE_GRAPH_MATCHER_H_
