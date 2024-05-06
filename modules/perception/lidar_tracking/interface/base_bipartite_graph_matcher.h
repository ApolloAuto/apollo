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

#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"

#include "cyber/common/macros.h"
#include "modules/perception/common/algorithm/graph/secure_matrix.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::perception::BaseInitOptions;

struct BipartiteGraphMatcherInitOptions : public BaseInitOptions {};

struct BipartiteGraphMatcherOptions {
  float cost_thresh = 4.0f;
  float bound_value = 100.0f;
};

class BaseBipartiteGraphMatcher {
 public:
  typedef std::pair<size_t, size_t> NodeNodePair;
  BaseBipartiteGraphMatcher() = default;
  virtual ~BaseBipartiteGraphMatcher() = default;

  /**
   * @brief Match bipartite graph
   *
   * @param options
   * @param assignments matched pair of objects & tracks
   * @param unassigned_rows unmatched rows
   * @param unassigned_cols unmatched cols
   */
  virtual void Match(const BipartiteGraphMatcherOptions &options,
                     std::vector<NodeNodePair> *assignments,
                     std::vector<size_t> *unassigned_rows,
                     std::vector<size_t> *unassigned_cols) = 0;
  /**
   * @brief Get class name
   *
   * @return std::string
   */
  virtual std::string Name() const = 0;

  /**
   * @brief Get cost matrix
   *
   * @return algorithm::SecureMat<float>*
   */
  virtual algorithm::SecureMat<float> *cost_matrix() { return cost_matrix_; }

 protected:
  algorithm::SecureMat<float> *cost_matrix_ = nullptr;
  float max_match_distance_ = 0.0f;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseBipartiteGraphMatcher);
};  // class BaseBipartiteGraphMatcher

PERCEPTION_REGISTER_REGISTERER(BaseBipartiteGraphMatcher);
#define PERCEPTION_REGISTER_BIPARTITEGRAPHMATCHER(name) \
  PERCEPTION_REGISTER_CLASS(BaseBipartiteGraphMatcher, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
