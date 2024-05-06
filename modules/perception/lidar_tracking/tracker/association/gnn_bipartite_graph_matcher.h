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
#include <vector>

#include "modules/perception/common/algorithm/graph/secure_matrix.h"
#include "modules/perception/lidar_tracking/interface/base_bipartite_graph_matcher.h"

namespace apollo {
namespace perception {
namespace lidar {

class MatchCost {
 public:
  MatchCost(size_t ridx, size_t cidx, double cost);
  /**
   * @brief Access RowIdx
   *
   * @return size_t
   */
  size_t RowIdx() const;
  /**
   * @brief Access ColIdx
   *
   * @return size_t
   */
  size_t ColIdx() const;
  /**
   * @brief Access Cost
   *
   * @return double
   */
  double Cost() const;

  friend bool operator<(const MatchCost& m1, const MatchCost& m2);
  friend std::ostream& operator<<(std::ostream& os, const MatchCost& m);

 private:
  size_t row_idx_ = 0;
  size_t col_idx_ = 0;
  double cost_ = 0.0;
};

class GnnBipartiteGraphMatcher : public BaseBipartiteGraphMatcher {
 public:
  explicit GnnBipartiteGraphMatcher(size_t max_size = 1000);
  ~GnnBipartiteGraphMatcher();

  /**
   * @brief Match interface
   *
   * @param options match params
   * @param assignments matched pair of objects & tracks
   * @param unassigned_rows unmatched rows
   * @param unassigned_cols unmatched cols
   */
  void Match(const BipartiteGraphMatcherOptions& options,
             std::vector<NodeNodePair>* assignments,
             std::vector<size_t>* unassigned_rows,
             std::vector<size_t>* unassigned_cols);
  /**
   * @brief Get class name
   *
   * @return std::string
   */
  std::string Name() const { return "GnnBipartiteGraphMatcher"; }

 protected:
  std::vector<int> row_tag_;
  std::vector<int> col_tag_;
  std::vector<int> ignore_track_;
};  // class GnnBipartiteGraphMatcher

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
