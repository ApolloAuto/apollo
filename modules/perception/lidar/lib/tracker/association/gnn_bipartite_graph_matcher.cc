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

#include "modules/perception/lidar/lib/tracker/association/gnn_bipartite_graph_matcher.h"

#include <algorithm>
#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace lidar {

MatchCost::MatchCost(size_t ridx, size_t cidx, double cost)
    : row_idx_(ridx), col_idx_(cidx), cost_(cost) {}

size_t MatchCost::RowIdx() const { return row_idx_; }

size_t MatchCost::ColIdx() const { return col_idx_; }

double MatchCost::Cost() const { return cost_; }

bool operator<(const MatchCost& m1, const MatchCost& m2) {
  return m1.cost_ < m2.cost_;
}

std::ostream& operator<<(std::ostream& os, const MatchCost& m) {
  os << "MatchCost ridx:" << m.RowIdx() << " cidx:" << m.ColIdx()
     << " Cost:" << m.Cost();
  return os;
}

GnnBipartiteGraphMatcher::GnnBipartiteGraphMatcher(size_t max_size) {
  row_tag_.reserve(max_size);
  col_tag_.reserve(max_size);
  cost_matrix_ = new common::SecureMat<float>();
}

GnnBipartiteGraphMatcher::~GnnBipartiteGraphMatcher() {
  if (cost_matrix_ != nullptr) {
    delete cost_matrix_;
  }
}

void GnnBipartiteGraphMatcher::Match(
    const BipartiteGraphMatcherOptions& options,
    std::vector<NodeNodePair>* assignments,
    std::vector<size_t>* unassigned_rows,
    std::vector<size_t>* unassigned_cols) {
  assignments->clear();
  unassigned_rows->clear();
  unassigned_cols->clear();
  row_tag_.clear();
  col_tag_.clear();
  common::SecureMat<float>* cost_matrix = cost_matrix_;
  float max_dist = options.cost_thresh;
  int num_rows = static_cast<int>(cost_matrix->height());
  int num_cols = static_cast<int>(cost_matrix->width());
  row_tag_.assign(num_rows, 0);
  col_tag_.assign(num_cols, 0);

  std::vector<MatchCost> match_costs;
  for (int r = 0; r < num_rows; r++) {
    for (int c = 0; c < num_cols; c++) {
      if ((*cost_matrix)(r, c) < max_dist) {
        MatchCost item(r, c, (*cost_matrix)(r, c));
        match_costs.push_back(item);
      }
    }
  }

  // sort costs in ascending order
  std::sort(match_costs.begin(), match_costs.end());

  // gnn
  for (size_t i = 0; i < match_costs.size(); ++i) {
    size_t rid = match_costs[i].RowIdx();
    size_t cid = match_costs[i].ColIdx();
    if (row_tag_[rid] == 0 && col_tag_[cid] == 0) {
      row_tag_[rid] = 1;
      col_tag_[cid] = 1;
      assignments->push_back(std::make_pair(rid, cid));
    }
  }

  for (int i = 0; i < num_rows; i++) {
    if (row_tag_[i] == 0) {
      unassigned_rows->push_back(i);
    }
  }

  for (int i = 0; i < num_cols; i++) {
    if (col_tag_[i] == 0) {
      unassigned_cols->push_back(i);
    }
  }
}

PERCEPTION_REGISTER_BIPARTITEGRAPHMATCHER(GnnBipartiteGraphMatcher);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
