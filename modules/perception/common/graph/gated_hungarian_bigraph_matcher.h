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
#include <functional>
#include <map>
#include <utility>
#include <vector>

#include "cyber/common/log.h"

#include "modules/perception/common/graph/connected_component_analysis.h"
#include "modules/perception/common/graph/hungarian_optimizer.h"

namespace apollo {
namespace perception {
namespace common {

template <typename T>
class GatedHungarianMatcher {
 public:
  enum class OptimizeFlag { OPTMAX, OPTMIN };

  explicit GatedHungarianMatcher(int max_matching_size = 1000) {
    global_costs_.Reserve(max_matching_size, max_matching_size);
    optimizer_.costs()->Reserve(max_matching_size, max_matching_size);
  }
  ~GatedHungarianMatcher() {}

  /* @brief: global_costs is the memory we reserved for the updating of
   * costs of matching. it could & need be updated outside the matcher,
   * before each matching. use it carefully, and make sure all the
   * elements of the global_costs is updated as you presumed. resize it
   * and update it completely is STRONG RECOMMENDED!! P.S. resizing SecureMat
   * would not alloc new memory, if the resizing size is smaller than the
   * size reserved. */
  const SecureMat<T>& global_costs() const { return global_costs_; }
  SecureMat<T>* mutable_global_costs() { return &global_costs_; }

  void Match(T cost_thresh, OptimizeFlag opt_flag,
             std::vector<std::pair<size_t, size_t>>* assignments,
             std::vector<size_t>* unassigned_rows,
             std::vector<size_t>* unassigned_cols);

  void Match(T cost_thresh, T bound_value, OptimizeFlag opt_flag,
             std::vector<std::pair<size_t, size_t>>* assignments,
             std::vector<size_t>* unassigned_rows,
             std::vector<size_t>* unassigned_cols);

 private:
  /* Step 1:
   * a. get number of rows & cols
   * b. determine function of comparison */
  void MatchInit();

  /* Step 2:
   * to acclerate matching process, split input cost graph into several
   * small sub-parts. */
  void ComputeConnectedComponents(
      std::vector<std::vector<size_t>>* row_components,
      std::vector<std::vector<size_t>>* col_components) const;

  /* Step 3:
   * optimize single connected component, which is part of the global one */
  void OptimizeConnectedComponent(const std::vector<size_t>& row_component,
                                  const std::vector<size_t>& col_component);

  /* Step 4:
   * generate the set of unassigned row or col index. */
  void GenerateUnassignedData(std::vector<size_t>* unassigned_rows,
                              std::vector<size_t>* unassigned_cols) const;

  /* @brief: core function for updating the local cost matrix from global one,
   * we get queryed local costs and write them in the memeory of costs of
   * optimizer directly
   * @params[IN] row_component: the set of index of rows of sub-graph
   * @params[IN] col_component: the set of index of cols of sub-graph
   * @return: nothing */
  void UpdateGatingLocalCostsMat(const std::vector<size_t>& row_component,
                                 const std::vector<size_t>& col_component);

  void OptimizeAdapter(
      std::vector<std::pair<size_t, size_t>>* local_assignments);

  /* Hungarian optimizer */
  HungarianOptimizer<T> optimizer_;

  /* global costs matrix */
  SecureMat<T> global_costs_;

  /* input data */
  T cost_thresh_ = 0.0;
  T bound_value_ = 0.0;
  OptimizeFlag opt_flag_ = OptimizeFlag::OPTMIN;

  /* output data */
  mutable std::vector<std::pair<size_t, size_t>>* assignments_ptr_ = nullptr;

  /* size of component */
  size_t rows_num_ = 0;
  size_t cols_num_ = 0;

  /* the rhs is always better than lhs */
  std::function<bool(T, T)> compare_fun_;
  std::function<bool(T)> is_valid_cost_;
};  // class GatedHungarianMatcher

template <typename T>
void GatedHungarianMatcher<T>::Match(
    T cost_thresh, OptimizeFlag opt_flag,
    std::vector<std::pair<size_t, size_t>>* assignments,
    std::vector<size_t>* unassigned_rows,
    std::vector<size_t>* unassigned_cols) {
  Match(cost_thresh, cost_thresh, opt_flag, assignments, unassigned_rows,
        unassigned_cols);
}

template <typename T>
void GatedHungarianMatcher<T>::Match(
    T cost_thresh, T bound_value, OptimizeFlag opt_flag,
    std::vector<std::pair<size_t, size_t>>* assignments,
    std::vector<size_t>* unassigned_rows,
    std::vector<size_t>* unassigned_cols) {
  CHECK_NOTNULL(assignments);
  CHECK_NOTNULL(unassigned_rows);
  CHECK_NOTNULL(unassigned_cols);

  /* initialize matcher */
  cost_thresh_ = cost_thresh;
  opt_flag_ = opt_flag;
  bound_value_ = bound_value;
  assignments_ptr_ = assignments;
  MatchInit();

  /* compute components */
  std::vector<std::vector<size_t>> row_components;
  std::vector<std::vector<size_t>> col_components;
  this->ComputeConnectedComponents(&row_components, &col_components);
  CHECK_EQ(row_components.size(), col_components.size());

  /* compute assignments */
  assignments_ptr_->clear();
  assignments_ptr_->reserve(std::max(rows_num_, cols_num_));
  for (size_t i = 0; i < row_components.size(); ++i) {
    this->OptimizeConnectedComponent(row_components[i], col_components[i]);
  }

  this->GenerateUnassignedData(unassigned_rows, unassigned_cols);
}

template <typename T>
void GatedHungarianMatcher<T>::MatchInit() {
  /* get number of rows & cols */
  rows_num_ = global_costs_.height();
  cols_num_ = (rows_num_ == 0) ? 0 : global_costs_.width();

  /* determine function of comparison */
  static std::map<OptimizeFlag, std::function<bool(T, T)>> compare_fun_map = {
      {OptimizeFlag::OPTMAX, std::less<T>()},
      {OptimizeFlag::OPTMIN, std::greater<T>()},
  };
  auto find_ret = compare_fun_map.find(opt_flag_);
  CHECK(find_ret != compare_fun_map.end());
  compare_fun_ = find_ret->second;
  is_valid_cost_ = std::bind1st(compare_fun_, cost_thresh_);

  /* check the validity of bound_value */
  CHECK(!is_valid_cost_(bound_value_));
}

template <typename T>
void GatedHungarianMatcher<T>::ComputeConnectedComponents(
    std::vector<std::vector<size_t>>* row_components,
    std::vector<std::vector<size_t>>* col_components) const {
  CHECK_NOTNULL(row_components);
  CHECK_NOTNULL(col_components);

  std::vector<std::vector<int>> nb_graph;
  nb_graph.resize(rows_num_ + cols_num_);
  for (int i = 0; i < rows_num_; ++i) {
    for (int j = 0; j < cols_num_; ++j) {
      if (is_valid_cost_(global_costs_(i, j))) {
        nb_graph[i].push_back(static_cast<int>(rows_num_) + j);
        nb_graph[j + rows_num_].push_back(i);
      }
    }
  }

  std::vector<std::vector<int>> components;
  ConnectedComponentAnalysis(nb_graph, &components);
  row_components->clear();
  row_components->resize(components.size());
  col_components->clear();
  col_components->resize(components.size());
  for (size_t i = 0; i < components.size(); ++i) {
    for (size_t j = 0; j < components[i].size(); ++j) {
      int id = components[i][j];
      if (id < rows_num_) {
        row_components->at(i).push_back(id);
      } else {
        id -= static_cast<int>(rows_num_);
        col_components->at(i).push_back(id);
      }
    }
  }
}

template <typename T>
void GatedHungarianMatcher<T>::OptimizeConnectedComponent(
    const std::vector<size_t>& row_component,
    const std::vector<size_t>& col_component) {
  size_t local_rows_num = row_component.size();
  size_t local_cols_num = col_component.size();

  /* simple case 1: no possible matches */
  if (!local_rows_num || !local_cols_num) {
    return;
  }
  /* simple case 2: 1v1 pair with no ambiguousness */
  if (local_rows_num == 1 && local_cols_num == 1) {
    size_t idx_r = row_component[0];
    size_t idx_c = col_component[0];
    if (is_valid_cost_(global_costs_(idx_r, idx_c))) {
      assignments_ptr_->push_back(std::make_pair(idx_r, idx_c));
    }
    return;
  }

  /* update local cost matrix */
  UpdateGatingLocalCostsMat(row_component, col_component);

  /* get local assignments */
  std::vector<std::pair<size_t, size_t>> local_assignments;
  OptimizeAdapter(&local_assignments);

  /* parse local assginments into global ones */
  for (size_t i = 0; i < local_assignments.size(); ++i) {
    auto local_assignment = local_assignments[i];
    size_t global_row_idx = row_component[local_assignment.first];
    size_t global_col_idx = col_component[local_assignment.second];
    if (!is_valid_cost_(global_costs_(global_row_idx, global_col_idx))) {
      continue;
    }
    assignments_ptr_->push_back(std::make_pair(global_row_idx, global_col_idx));
  }
}

template <typename T>
void GatedHungarianMatcher<T>::GenerateUnassignedData(
    std::vector<size_t>* unassigned_rows,
    std::vector<size_t>* unassigned_cols) const {
  CHECK_NOTNULL(unassigned_rows);
  CHECK_NOTNULL(unassigned_cols);

  const auto assignments = *assignments_ptr_;
  unassigned_rows->clear(), unassigned_rows->reserve(rows_num_);
  unassigned_cols->clear(), unassigned_cols->reserve(cols_num_);
  std::vector<bool> row_assignment_flags(rows_num_, false);
  std::vector<bool> col_assignment_flags(cols_num_, false);
  for (const auto& assignment : assignments) {
    row_assignment_flags[assignment.first] = true;
    col_assignment_flags[assignment.second] = true;
  }
  for (size_t i = 0; i < row_assignment_flags.size(); ++i) {
    if (!row_assignment_flags[i]) {
      unassigned_rows->push_back(i);
    }
  }
  for (size_t i = 0; i < col_assignment_flags.size(); ++i) {
    if (!col_assignment_flags[i]) {
      unassigned_cols->push_back(i);
    }
  }
}

template <typename T>
void GatedHungarianMatcher<T>::UpdateGatingLocalCostsMat(
    const std::vector<size_t>& row_component,
    const std::vector<size_t>& col_component) {
  /* set the invalid cost to bound value */
  SecureMat<T>* local_costs = optimizer_.costs();
  local_costs->Resize(row_component.size(), col_component.size());
  for (size_t i = 0; i < row_component.size(); ++i) {
    for (size_t j = 0; j < col_component.size(); ++j) {
      T& current_cost = global_costs_(row_component[i], col_component[j]);
      if (is_valid_cost_(current_cost)) {
        (*local_costs)(i, j) = current_cost;
      } else {
        (*local_costs)(i, j) = bound_value_;
      }
    }
  }
}

template <typename T>
void GatedHungarianMatcher<T>::OptimizeAdapter(
    std::vector<std::pair<size_t, size_t>>* local_assignments) {
  CHECK_NOTNULL(local_assignments);
  if (opt_flag_ == OptimizeFlag::OPTMAX) {
    optimizer_.Maximize(local_assignments);
  } else {
    optimizer_.Minimize(local_assignments);
  }
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
