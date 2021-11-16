/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#define USR_INF 999999999999

namespace apollo {
namespace v2x {
namespace ft {

class KMkernal {
 public:
  KMkernal() = default;
  ~KMkernal() = default;
  template <typename T>
  bool GetKMResult(const T& association_mat,
                   std::vector<std::pair<int, int>>* match_cps,
                   bool need_reverse = false);

 private:
  int u_size_;
  int v_size_;
  double* ex_u_;
  double* ex_v_;
  int* v_matched_;
  double* v_slack_;
  std::set<int> used_u_;
  std::set<int> used_v_;
  template <typename T>
  bool FindCP(const T& mat, int i);
};
template <typename T>
bool KMkernal::GetKMResult(const T& association_mat,
                           std::vector<std::pair<int, int>>* match_cps,
                           bool need_reverse) {
  match_cps->clear();
  u_size_ = association_mat.rows();
  v_size_ = association_mat.cols();
  if (u_size_ > v_size_) return false;
  ex_u_ = new double[u_size_];
  ex_v_ = new double[v_size_];
  v_matched_ = new int[v_size_];
  std::fill(v_matched_, v_matched_ + v_size_, -1);
  memset(ex_v_, 0, v_size_ * sizeof(double));
  for (int i = 0; i < u_size_; ++i) {
    ex_u_[i] = association_mat(i, 0);
    for (int j = 1; j < v_size_; ++j) {
      ex_u_[i] = std::max(static_cast<float>(ex_u_[i]), association_mat(i, j));
    }
  }
  for (int i = 0; i < u_size_; ++i) {
    if (ex_u_[i] <= 0) {
      if (need_reverse)
        match_cps->push_back(std::make_pair(-1, i));
      else
        match_cps->push_back(std::make_pair(i, -1));
      continue;
    }
    v_slack_ = new double[v_size_];
    std::fill(v_slack_, v_slack_ + v_size_, USR_INF);
    while (1) {
      used_u_.clear();
      used_v_.clear();
      if (FindCP(association_mat, i)) break;
      double d = USR_INF;
      for (int j = 0; j < v_size_; ++j)
        if (used_v_.find(j) == used_v_.end()) d = std::min(d, v_slack_[j]);
      for (auto it = used_u_.begin(); it != used_u_.end(); it++) {
        ex_u_[*it] -= d;
      }
      for (int j = 0; j < v_size_; ++j) {
        if (used_v_.find(j) != used_v_.end())
          ex_v_[j] += d;
        else
          v_slack_[j] -= d;
      }
    }
    delete[] v_slack_;
  }
  if (need_reverse) {
    for (int j = 0; j < v_size_; ++j) {
      if (v_matched_[j] == -1) {
        match_cps->push_back(std::make_pair(j, -1));
      } else if (association_mat(v_matched_[j], j) > 0) {
        match_cps->push_back(std::make_pair(j, v_matched_[j]));
      } else {
        match_cps->push_back(std::make_pair(-1, v_matched_[j]));
        match_cps->push_back(std::make_pair(j, -1));
      }
    }
  } else {
    for (int j = 0; j < v_size_; ++j) {
      if (v_matched_[j] == -1) {
        match_cps->push_back(std::make_pair(-1, j));
      } else if (association_mat(v_matched_[j], j) > 0) {
        match_cps->push_back(std::make_pair(v_matched_[j], j));
      } else {
        match_cps->push_back(std::make_pair(v_matched_[j], -1));
        match_cps->push_back(std::make_pair(-1, j));
      }
    }
  }
  delete[] ex_u_;
  delete[] ex_v_;
  delete[] v_matched_;
  return true;
}

template <typename T>
bool KMkernal::FindCP(const T& mat, int i) {
  used_u_.insert(i);
  for (int j = 0; j < v_size_; ++j) {
    if (used_v_.find(j) != used_v_.end()) {
      continue;
    }
    double gap = ex_u_[i] + ex_v_[j] - mat(i, j);
    if (gap <= 0) {
      // res = 0;
      used_v_.insert(j);
      bool match_success = v_matched_[j] == -1 || FindCP(mat, v_matched_[j]);
      if (match_success) {
        v_matched_[j] = i;
        return true;
      }
    } else {
      v_slack_[j] = std::min(v_slack_[j], gap);
    }
  }
  return false;
}

}  // namespace ft
}  // namespace v2x
}  // namespace apollo
