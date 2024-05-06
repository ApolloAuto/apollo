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

#include <vector>

namespace apollo {
namespace perception {
namespace algorithm {

// Note: a partition of a set U is defined to be a collection P of subsets of U
// such that U is the union of all sets in P and P is pairwise disjoint. U is
// called the universe of P and P is a partition of U
class Universe {
  struct Element {
    int rank = 0;
    int p = 0;
    int size = 1;
  };

 public:
  Universe() = default;
  explicit Universe(const int elements_num);

  ~Universe() = default;

  // @brief: reset each element
  void Reset(const int elements_num);

  // @brief: find and return input element's parent
  int Find(const int x);

  // @brief: join the two elements into one set
  void Join(const int x, const int y);

  // @brief: get size of input element
  int GetSize(const int x) const { return elts_[x].size; }

  // @brief: get sets number
  int GetSetsNum() const { return sets_num_; }

 private:
  std::vector<Element> elts_;
  int sets_num_ = 0;
};

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
