/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
namespace routing {

template <typename T>
int BinarySearchForSLarger(const std::vector<T>& sorted_vec, double value_s) {
  if (sorted_vec.empty()) {
    return -1;
  }
  int start_index = 0;
  int end_index = static_cast<int>(sorted_vec.size()) - 1;
  double internal_s = 0.0;
  int middle_index = 0;
  while (end_index - start_index > 1) {
    middle_index = (start_index + end_index) / 2;
    internal_s = sorted_vec[middle_index].StartS();
    if (internal_s > value_s) {
      end_index = middle_index;
    } else {
      start_index = middle_index;
    }
  }
  double end_s = sorted_vec[start_index].EndS();
  if (value_s <= end_s) {
    return start_index;
  }
  return end_index;
}

template <typename T>
int BinarySearchForSSmaller(const std::vector<T>& sorted_vec, double value_s) {
  if (sorted_vec.empty()) {
    return -1;
  }
  int start_index = 0;
  int end_index = static_cast<int>(sorted_vec.size()) - 1;
  double internal_s = 0.0;
  int middle_index = 0;
  while (end_index - start_index > 1) {
    middle_index = (start_index + end_index) / 2;
    internal_s = sorted_vec[middle_index].EndS();
    if (internal_s > value_s) {
      end_index = middle_index;
    } else {
      start_index = middle_index;
    }
  }
  double start_s = sorted_vec[end_index].StartS();
  if (value_s > start_s) {
    return end_index;
  }
  return start_index;
}

template <typename T>
int BinarySearchCheckValidSIndex(const std::vector<T>& sorted_vec, int index,
                                 double value_s) {
  if (index == -1) {
    return -1;
  }
  double start_s = sorted_vec[index].StartS();
  double end_s = sorted_vec[index].EndS();
  static const double distance_error = 0.02;
  if (start_s <= value_s + distance_error &&
      end_s >= value_s - distance_error) {
    return index;
  }
  return -1;
}

template <typename T>
int BinarySearchForStartS(const std::vector<T>& sorted_vec, double value_s) {
  int index = BinarySearchForSLarger(sorted_vec, value_s);
  return BinarySearchCheckValidSIndex(sorted_vec, index, value_s);
}

template <typename T>
int BinarySearchForEndS(const std::vector<T>& sorted_vec, double value_s) {
  int index = BinarySearchForSSmaller(sorted_vec, value_s);
  return BinarySearchCheckValidSIndex(sorted_vec, index, value_s);
}

}  // namespace routing
}  // namespace apollo
