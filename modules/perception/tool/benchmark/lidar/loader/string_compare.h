/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace apollo {
namespace perception {
namespace benchmark {

static void string_split(const std::string& str, char c,
                         std::vector<std::string>* strs) {
  strs->clear();
  std::string term;
  for (auto& cur_char : str) {
    if (cur_char != c) {
      term += cur_char;
      continue;
    }
    // find one
    if (!term.empty()) {
      strs->push_back(term);
      term.clear();
    }
  }
  if (!term.empty()) {
    strs->push_back(term);
  }
}

static bool string_compare_by_length(const std::string& lhs,
                                     const std::string& rhs) {
  if (lhs.length() < rhs.length()) {
    return true;
  } else if (lhs.length() == rhs.length()) {
    return lhs <= rhs;
  } else {
    return false;
  }
}

static void sort_strings_by_split_length(
    const std::vector<std::string>& strs,
    std::vector<std::size_t>* sorted_indices) {
  struct StringHelper {
    std::string str;
    std::vector<std::string> splits;
    std::size_t id;
  };
  std::vector<StringHelper> helper(strs.size());
  for (std::size_t i = 0; i < strs.size(); ++i) {
    helper[i].str = strs.at(i);
    string_split(helper[i].str, '/', &helper[i].splits);
    helper[i].id = i;
  }
  std::sort(helper.begin(), helper.end(),
            [](const StringHelper& lhs, const StringHelper& rhs) {
              if (lhs.splits.size() < rhs.splits.size()) {
                return true;
              } else if (lhs.splits.size() > rhs.splits.size()) {
                return false;
              }
              for (std::size_t i = 0; i < lhs.splits.size(); ++i) {
                if (lhs.splits[i] == rhs.splits[i]) {
                  continue;
                } else {
                  return string_compare_by_length(lhs.splits[i], rhs.splits[i]);
                }
              }
              return true;
            });
  sorted_indices->resize(strs.size());
  for (std::size_t i = 0; i < strs.size(); ++i) {
    sorted_indices->at(i) = helper[i].id;
  }
}

template <typename T>
void shuffle_by_indices(std::vector<T>* src,
                        const std::vector<std::size_t>& indices) {
  if (src->size() != indices.size()) {
    return;
  }
  std::vector<T> dst;
  dst.reserve(indices.size());
  for (auto& id : indices) {
    dst.push_back(std::move(src->at(id)));
  }
  src->clear();
  for (auto& data : dst) {
    src->push_back(std::move(data));
  }
}

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
