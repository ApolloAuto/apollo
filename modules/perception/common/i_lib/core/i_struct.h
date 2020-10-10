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

#include "modules/perception/common/i_lib/core/i_alloc.h"

namespace apollo {
namespace perception {
namespace common {
template <typename T1, typename T2>
struct Pair {
  T1 first;
  T2 second;
  Pair() {}
  Pair(const Pair<T1, T2> &p) {
    first = p.first;
    second = p.second;
  }
  Pair &operator=(const Pair<T1, T2> &p) {
    this->first = p.first;
    this->second = p.second;
    return (*this);
  }
  Pair(const T1 &a, const T2 &b) : first(a), second(b) {}
};

template <typename T1, typename T2>
inline Pair<T1, T2> IMakePair(const T1 &a, const T2 &b) {
  return Pair<T1, T2>(a, b);
}

template <typename T1, typename T2>
inline bool ILessPairFirstElement(const Pair<T1, T2> &a,
                                  const Pair<T1, T2> &b) {
  return a.first < b.first;
}

template <typename T1, typename T2>
inline bool ILessPairSecondElement(const Pair<T1, T2> &a,
                                   const Pair<T1, T2> &b) {
  return a.second < b.second;
}

template <typename T1, typename T2>
inline bool ILargerPairFirstElement(const Pair<T1, T2> &a,
                                    const Pair<T1, T2> &b) {
  return a.first > b.first;
}

template <typename T1, typename T2>
inline bool ILargerPairSecondElement(const Pair<T1, T2> &a,
                                     const Pair<T1, T2> &b) {
  return a.second > b.second;
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
