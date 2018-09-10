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

#ifndef CYBERTRON_DATA_DATA_FUSION_H_
#define CYBERTRON_DATA_DATA_FUSION_H_

#include <deque>
#include <memory>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <vector>

#include "cybertron/data/data_cache.h"

namespace apollo {
namespace cybertron {
namespace data {

class DataFusion {
 public:
  DataFusion() : window_time_(100 * 1000 * 1000) {}  // 100ms

  ~DataFusion() {}

  template <typename M0, typename M1>
  bool process(const MetaDataPtr<M0>& msg0, MetaDataPtr<M1>& msg1,
               const MetaMsgVec<M1>& msg1_vec) {
    GetPivotPoints(msg0);
    return GetFirstMatch(msg1, msg1_vec);
  }

  template <typename M0, typename M1, typename M2>
  bool process(const MetaDataPtr<M0>& msg0, MetaDataPtr<M1>& msg1,
               const MetaMsgVec<M1>& msg1_vec, MetaDataPtr<M2>& msg2,
               const MetaMsgVec<M2>& msg2_vector) {
    GetPivotPoints(msg0);

    return GetFirstMatch(msg1, msg1_vec) && GetFirstMatch(msg2, msg2_vector);
  }

  template <typename M0, typename M1, typename M2, typename M3>
  bool process(const MetaDataPtr<M0>& msg0, MetaDataPtr<M1>& msg1,
               const MetaMsgVec<M1>& msg1_vec, MetaDataPtr<M2>& msg2,
               const MetaMsgVec<M2>& msg2_vector, MetaDataPtr<M3>& msg3,
               const MetaMsgVec<M3>& msg3_vec) {
    GetPivotPoints(msg0);

    return GetFirstMatch(msg1, msg1_vec) && GetFirstMatch(msg2, msg2_vector) &&
           GetFirstMatch(msg3, msg3_vec);
  }

 private:
  DataFusion(DataFusion&) = delete;
  DataFusion& operator=(DataFusion&) = delete;

  template <typename M>
  uint64_t GetTimestamp(const MetaDataPtr<M>& msg) {
    return msg->time_stamp;
  }

  template <typename M0>
  void GetPivotPoints(const MetaDataPtr<M0>& msg0) {
    pivot_start_point_ = GetTimestamp(msg0) - window_time_ / 2;
    pivot_end_point_ = GetTimestamp(msg0) + window_time_ / 2;
  }

  template <typename M1>
  bool GetFirstMatch(MetaDataPtr<M1>& m1, const MetaMsgVec<M1>& msg1_vec) {
    for (const auto& msg : msg1_vec) {
      if (GetTimestamp(msg) > pivot_start_point_ &&
          GetTimestamp(msg) <= pivot_end_point_) {
        m1 = msg;
        return true;
      }
    }
    return false;
  }

 private:
  uint64_t window_time_;
  uint64_t pivot_start_point_ = 0;
  uint64_t pivot_end_point_ = 0;
};

}  // namespace data
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_DATA_DATA_FUSION_H_
