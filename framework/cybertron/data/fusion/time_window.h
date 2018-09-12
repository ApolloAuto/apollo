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

#ifndef CYBERTRON_DATA_FUSION_TIME_WINDOW_H_
#define CYBERTRON_DATA_FUSION_TIME_WINDOW_H_

#include <deque>
#include <memory>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <vector>

#include "cybertron/data/channel_buffer.h"
#include "cybertron/data/fusion/data_fusion.h"

namespace apollo {
namespace cybertron {
namespace data {
namespace fusion {

class TimeWindowBase {
 protected:
  static const uint64_t FUSION_QUEUE_SIZE = 10;

  template <typename M>
  void GetPivotPoints(const std::shared_ptr<M>& msg0) {
    pivot_start_point_ = message::GetTimestamp(msg0) - window_time_ / 2;
    pivot_end_point_ = message::GetTimestamp(msg0) + window_time_ / 2;
  }

  template <typename M>
  bool GetFirstMatch(ChannelBuffer<M>& buffer, std::shared_ptr<M>& m) {
    std::vector<std::shared_ptr<M>> msg_vec;
    if (buffer.FetchMulti(FUSION_QUEUE_SIZE, &msg_vec)) {
      for (const auto& msg : msg_vec) {
        if (message::GetTimestamp(msg) > pivot_start_point_ &&
            message::GetTimestamp(msg) <= pivot_end_point_) {
          m = msg;
          return true;
        }
      }
    }
    return false;
  }

  uint64_t window_time_ = 100 * 1000 * 1000;  // default 100ms
  uint64_t pivot_start_point_ = 0;
  uint64_t pivot_end_point_ = 0;
};

template <typename M0, typename M1 = NullType, typename M2 = NullType,
          typename M3 = NullType>
class TimeWindow : public DataFusion<M0, M1, M2, M3>, public TimeWindowBase {
 public:
  TimeWindow(const ChannelBuffer<M0>& buffer_0,
             const ChannelBuffer<M1>& buffer_1,
             const ChannelBuffer<M2>& buffer_2,
             const ChannelBuffer<M3>& buffer_3)
      : buffer_m0_(buffer_0),
        buffer_m1_(buffer_1),
        buffer_m2_(buffer_2),
        buffer_m3_(buffer_3) {}

  bool Fusion(uint64_t* index, std::shared_ptr<M0>& m0, std::shared_ptr<M1>& m1,
              std::shared_ptr<M2>& m2, std::shared_ptr<M3>& m3) {
    if (buffer_m0_.Fetch(index, m0)) {
      GetPivotPoints(m0);
      return GetFirstMatch(buffer_m1_, m1) && GetFirstMatch(buffer_m2_, m2) &&
             GetFirstMatch(buffer_m3_, m3);
    }
    return false;
  }

 private:
  ChannelBuffer<M0> buffer_m0_;
  ChannelBuffer<M1> buffer_m1_;
  ChannelBuffer<M2> buffer_m2_;
  ChannelBuffer<M3> buffer_m3_;
};

template <typename M0, typename M1, typename M2>
class TimeWindow<M0, M1, M2, NullType> : public DataFusion<M0, M1, M2>,
                                         public TimeWindowBase {
 public:
  TimeWindow(const ChannelBuffer<M0>& buffer_0,
             const ChannelBuffer<M1>& buffer_1,
             const ChannelBuffer<M2>& buffer_2)
      : buffer_m0_(buffer_0), buffer_m1_(buffer_1), buffer_m2_(buffer_2) {}

  bool Fusion(uint64_t* index, std::shared_ptr<M0>& m0, std::shared_ptr<M1>& m1,
              std::shared_ptr<M2>& m2) {
    if (buffer_m0_.Fetch(index, m0)) {
      GetPivotPoints(m0);
      return GetFirstMatch(buffer_m1_, m1) && GetFirstMatch(buffer_m2_, m2);
    }
    return false;
  }

 private:
  ChannelBuffer<M0> buffer_m0_;
  ChannelBuffer<M1> buffer_m1_;
  ChannelBuffer<M2> buffer_m2_;
};

template <typename M0, typename M1>
class TimeWindow<M0, M1, NullType, NullType> : public DataFusion<M0, M1>,
                                               public TimeWindowBase {
 public:
  TimeWindow(const ChannelBuffer<M0>& buffer_0,
             const ChannelBuffer<M1>& buffer_1)
      : buffer_m0_(buffer_0), buffer_m1_(buffer_1) {}

  bool Fusion(uint64_t* index, std::shared_ptr<M0>& m0,
              std::shared_ptr<M1>& m1) {
    if (buffer_m0_.Fetch(index, m0)) {
      GetPivotPoints(m0);
      return GetFirstMatch(buffer_m1_, m1);
    }
    return false;
  }

 private:
  ChannelBuffer<M0> buffer_m0_;
  ChannelBuffer<M1> buffer_m1_;
};

}  // namespace fusion
}  // namespace data
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_DATA_FUSION_TIME_WINDOW_H_
