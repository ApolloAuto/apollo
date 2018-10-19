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

#ifndef CYBER_DATA_FUSION_ALL_LATEST_H_
#define CYBER_DATA_FUSION_ALL_LATEST_H_

#include <deque>
#include <memory>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <vector>

#include "cyber/common/types.h"
#include "cyber/data/channel_buffer.h"
#include "cyber/data/fusion/data_fusion.h"

namespace apollo {
namespace cyber {
namespace data {
namespace fusion {

template <typename M0, typename M1 = NullType, typename M2 = NullType,
          typename M3 = NullType>
class AllLatest : public DataFusion<M0, M1, M2, M3> {
 public:
  AllLatest(const ChannelBuffer<M0>& buffer_0,
            const ChannelBuffer<M1>& buffer_1,
            const ChannelBuffer<M2>& buffer_2,
            const ChannelBuffer<M3>& buffer_3)
      : buffer_m0_(buffer_0),
        buffer_m1_(buffer_1),
        buffer_m2_(buffer_2),
        buffer_m3_(buffer_3) {}
  bool Fusion(uint64_t* index, std::shared_ptr<M0>& m0, std::shared_ptr<M1>& m1,
              std::shared_ptr<M2>& m2, std::shared_ptr<M3>& m3) override {
    return buffer_m0_.Fetch(index, m0) && buffer_m1_.Latest(m1) &&
           buffer_m2_.Latest(m2) && buffer_m3_.Latest(m3);
  }

 private:
  ChannelBuffer<M0> buffer_m0_;
  ChannelBuffer<M1> buffer_m1_;
  ChannelBuffer<M2> buffer_m2_;
  ChannelBuffer<M3> buffer_m3_;
};

template <typename M0, typename M1, typename M2>
class AllLatest<M0, M1, M2, NullType> : public DataFusion<M0, M1, M2> {
 public:
  AllLatest(const ChannelBuffer<M0>& buffer_0,
            const ChannelBuffer<M1>& buffer_1,
            const ChannelBuffer<M2>& buffer_2)
      : buffer_m0_(buffer_0), buffer_m1_(buffer_1), buffer_m2_(buffer_2) {}

  bool Fusion(uint64_t* index, std::shared_ptr<M0>& m0, std::shared_ptr<M1>& m1,
              std::shared_ptr<M2>& m2) override {
    return buffer_m0_.Fetch(index, m0) && buffer_m1_.Latest(m1) &&
           buffer_m2_.Latest(m2);
  }

 private:
  ChannelBuffer<M0> buffer_m0_;
  ChannelBuffer<M1> buffer_m1_;
  ChannelBuffer<M2> buffer_m2_;
};

template <typename M0, typename M1>
class AllLatest<M0, M1, NullType, NullType> : public DataFusion<M0, M1> {
 public:
  AllLatest(const ChannelBuffer<M0>& buffer_0,
            const ChannelBuffer<M1>& buffer_1)
      : buffer_m0_(buffer_0), buffer_m1_(buffer_1) {}

  bool Fusion(uint64_t* index, std::shared_ptr<M0>& m0,
              std::shared_ptr<M1>& m1) override {
    return buffer_m0_.Fetch(index, m0) && buffer_m1_.Latest(m1);
  }

 private:
  ChannelBuffer<M0> buffer_m0_;
  ChannelBuffer<M1> buffer_m1_;
};

}  // namespace fusion
}  // namespace data
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_DATA_FUSION_ALL_LATEST_H_
