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
#include <tuple>
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
  using FusionDataType = std::tuple<std::shared_ptr<M0>, std::shared_ptr<M1>,
                                    std::shared_ptr<M2>, std::shared_ptr<M3>>;

 public:
  AllLatest(const ChannelBuffer<M0>& buffer_0,
            const ChannelBuffer<M1>& buffer_1,
            const ChannelBuffer<M2>& buffer_2,
            const ChannelBuffer<M3>& buffer_3)
      : buffer_m0_(buffer_0),
        buffer_m1_(buffer_1),
        buffer_m2_(buffer_2),
        buffer_m3_(buffer_3),
        buffer_fusion_(buffer_m0_.channel_id(),
                       new CacheBuffer<std::shared_ptr<FusionDataType>>(
                           buffer_0.Buffer()->Capacity() - uint64_t(1))) {
    buffer_m0_.Buffer()->SetFusionCallback(
        [this](const std::shared_ptr<M0>& m0) {
          std::shared_ptr<M1> m1;
          std::shared_ptr<M2> m2;
          std::shared_ptr<M3> m3;
          if (!buffer_m1_.Latest(m1) || !buffer_m2_.Latest(m2) ||
              !buffer_m3_.Latest(m3)) {
            return;
          }

          auto data = std::make_shared<FusionDataType>(m0, m1, m2, m3);
          std::lock_guard<std::mutex> lg(buffer_fusion_.Buffer()->Mutex());
          buffer_fusion_.Buffer()->Fill(data);
        });
  }

  bool Fusion(uint64_t* index, std::shared_ptr<M0>& m0, std::shared_ptr<M1>& m1,
              std::shared_ptr<M2>& m2, std::shared_ptr<M3>& m3) override {
    std::shared_ptr<FusionDataType> fusion_data;
    if (!buffer_fusion_.Fetch(index, fusion_data)) {
      return false;
    }
    m0 = std::get<0>(*fusion_data);
    m1 = std::get<1>(*fusion_data);
    m2 = std::get<2>(*fusion_data);
    m3 = std::get<3>(*fusion_data);
    return true;
  }

 private:
  ChannelBuffer<M0> buffer_m0_;
  ChannelBuffer<M1> buffer_m1_;
  ChannelBuffer<M2> buffer_m2_;
  ChannelBuffer<M3> buffer_m3_;
  ChannelBuffer<FusionDataType> buffer_fusion_;
};

template <typename M0, typename M1, typename M2>
class AllLatest<M0, M1, M2, NullType> : public DataFusion<M0, M1, M2> {
  using FusionDataType =
      std::tuple<std::shared_ptr<M0>, std::shared_ptr<M1>, std::shared_ptr<M2>>;

 public:
  AllLatest(const ChannelBuffer<M0>& buffer_0,
            const ChannelBuffer<M1>& buffer_1,
            const ChannelBuffer<M2>& buffer_2)
      : buffer_m0_(buffer_0),
        buffer_m1_(buffer_1),
        buffer_m2_(buffer_2),
        buffer_fusion_(buffer_m0_.channel_id(),
                       new CacheBuffer<std::shared_ptr<FusionDataType>>(
                           buffer_0.Buffer()->Capacity() - uint64_t(1))) {
    buffer_m0_.Buffer()->SetFusionCallback(
        [this](const std::shared_ptr<M0>& m0) {
          std::shared_ptr<M1> m1;
          std::shared_ptr<M2> m2;
          if (!buffer_m1_.Latest(m1) || !buffer_m2_.Latest(m2)) {
            return;
          }

          auto data = std::make_shared<FusionDataType>(m0, m1, m2);
          std::lock_guard<std::mutex> lg(buffer_fusion_.Buffer()->Mutex());
          buffer_fusion_.Buffer()->Fill(data);
        });
  }

  bool Fusion(uint64_t* index, std::shared_ptr<M0>& m0, std::shared_ptr<M1>& m1,
              std::shared_ptr<M2>& m2) override {
    std::shared_ptr<FusionDataType> fusion_data;
    if (!buffer_fusion_.Fetch(index, fusion_data)) {
      return false;
    }
    m0 = std::get<0>(*fusion_data);
    m1 = std::get<1>(*fusion_data);
    m2 = std::get<2>(*fusion_data);
    return true;
  }

 private:
  ChannelBuffer<M0> buffer_m0_;
  ChannelBuffer<M1> buffer_m1_;
  ChannelBuffer<M2> buffer_m2_;
  ChannelBuffer<FusionDataType> buffer_fusion_;
};

template <typename M0, typename M1>
class AllLatest<M0, M1, NullType, NullType> : public DataFusion<M0, M1> {
  using FusionDataType = std::tuple<std::shared_ptr<M0>, std::shared_ptr<M1>>;

 public:
  AllLatest(const ChannelBuffer<M0>& buffer_0,
            const ChannelBuffer<M1>& buffer_1)
      : buffer_m0_(buffer_0),
        buffer_m1_(buffer_1),
        buffer_fusion_(buffer_m0_.channel_id(),
                       new CacheBuffer<std::shared_ptr<FusionDataType>>(
                           buffer_0.Buffer()->Capacity() - uint64_t(1))) {
    buffer_m0_.Buffer()->SetFusionCallback(
        [this](const std::shared_ptr<M0>& m0) {
          std::shared_ptr<M1> m1;
          if (!buffer_m1_.Latest(m1)) {
            return;
          }

          auto data = std::make_shared<FusionDataType>(m0, m1);
          std::lock_guard<std::mutex> lg(buffer_fusion_.Buffer()->Mutex());
          buffer_fusion_.Buffer()->Fill(data);
        });
  }

  bool Fusion(uint64_t* index, std::shared_ptr<M0>& m0,
              std::shared_ptr<M1>& m1) override {
    std::shared_ptr<FusionDataType> fusion_data;
    if (!buffer_fusion_.Fetch(index, fusion_data)) {
      return false;
    }
    m0 = std::get<0>(*fusion_data);
    m1 = std::get<1>(*fusion_data);
    return true;
  }

 private:
  ChannelBuffer<M0> buffer_m0_;
  ChannelBuffer<M1> buffer_m1_;
  ChannelBuffer<FusionDataType> buffer_fusion_;
};

}  // namespace fusion
}  // namespace data
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_DATA_FUSION_ALL_LATEST_H_
