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

template <typename M0, typename ...M>
class AllLatest : public DataFusion<M0, M...> {
  using FusionDataType = std::tuple<std::shared_ptr<M0>, std::shared_ptr<M>...>;

 public:
  AllLatest(const ChannelBuffer<M0>& buffer_0,
            const ChannelBuffer<M>& ...buffers)
        : AllLatest(buffer_0, buffers...
                    , std::make_index_sequence<sizeof...(M)>{}) {
  }

  bool Fusion(uint64_t* index,
              std::shared_ptr<M0>& m0,
              std::shared_ptr<M>& ...m) override {
    std::shared_ptr<FusionDataType> fusion_data;
    if (!buffer_fusion_.Fetch(index, fusion_data)) {
      return false;
    }

    Fusion(fusion_data, m0, m...,
           std::make_index_sequence<sizeof...(M)>{});
    return true;
  }

 private:
  template<size_t ...N>
  AllLatest(const ChannelBuffer<M0>& buffer_0,
            const ChannelBuffer<M>& ...buffers,
            std::index_sequence<N...>)
      : buffer_m0_(buffer_0)
      , buffer_aux_(std::make_tuple(buffers...))
      , buffer_fusion_(buffer_0.channel_id(),
                       new CacheBuffer<std::shared_ptr<FusionDataType>>(
                           buffer_0.Buffer()->Capacity() - uint64_t(1))) {
    buffer_m0_.Buffer()->SetFusionCallback(
        [this](const std::shared_ptr<M0>& m0) {
          auto data = std::make_shared<FusionDataType>();

          std::get<0>(*data) = m0;
          std::vector<bool> is_got_data = {std::get<N>(buffer_aux_)
                                          .Latest(std::get<N+1>(*data))...};
          if (is_got_data.end()
                == std::find(is_got_data.begin(), is_got_data.end(), false)) {
            std::lock_guard<std::mutex> lg(buffer_fusion_.Buffer()->Mutex());
            buffer_fusion_.Buffer()->Fill(data);
          }
        });
  }

  template<size_t ...N>
  void Fusion(const std::shared_ptr<FusionDataType> &fusion_data,
              std::shared_ptr<M0>& m0, std::shared_ptr<M>& ...m, // NOLINT
              std::index_sequence<N...>) {
    m0 = std::get<0>(*fusion_data);
    {
      int dummy[] = {((m = std::get<N+1>(*fusion_data)), 0)...};
      (void)dummy;
    }
  }


  ChannelBuffer<M0> buffer_m0_;
  std::tuple<ChannelBuffer<M>...> buffer_aux_;
  ChannelBuffer<FusionDataType> buffer_fusion_;
};

}  // namespace fusion
}  // namespace data
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_DATA_FUSION_ALL_LATEST_H_
