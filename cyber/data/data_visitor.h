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

#ifndef CYBER_DATA_DATA_VISITOR_H_
#define CYBER_DATA_DATA_VISITOR_H_

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>
#include <utility>
#include <tuple>

#include "cyber/common/log.h"
#include "cyber/data/channel_buffer.h"
#include "cyber/data/data_dispatcher.h"
#include "cyber/data/data_visitor_base.h"
#include "cyber/data/fusion/all_latest.h"
#include "cyber/data/fusion/data_fusion.h"

namespace apollo {
namespace cyber {
namespace data {

struct VisitorConfig {
  VisitorConfig(uint64_t id, uint32_t size)
      : channel_id(id), queue_size(size) {}
  uint64_t channel_id;
  uint32_t queue_size;
};

template <typename T>
using BufferType = CacheBuffer<std::shared_ptr<T>>;

template <typename ...M>
class DataVisitor : public DataVisitorBase {
 public:
  explicit DataVisitor(const std::vector<VisitorConfig>& configs)
  : DataVisitor(configs, std::make_index_sequence<sizeof...(M)>{}) {
  }

  ~DataVisitor() {
    if (data_fusion_) {
      delete data_fusion_;
      data_fusion_ = nullptr;
    }
  }

  bool TryFetch(std::shared_ptr<M>& ...m) {                         // NOLINT
    if (data_fusion_->Fusion(&next_msg_index_, m...)) {
      next_msg_index_++;
      return true;
    }
    return false;
  }

 private:
  template<size_t ...N>
  explicit DataVisitor(const std::vector<VisitorConfig>& configs,
                       std::index_sequence<N...>)
  : buffers_(std::make_tuple(ChannelBuffer<M>(
                  configs[N].channel_id,
                  new BufferType<M>(configs[N].queue_size))...)) {
    {
      int dummy[] = {(
          DataDispatcher<M>::Instance()->AddBuffer(
              std::get<N>(buffers_)), 0)... };
      (void)dummy;
    }
    data_notifier_->AddNotifier(std::get<0>(buffers_).channel_id(), notifier_);
    data_fusion_ =
        new fusion::AllLatest<M...>(std::get<N>(buffers_)...);
  }

  std::tuple<ChannelBuffer<M> ...> buffers_;
  fusion::DataFusion<M ...>* data_fusion_ = nullptr;
};

template <typename M0>
class DataVisitor<M0>: public DataVisitorBase {
 public:
  explicit DataVisitor(const VisitorConfig& configs)
      : buffer_(configs.channel_id, new BufferType<M0>(configs.queue_size)) {
    DataDispatcher<M0>::Instance()->AddBuffer(buffer_);
    data_notifier_->AddNotifier(buffer_.channel_id(), notifier_);
  }

  explicit DataVisitor(const std::vector<VisitorConfig>& configs)
      : DataVisitor(configs[0]) {
  }

  DataVisitor(uint64_t channel_id, uint32_t queue_size)
      : buffer_(channel_id, new BufferType<M0>(queue_size)) {
    DataDispatcher<M0>::Instance()->AddBuffer(buffer_);
    data_notifier_->AddNotifier(buffer_.channel_id(), notifier_);
  }

  bool TryFetch(std::shared_ptr<M0>& m0) {  // NOLINT
    if (buffer_.Fetch(&next_msg_index_, m0)) {
      next_msg_index_++;
      return true;
    }
    return false;
  }

 private:
  ChannelBuffer<M0> buffer_;
};

}  // namespace data
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_DATA_DATA_VISITOR_H_
