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

#include "cybertron/data/data_visitor.h"

#include "cybertron/common/global_data.h"
#include "cybertron/data/data_fusion_notifier.h"

namespace apollo {
namespace cybertron {
namespace data {

DataVisitor::DataVisitor(std::vector<uint64_t>&& channel_vec,
                         const uint64_t& queue_size,
                         const FusionType& fusion_type) {
  data_flag_.channel_vec = channel_vec;
  data_flag_.queue_size = queue_size;
  data_flag_.fusion_type = fusion_type;
  Initialize();
}

DataVisitor::DataVisitor(const uint64_t& channel, const uint64_t& queue_size,
                         const FusionType& fusion_type) {
  data_flag_.channel_vec.push_back(std::move(channel));
  data_flag_.queue_size = queue_size;
  data_flag_.fusion_type = fusion_type;
  Initialize();
}

void DataVisitor::Initialize() {
  data_cache_->InitChannelCache(data_flag_.channel_vec);
  DataFusionNotifier notifier;
  notifier.RegisterCallback(data_flag_.channel_vec, [=]() {
    if (notify_) {
      notify_callback_();
    }
  });
  data_cache_->RegisterFusionNotifier(notifier);
}

}  // namespace data
}  // namespace cybertron
}  // namespace apollo
