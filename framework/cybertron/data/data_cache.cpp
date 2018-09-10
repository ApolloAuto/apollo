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

#include "cybertron/data/data_cache.h"

namespace apollo {
namespace cybertron {
namespace data {

bool DataCache::HasMessage(uint64_t channel_id) {
  BufferValueType* buffer = nullptr;
  if (channel_data_.Get(channel_id, &buffer)) {
    ReadLockGuard lock(buffer->RWLock());
    return !buffer->Empty();
  } else {
    return false;
  }
}

void DataCache::RegisterFusionNotifier(const DataFusionNotifier& notifier) {
  std::vector<DataFusionNotifier>* notifiers = nullptr;
  if (fusion_map_.Get(notifier.MainChannel(), &notifiers)) {
    notifiers->emplace_back(notifier);
  } else {
    std::vector<DataFusionNotifier> notifier_vec;
    notifier_vec.reserve(4);
    notifier_vec.emplace_back(notifier);
    fusion_map_.Set(notifier.MainChannel(), std::move(notifier_vec));
  }
}

}  // namespace data
}  // namespace cybertron
}  // namespace apollo
