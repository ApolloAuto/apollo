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

#ifndef CYBERTRON_DATA_DATA_DISPATCHER_H_
#define CYBERTRON_DATA_DATA_DISPATCHER_H_

#include <memory>
#include <mutex>
#include <vector>

#include "cybertron/common/log.h"
#include "cybertron/common/macros.h"
#include "cybertron/data/cache_buffer.h"
#include "cybertron/data/channel_buffer.h"
#include "cybertron/data/data_notifier.h"
#include "cybertron/time/time.h"
#include "cybertron/init.h"

namespace apollo {
namespace cybertron {
namespace data {

using apollo::cybertron::Time;
using apollo::cybertron::base::AtomicHashMap;

template <typename T>
class DataDispatcher {
 public:
  using BufferVector =
      std::vector<std::weak_ptr<CacheBuffer<std::shared_ptr<T>>>>;
  ~DataDispatcher() {}

  void AddBuffer(const ChannelBuffer<T>& channel_buffer);

  bool Dispatch(const uint64_t channel_id, const std::shared_ptr<T>& msg);

 private:
  std::shared_ptr<DataNotifier> notifier_ = DataNotifier::Instance();
  std::mutex buffers_map_mutex_;
  AtomicHashMap<uint64_t, BufferVector> buffers_map_;

  DECLARE_SINGLETON(DataDispatcher)
};

template <typename T>
inline DataDispatcher<T>::DataDispatcher() {}

template <typename T>
void DataDispatcher<T>::AddBuffer(const ChannelBuffer<T>& channel_buffer) {
  std::lock_guard<std::mutex> lock(buffers_map_mutex_);
  auto buffer = channel_buffer.Buffer();
  BufferVector* buffers = nullptr;
  if (buffers_map_.Get(channel_buffer.ChannelId(), &buffers)) {
    buffers->emplace_back(buffer);
  } else {
    BufferVector new_buffers = {buffer};
    buffers_map_.Set(channel_buffer.ChannelId(), new_buffers);
  }
}

template <typename T>
bool DataDispatcher<T>::Dispatch(const uint64_t channel_id,
                                 const std::shared_ptr<T>& msg) {
  BufferVector* buffers = nullptr;
  if (apollo::cybertron::IsShutdown()) {
    return false;
  }
  if (buffers_map_.Get(channel_id, &buffers)) {
    for (auto& buffer_wptr : *buffers) {
      if (auto buffer = buffer_wptr.lock()) {
        std::lock_guard<std::mutex> lock(buffer->Mutex());
        buffer->Fill(msg);
      }
    }
  } else {
    return false;
  }
  return notifier_->Notify(channel_id);
}

}  // namespace data
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_DATA_DATA_DISPATCHER_H_
