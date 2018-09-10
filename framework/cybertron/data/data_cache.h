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

#ifndef CYBERTRON_DATA_DATA_CACHE_H_
#define CYBERTRON_DATA_DATA_CACHE_H_

#include <sys/time.h>
#include <algorithm>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cybertron/base/atomic_rw_lock.h"
#include "cybertron/base/concurrent_object_pool.h"
#include "cybertron/common/global_data.h"
#include "cybertron/common/log.h"
#include "cybertron/common/macros.h"
#include "cybertron/data/any.h"
#include "cybertron/data/cache_buffer.h"
#include "cybertron/data/data_fusion_notifier.h"
#include "cybertron/data/meta_data.h"
#include "cybertron/time/time.h"
#include "cybertron/croutine/croutine.h"
#include "cybertron/event/perf_event_cache.h"

namespace apollo {
namespace cybertron {
namespace data {

using apollo::cybertron::Time;
using apollo::cybertron::base::AtomicHashMap;
using apollo::cybertron::common::GlobalData;
using apollo::cybertron::base::ReadLockGuard;
using apollo::cybertron::base::WriteLockGuard;
using apollo::cybertron::event::PerfEventCache;


template <typename T>
using CCObjectPool = apollo::cybertron::base::CCObjectPool<T>;
template <typename M>
using MetaMsgVec = std::vector<MetaDataPtr<M>>;

class DataCache {
 public:
  static const uint64_t CHANNEL_CACHE_SIZE = 64;
  static const uint64_t FUSION_QUEUE_SIZE = 10;
  using BufferValueType = CacheBuffer<Any, CHANNEL_CACHE_SIZE>;
  ~DataCache() {}

  void Shutdown() {}

  void InitChannelCache(const uint64_t channel_id) {
    channel_data_.Set(channel_id);
  }

  void InitChannelCache(const std::vector<uint64_t>& channel_vec) {
    for (auto& channel_id : channel_vec) {
      channel_data_.Set(channel_id);
    }
  }

  template <typename T>
  bool WriteDataCache(const uint64_t channel_id, const std::shared_ptr<T>& msg);

  void RegisterFusionNotifier(const DataFusionNotifier& notifier);

  bool HasMessage(uint64_t channel_id);

  template <typename T>
  bool ReadDataCache(const uint64_t channel_id, const uint64_t queue_size,
                     uint64_t* seq_id, uint64_t* drop_num, MetaDataPtr<T>& msg);

  template <typename T>
  bool ReadDataCache(const uint64_t channel_id, uint64_t* seq_id,
                     MetaDataPtr<T>& msg);

  template <typename T>
  bool ReadDataCache(const uint64_t channel_id, MetaDataPtr<T>& msg);

  template <typename T>
  bool ReadDataCache(const uint64_t channel_id, MetaMsgVec<T>& msg_vector);

 private:
  AtomicHashMap<uint64_t, BufferValueType> channel_data_;
  AtomicHashMap<uint64_t, std::vector<DataFusionNotifier>> fusion_map_;

  DECLARE_SINGLETON(DataCache)
};

inline DataCache::DataCache() {}

template <typename T>
bool DataCache::WriteDataCache(const uint64_t channel_id,
                               const std::shared_ptr<T>& msg) {
  if (msg == nullptr) {
      return false;
    }
  auto write_cache_stamp = Time::Now().ToNanosecond();
  static auto pool =
      CCObjectPool<MetaData<T>>::Instance(CHANNEL_CACHE_SIZE * 2);
  auto meta_data = pool->GetObject();
  if (!meta_data) {
    meta_data = std::make_shared<MetaData<T>>();
  }
  meta_data->channel_id = channel_id;
  meta_data->time_stamp = Time::Now().ToNanosecond();
  meta_data->message = msg;
  
  BufferValueType* cache = nullptr;
  channel_data_.Get(channel_id, &cache);
  {
    WriteLockGuard lg(cache->RWLock());
    cache->Fill(std::move(Any(meta_data)));
  }
  PerfEventCache::Instance()->AddTransportEvent(1, 0, write_cache_stamp);
  auto notify_stamp = Time::Now().ToNanosecond();;
  std::vector<DataFusionNotifier>* notifiers = nullptr;
  if (fusion_map_.Get(channel_id, &notifiers)) {
    for (auto& notifier : *notifiers) {
      notifier.Notify(channel_id);
    }
  }
  PerfEventCache::Instance()->AddTransportEvent(2, 0, notify_stamp);
  return true;
}

template <typename T>
bool DataCache::ReadDataCache(const uint64_t channel_id,
                              const uint64_t queue_size, uint64_t* seq_id,
                              uint64_t* drop_num, MetaDataPtr<T>& msg) {
  BufferValueType* cache = nullptr;
  channel_data_.Get(channel_id, &cache);
  ReadLockGuard lg(cache->RWLock());
  if (cache->Empty()) {
    return false;
  }
  auto oldest_index = std::min(cache->Head(), cache->Tail() - queue_size + 1);
  if (*seq_id == 0) {
    *seq_id = cache->Tail();
  } else if (*seq_id == cache->Tail() + 1) {
    // no new msg
    return false;
  } else if (*seq_id < oldest_index) {
    auto interval = oldest_index - *seq_id;
    *drop_num += interval;
    *seq_id = oldest_index;
    AINFO << "drop message channel[" << GlobalData::GetChannelById(channel_id)
          << "] drop_message_num[" << interval << "] pre_index[" << *seq_id
          << "] current_index[" << oldest_index << "] ";
  }

  try {
    msg = AnyCast<MetaDataPtr<T>>((*cache)[*seq_id]);
    return true;
  } catch (const BadAnyCast) {
    AWARN << "msg bad any cast.";
    return false;
  }
}

template <typename T>
bool DataCache::ReadDataCache(const uint64_t channel_id, uint64_t* seq_id,
                              MetaDataPtr<T>& msg) {
  BufferValueType* cache = nullptr;
  channel_data_.Get(channel_id, &cache);
  ReadLockGuard lg(cache->RWLock());
  if (cache->Empty()) {
    return false;
  }

  if (*seq_id == 0) {
    *seq_id = cache->Tail();
  }

  try {
    msg = AnyCast<MetaDataPtr<T>>((*cache)[*seq_id]);
    return true;
  } catch (const BadAnyCast) {
    AWARN << "msg bad any cast.";
    return false;
  }
}

template <typename T>
bool DataCache::ReadDataCache(const uint64_t channel_id, MetaDataPtr<T>& msg) {
  BufferValueType* cache = nullptr;
  channel_data_.Get(channel_id, &cache);
  ReadLockGuard lg(cache->RWLock());
  if (cache->Empty()) {
    return false;
  }

  try {
    msg = AnyCast<MetaDataPtr<T>>(cache->Back());
    return true;
  } catch (const BadAnyCast) {
    AWARN << "msg bad any cast.";
    return false;
  }
}

template <typename T>
bool DataCache::ReadDataCache(const uint64_t channel_id,
                              MetaMsgVec<T>& msg_vector) {
  BufferValueType* cache = nullptr;
  channel_data_.Get(channel_id, &cache);
  ReadLockGuard lg(cache->RWLock());
  if (cache->Empty()) {
    return false;
  }

  auto num = std::min(cache->Size(), static_cast<uint64_t>(FUSION_QUEUE_SIZE));
  msg_vector.reserve(num);
  for (auto index = cache->Tail() - num + 1; index <= cache->Tail(); ++index) {
    try {
      msg_vector.emplace_back(AnyCast<MetaDataPtr<T>>((*cache)[index]));
    } catch (const BadAnyCast) {
      AWARN << "msg bad any cast.";
      return false;
    }
  }
  return true;
}

}  // namespace data
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_DATA_DATA_CACHE_H_
