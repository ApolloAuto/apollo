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

#ifndef CYBERTRON_DATA_DATA_VISITOR_H_
#define CYBERTRON_DATA_DATA_VISITOR_H_

#include <sys/time.h>
#include <functional>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "cybertron/common/log.h"
#include "cybertron/data/data_cache.h"
#include "cybertron/data/data_fusion.h"
#include "cybertron/data/meta_data.h"
#include "cybertron/proto/component_config.pb.h"

namespace apollo {
namespace cybertron {
namespace data {

using apollo::cybertron::proto::FusionType;

struct DataFlag {
  std::vector<uint64_t> channel_vec;
  uint64_t queue_size = 0;
  uint64_t seq_id = 0;  // main channel
  FusionType fusion_type;
};

struct DataStatistics {
  double wait_time = 0;
  uint64_t drop_num = 0;
  uint64_t queue_size = 0;
  uint64_t pop_num = 0;
};

class DataVisitor {
 public:
  DataVisitor(std::vector<uint64_t>&& channel_vec,
              const uint64_t& queue_size = 1,
              const FusionType& fusion_type = FusionType::NEWEST);
  DataVisitor(const uint64_t& channel, const uint64_t& queue_size = 1,
              const FusionType& fusion_type = FusionType::NEWEST);

  template <typename M0>
  bool TryFetch(std::shared_ptr<M0>& m0);

  template <typename M0, typename M1>
  bool TryFetch(std::shared_ptr<M0>& m0, std::shared_ptr<M1>& m1);

  template <typename M0, typename M1, typename M2>
  bool TryFetch(std::shared_ptr<M0>& m0, std::shared_ptr<M1>& m1,
                std::shared_ptr<M2>& m2);

  template <typename M0, typename M1, typename M2, typename M3>
  bool TryFetch(std::shared_ptr<M0>& m0, std::shared_ptr<M1>& m1,
                std::shared_ptr<M2>& m2, std::shared_ptr<M3>& m3);

  void RegisterCallback(std::function<void()>&& callback) {
    notify_ = true;
    notify_callback_ = callback;
  }

  DataStatistics GetStatistics() { return data_statistics_; }

  DataFlag& Flag() { return data_flag_; }

 private:
  void Initialize();

  std::shared_ptr<DataCache> data_cache_ = DataCache::Instance();
  DataFusion data_fusion_;
  DataFlag data_flag_;
  DataStatistics data_statistics_;
  int pop_num_ = 0;
  bool notify_ = false;
  std::function<void()> notify_callback_;
};

template <typename M0>
bool DataVisitor::TryFetch(std::shared_ptr<M0>& m0) {
  auto msg = std::shared_ptr<MetaData<M0>>();
  if (data_cache_->ReadDataCache(data_flag_.channel_vec[0],
                                 data_flag_.queue_size, &data_flag_.seq_id,
                                 &data_statistics_.drop_num, msg)) {
    m0 = msg->message;
    ++data_flag_.seq_id;
    data_statistics_.pop_num++;
    return true;
  }

  return false;
}

template <typename M0, typename M1>
bool DataVisitor::TryFetch(std::shared_ptr<M0>& m0, std::shared_ptr<M1>& m1) {
  bool ret = false;
  auto msg0 = std::shared_ptr<MetaData<M0>>();
  auto msg1 = std::shared_ptr<MetaData<M1>>();
  if (data_flag_.fusion_type == FusionType::NEWEST) {
    if (data_cache_->ReadDataCache(data_flag_.channel_vec[0],
                                   data_flag_.queue_size, &data_flag_.seq_id,
                                   &data_statistics_.drop_num, msg0) &&
        data_cache_->ReadDataCache(data_flag_.channel_vec[1], msg1)) {
      ret = true;
    }
  } else if (data_flag_.fusion_type == FusionType::TIME_WINDOW) {
    MetaMsgVec<M1> msgVecM1;
    if (data_cache_->ReadDataCache(data_flag_.channel_vec[0],
                                   data_flag_.queue_size, &data_flag_.seq_id,
                                   &data_statistics_.drop_num, msg0) &&
        data_cache_->ReadDataCache(data_flag_.channel_vec[1], msgVecM1)) {
      if (data_fusion_.process(msg0, msg1, msgVecM1)) {
        ret = true;
      }
    }
  }

  if (ret) {
    m0 = msg0->message;
    m1 = msg1->message;
    ++data_flag_.seq_id;
    data_statistics_.pop_num++;
  }
  return ret;
}

template <typename M0, typename M1, typename M2>
bool DataVisitor::TryFetch(std::shared_ptr<M0>& m0, std::shared_ptr<M1>& m1,
                           std::shared_ptr<M2>& m2) {
  bool ret = false;
  auto msg0 = std::shared_ptr<MetaData<M0>>();
  auto msg1 = std::shared_ptr<MetaData<M1>>();
  auto msg2 = std::shared_ptr<MetaData<M2>>();

  if (data_flag_.fusion_type == FusionType::NEWEST) {
    if (data_cache_->ReadDataCache(data_flag_.channel_vec[0],
                                   data_flag_.queue_size, &data_flag_.seq_id,
                                   &data_statistics_.drop_num, msg0) &&
        data_cache_->ReadDataCache(data_flag_.channel_vec[1], msg1) &&
        data_cache_->ReadDataCache(data_flag_.channel_vec[2], msg2)) {
      ret = true;
    }
  } else if (data_flag_.fusion_type == FusionType::TIME_WINDOW) {
    MetaMsgVec<M1> msgVecM1;
    MetaMsgVec<M2> msgVecM2;
    if (data_cache_->ReadDataCache(data_flag_.channel_vec[0],
                                   data_flag_.queue_size, &data_flag_.seq_id,
                                   &data_statistics_.drop_num, msg0) &&
        data_cache_->ReadDataCache(data_flag_.channel_vec[1], msgVecM1) &&
        data_cache_->ReadDataCache(data_flag_.channel_vec[2], msgVecM2)) {
      if (data_fusion_.process(msg0, msg1, msgVecM1, msg2, msgVecM2)) {
        ret = true;
      }
    }
  }

  if (ret) {
    m0 = msg0->message;
    m1 = msg1->message;
    m2 = msg2->message;
    ++data_flag_.seq_id;
    data_statistics_.pop_num++;
  }

  return ret;
}

template <typename M0, typename M1, typename M2, typename M3>
bool DataVisitor::TryFetch(std::shared_ptr<M0>& m0, std::shared_ptr<M1>& m1,
                           std::shared_ptr<M2>& m2, std::shared_ptr<M3>& m3) {
  bool ret = false;
  auto msg0 = std::shared_ptr<MetaData<M0>>();
  auto msg1 = std::shared_ptr<MetaData<M1>>();
  auto msg2 = std::shared_ptr<MetaData<M2>>();
  auto msg3 = std::shared_ptr<MetaData<M3>>();

  if (data_flag_.fusion_type == FusionType::NEWEST) {
    if (data_cache_->ReadDataCache(data_flag_.channel_vec[0],
                                   data_flag_.queue_size, &data_flag_.seq_id,
                                   &data_statistics_.drop_num, msg0) &&
        data_cache_->ReadDataCache(data_flag_.channel_vec[1], msg1) &&
        data_cache_->ReadDataCache(data_flag_.channel_vec[2], msg2) &&
        data_cache_->ReadDataCache(data_flag_.channel_vec[3], msg3)) {
      ret = true;
    }
  } else if (data_flag_.fusion_type == FusionType::TIME_WINDOW) {
    MetaMsgVec<M1> msgVecM1;
    MetaMsgVec<M2> msgVecM2;
    MetaMsgVec<M3> msgVecM3;
    if (data_cache_->ReadDataCache(data_flag_.channel_vec[0],
                                   data_flag_.queue_size, &data_flag_.seq_id,
                                   &data_statistics_.drop_num, msg0) &&
        data_cache_->ReadDataCache(data_flag_.channel_vec[1], msgVecM1) &&
        data_cache_->ReadDataCache(data_flag_.channel_vec[2], msgVecM2) &&
        data_cache_->ReadDataCache(data_flag_.channel_vec[3], msgVecM3)) {
      if (data_fusion_.process(msg0, msg1, msgVecM1, msg2, msgVecM2, msg3,
                               msgVecM3)) {
        ret = true;
      }
    }
  }

  if (ret) {
    m0 = msg0->message;
    m1 = msg1->message;
    m2 = msg2->message;
    m3 = msg3->message;
    ++data_flag_.seq_id;
    data_statistics_.pop_num++;
  }
  return ret;
}

}  // namespace data
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_DATA_DATA_VISITOR_H_
