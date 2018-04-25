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

#ifndef MODULES_PERCEPTION_ONBOARD_COMMON_SHARED_DATA_H_
#define MODULES_PERCEPTION_ONBOARD_COMMON_SHARED_DATA_H_

#include <algorithm>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "boost/format.hpp"
#include "gflags/gflags.h"

#include "modules/perception/lib/base/mutex.h"
#include "modules/perception/onboard/shared_data.h"

namespace apollo {
namespace perception {

// define shared data pointer
template <class T>
using SharedDataPtr = std::shared_ptr<T>;

template <class T>
using SharedDataConstPtr = std::shared_ptr<const T>;

DECLARE_int32(shared_data_stale_time);
DECLARE_int32(stamp_enlarge_factor);

struct CommonSharedDataKey {
  CommonSharedDataKey() = default;
  CommonSharedDataKey(const double &ts, const std::string &id)
      : timestamp(ts), device_id(id) {}
  virtual std::string ToString() const {
    return device_id +
           (boost::format("%ld") %
            static_cast<int64_t>(timestamp * FLAGS_stamp_enlarge_factor))
               .str();
  }
  double timestamp = 0.0;
  std::string device_id = "";
};

struct CommonSharedDataStat {
  std::string ToString() const {
    std::ostringstream oss;
    oss << "add_cnt:" << add_cnt << " remove_cnt:" << remove_cnt
        << " get_cnt:" << get_cnt;
    return oss.str();
  }

  uint64_t add_cnt = 0;
  uint64_t remove_cnt = 0;
  uint64_t get_cnt = 0;
};

// define shared data template for common usage
template <class M>
class CommonSharedData : public SharedData {
 public:
  CommonSharedData() {}
  virtual ~CommonSharedData() {}

  bool Init() override {
    latest_timestamp_ = std::numeric_limits<double>::min();
    return true;
  }
  // @brief: you must impl your own name func
  // @return: name of your own class
  virtual std::string name() const = 0;

  // @brief: reset the shared data, clear data
  void Reset() override;

  void RemoveStaleData() override;

  // @brief: add new key shared data
  // @param [in]: key
  // @param [in]: value
  // @return : true or false
  bool Add(const std::string &key, const SharedDataPtr<M> &data);

  bool Add(const CommonSharedDataKey &key, const SharedDataPtr<M> &data);

  // @brief: get shared data for the given key
  // @param [in]: key
  // @param [out]: value with the key
  // @return : true or false
  bool Get(const std::string &key, SharedDataPtr<M> *data);

  bool Get(const CommonSharedDataKey &key, SharedDataPtr<M> *data);

  double GetLatestTimestamp() const;
  // @brief: remove shared data with the given key
  // @param [in]: key
  // @return : true or false
  bool Remove(const std::string &key);

  bool Remove(const CommonSharedDataKey &key);

  // @brief: get the data then remove it
  // @param [in]: key
  // @param [out]: value with the key
  // @return : true or false
  bool Pop(const std::string &key, SharedDataPtr<M> *data);

  bool Pop(const CommonSharedDataKey &key, SharedDataPtr<M> *data);

  // @brief: num of data stored in shared data
  // @return: num of data
  unsigned Size() const { return data_map_.size(); }

  CommonSharedDataStat GetStat() const { return stat_; }

 private:
  typedef std::unordered_map<std::string, SharedDataPtr<M>> SharedDataMap;
  typedef std::pair<std::string, SharedDataPtr<M>> SharedDataPair;
  typedef std::unordered_map<std::string, uint64_t>
      DataAddedTimeMap;  // precision in second
  typedef std::pair<std::string, uint64_t> DataKeyTimestampPair;

  SharedDataMap data_map_;
  Mutex mutex_;
  CommonSharedDataStat stat_;
  DataAddedTimeMap data_added_time_map_;
  double latest_timestamp_ = std::numeric_limits<double>::min();

  DISALLOW_COPY_AND_ASSIGN(CommonSharedData);
};

template <class M>
void CommonSharedData<M>::Reset() {
  MutexLock lock(&mutex_);
  AINFO << "Reset " << name() << ", map size: " << data_map_.size();
  data_map_.clear();
  data_added_time_map_.clear();
  latest_timestamp_ = std::numeric_limits<double>::min();
}

template <class M>
void CommonSharedData<M>::RemoveStaleData() {
  MutexLock lock(&mutex_);
  const uint64_t now = ::time(NULL);
  bool has_change = false;
  for (auto iter = data_added_time_map_.begin();
       iter != data_added_time_map_.end();) {
    if (now - iter->second >
        static_cast<uint64_t>(FLAGS_shared_data_stale_time)) {
      const size_t erase_cnt = data_map_.erase(iter->first);
      if (erase_cnt != 1u) {
        AWARN << "data_map_ erase cnt:" << erase_cnt << " key:" << iter->first;
        return;
      }
      iter = data_added_time_map_.erase(iter);
      ++stat_.remove_cnt;
      has_change = true;
    } else {
      ++iter;
    }
  }
  if (has_change) {
    AINFO << "SharedData remove_stale_data name:" << name() << " stat:["
          << stat_.ToString() << "]";
  }
}

template <class M>
bool CommonSharedData<M>::Add(const std::string &key,
                              const SharedDataPtr<M> &data) {
  MutexLock lock(&mutex_);
  auto ret = data_map_.emplace(SharedDataPair(key, data));
  if (!ret.second) {
    AWARN << "Duplicate key: " << key;
    return false;
  }

  const uint64_t timestamp = ::time(NULL);
  data_added_time_map_.emplace(DataKeyTimestampPair(key, timestamp));
  ++stat_.add_cnt;
  return true;
}

template <class M>
bool CommonSharedData<M>::Add(const CommonSharedDataKey &key,
                              const SharedDataPtr<M> &data) {
  // update latest_timestamp for SharedData
  latest_timestamp_ = key.timestamp;
  return Add(key.ToString(), data);
}

template <class M>
bool CommonSharedData<M>::Get(const std::string &key, SharedDataPtr<M> *data) {
  MutexLock lock(&mutex_);
  auto citer = data_map_.find(key);
  if (citer == data_map_.end()) {
    AWARN << "Failed to get shared data. key: " << key;
    return false;
  }
  *data = citer->second;
  ++stat_.get_cnt;
  return true;
}

template <class M>
bool CommonSharedData<M>::Get(const CommonSharedDataKey &key,
                              SharedDataPtr<M> *data) {
  return Get(key.ToString(), data);
}

template <class M>
double CommonSharedData<M>::GetLatestTimestamp() const {
  return latest_timestamp_;
}

template <class M>
bool CommonSharedData<M>::Remove(const std::string &key) {
  MutexLock lock(&mutex_);
  const size_t num = data_map_.erase(key);
  if (num != 1u) {
    AWARN << "Only one element should be deleted with key: " << key
          << ", but num: " << num;
    return false;
  }

  const size_t erase_cnt = data_added_time_map_.erase(key);
  if (erase_cnt != 1u) {
    AWARN << "data_added_time_map_ erase cnt:" << erase_cnt << " key:" << key;
    return false;
  }
  ++stat_.remove_cnt;
  return true;
}

template <class M>
bool CommonSharedData<M>::Remove(const CommonSharedDataKey &key) {
  return Remove(key.ToString());
}

template <class M>
bool CommonSharedData<M>::Pop(const std::string &key, SharedDataPtr<M> *data) {
  MutexLock lock(&mutex_);
  auto citer = data_map_.find(key);
  if (citer == data_map_.end()) {
    AWARN << "Failed to get shared data. key: " << key;
    return false;
  }
  *data = citer->second;
  const size_t num = data_map_.erase(key);
  if (num != 1u) {
    AWARN << "Only one element should be deleted with key: " << key
          << ", but num: " << num;
    return false;
  }
  const size_t erase_cnt = data_added_time_map_.erase(key);
  if (erase_cnt != 1u) {
    AWARN << "data_added_time_map_ erase cnt:" << erase_cnt << " key:" << key;
    return false;
  }
  ++stat_.get_cnt;
  ++stat_.remove_cnt;
  return true;
}

template <class M>
bool CommonSharedData<M>::Pop(const CommonSharedDataKey &key,
                              SharedDataPtr<M> *data) {
  return Pop(key.ToString(), data);
}

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_ONBOARD_COMMON_SHARED_DATA_H_
