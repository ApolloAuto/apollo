#ifndef apollo_PERCEPTION_ONBOARD_COMMON_SHARED_DATA_H
#define apollo_PERCEPTION_ONBOARD_COMMON_SHARED_DATA_H

#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <boost/format.hpp>
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
  CommonSharedDataKey(const double& ts, const std::string& id)
      : timestamp(ts), device_id(id) {}
  virtual std::string to_string() const {
    return device_id +
           (boost::format("%ld") %
            static_cast<long>(timestamp * FLAGS_stamp_enlarge_factor))
               .str();
  }
  double timestamp = 0.0;
  std::string device_id = "";
};

struct CommonSharedDataStat {
  std::string to_string() const {
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

  virtual bool init() override {
    return true;
  }
  // @brief: you must impl your own name func
  // @return: name of your own class
  virtual std::string name() const = 0;

  // @brief: reset the shared data, clear data
  virtual void reset() override;

  virtual void remove_stale_data() override;

  // @brief: add new key shared data
  // @param [in]: key
  // @param [in]: value
  // @return : true or false
  bool add(const std::string& key, const SharedDataPtr<M>& data);
  bool add(const CommonSharedDataKey& key, const SharedDataPtr<M>& data);

  // @brief: get shared data for the given key
  // @param [in]: key
  // @param [out]: value with the key
  // @return : true or false
  bool get(const std::string& key, SharedDataPtr<M>* data);
  bool get(const CommonSharedDataKey& key, SharedDataPtr<M>* data);

  // @brief: remove shared data with the given key
  // @param [in]: key
  // @return : true or false
  bool remove(const std::string& key);
  bool remove(const CommonSharedDataKey& key);

  // @brief: get the data then remove it
  // @param [in]: key
  // @param [out]: value with the key
  // @return : true or false
  bool pop(const std::string& key, SharedDataPtr<M>* data);
  bool pop(const CommonSharedDataKey& key, SharedDataPtr<M>* data);

  // @brief: num of data stored in shared data
  // @return: num of data
  unsigned size() const {
    return _data_map.size();
  }

  CommonSharedDataStat get_stat() const {
    return _stat;
  }

 private:
  typedef std::map<std::string, SharedDataPtr<M>> SharedDataMap;
  typedef std::pair<std::string, SharedDataPtr<M>> SharedDataPair;
  typedef std::map<std::string, uint64_t>
      DataAddedTimeMap;  // precision in second
  typedef std::pair<std::string, uint64_t> DataKeyTimestampPair;

  SharedDataMap _data_map;
  Mutex _mutex;
  CommonSharedDataStat _stat;
  DataAddedTimeMap _data_added_time_map;

  DISALLOW_COPY_AND_ASSIGN(CommonSharedData);
};

template <class M>
void CommonSharedData<M>::reset() {
  MutexLock lock(&_mutex);
  AINFO << "Reset " << name() << ", map size: " << _data_map.size();
  _data_map.clear();
  _data_added_time_map.clear();
}

template <class M>
void CommonSharedData<M>::remove_stale_data() {
  MutexLock lock(&_mutex);
  const uint64_t now = ::time(NULL);
  bool has_change = false;
  for (auto iter = _data_added_time_map.begin();
       iter != _data_added_time_map.end();) {
    if (now - iter->second > FLAGS_shared_data_stale_time) {
      const size_t erase_cnt = _data_map.erase(iter->first);
      if (erase_cnt != 1u) {
        AWARN << "_data_map erase cnt:" << erase_cnt << " key:" << iter->first;
        return;
      }
      iter = _data_added_time_map.erase(iter);
      ++_stat.remove_cnt;
      has_change = true;
    } else {
      ++iter;
    }
  }
  if (has_change) {
    AINFO << "SharedData remove_stale_data name:" << name() << " stat:["
          << _stat.to_string() << "]";
  }
}

template <class M>
bool CommonSharedData<M>::add(const std::string& key,
                              const SharedDataPtr<M>& data) {
  MutexLock lock(&_mutex);
  auto ret = _data_map.emplace(SharedDataPair(key, data));
  if (!ret.second) {
    AWARN << "Duplicate key: " << key;
    return false;
  }

  const uint64_t timestamp = ::time(NULL);
  _data_added_time_map.emplace(DataKeyTimestampPair(key, timestamp));

  ++_stat.add_cnt;
  return true;
}

template <class M>
bool CommonSharedData<M>::add(const CommonSharedDataKey& key,
                              const SharedDataPtr<M>& data) {
  return add(key.to_string(), data);
}

template <class M>
bool CommonSharedData<M>::get(const std::string& key, SharedDataPtr<M>* data) {
  MutexLock lock(&_mutex);
  auto citer = _data_map.find(key);
  if (citer == _data_map.end()) {
    AWARN << "Failed to get shared data. key: " << key;
    return false;
  }
  *data = citer->second;
  ++_stat.get_cnt;
  return true;
}

template <class M>
bool CommonSharedData<M>::get(const CommonSharedDataKey& key,
                              SharedDataPtr<M>* data) {
  return get(key.to_string(), data);
}

template <class M>
bool CommonSharedData<M>::remove(const std::string& key) {
  MutexLock lock(&_mutex);
  const size_t num = _data_map.erase(key);
  if (num != 1u) {
    AWARN << "Only one element should be deleted with key: " << key
          << ", but num: " << num;
    return false;
  }

  const size_t erase_cnt = _data_added_time_map.erase(key);
  if (erase_cnt != 1u) {
    AWARN << "_data_added_time_map erase cnt:" << erase_cnt << " key:" << key;
    return false;
  }
  ++_stat.remove_cnt;
  return true;
}

template <class M>
bool CommonSharedData<M>::remove(const CommonSharedDataKey& key) {
  return remove(key.to_string());
}

template <class M>
bool CommonSharedData<M>::pop(const std::string& key, SharedDataPtr<M>* data) {
  MutexLock lock(&_mutex);
  auto citer = _data_map.find(key);
  if (citer == _data_map.end()) {
    AWARN << "Failed to get shared data. key: " << key;
    return false;
  }
  *data = citer->second;
  const size_t num = _data_map.erase(key);
  if (num != 1u) {
    AWARN << "Only one element should be deleted with key: " << key
          << ", but num: " << num;
    return false;
  }
  const size_t erase_cnt = _data_added_time_map.erase(key);
  if (erase_cnt != 1u) {
    AWARN << "_data_added_time_map erase cnt:" << erase_cnt << " key:" << key;
    return false;
  }
  ++_stat.get_cnt;
  ++_stat.remove_cnt;
  return true;
}

template <class M>
bool CommonSharedData<M>::pop(const CommonSharedDataKey& key,
                              SharedDataPtr<M>* data) {
  return pop(key.to_string(), data);
}

}  // namespace perception
}  // namespace apollo

#endif  // apollo_PERCEPTION_ONBOARD_COMMON_SHARED_DATA_H
