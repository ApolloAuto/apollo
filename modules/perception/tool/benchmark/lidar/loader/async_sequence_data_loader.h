/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#pragma once
#include <algorithm>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "modules/perception/tool/benchmark/lidar/ctpl/ctpl.h"
#include "modules/perception/tool/benchmark/lidar/loader/sequence_data_loader.h"

namespace apollo {
namespace perception {
namespace benchmark {

template <class Dtype>
struct Cache {
  std::shared_ptr<Dtype> data;
  bool loaded = false;
  bool load_success = false;
  std::future<void> status;
};

// @brief General asynchronous data loader class, class DataType must implement
// "bool load(const std::vector<std::string>& filenames)" member function
template <class DataType>
class AsyncSequenceDataLoader : public SequenceDataLoader<DataType> {
 public:
  AsyncSequenceDataLoader() = default;
  ~AsyncSequenceDataLoader() = default;
  void set(std::size_t cache_size, std::size_t prefetch_size,
           std::size_t thread_num) {
    std::lock_guard<std::mutex> lock(_mutex);
    _fixed_cache_size = cache_size;
    _prefetch_data_size = prefetch_size;
    if (_thread_pool == nullptr) {
      _thread_pool.reset(new ctpl::thread_pool(static_cast<int>(thread_num)));
    } else {
      _thread_pool->stop(true);
      _thread_pool->start(static_cast<int>(thread_num));
    }
    _cached_data.clear();
  }
  bool query_next(std::shared_ptr<DataType>& data) override;  // NOLINT
  bool query_last(std::shared_ptr<DataType>& data) override;  // NOLINT

 protected:
  using CachePtr = std::shared_ptr<Cache<DataType>>;
  using SequenceDataLoader<DataType>::_initialized;
  using SequenceDataLoader<DataType>::_idx;
  using SequenceDataLoader<DataType>::_filenames;

 protected:
  std::size_t _fixed_cache_size = 50;
  std::size_t _prefetch_data_size = 5;

 private:
  std::map<int, CachePtr> _cached_data;
  std::unique_ptr<ctpl::thread_pool> _thread_pool;
  std::mutex _mutex;
};

// @brief pipeline for query next and query last
// case 1. not load, then loading
// case 2. pre-loading, then wait
// case 3. loaded, then do nothing
// finally trigger prefetching

template <class DataType>
bool AsyncSequenceDataLoader<DataType>::query_next(
    std::shared_ptr<DataType>& data) {  // NOLINT
  if (_thread_pool == nullptr) {
    return false;
  }
  if (!_initialized) {
    return false;
  }
  std::lock_guard<std::mutex> lock(_mutex);
  ++_idx;
  if (_idx >= static_cast<int>(_filenames[0].size())) {
    return false;
  } else if (_idx < 0) {
    _idx = 0;
  }

  bool load_success = false;
  // load current idx
  auto iter = _cached_data.find(_idx);
  // not load yet
  if (iter == _cached_data.end()) {
    std::cerr << "Fail to prefetch, start loading..." << std::endl;
    CachePtr cache_ptr(new Cache<DataType>);
    cache_ptr->data.reset(new DataType);
    std::vector<std::string> files;
    for (auto& names : _filenames) {
      files.push_back(names[_idx]);
    }
    cache_ptr->loaded = true;
    cache_ptr->load_success = cache_ptr->data->load(files);
    _cached_data.emplace(_idx, cache_ptr);
    data = cache_ptr->data;
    load_success = cache_ptr->load_success;
  } else {
    // loaded
    if (!iter->second->loaded) {
      iter->second->status.wait();
      iter->second->loaded = true;
    }
    data = iter->second->data;
    load_success = iter->second->load_success;
  }
  // prefetch next data
  for (int i = _idx + 1;
       i <= std::min(static_cast<std::size_t>(_idx) + _prefetch_data_size,
                     _filenames[0].size() - 1);
       ++i) {
    auto prefetch_iter = _cached_data.find(i);
    if (prefetch_iter == _cached_data.end()) {
      CachePtr cache_ptr(new Cache<DataType>);
      cache_ptr->data.reset(new DataType);
      std::vector<std::string> files;
      for (auto& names : _filenames) {
        files.push_back(names[i]);
      }
      cache_ptr->status = _thread_pool->push([cache_ptr, files](int id) {
        cache_ptr->load_success = cache_ptr->data->load(files);
      });
      _cached_data.emplace(i, cache_ptr);
    }
  }
  // clean stale data on the other end
  for (auto citer = _cached_data.begin();
       citer != _cached_data.end() &&
       _cached_data.size() > _fixed_cache_size;) {
    if (citer->second->loaded && _idx != citer->first) {
      citer->second->data->release();
      _cached_data.erase(citer++);
    } else {
      break;
    }
  }
  return load_success;
}

template <class DataType>
bool AsyncSequenceDataLoader<DataType>::query_last(
    std::shared_ptr<DataType>& data) {  // NOLINT
  if (_thread_pool == nullptr) {
    return false;
  }
  if (!_initialized) {
    return false;
  }
  std::lock_guard<std::mutex> lock(_mutex);
  if (data == nullptr) {
    data.reset(new DataType);
  }
  --_idx;
  if (_idx < 0) {
    return false;
  } else if (_idx >= static_cast<int>(_filenames[0].size())) {
    _idx = static_cast<int>(_filenames[0].size() - 1);
  }
  bool load_success = false;
  // load current idx
  auto iter = _cached_data.find(_idx);
  // not load yet
  if (iter == _cached_data.end()) {
    std::cerr << "Fail to prefetch, start loading..." << std::endl;
    CachePtr cache_ptr(new Cache<DataType>);
    cache_ptr->data.reset(new DataType);
    cache_ptr->loaded = true;
    std::vector<std::string> files;
    for (auto& names : _filenames) {
      files.push_back(names[_idx]);
    }
    cache_ptr->load_success = cache_ptr->data->load(files);
    _cached_data.emplace(_idx, cache_ptr);
    data = cache_ptr->data;
    load_success = cache_ptr->load_success;
  } else {
    // loaded
    if (!iter->second->loaded) {
      iter->second->status.wait();
      iter->second->loaded = true;
    }
    data = iter->second->data;
    load_success = iter->second->load_success;
  }
  // prefetch last data
  for (int i = _idx - 1;
       i >= std::max(_idx - static_cast<int>(_prefetch_data_size), 0); --i) {
    auto prefetch_iter = _cached_data.find(i);
    if (prefetch_iter == _cached_data.end()) {
      CachePtr cache_ptr(new Cache<DataType>);
      cache_ptr->data.reset(new DataType);
      std::vector<std::string> files;
      for (auto& names : _filenames) {
        files.push_back(names[i]);
      }
      cache_ptr->status = _thread_pool->push([cache_ptr, files](int id) {
        cache_ptr->load_success = cache_ptr->data->load(files);
      });
      _cached_data.emplace(i, cache_ptr);
    }
  }
  // clean stale data on the other end
  for (auto citer = _cached_data.rbegin();
       citer != _cached_data.rend() &&
       _cached_data.size() > _fixed_cache_size;) {
    if (citer->second->loaded && citer->first != _idx) {
      citer->second->data->release();
      _cached_data.erase((++citer).base());
    } else {
      break;
    }
  }
  return load_success;
}

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
