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

/**
 * @file
 */

#pragma once

#include <future>
#include <memory>
#include <utility>
#include <vector>

#include "cyber/base/bounded_queue.h"
#include "cyber/common/log.h"

namespace apollo {
namespace prediction {

class BaseThreadPool {
 public:
  BaseThreadPool(int thread_num, int next_thread_pool_level);

  void Stop();

  ~BaseThreadPool();

  template <typename InputIter, typename F>
  void ForEach(InputIter begin, InputIter end, F f) {
    std::vector<std::future<void>> futures;
    for (auto iter = begin; iter != end; ++iter) {
      auto& elem = *iter;
      futures.emplace_back(this->Post([&] { f(elem); }));
    }
    for (auto& future : futures) {
      if (future.valid()) {
        future.get();
      } else {
        AERROR << "Future is invalid.";
      }
    }
  }

  template <typename FuncType>
  std::future<typename std::result_of<FuncType()>::type> Post(FuncType&& func) {
    typedef typename std::result_of<FuncType()>::type ReturnType;
    typedef typename std::packaged_task<ReturnType()> TaskType;
    // Post requires that the functions in it are copy-constructible.
    // We used a shared pointer for the packaged_task,
    // Since it's only movable and non-copyable
    std::shared_ptr<TaskType> task =
        std::make_shared<TaskType>(std::move(func));
    std::future<ReturnType> returned_future = task->get_future();

    // Note: variables eg. `task` must be copied here because of the lifetime
    if (stopped_) {
      return std::future<ReturnType>();
    }
    task_queue_.Enqueue([task]() { (*task)(); });
    return returned_future;
  }

  static std::vector<int> THREAD_POOL_CAPACITY;

 private:
  std::vector<std::thread> workers_;
  apollo::cyber::base::BoundedQueue<std::function<void()>> task_queue_;
  std::atomic_bool stopped_;
};

template <int LEVEL>
class LevelThreadPool : public BaseThreadPool {
 public:
  static LevelThreadPool* Instance() {
    static LevelThreadPool<LEVEL> pool;
    return &pool;
  }

 private:
  LevelThreadPool() : BaseThreadPool(THREAD_POOL_CAPACITY[LEVEL], LEVEL + 1) {
    ADEBUG << "Level = " << LEVEL
           << "; thread pool capacity = " << THREAD_POOL_CAPACITY[LEVEL];
  }
};

class PredictionThreadPool {
 public:
  static BaseThreadPool* Instance();

  static thread_local int s_thread_pool_level;

  template <typename InputIter, typename F>
  static void ForEach(InputIter begin, InputIter end, F f) {
    Instance()->ForEach(begin, end, f);
  }
};

}  // namespace prediction
}  // namespace apollo
