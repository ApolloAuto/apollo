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

#include <algorithm>
#include <future>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "boost/asio.hpp"
#include "boost/thread.hpp"

#include "cyber/common/log.h"

namespace apollo {
namespace prediction {

class PredictionThreadPool {
 public:
  PredictionThreadPool(int thread_num, int thread_pool_index);

  ~PredictionThreadPool() = default;

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
    io_service_.post([=] { (*task)(); });
    return returned_future;
  }

 private:
  boost::thread_group thread_group_;
  boost::asio::io_service io_service_;
  boost::asio::io_service::work work_;
};

}  // namespace prediction
}  // namespace apollo
