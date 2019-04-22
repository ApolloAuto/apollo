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

#include "modules/prediction/common/prediction_thread_pool.h"

#include "boost/preprocessor/repeat.hpp"

namespace apollo {
namespace prediction {

thread_local int PredictionThreadPool::s_thread_pool_level = 0;
std::vector<int> BaseThreadPool::THREAD_POOL_CAPACITY = {10, 10, 10};

BaseThreadPool::BaseThreadPool(
    int thread_num, int next_thread_pool_level) : work_(io_service_) {
  for (int i = 0; i < thread_num; ++i) {
    thread_group_.create_thread([this, next_thread_pool_level, i] {
      PredictionThreadPool::s_thread_pool_level = next_thread_pool_level;
      this->io_service_.run();
    });
  }
}

void BaseThreadPool::Stop() {
  io_service_.stop();
  thread_group_.join_all();
  stopped_ = true;
}

BaseThreadPool::~BaseThreadPool() {
  if (!stopped_) {
    try {
      Stop();
    } catch (std::exception& e) {
      AERROR << "Stop thread pool failed. " << e.what();
    }
  }
}

BaseThreadPool* PredictionThreadPool::Instance() {
  int pool_level = s_thread_pool_level;
  int max_thread_pool_level = static_cast<int>(
      BaseThreadPool::THREAD_POOL_CAPACITY.size());
  CHECK_LT(pool_level, max_thread_pool_level);
  int index = s_thread_pool_level;
  switch (index) {
    case 0: {
      return LevelThreadPool<0>::Instance();
    }
    case 1: {
      return LevelThreadPool<1>::Instance();
    }
    case 2: {
      return LevelThreadPool<2>::Instance();
    }
  }
  AERROR << "Should not hit here";
  return LevelThreadPool<0>::Instance();
}

}  // namespace prediction
}  // namespace apollo
