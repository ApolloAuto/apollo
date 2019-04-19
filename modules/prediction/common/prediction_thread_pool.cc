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

#include "boost/preprocessor/repeat.hpp"

#include "modules/prediction/common/prediction_thread_pool.h"

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

}  // namespace prediction
}  // namespace apollo
