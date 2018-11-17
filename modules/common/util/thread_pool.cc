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

/**
* @file
**/

#include "glog/logging.h"

#include "modules/common/util/thread_pool.h"

namespace apollo {
namespace common {
namespace util {

ThreadPool::ThreadPool() {}

void ThreadPool::Init(int pool_size) {
  instance()->pool_.reset(new ctpl::thread_pool(pool_size));
}

ctpl::thread_pool* ThreadPool::pool() {
  return CHECK_NOTNULL(instance()->pool_.get());
}

void ThreadPool::Stop() {
  if (instance()->pool_) {
    instance()->pool_->stop(true);
  }
}

}  // namespace util
}  // namespace common
}  // namespace apollo
