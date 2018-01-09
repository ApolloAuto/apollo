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

/**
 * @file
 **/

#ifndef MODULES_PLANNING_COMMON_PLANNING_THREAD_POOL_H_
#define MODULES_PLANNING_COMMON_PLANNING_THREAD_POOL_H_

#include <memory>

#include "modules/common/macro.h"
#include "modules/common/util/ctpl_stl.h"

namespace apollo {
namespace planning {

/**
 * @class PlanningThreadPool
 *
 * @brief A singleton class that contains thread pool for planning
 */

class PlanningThreadPool {
 public:
  void Init();
  common::util::ThreadPool* mutable_thread_pool() {
    return thread_pool_.get();
  }

 private:
  std::unique_ptr<common::util::ThreadPool> thread_pool_;
  bool is_initialized = false;

  DECLARE_SINGLETON(PlanningThreadPool);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_PLANNING_THREAD_POOL_H_
