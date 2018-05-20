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
 * @file planning_thread_pool.cc
 **/

#include "modules/planning/common/planning_thread_pool.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

PlanningThreadPool::PlanningThreadPool() {}

void PlanningThreadPool::Init() {
  if (is_initialized) {
    return;
  }
  thread_pool_.reset(
      new common::util::ThreadPool(FLAGS_num_thread_planning_thread_pool));
  is_initialized = true;
}

void PlanningThreadPool::Synchronize() {
  for (auto& future : futures_) {
    future.wait();
  }
  futures_.clear();
}

}  // namespace planning
}  // namespace apollo
