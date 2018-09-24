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
#include "modules/perception/lib/thread/thread_worker.h"

namespace apollo {
namespace perception {
namespace lib {

void ThreadWorker::Bind(const std::function<bool()> &func) {
  work_func_ = func;
}

void ThreadWorker::Start() {
  if (thread_ptr_ == nullptr) {
    thread_ptr_.reset(new std::thread(&ThreadWorker::Core, this));
  }
  std::lock_guard<std::mutex> lock(mutex_);
  work_flag_ = false;
  exit_flag_ = false;
}

void ThreadWorker::WakeUp() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    work_flag_ = true;
  }
  condition_.notify_one();
}

void ThreadWorker::Join() {
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [&]() { return !work_flag_; });
}

void ThreadWorker::Release() {
  if (thread_ptr_ == nullptr) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    work_flag_ = true;
    exit_flag_ = true;
  }
  condition_.notify_one();
  thread_ptr_->join();
  thread_ptr_.reset(nullptr);
}

void ThreadWorker::Core() {
  while (true) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [&]() { return work_flag_; });
    }
    if (exit_flag_) {
      break;
    }
    work_func_();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      work_flag_ = false;
    }
    condition_.notify_one();
  }
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
