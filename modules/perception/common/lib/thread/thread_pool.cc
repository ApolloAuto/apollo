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
#include "modules/perception/common/lib/thread/thread_pool.h"

namespace apollo {
namespace perception {
namespace lib {

// Just use the protobuf Closure.
using google::protobuf::Closure;

using std::vector;

ThreadPool::ThreadPool(int num_workers)
    : num_workers_(num_workers),
      num_available_workers_(num_workers),
      task_queue_(num_workers),
      started_(false) {
  workers_.reserve(num_workers_);
  for (int idx = 0; idx < num_workers_; ++idx) {
    ThreadPoolWorker *worker = new ThreadPoolWorker(this);
    workers_.push_back(worker);
  }
}

ThreadPool::~ThreadPool() {
  if (!started_) {
    return;
  }

  for (int idx = 0; idx < num_workers_; ++idx) {
    Add(NULL);
  }

  for (int idx = 0; idx < num_workers_; ++idx) {
    workers_[idx]->Join();
    delete workers_[idx];
  }
}

void ThreadPool::Start() {
  for (int idx = 0; idx < num_workers_; ++idx) {
    workers_[idx]->Start();
  }
  started_ = true;
}

void ThreadPool::Add(Closure *closure) { task_queue_.Push(closure); }

void ThreadPool::Add(const vector<Closure *> &closures) {
  for (size_t idx = 0; idx < closures.size(); ++idx) {
    Add(closures[idx]);
  }
}

void ThreadPoolWorker::Run() {
  while (true) {
    Closure *closure = nullptr;
    thread_pool_->task_queue_.Pop(&closure);
    if (closure == nullptr) {
      break;
    }

    {
      MutexLock lock(&(thread_pool_->mutex_));
      --(thread_pool_->num_available_workers_);
    }

    closure->Run();

    {
      MutexLock lock(&(thread_pool_->mutex_));
      ++(thread_pool_->num_available_workers_);
    }
  }
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
