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
#pragma once

#include <vector>

#include "google/protobuf/stubs/callback.h"
#include "google/protobuf/stubs/common.h"

#include "modules/perception/common/lib/thread/concurrent_queue.h"
#include "modules/perception/common/lib/thread/mutex.h"
#include "modules/perception/common/lib/thread/thread.h"

namespace apollo {
namespace perception {
namespace lib {

class ThreadPoolWorker;

class ThreadPool {
 public:
  explicit ThreadPool(int num_workers);

  ~ThreadPool();

  void Start();

  void Add(google::protobuf::Closure *closure);
  void Add(const std::vector<google::protobuf::Closure *> &closures);

  int num_workers() const { return num_workers_; }

  int num_available_workers() const { return num_available_workers_; }

  ThreadPool(const ThreadPool &) = delete;
  ThreadPool &operator=(const ThreadPool &) = delete;

 private:
  friend class ThreadPoolWorker;

  std::vector<ThreadPoolWorker *> workers_;
  int num_workers_;
  int num_available_workers_;
  Mutex mutex_;

  FixedSizeConQueue<google::protobuf::Closure *> task_queue_;
  bool started_;
};

class ThreadPoolWorker : public Thread {
 public:
  explicit ThreadPoolWorker(ThreadPool *thread_pool)
      : Thread(true, "ThreadPoolWorker"), thread_pool_(thread_pool) {}

  virtual ~ThreadPoolWorker() {}

  ThreadPoolWorker(const ThreadPoolWorker &) = delete;
  ThreadPoolWorker &operator=(const ThreadPoolWorker &) = delete;

 protected:
  void Run() override;

 private:
  ThreadPool *thread_pool_;
};

}  // namespace lib
}  // namespace perception
}  // namespace apollo
