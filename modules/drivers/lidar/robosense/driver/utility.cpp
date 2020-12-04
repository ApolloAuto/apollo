/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/drivers/lidar/robosense/driver/utility.h"
namespace apollo {
namespace drivers {
namespace robosense {

ThreadPool::Ptr ThreadPool::instance_ptr = nullptr;
std::mutex ThreadPool::instance_mutex;
ThreadPool::ThreadPool() : stoped{false} {
  idl_thr_num = MAX_THREAD_NUM;
  for (int i = 0; i < idl_thr_num; ++i) {
    pool.emplace_back([this] {
      while (!this->stoped) {
        std::function<void()> task;
        {
          std::unique_lock<std::mutex> lock{this->m_lock};
          this->cv_task.wait(lock, [this] {
            return this->stoped.load() || !this->tasks.empty();
          });
          if (this->stoped && this->tasks.empty()) return;
          task = std::move(this->tasks.front());
          this->tasks.pop();
        }
        idl_thr_num--;
        task();
        idl_thr_num++;
      }
    });
  }
}

ThreadPool::Ptr ThreadPool::getInstance() {
  if (instance_ptr == nullptr) {
    std::lock_guard<std::mutex> lk(instance_mutex);
    if (instance_ptr == nullptr) {
      instance_ptr = std::shared_ptr<ThreadPool>(new ThreadPool);
    }
  }
  return instance_ptr;
}
ThreadPool::~ThreadPool() {
  stoped.store(true);
  cv_task.notify_all();
  for (std::thread &thread : pool) {
    thread.join();
  }
}

int ThreadPool::idlCount() { return idl_thr_num; }

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
