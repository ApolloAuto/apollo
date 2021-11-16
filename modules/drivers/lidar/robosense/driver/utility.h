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
#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>
#define MAX_THREAD_NUM 4

namespace apollo {
namespace drivers {
namespace robosense {
template <typename T>
class Queue {
 public:
  Queue() { is_task_finished = true; }
  void push(const T &value) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_quque.push(value);
  }

  void pop() {
    if (!m_quque.empty()) {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_quque.pop();
    }
  }

  void clear() {
    std::queue<T> empty;
    std::lock_guard<std::mutex> lock(m_mutex);
    swap(empty, m_quque);
  }

 public:
  std::queue<T> m_quque;
  std::atomic<bool> is_task_finished;

 private:
  mutable std::mutex m_mutex;
};

struct Thread {
  Thread() { start = false; }
  std::shared_ptr<std::thread> m_thread;
  std::atomic<bool> start;
};
class ThreadPool {
 private:
  inline ThreadPool();

 public:
  typedef std::shared_ptr<ThreadPool> Ptr;
  ThreadPool(ThreadPool &) = delete;
  ThreadPool &operator=(const ThreadPool &) = delete;
  ~ThreadPool();

 public:
  static Ptr getInstance();
  int idlCount();
  template <class F, class... Args>
  inline auto commit(F &&f, Args &&... args)
      -> std::future<decltype(f(args...))> {
    if (stoped.load())
      throw std::runtime_error("Commit on ThreadPool is stopped.");
    using RetType = decltype(f(args...));
    auto task = std::make_shared<std::packaged_task<RetType()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...));  // wtf !
    std::future<RetType> future = task->get_future();
    {
      std::lock_guard<std::mutex> lock{m_lock};
      tasks.emplace([task]() { (*task)(); });
    }
    cv_task.notify_one();
    return future;
  }

 private:
  using Task = std::function<void()>;
  std::vector<std::thread> pool;
  std::queue<Task> tasks;
  std::mutex m_lock;
  std::condition_variable cv_task;
  std::atomic<bool> stoped;
  std::atomic<int> idl_thr_num;
  static Ptr instance_ptr;
  static std::mutex instance_mutex;
};

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
