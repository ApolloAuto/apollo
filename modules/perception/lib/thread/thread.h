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

#include <string>

namespace apollo {
namespace perception {
namespace lib {

class Thread {
 public:
  explicit Thread(bool joinable = false, const std::string &name = "Thread")
      : tid_(0), started_(false), joinable_(joinable), thread_name_(name) {}

  pthread_t tid() const { return tid_; }

  void set_joinable(bool joinable) {
    if (!started_) {
      joinable_ = joinable;
    }
  }

  void Start();

  void Join();

  bool IsAlive();

  std::string get_thread_name() const { return thread_name_; }
  void set_thread_name(const std::string &name) { thread_name_ = name; }

  Thread(const Thread &) = delete;
  Thread &operator=(const Thread &) = delete;

 protected:
  virtual void Run() = 0;

  static void *ThreadRunner(void *arg) {
    Thread *t = reinterpret_cast<Thread *>(arg);
    t->Run();
    return nullptr;
  }

  pthread_t tid_;
  bool started_;
  bool joinable_;
  std::string thread_name_;

 private:
};

}  // namespace lib
}  // namespace perception
}  // namespace apollo
