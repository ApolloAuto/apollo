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

#include <condition_variable>
#include <functional>
#include <memory>
#include <thread>

namespace apollo {
namespace perception {
namespace lib {

class ThreadWorker {
 public:
  ThreadWorker() = default;
  ~ThreadWorker() { Release(); }

  // bind a bool returned function, should be called before start
  void Bind(const std::function<bool()> &func);

  // start the thread loopy running
  void Start();

  // wake up the thread to do something
  void WakeUp();

  // wait util the one time execution is done
  void Join();

  // release the thread resources
  void Release();

 private:
  // the main function of thread
  void Core();

 private:
  std::unique_ptr<std::thread> thread_ptr_;
  std::mutex mutex_;
  std::condition_variable condition_;

  bool work_flag_ = false;
  bool exit_flag_ = true;

  std::function<bool()> work_func_;
};

}  // namespace lib
}  // namespace perception
}  // namespace apollo
