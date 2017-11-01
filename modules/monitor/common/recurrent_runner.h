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

#ifndef MODULES_MONITOR_COMMON_RECURRENT_RUNNER_H_
#define MODULES_MONITOR_COMMON_RECURRENT_RUNNER_H_

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "modules/monitor/proto/system_status.pb.h"

/**
 * @namespace apollo::monitor
 * @brief apollo::monitor
 */
namespace apollo {
namespace monitor {

class RecurrentRunner {
 public:
  RecurrentRunner(const std::string &name, const double interval);
  virtual ~RecurrentRunner() = default;
  void Tick(const double current_time);

  virtual void RunOnce(const double current_time) = 0;

 protected:
  std::string name_;

 private:
  double interval_;
  double next_round_ = 0;
};

class RecurrentRunnerThread {
 public:
  explicit RecurrentRunnerThread(const double interval);

  void RegisterRunner(std::unique_ptr<RecurrentRunner> runner);
  void Start();
  void Stop();

 private:
  int64_t interval_ms_;
  std::vector<std::unique_ptr<RecurrentRunner>> runners_;
  std::unique_ptr<std::thread> thread_;

  bool stop_ = false;
  std::mutex stop_mutex_;
};

}  // namespace monitor
}  // namespace apollo

#endif  // MODULES_MONITOR_COMMON_RECURRENT_RUNNER_H_
