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

#ifndef MODULES_MONITOR_COMMON_MESSAGE_OBSERVER_H_
#define MODULES_MONITOR_COMMON_MESSAGE_OBSERVER_H_

#include <memory>
#include <mutex>
#include <string>

#include "cybertron/cybertron.h"
#include "modules/common/adapters/adapter_gflags.h"

/**
 * @namespace apollo::monitor
 * @brief apollo::monitor
 */
namespace apollo {
namespace monitor {

template <class T>
class MessageObserver {
 public:
  MessageObserver(const std::string& channel, apollo::cybertron::Node* node) {
    node->CreateReader<T>(channel, [this](const std::shared_ptr<T>& msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      msg_ = msg;
    });
  }

  const std::shared_ptr<T> GetLatest() {
    std::lock_guard<std::mutex> lock(mutex_);
    return msg_;
  }

 private:
  std::shared_ptr<T> msg_ = nullptr;
  std::mutex mutex_;
};

}  // namespace monitor
}  // namespace apollo

#endif  // MODULES_MONITOR_COMMON_MESSAGE_OBSERVER_H_
