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

#ifndef CYBER_SERVICE_CLIENT_BASE_H_
#define CYBER_SERVICE_CLIENT_BASE_H_

#include <chrono>
#include <string>

#include "cyber/common/macros.h"

namespace apollo {
namespace cyber {

/**
 * @class ClientBase
 * @brief Base class of Client
 *
 */
class ClientBase {
 public:
  /**
   * @brief Construct a new Client Base object
   *
   * @param service_name the service we can request
   */
  explicit ClientBase(const std::string& service_name)
      : service_name_(service_name) {}
  virtual ~ClientBase() {}

  /**
   * @brief Destroy the Client
   */
  virtual void Destroy() = 0;

  /**
   * @brief Get the service name
   */
  const std::string& ServiceName() const { return service_name_; }

  /**
   * @brief Ensure whether there is any Service named `service_name_`
   */
  virtual bool ServiceIsReady() const = 0;

 protected:
  std::string service_name_;

  bool WaitForServiceNanoseconds(std::chrono::nanoseconds time_out) {
    bool has_service = false;
    auto step_duration = std::chrono::nanoseconds(5 * 1000 * 1000);
    while (time_out.count() > 0) {
      has_service = service_discovery::TopologyManager::Instance()
                        ->service_manager()
                        ->HasService(service_name_);
      if (!has_service) {
        std::this_thread::sleep_for(step_duration);
        time_out -= step_duration;
      } else {
        break;
      }
    }
    return has_service;
  }
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SERVICE_CLIENT_BASE_H_
