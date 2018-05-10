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

#ifndef MODULES_MONITOR_MONITOR_H_
#define MODULES_MONITOR_MONITOR_H_

#include <memory>
#include <string>

#include "modules/common/apollo_app.h"
#include "modules/monitor/common/recurrent_runner.h"
#include "modules/monitor/proto/system_status.pb.h"

/**
 * @namespace apollo::monitor
 * @brief apollo::monitor
 */
namespace apollo {
namespace monitor {

class Monitor : public apollo::common::ApolloApp {
 public:
  Monitor();
  std::string Name() const override { return "SystemMonitor"; }

  apollo::common::Status Init() override;
  apollo::common::Status Start() override;
  void Stop() override;

 private:
  RecurrentRunnerThread monitor_thread_;
};

}  // namespace monitor
}  // namespace apollo

#endif  // MODULES_MONITOR_MONITOR_H_
