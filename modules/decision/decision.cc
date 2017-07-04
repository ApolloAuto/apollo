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

#include "modules/decision/decision.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/time/time.h"
#include "modules/decision/common/decision_gflags.h"

namespace apollo {
namespace decision {

using apollo::common::adapter::AdapterManager;
using apollo::common::time::Clock;

std::string Decision::Name() const {
  return FLAGS_decision_module_name;
}

apollo::common::Status Decision::Init() {
  return apollo::common::Status::OK();
}

apollo::common::Status Decision::Start() {
  AdapterManager::Init();

  // start ROS timer, one-shot = false, auto-start = true
  const double duration = 1.0 / FLAGS_decision_publish_freq;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &Decision::OnTimer, this);
  return apollo::common::Status::OK();
}

void Decision::Stop() {  // spinner_.stop();
}

void Decision::OnTimer(const ros::TimerEvent&) {
  PublishDecision();
}

void Decision::PublishDecision() {
  DecisionResult decision_result;
  AdapterManager::FillDecisionHeader(Name(), decision_result.mutable_header());

  AdapterManager::PublishDecision(decision_result);
}

}  // namespace decision
}  // namespace apollo
