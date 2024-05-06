/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/planning/traffic_rules/stop_sign/proto/stop_sign.pb.h"

#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_rule.h"

namespace apollo {
namespace planning {

class StopSign : public TrafficRule {
 public:
  bool Init(const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;
  virtual ~StopSign() = default;
  common::Status ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);
  void Reset() override {}

 private:
  StopSignConfig config_;
  void MakeDecisions(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);

 private:
  static constexpr char const* STOP_SIGN_VO_ID_PREFIX = "SS_";
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::StopSign, TrafficRule)

}  // namespace planning
}  // namespace apollo
