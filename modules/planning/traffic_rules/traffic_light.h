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

#include "modules/planning/traffic_rules/traffic_rule.h"

namespace apollo {
namespace planning {

class TrafficLight : public TrafficRule {
 public:
  TrafficLight(const TrafficRuleConfig& config,
               const std::shared_ptr<DependencyInjector>& injector);

  virtual ~TrafficLight() = default;

  common::Status ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);

 private:
  void MakeDecisions(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);

 private:
  static constexpr char const* TRAFFIC_LIGHT_VO_ID_PREFIX = "TL_";
};

}  // namespace planning
}  // namespace apollo
