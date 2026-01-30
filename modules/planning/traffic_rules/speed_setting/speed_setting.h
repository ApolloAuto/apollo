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
 * @file speed_setting.h
 **/

#pragma once

#include "modules/common/status/status.h"
#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_rule.h"

namespace apollo {
namespace planning {

class SpeedSetting : public TrafficRule {
public:
    SpeedSetting();

    virtual ~SpeedSetting() = default;

    common::Status ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info);

    void Reset() override {}

private:
    double last_cruise_speed_;
    uint32_t last_sequence_num_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::SpeedSetting, TrafficRule)

}  // namespace planning
}  // namespace apollo
