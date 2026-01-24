/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 * @file s_shape_speed.h
 **/

#pragma once

#include "modules/planning/planning_base/common/speed/speed_data.h"

namespace apollo {
namespace planning {

class SpeedPlanner {
public:
    static void FastStop(double v0, double a0, double max_deceleration, SpeedData* speed_data);

    static void SmoothStop(
            double v0,
            double a0,
            double max_deceleration,
            double max_pos_jerk,
            double max_neg_jerk,
            SpeedData* speed_data);
};

}  // namespace planning
}  // namespace apollo
