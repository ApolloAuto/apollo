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

/**
 * @file trajectory_analyzer.h
 * @brief Defines the TrajectoryAnalyzer class.
 */

#pragma once

#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"

#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"

#include "modules/control/control_component/controller_task_base/common/trajectory_analyzer.h"

/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace apollo {
namespace control {

// using apollo::common::TrajectoryPoint;

/**
 * @class TrajectoryAnalyzerExtend
 * @brief process point query and conversion related to trajectory
 */
class TrajectoryAnalyzerExtend : public TrajectoryAnalyzer {
public:
    /**
     * @brief constructor
     */
    TrajectoryAnalyzerExtend() = default;

    TrajectoryAnalyzerExtend(const planning::ADCTrajectory *planning_published_trajectory) :
            TrajectoryAnalyzer(planning_published_trajectory) {}

    /**
     * @brief destructor
     */
    ~TrajectoryAnalyzerExtend() = default;

    /**
     * @brief query a point of trajectory that its s distance for reference trajectory point is closest
     * to the give reference trajectory point and relative s distance. The relative s distance is relative to the
     * reference trajectory point
     * @param t relative s distance for query
     * @return a point of trajectory
     */
    common::TrajectoryPoint QueryNearestPointByRelativeStation(
            const common::TrajectoryPoint &ref_point,
            const double &relative_station) const;
};

}  // namespace control
}  // namespace apollo
