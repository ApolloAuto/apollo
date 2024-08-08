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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/dreamview_msgs/chart.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/planning/planning_base/proto/planning_config.pb.h"

#include "modules/common/math/vec2d.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_decider.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

class Planner;
class Frame;
class PublishableTrajectory;
class DependencyInjector;
class HDMap;
class LocalView;

using apollo::common::SLPoint;
using apollo::common::math::Vec2d;
/**
 * @class planning
 *
 * @brief PlanningBase module main class.
 */
class PlanningBase {
 public:
  PlanningBase() = delete;

  explicit PlanningBase(const std::shared_ptr<DependencyInjector>& injector);

  virtual ~PlanningBase();

  virtual apollo::common::Status Init(const PlanningConfig& config);

  virtual std::string Name() const = 0;

  virtual void RunOnce(const LocalView& local_view,
                       ADCTrajectory* const adc_trajectory) = 0;

  /**
   * @brief Plan the trajectory given current vehicle state
   */
  virtual apollo::common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const trajectory) = 0;

  /**
   * @brief Check if vehicle reaches the end point of the RoutingRequest.
   *
   * @return True if vehicle reaches the end point.
   */
  bool IsPlanningFinished(
      const ADCTrajectory::TrajectoryType& current_trajectory_type) const;

  bool GenerateWidthOfLane(const Vec2d& current_location, Vec2d& left_point,
                           Vec2d& right_point);

 protected:
  virtual void FillPlanningPb(const double timestamp,
                              ADCTrajectory* const trajectory_pb);

  void LoadPlanner();

  LocalView local_view_;
  const hdmap::HDMap* hdmap_ = nullptr;

  double start_time_ = 0.0;
  size_t seq_num_ = 0;

  PlanningConfig config_;
  TrafficDecider traffic_decider_;
  std::unique_ptr<Frame> frame_;
  std::shared_ptr<Planner> planner_;
  std::unique_ptr<PublishableTrajectory> last_publishable_trajectory_;
  std::shared_ptr<DependencyInjector> injector_;
};

}  // namespace planning
}  // namespace apollo
