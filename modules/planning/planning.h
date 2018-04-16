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

#ifndef MODULES_PLANNING_PLANNING_H_
#define MODULES_PLANNING_PLANNING_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/traffic_rule_config.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/apollo_app.h"
#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/planner/planner.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class planning
 *
 * @brief Planning module main class. It processes GPS and IMU as input,
 * to generate planning info.
 */
class Planning : public apollo::common::ApolloApp {
 public:
  /**
   * @brief module name
   * @return module name
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  apollo::common::Status Init() override;

  /**
   * @brief module start function
   * @return start status
   */
  apollo::common::Status Start() override;

  /**
   * @brief module stop function
   */
  void Stop() override;

  /**
   * @brief main logic of the planning module, runs periodically triggered by
   * timer.
   */
  void RunOnce();

  /**
   * @brief record last planning trajectory
   */
  void SetLastPublishableTrajectory(const ADCTrajectory& adc_trajectory);

 private:
  // Watch dog timer
  void OnTimer(const ros::TimerEvent&);

  void PublishPlanningPb(ADCTrajectory* trajectory_pb, double timestamp);

  /**
   * @brief Fill the header and publish the planning message.
   */
  void Publish(planning::ADCTrajectory* trajectory) {
    using apollo::common::adapter::AdapterManager;
    AdapterManager::FillPlanningHeader(Name(), trajectory);
    AdapterManager::PublishPlanning(*trajectory);
  }

  void RegisterPlanners();

  /**
   * @brief Plan the trajectory given current vehicle state
   */
  common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* trajectory);

  common::Status InitFrame(const uint32_t sequence_num,
                           const common::TrajectoryPoint& planning_start_point,
                           const double start_time,
                           const common::VehicleState& vehicle_state);

  bool IsVehicleStateValid(const common::VehicleState& vehicle_state);
  void ExportReferenceLineDebug(planning_internal::Debug* debug);

  void SetFallbackCruiseTrajectory(ADCTrajectory* cruise_trajectory);

  double start_time_ = 0.0;

  apollo::common::util::Factory<PlanningConfig::PlannerType, Planner>
      planner_factory_;

  PlanningConfig config_;

  TrafficRuleConfigs traffic_rule_configs_;

  const hdmap::HDMap* hdmap_ = nullptr;

  std::unique_ptr<Frame> frame_;

  std::unique_ptr<Planner> planner_;

  std::unique_ptr<PublishableTrajectory> last_publishable_trajectory_;

  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;

  ros::Timer timer_;
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_PLANNING_H_ */
