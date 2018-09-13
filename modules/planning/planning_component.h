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

#ifndef MODULES_PLANNING_PLANNING_COMPONENT_H_
#define MODULES_PLANNING_PLANNING_COMPONENT_H_

#include <memory>
#include <mutex>

#include "cybertron/class_loader/class_loader.h"
#include "cybertron/component/component.h"
#include "cybertron/message/raw_message.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_conf.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/planning/planning_base.h"

namespace apollo {
namespace planning {

class PlanningComponent final
    : public cybertron::Component<prediction::PredictionObstacles,
                                  canbus::Chassis,
                                  localization::LocalizationEstimate> {
 public:
  PlanningComponent() = default;
  ~PlanningComponent() = default;

 public:
  bool Init() override;

  bool Proc(const std::shared_ptr<prediction::PredictionObstacles>&
                prediction_obstacles,
            const std::shared_ptr<canbus::Chassis>& chassis,
            const std::shared_ptr<localization::LocalizationEstimate>&
                localization_estimate) override;

 private:
  PlanningConf planning_conf_;
  std::shared_ptr<cybertron::Reader<perception::TrafficLightDetection>>
      traffic_light_reader_;
  std::shared_ptr<cybertron::Reader<routing::RoutingResponse>> routing_reader_;

  std::shared_ptr<cybertron::Writer<ADCTrajectory>> writer_;

  std::mutex mutex_;
  perception::TrafficLightDetection traffic_light_;
  routing::RoutingResponse routing_;

  LocalView local_view_;

  std::unique_ptr<PlanningBase> planning_base_;
};

CYBERTRON_REGISTER_COMPONENT(PlanningComponent)

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNING_COMPONENT_H_
