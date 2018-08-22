/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.

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

#include "modules/control/controller/lat_controller.h"

#include <memory>
#include <string>
#include <utility>
#include <iostream>
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/planning/proto/planning.pb.h"
#include "libfuzzer/libfuzzer_macro.h"

protobuf_mutator::protobuf::LogSilencer log_silincer;

namespace apollo {
namespace control {

using apollo::common::time::Clock;
using PlanningTrajectoryPb = apollo::planning::ADCTrajectory;
using LocalizationPb = apollo::localization::LocalizationEstimate;
using ChassisPb = apollo::canbus::Chassis;
using apollo::common::VehicleStateProvider;

/******************************************************
* Fuzzer class should inherit the target class in order
* to invoke the target member function
******************************************************/

class LatControllerFuzzer : LatController {
 public:
  virtual void SetUp() {
    FLAGS_v = 3;
    std::string control_conf_file =
        "modules/control/testdata/conf/lincoln.pb.txt";
    ControlConf control_conf;
    CHECK(apollo::common::util::GetProtoFromFile(control_conf_file,
                                                 &control_conf));
    lateral_conf_ = control_conf.lat_controller_conf();

    timestamp_ = Clock::NowInSeconds();
  }
  void ComputeLateralErrors(const double x, const double y, const double theta,
                            const double linear_v, const double angular_v,
                            const TrajectoryAnalyzer &trajectory_analyzer,
                            SimpleLateralDebug *debug) {
    LatController::ComputeLateralErrors(x, y, theta, linear_v, angular_v,
                                        trajectory_analyzer, debug);
  }
  void target(planning::ADCTrajectory planning_trajectory_pb);

 protected:
  LocalizationPb LoadLocalizationPb(const std::string &filename) {
    LocalizationPb localization_pb;
    CHECK(apollo::common::util::GetProtoFromFile(filename, &localization_pb))
        << "Failed to open file " << filename;
    localization_pb.mutable_header()->set_timestamp_sec(timestamp_);
    return localization_pb;
  }

  ChassisPb LoadChassisPb(const std::string &filename) {
    ChassisPb chassis_pb;
    CHECK(apollo::common::util::GetProtoFromFile(filename, &chassis_pb))
        << "Failed to open file " << filename;
    chassis_pb.mutable_header()->set_timestamp_sec(timestamp_);
    return chassis_pb;
  }

  PlanningTrajectoryPb LoadPlanningTrajectoryPb(const std::string &filename) {
    PlanningTrajectoryPb planning_trajectory_pb;
    CHECK(apollo::common::util::GetProtoFromFile(filename,
                                                 &planning_trajectory_pb))
        << "Failed to open file " << filename;
    planning_trajectory_pb.mutable_header()->set_timestamp_sec(timestamp_);
    return planning_trajectory_pb;
  }
  double timestamp_ = 0.0;
  LatControllerConf lateral_conf_;
};
/******************************************************
* Implements the target function that takes the protobuf
* message structure. This function is modified from the 
* original TEST_F gtest function. 
******************************************************/
void LatControllerFuzzer::target(planning::ADCTrajectory
    planning_trajectory_pb) {
  timestamp_ = Clock::NowInSeconds();

  auto localization_pb = LoadLocalizationPb(
      "modules/control/testdata/lateral_controller_test/"
      "1_localization.pb.txt");
  auto chassis_pb = LoadChassisPb(
      "modules/control/testdata/lateral_controller_test/1_chassis.pb.txt");
  FLAGS_enable_map_reference_unify = false;
  auto *vehicle_state = VehicleStateProvider::instance();
  vehicle_state->Update(localization_pb, chassis_pb);

  /*auto planning_trajectory_pb = LoadPlanningTrajectoryPb(
      "modules/control/testdata/lateral_controller_test/"
      "1_planning.pb.txt");*/
  TrajectoryAnalyzer trajectory_analyzer(&planning_trajectory_pb);

  ControlCommand cmd;
  SimpleLateralDebug *debug = cmd.mutable_debug()->mutable_simple_lat_debug();
  ComputeLateralErrors(
      vehicle_state->x(), vehicle_state->y(), vehicle_state->heading(),
      vehicle_state->linear_velocity(), vehicle_state->angular_velocity(),
      trajectory_analyzer, debug);
}

}  // namespace control
}  // namespace apollo

/******************************************************
* The test driver function. feed the mutated message to
* the target function. In addition, some doman specific
* constraints can be added, befor feeding the message.
* The constraints can also help bypass discovered bugs, 
* that are not patched yet. 
******************************************************/
DEFINE_PROTO_FUZZER(const apollo::planning::ADCTrajectory& message) {
  apollo::control::LatControllerFuzzer fuzzer;
  if (message.header().module_name() != "") {
    fuzzer.target(message);
  }
}
