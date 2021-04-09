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

#include "modules/control/controller/lon_controller.h"

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

const char data_path[] =
    "modules/control/testdata/longitudinal_controller_test/";

class LonControllerFuzzer : LonController {
 public:
  virtual void SetUp() {
    FLAGS_v = 3;

    ControlConf control_conf;
    std::string control_conf_file =
        "modules/control/testdata/conf/lincoln.pb.txt";

    CHECK(apollo::common::util::GetProtoFromFile(control_conf_file,
                                                 &control_conf));
    longitudinal_conf_ = control_conf.lon_controller_conf();

    timestamp_ = Clock::NowInSeconds();

    controller_.reset(new LonController());
  }

  void ComputeLongitudinalErrors(const TrajectoryAnalyzer *trajectory,
                                 const double preview_time,
                                 SimpleLongitudinalDebug *debug) {
    LonController::ComputeLongitudinalErrors(trajectory, preview_time, debug);
  }

  common::Status Init(const ControlConf *control_conf) {
    return LonController::Init(control_conf);
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
  LonControllerConf longitudinal_conf_;
  std::unique_ptr<LonController> controller_;
  double timestamp_ = 0.0;
}lon_controller_fuzzer;


void LonControllerFuzzer::target(planning::ADCTrajectory trajectory_pb) {
  FLAGS_enable_map_reference_unify = false;

  auto localization_pb =
      LoadLocalizationPb(std::string(data_path) + "1_localization.pb.txt");
  auto chassis_pb = LoadChassisPb(std::string(data_path) + "1_chassis.pb.txt");
  double time_now = Clock::NowInSeconds();
  trajectory_pb.mutable_header()->set_timestamp_sec(time_now);

  auto *vehicle_state = VehicleStateProvider::instance();
  vehicle_state->Update(localization_pb, chassis_pb);
  TrajectoryAnalyzer trajectory_analyzer(&trajectory_pb);

  double ts = longitudinal_conf_.ts();
  double preview_time = longitudinal_conf_.preview_window() * ts;

  SimpleLongitudinalDebug debug;
  ComputeLongitudinalErrors(&trajectory_analyzer, preview_time, &debug);
}

}  // namespace control
}  // namespace apollo

DEFINE_PROTO_FUZZER(const apollo::planning::ADCTrajectory& message) {
  if (message.header().module_name() != "" &&
    !message.trajectory_point().empty()) {
    apollo::control::lon_controller_fuzzer.target(message);
  }
}
