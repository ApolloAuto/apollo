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

#include "gmock/gmock.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"

#include "modules/control/proto/control_conf.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/common/control_gflags.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace control {

using apollo::common::time::Clock;
using LocalizationPb = localization::LocalizationEstimate;
using ChassisPb = canbus::Chassis;
using TrajectoryPb = planning::ADCTrajectory;
using apollo::common::VehicleStateProvider;

const char data_path[] =
    "modules/control/testdata/longitudinal_controller_test/";

class LonControllerTest : public ::testing::Test, LonController {
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

 protected:
  LocalizationPb LoadLocalizationPb(const std::string &filename) {
    LocalizationPb localization;
    CHECK(apollo::common::util::GetProtoFromFile(filename, &localization))
        << "Failed to open file " << filename;
    localization.mutable_header()->set_timestamp_sec(timestamp_);
    return localization;
  }

  ChassisPb LoadChassisPb(const std::string &filename) {
    ChassisPb chassis_pb;
    CHECK(apollo::common::util::GetProtoFromFile(filename, &chassis_pb))
        << "Failed to open file " << filename;
    chassis_pb.mutable_header()->set_timestamp_sec(timestamp_);
    return chassis_pb;
  }

  TrajectoryPb LoadPlanningTrajectoryPb(const std::string &filename) {
    TrajectoryPb trajectory_pb;
    CHECK(apollo::common::util::GetProtoFromFile(filename, &trajectory_pb))
        << "Failed to open file " << filename;

    trajectory_pb.mutable_header()->set_timestamp_sec(timestamp_);
    return trajectory_pb;
  }

  LonControllerConf longitudinal_conf_;
  std::unique_ptr<LonController> controller_;
  double timestamp_ = 0.0;
};

TEST_F(LonControllerTest, ComputeLongitudinalErrors) {
  FLAGS_enable_map_reference_unify = false;

  auto localization_pb =
      LoadLocalizationPb(std::string(data_path) + "1_localization.pb.txt");
  auto chassis_pb = LoadChassisPb(std::string(data_path) + "1_chassis.pb.txt");
  auto trajectory_pb =
      LoadPlanningTrajectoryPb(std::string(data_path) + "1_planning.pb.txt");

  double time_now = Clock::NowInSeconds();
  trajectory_pb.mutable_header()->set_timestamp_sec(time_now);

  auto *vehicle_state = VehicleStateProvider::instance();
  vehicle_state->Update(localization_pb, chassis_pb);
  TrajectoryAnalyzer trajectory_analyzer(&trajectory_pb);

  double ts = longitudinal_conf_.ts();
  double preview_time = longitudinal_conf_.preview_window() * ts;

  SimpleLongitudinalDebug debug;
  ComputeLongitudinalErrors(&trajectory_analyzer, preview_time, &debug);

  double station_reference_expected = 0.16716666937000002;
  double speed_reference_expected = 1.70833337307;
  double station_error_expected = -0.0128094011029;
  double speed_error_expected = 1.70833337307;

  double preview_station_error_expected =
      0.91472222328000008 - station_reference_expected;
  double preview_speed_reference_expected = 2.00277781487;
  double preview_speed_error_expected = 2.00277781487;
  double preview_acceleration_reference_expected = 0.405916936513;

  EXPECT_NEAR(debug.station_error(), station_error_expected, 0.001);
  EXPECT_NEAR(debug.speed_error(), speed_error_expected, 0.001);
  EXPECT_NEAR(debug.preview_station_error(), preview_station_error_expected,
              0.02);
  EXPECT_NEAR(debug.preview_speed_reference(), preview_speed_reference_expected,
              0.001);
  EXPECT_NEAR(debug.preview_speed_error(), preview_speed_error_expected, 0.001);
  EXPECT_NEAR(debug.preview_acceleration_reference(),
              preview_acceleration_reference_expected, 0.001);
  EXPECT_NEAR(debug.station_reference(), station_reference_expected, 0.001);
  EXPECT_NEAR(debug.speed_reference(), speed_reference_expected, 0.001);
}

TEST_F(LonControllerTest, Init) {
  common::Status status = Init(nullptr);
  EXPECT_EQ(status.code() == common::ErrorCode::CONTROL_INIT_ERROR, true);
}

}  // namespace control
}  // namespace apollo
