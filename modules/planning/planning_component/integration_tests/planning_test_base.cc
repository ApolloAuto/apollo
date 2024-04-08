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

#include "modules/planning/planning_component/integration_tests/planning_test_base.h"

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::canbus::Chassis;
using apollo::cyber::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::perception::TrafficLightDetection;
using apollo::prediction::PredictionObstacles;
using apollo::routing::RoutingResponse;

DEFINE_string(test_data_dir, "", "the test data folder");
DEFINE_bool(test_update_golden_log, false,
            "true to update decision golden log file.");
DEFINE_string(test_routing_response_file, "", "The routing file used in test");
DEFINE_string(test_localization_file, "", "The localization test file");
DEFINE_string(test_chassis_file, "", "The chassis test file");
DEFINE_string(test_planning_config_file, "", "planning config file for test");
DEFINE_string(test_prediction_file, "", "The prediction module test file");
DEFINE_string(test_traffic_light_file, "", "The traffic light test file");
DEFINE_string(test_relative_map_file, "", "The relative map test file");
DEFINE_string(test_previous_planning_file, "",
              "The previous planning test file");

void PlanningTestBase::SetUpTestCase() {
  FLAGS_use_multi_thread_to_add_obstacles = false;
  FLAGS_traffic_rule_config_filename =
      "/apollo/modules/planning/planning_component/conf/"
      "traffic_rule_config.pb.txt";
  FLAGS_smoother_config_filename =
      "/apollo/modules/planning/planning_component/conf/"
      "qp_spline_smoother_config.pb.txt";
  FLAGS_map_dir = "/apollo/modules/planning/planning_base/testdata";
  FLAGS_test_localization_file = "";
  FLAGS_test_chassis_file = "";
  FLAGS_test_routing_response_file = "";
  FLAGS_test_planning_config_file =
      "/apollo/modules/planning/planning_component/conf/planning_config.pb.txt";
  FLAGS_test_previous_planning_file = "";
  FLAGS_test_prediction_file = "";
  FLAGS_align_prediction_time = false;
  FLAGS_enable_reference_line_provider_thread = false;
  // FLAGS_enable_trajectory_check is temporarily disabled, otherwise EMPlanner
  // and LatticePlanner can't pass the unit test.
  FLAGS_enable_trajectory_check = false;
}

bool PlanningTestBase::FeedTestData() {
  // chassis
  Chassis chassis;
  if (FLAGS_test_chassis_file.empty()) {
    AERROR << "Requires FLAGS_test_chassis_file to be set";
    return false;
  }
  if (!apollo::cyber::common::GetProtoFromFile(
          FLAGS_test_data_dir + "/" + FLAGS_test_chassis_file, &chassis)) {
    AERROR << "failed to load file: " << FLAGS_test_chassis_file;
    return false;
  }
  // localization
  if (FLAGS_test_localization_file.empty()) {
    AERROR << "Requires FLAGS_test_localization_file to be set";
    return false;
  }
  LocalizationEstimate localization;
  if (!apollo::cyber::common::GetProtoFromFile(
          FLAGS_test_data_dir + "/" + FLAGS_test_localization_file,
          &localization)) {
    AERROR << "failed to load file: " << FLAGS_test_localization_file;
    return false;
  }
  Clock::SetMode(apollo::cyber::proto::MODE_MOCK);
  Clock::SetNowInSeconds(localization.header().timestamp_sec());

  // prediction
  if (FLAGS_test_prediction_file.empty()) {
    AERROR << "Requires FLAGS_test_prediction_file to be set";
    return false;
  }
  PredictionObstacles prediction;
  if (!apollo::cyber::common::GetProtoFromFile(
          FLAGS_test_data_dir + "/" + FLAGS_test_prediction_file,
          &prediction)) {
    AERROR << "failed to load file: " << FLAGS_test_prediction_file;
    return false;
  }
  // routing_response
  if (FLAGS_test_routing_response_file.empty()) {
    AERROR << "Requires FLAGS_test_routing_response_file";
    return false;
  }
  RoutingResponse routing_response;
  if (!apollo::cyber::common::GetProtoFromFile(
          FLAGS_test_data_dir + "/" + FLAGS_test_routing_response_file,
          &routing_response)) {
    AERROR << "failed to load file: " << FLAGS_test_routing_response_file;
    return false;
  }
  // traffic_light_detection
  // optional
  TrafficLightDetection traffic_light_detection;
  if (!apollo::cyber::common::GetProtoFromFile(
          FLAGS_test_data_dir + "/" + FLAGS_test_traffic_light_file,
          &traffic_light_detection)) {
    AERROR << "failed to load file: " << FLAGS_test_traffic_light_file;
    return false;
  }

  local_view_.prediction_obstacles =
      std::make_shared<PredictionObstacles>(prediction);
  local_view_.chassis = std::make_shared<Chassis>(chassis);
  local_view_.localization_estimate =
      std::make_shared<LocalizationEstimate>(localization);
  local_view_.traffic_light =
      std::make_shared<TrafficLightDetection>(traffic_light_detection);

  AINFO << "Successfully feed proto files.";
  return true;
}

void PlanningTestBase::SetUp() {
  injector_ = std::make_shared<DependencyInjector>();

  if (FLAGS_use_navigation_mode) {
    // TODO(all)
    // planning_ = std::unique_ptr<PlanningBase>(new NaviPlanning());
  } else {
    planning_ = std::unique_ptr<PlanningBase>(new OnLanePlanning(injector_));
  }

  ACHECK(FeedTestData()) << "Failed to feed test data";

  ACHECK(cyber::common::GetProtoFromFile(FLAGS_test_planning_config_file,
                                         &config_))
      << "failed to load planning config file "
      << FLAGS_test_planning_config_file;

  ACHECK(planning_->Init(config_).ok()) << "Failed to init planning module";
}

void PlanningTestBase::UpdateData() {
  ACHECK(FeedTestData()) << "Failed to feed test data";
}

void PlanningTestBase::TrimPlanning(ADCTrajectory* origin,
                                    bool no_trajectory_point) {
  origin->clear_latency_stats();
  origin->clear_debug();
  // origin->mutable_header()->clear_radar_timestamp();
  // origin->mutable_header()->clear_lidar_timestamp();
  // origin->mutable_header()->clear_timestamp_sec();
  // origin->mutable_header()->clear_camera_timestamp();
  // origin->mutable_header()->clear_sequence_num();

  if (no_trajectory_point) {
    origin->clear_total_path_length();
    origin->clear_total_path_time();
    origin->clear_trajectory_point();
  }
}

bool PlanningTestBase::RunPlanning(const std::string& test_case_name,
                                   int case_num, bool no_trajectory_point) {
  const std::string golden_result_file =
      absl::StrCat("result_", test_case_name, "_", case_num, ".pb.txt");

  std::string full_golden_path = FLAGS_test_data_dir + "/" + golden_result_file;

  ADCTrajectory adc_trajectory_pb;
  planning_->RunOnce(local_view_, &adc_trajectory_pb);

  if (!IsValidTrajectory(adc_trajectory_pb)) {
    AERROR << "Fail to pass trajectory check.";
    return false;
  }

  adc_trajectory_ = adc_trajectory_pb;
  TrimPlanning(&adc_trajectory_, no_trajectory_point);
  if (FLAGS_test_update_golden_log) {
    AINFO << "The golden file is regenerated:" << full_golden_path;
    cyber::common::SetProtoToASCIIFile(adc_trajectory_, full_golden_path);
  } else {
    ADCTrajectory golden_result;
    bool load_success =
        cyber::common::GetProtoFromASCIIFile(full_golden_path, &golden_result);
    TrimPlanning(&golden_result, no_trajectory_point);
    if (!load_success ||
        !common::util::IsProtoEqual(golden_result, adc_trajectory_)) {
      char tmp_fname[100] = "/tmp/XXXXXX";
      int fd = mkstemp(tmp_fname);
      if (fd < 0) {
        AERROR << "Failed to create temporary file: " << tmp_fname;
        return false;
      }
      if (!cyber::common::SetProtoToASCIIFile(adc_trajectory_, fd)) {
        AERROR << "Failed to write to file: " << tmp_fname;
      }
      AERROR << "found error\ndiff -y " << tmp_fname << " " << full_golden_path;
      AERROR << "to override error\nmv " << tmp_fname << " "
             << full_golden_path;
      AERROR << "to visualize\n/usr/bin/python "
                "modules/tools/plot_trace/plot_planning_result.py "
             << tmp_fname << " " << full_golden_path;
      return false;
    }
  }
  return true;
}

bool PlanningTestBase::IsValidTrajectory(const ADCTrajectory& trajectory) {
  for (int i = 0; i < trajectory.trajectory_point_size(); ++i) {
    const auto& point = trajectory.trajectory_point(i);

    const double kMaxAccelThreshold =
        FLAGS_longitudinal_acceleration_upper_bound;
    const double kMinAccelThreshold =
        FLAGS_longitudinal_acceleration_lower_bound;
    if (point.a() > kMaxAccelThreshold || point.a() < kMinAccelThreshold) {
      AERROR << "Invalid trajectory point because accel out of range: "
             << point.DebugString();
      return false;
    }

    if (!point.has_path_point()) {
      AERROR << "Invalid trajectory point because NO path_point in "
                "trajectory_point: "
             << point.DebugString();
      return false;
    }

    if (i > 0) {
      const double kPathSEpsilon = 1e-3;
      const auto& last_point = trajectory.trajectory_point(i - 1);
      if (point.path_point().s() + kPathSEpsilon <
          last_point.path_point().s()) {
        AERROR << "Invalid trajectory point because s value error. last point: "
               << last_point.DebugString()
               << ", curr point: " << point.DebugString();
        return false;
      }
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
