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

#include "modules/planning/integration_tests/planning_test_base.h"

#include <cstdlib>

#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "modules/common/log.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using common::adapter::AdapterManager;

DEFINE_string(test_data_dir, "", "the test data folder");
DEFINE_bool(test_update_golden_log, false,
            "true to update decision golden log file.");
DEFINE_string(test_routing_response_file, "garage_routing.pb.txt",
              "The routing file used in test");
DEFINE_string(test_localization_file, "garage_localization.pb.txt",
              "The localization test file");
DEFINE_string(test_chassis_file, "garage_chassis.pb.txt",
              "The chassis test file");
DEFINE_string(test_prediction_file, "", "The prediction module test file");
DEFINE_string(test_traffic_light_file, "", "The traffic light test file");

DEFINE_string(test_previous_planning_file, "",
              "The previous planning test file");

void PlanningTestBase::SetUpTestCase() {
  FLAGS_planning_config_file = "modules/planning/conf/planning_config.pb.txt";
  FLAGS_planning_adapter_config_filename =
      "modules/planning/testdata/conf/adapter.conf";
  FLAGS_map_dir = "modules/planning/testdata";
  FLAGS_test_localization_file = "garage_localization.pb.txt";
  FLAGS_test_chassis_file = "garage_chassis.pb.txt";
  FLAGS_test_prediction_file = "garage_prediction.pb.txt";
  FLAGS_align_prediction_time = false;
  FLAGS_enable_reference_line_provider_thread = false;
  FLAGS_enable_trajectory_check = true;
}

bool PlanningTestBase::SetUpAdapters() {
  if (!AdapterManager::Initialized()) {
    AdapterManager::Init(FLAGS_planning_adapter_config_filename);
  }
  if (!AdapterManager::GetRoutingResponse()) {
    AERROR << "routing is not registered in adapter manager, check adapter "
              "config file."
           << FLAGS_planning_adapter_config_filename;
    return false;
  }
  auto routing_response_file =
      FLAGS_test_data_dir + "/" + FLAGS_test_routing_response_file;
  if (!AdapterManager::FeedRoutingResponseFile(routing_response_file)) {
    AERROR << "failed to routing file: " << routing_response_file;
    return false;
  }
  AINFO << "Using Routing " << routing_response_file;
  auto localization_file =
      FLAGS_test_data_dir + "/" + FLAGS_test_localization_file;
  if (!AdapterManager::FeedLocalizationFile(localization_file)) {
    AERROR << "Failed to load localization file: " << localization_file;
    return false;
  }
  AINFO << "Using Localization file: " << localization_file;
  auto chassis_file = FLAGS_test_data_dir + "/" + FLAGS_test_chassis_file;
  if (!AdapterManager::FeedChassisFile(chassis_file)) {
    AERROR << "Failed to load chassis file: " << chassis_file;
    return false;
  }
  AINFO << "Using Chassis file: " << chassis_file;
  if (FLAGS_enable_prediction && !FLAGS_test_prediction_file.empty()) {
    auto prediction_file =
        FLAGS_test_data_dir + "/" + FLAGS_test_prediction_file;
    if (!AdapterManager::FeedPredictionFile(prediction_file)) {
      AERROR << "Failed to load prediction file: " << prediction_file;
      return false;
    }
    AINFO << "Using Prediction file: " << prediction_file;
  } else {
    AINFO << "Prediction is disabled";
  }
  if (FLAGS_enable_traffic_light && !FLAGS_test_traffic_light_file.empty()) {
    auto traffic_light_file =
        FLAGS_test_data_dir + "/" + FLAGS_test_traffic_light_file;
    if (!AdapterManager::FeedTrafficLightDetectionFile(traffic_light_file)) {
      AERROR << "Failed to load traffic light file: " << traffic_light_file;
      return false;
    }
    AINFO << "Using Traffic Light file: " << traffic_light_file;
  } else {
    AINFO << "Traffic Light is disabled";
  }
  return true;
}

void PlanningTestBase::SetUp() {
  planning_.Stop();
  CHECK(SetUpAdapters()) << "Failed to setup adapters";
  CHECK(planning_.Init().ok()) << "Failed to init planning module";
  if (!FLAGS_test_previous_planning_file.empty()) {
    const auto prev_planning_file =
        FLAGS_test_data_dir + "/" + FLAGS_test_previous_planning_file;
    ADCTrajectory prev_planning;
    CHECK(common::util::GetProtoFromFile(prev_planning_file, &prev_planning));
    planning_.SetLastPublishableTrajectory(prev_planning);
  }
}

void PlanningTestBase::TrimPlanning(ADCTrajectory* origin) {
  origin->clear_latency_stats();
  origin->clear_debug();
  origin->mutable_header()->clear_radar_timestamp();
  origin->mutable_header()->clear_lidar_timestamp();
  origin->mutable_header()->clear_timestamp_sec();
  origin->mutable_header()->clear_camera_timestamp();
  origin->mutable_header()->clear_sequence_num();
}

bool PlanningTestBase::RunPlanning(const std::string& test_case_name,
                                   int case_num) {
  const std::string golden_result_file = apollo::common::util::StrCat(
      "result_", test_case_name, "_", case_num, ".pb.txt");

  std::string full_golden_path = FLAGS_test_data_dir + "/" + golden_result_file;
  planning_.RunOnce();

  const ADCTrajectory* trajectory_pointer =
      AdapterManager::GetPlanning()->GetLatestPublished();

  if (!trajectory_pointer) {
    AERROR << " did not get latest adc trajectory";
    return false;
  }

  if (!IsValidTrajectory(*trajectory_pointer)) {
    AERROR << "Fail to pass trajectory check.";
    return false;
  }

  adc_trajectory_ = *trajectory_pointer;
  TrimPlanning(&adc_trajectory_);
  if (FLAGS_test_update_golden_log) {
    AINFO << "The golden file is regenerated:" << full_golden_path;
    common::util::SetProtoToASCIIFile(adc_trajectory_, full_golden_path);
  } else {
    ADCTrajectory golden_result;
    bool load_success =
        common::util::GetProtoFromASCIIFile(full_golden_path, &golden_result);
    TrimPlanning(&golden_result);
    if (!load_success ||
        !common::util::IsProtoEqual(golden_result, adc_trajectory_)) {
      char tmp_fname[100] = "/tmp/XXXXXX";
      int fd = mkstemp(tmp_fname);
      if (!common::util::SetProtoToASCIIFile(adc_trajectory_, fd)) {
        AERROR << "Failed to write to file " << tmp_fname;
      }
      AERROR << "found\ndiff " << tmp_fname << " " << full_golden_path;
      AERROR << "visualize diff\npython "
                "modules/tools/plot_trace/plot_planning_result.py "
             << tmp_fname << " " << full_golden_path;
      return false;
    }
  }
  return true;
}

bool PlanningTestBase::IsValidTrajectory(const ADCTrajectory& trajectory) {
  if (trajectory.trajectory_point().empty()) {
    AERROR << "trajectory has NO point.";
    return false;
  }

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
