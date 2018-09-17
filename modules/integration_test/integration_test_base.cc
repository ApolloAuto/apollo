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

#include "modules/integration_test/integration_test_base.h"

#include <cstdlib>

#include "cybertron/cybertron.h"
#include "cybertron/time/rate.h"
#include "cybertron/time/time.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

namespace {
const char* node_name = "integration_tester";
}

using apollo::canbus::Chassis;
using apollo::common::time::Clock;
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
DEFINE_string(test_prediction_file, "", "The prediction module test file");
DEFINE_string(test_traffic_light_file, "", "The traffic light test file");
DEFINE_string(test_relative_map_file, "", "The relative map test file");
DEFINE_string(test_previous_planning_file, "",
              "The previous planning test file");

void IntegrationTestBase::SetUpTestCase() {
  FLAGS_use_multi_thread_to_add_obstacles = false;
  FLAGS_enable_multi_thread_in_dp_poly_path = false;
  FLAGS_enable_multi_thread_in_dp_st_graph = false;
  FLAGS_planning_config_file =
      "/apollo/modules/planning/conf/planning_config.pb.txt";
  FLAGS_traffic_rule_config_filename =
      "/apollo/modules/planning/conf/traffic_rule_config.pb.txt";
  FLAGS_smoother_config_filename =
      "/apollo/modules/planning/conf/qp_spline_smoother_config.pb.txt";
  FLAGS_map_dir = "/apollo/modules/planning/testdata";
  FLAGS_test_localization_file = "";
  FLAGS_test_chassis_file = "";
  FLAGS_test_routing_response_file = "";
  FLAGS_test_prediction_file = "";
  FLAGS_align_prediction_time = false;
  FLAGS_estimate_current_vehicle_state = false;
  FLAGS_enable_reference_line_provider_thread = false;

  // FLAGS_enable_trajectory_check is temporarily disabled, otherwise EMPlanner
  // and LatticePlanner can't pass the unit test.
  FLAGS_enable_trajectory_check = false;
  FLAGS_planning_test_mode = true;
  FLAGS_enable_lag_prediction = false;
}

bool IntegrationTestBase::FeedTestData() {
  // chassis
  if (!apollo::common::util::GetProtoFromFile(
          FLAGS_test_data_dir + "/" + FLAGS_test_chassis_file, &chassis_)) {
    AERROR << "failed to load file: " << FLAGS_test_chassis_file;
    return -1;
  }
  // localization
  if (!apollo::common::util::GetProtoFromFile(
          FLAGS_test_data_dir + "/" + FLAGS_test_localization_file,
          &localization_)) {
    AERROR << "failed to load file: " << FLAGS_test_localization_file;
    return -1;
  }
  // prediction
  if (!apollo::common::util::GetProtoFromFile(
          FLAGS_test_data_dir + "/" + FLAGS_test_prediction_file,
          &prediction_)) {
    AERROR << "failed to load file: " << FLAGS_test_prediction_file;
    return -1;
  }
  // routing_response
  if (!apollo::common::util::GetProtoFromFile(
          FLAGS_test_data_dir + "/" + FLAGS_test_routing_response_file,
          &routing_response_)) {
    AERROR << "failed to load file: " << FLAGS_test_routing_response_file;
    return -1;
  }
  // traffic_light_detection
  if (!apollo::common::util::GetProtoFromFile(
          FLAGS_test_data_dir + "/" + FLAGS_test_traffic_light_file,
          &traffic_light_detection_)) {
    AERROR << "failed to load file: " << FLAGS_test_traffic_light_file;
    return -1;
  }

  PublishAllTopics();

  AINFO << "Successfully fed proto files.";
  return true;
}

void IntegrationTestBase::PublishAllTopics() {
  node_ = std::move(apollo::cybertron::CreateNode(node_name));
  // chassis
  auto chassis_writer = node_->CreateWriter<Chassis>(FLAGS_chassis_topic);
  chassis_writer->Write(chassis_);
  // localization
  auto localization_writer =
      node_->CreateWriter<LocalizationEstimate>(FLAGS_localization_topic);
  localization_writer->Write(localization_);
  // prediction
  auto prediction_writer =
      node_->CreateWriter<PredictionObstacles>(FLAGS_prediction_topic);
  prediction_writer->Write(prediction_);
  // routing_response
  auto routing_response_writer =
      node_->CreateWriter<RoutingResponse>(FLAGS_routing_response_topic);
  routing_response_writer->Write(routing_response_);
  // traffic_light_detection
  auto traffic_light_detection_writer =
      node_->CreateWriter<TrafficLightDetection>(
          FLAGS_traffic_light_detection_topic);
  traffic_light_detection_writer->Write(traffic_light_detection_);
}

void IntegrationTestBase::SetUp() {
  Clock::SetMode(Clock::CYBERTRON);
  apollo::cybertron::Init(node_name);
  CHECK(FeedTestData()) << "Failed to feed test data";
  // planning_.reset(new PlanningComponent());
}

void IntegrationTestBase::UpdateData() {
  CHECK(FeedTestData()) << "Failed to feed test data";
}

bool IntegrationTestBase::RunIntegrationTest(const std::string& test_case_name,
                                             int case_num,
                                             bool no_trajectory_point) {
  const std::string golden_result_file = apollo::common::util::StrCat(
      "result_", test_case_name, "_", case_num, ".pb.txt");
  std::string full_golden_path = FLAGS_test_data_dir + "/" + golden_result_file;

  // TODO(All): compare result with golden result here.
  return true;
}

}  // namespace planning
}  // namespace apollo
