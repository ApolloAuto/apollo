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
#include <memory>

#include "gtest/gtest.h"
#include "gflags/gflags.h"

#include "cybertron/cybertron.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/common/util/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/planning_component.h"

namespace apollo {
namespace planning {

using apollo::cybertron::ComponentConfig;
using apollo::cybertron::Reader;
using apollo::cybertron::Writer;

using apollo::canbus::Chassis;
using apollo::common::time::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::perception::TrafficLightDetection;
using apollo::planning::PlanningComponent;
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

class PlanningComponentTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    FLAGS_use_multi_thread_to_add_obstacles = false;
    FLAGS_enable_multi_thread_in_dp_poly_path = false;
    FLAGS_enable_multi_thread_in_dp_st_graph = false;
    FLAGS_planning_config_file =
        "/apollo/modules/planning/conf/planning_config.pb.txt";
    FLAGS_align_prediction_time = false;
    FLAGS_estimate_current_vehicle_state = false;
    FLAGS_enable_reference_line_provider_thread = false;
    FLAGS_planning_test_mode = true;
    // FLAGS_enable_lag_prediction = false;

    SetupCybertron();
  }

 protected:
  bool FeedTestData(LocalView* local_view);
  void SetupCybertron();
  bool RunPlanning(const std::string& test_case_name);
  void TrimPlanning(ADCTrajectory* origin);

 protected:
  bool is_cybertron_initialized_ = false;
  std::mutex mutex_;
  cybertron::ComponentConfig component_config_;

  // cybertron readers/writers
  std::shared_ptr<Writer<Chassis>> chassis_writer_;
  std::shared_ptr<Writer<LocalizationEstimate>> localization_writer_;
  std::shared_ptr<Writer<PredictionObstacles>> prediction_writer_;
  std::shared_ptr<Writer<RoutingResponse>> routing_response_writer_;
  std::shared_ptr<Writer<TrafficLightDetection>>
      traffic_light_detection_writer_;
  std::shared_ptr<Reader<ADCTrajectory>> planning_reader_;

  std::shared_ptr<PlanningComponent> planning_component_ = nullptr;
  ADCTrajectory adc_trajectory_;
};

void PlanningComponentTest::SetupCybertron() {
  if (is_cybertron_initialized_) {
    return;
  }

  // init cybertron framework
  apollo::cybertron::Init("planning_test");

  Clock::SetMode(Clock::CYBERTRON);

  component_config_.set_name("planning_test");
  // note: the sequence to add_readers() must be the same
  //       as what's in PlanningComponent::Proc()
  component_config_.add_readers()->set_channel(FLAGS_prediction_topic);
  component_config_.add_readers()->set_channel(FLAGS_chassis_topic);
  component_config_.add_readers()->set_channel(FLAGS_localization_topic);

  std::shared_ptr<apollo::cybertron::Node> node(
      apollo::cybertron::CreateNode("planning_test"));

  chassis_writer_ = node->CreateWriter<Chassis>(
      FLAGS_chassis_topic);
  localization_writer_ = node->CreateWriter<LocalizationEstimate>(
      FLAGS_localization_topic);
  prediction_writer_ = node->CreateWriter<PredictionObstacles>(
      FLAGS_prediction_topic);
  routing_response_writer_ = node->CreateWriter<RoutingResponse>(
      FLAGS_routing_response_topic);
  traffic_light_detection_writer_ =
      node->CreateWriter<TrafficLightDetection>(
          FLAGS_traffic_light_detection_topic);

  planning_reader_ = node->CreateReader<ADCTrajectory>(
      FLAGS_planning_trajectory_topic,
      [this](const std::shared_ptr<ADCTrajectory>& adc_trajectory) {
        ADEBUG << "Received planning data: run planning callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        adc_trajectory_.CopyFrom(*adc_trajectory);
      });

  is_cybertron_initialized_ = true;
}

bool PlanningComponentTest::FeedTestData(LocalView* local_view) {
  // chassis
  Chassis chassis;
  if (!apollo::common::util::GetProtoFromFile(
          FLAGS_test_data_dir + "/" + FLAGS_test_chassis_file, &chassis)) {
    AERROR << "failed to load file: " << FLAGS_test_chassis_file;
    return -1;
  }

  // localization
  LocalizationEstimate localization;
  if (!apollo::common::util::GetProtoFromFile(
      FLAGS_test_data_dir + "/" + FLAGS_test_localization_file,
      &localization)) {
    AERROR << "failed to load file: " << FLAGS_test_localization_file;
    return -1;
  }

  // prediction
  PredictionObstacles prediction;
  if (!apollo::common::util::GetProtoFromFile(
      FLAGS_test_data_dir + "/" + FLAGS_test_prediction_file,
      &prediction)) {
    AERROR << "failed to load file: " << FLAGS_test_prediction_file;
    return -1;
  }

  // routing_response
  RoutingResponse routing_response;
  if (!apollo::common::util::GetProtoFromFile(
      FLAGS_test_data_dir + "/" + FLAGS_test_routing_response_file,
      &routing_response)) {
    AERROR << "failed to load file: " << FLAGS_test_routing_response_file;
    return -1;
  }

  // traffic_light_detection
  TrafficLightDetection traffic_light_detection;
  if (!apollo::common::util::GetProtoFromFile(
      FLAGS_test_data_dir + "/" + FLAGS_test_traffic_light_file,
      &traffic_light_detection)) {
    AERROR << "failed to load file: " << FLAGS_test_traffic_light_file;
    // return -1;
  }


  local_view->chassis = std::make_shared<Chassis>(chassis);
  local_view->localization_estimate =
      std::make_shared<LocalizationEstimate>(localization);
  local_view->prediction_obstacles =
      std::make_shared<PredictionObstacles>(prediction);
  local_view->routing = std::make_shared<RoutingResponse>(routing_response);
  local_view->traffic_light =
      std::make_shared<TrafficLightDetection>(traffic_light_detection);

  AINFO << "Successfully feed proto files.";
  return true;
}

bool PlanningComponentTest::RunPlanning(const std::string& test_case_name) {
  LocalView local_view;
  CHECK(FeedTestData(&local_view)) << "Failed to feed test data";

  planning_component_.reset(new PlanningComponent());
  planning_component_->Initialize(component_config_);

  usleep(1000);  // sleep 1ms

  // feed topics
  routing_response_writer_->Write(local_view.routing);
  traffic_light_detection_writer_->Write(local_view.traffic_light);
  chassis_writer_->Write(local_view.chassis);
  localization_writer_->Write(local_view.localization_estimate);
  // note: main channel must be written last
  prediction_writer_->Write(local_view.prediction_obstacles);

  usleep(200000);  // sleep 200ms

  TrimPlanning(&adc_trajectory_);

  const std::string golden_result_file = apollo::common::util::StrCat(
      "result_", test_case_name, "_0.pb.txt");
  std::string full_golden_path = FLAGS_test_data_dir + "/" + golden_result_file;
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
      AERROR << "visualize diff\n/usr/bin/python "
                "modules/tools/plot_trace/plot_planning_result.py "
             << tmp_fname << " " << full_golden_path;
      return false;
    }
  }
  return true;
}

void PlanningComponentTest::TrimPlanning(ADCTrajectory* origin) {
  origin->clear_latency_stats();
  origin->clear_debug();
  origin->mutable_header()->clear_radar_timestamp();
  origin->mutable_header()->clear_lidar_timestamp();
  origin->mutable_header()->clear_timestamp_sec();
  origin->mutable_header()->clear_camera_timestamp();
  origin->mutable_header()->clear_sequence_num();
}

/*
 * test garage_test/stop_obstacle case, using the exact same test data
 */
TEST_F(PlanningComponentTest, garage_stop_obstacle) {
  FLAGS_test_data_dir = "modules/planning/testdata/garage_test";
  FLAGS_map_dir = "modules/planning/testdata/garage_map";
  FLAGS_base_map_filename = "base_map.txt";
  FLAGS_test_routing_response_file = "garage_routing.pb.txt";
  FLAGS_test_prediction_file = "stop_obstacle_prediction.pb.txt";
  FLAGS_test_localization_file = "stop_obstacle_localization.pb.txt";
  FLAGS_test_chassis_file = "stop_obstacle_chassis.pb.txt";

  bool run_planning_success = RunPlanning("planning_componnet_stop_obstacle");
  EXPECT_TRUE(run_planning_success);
}

}  // namespace planning
}  // namespace apollo
