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
#include "modules/common/vehicle_state/vehicle_state.h"
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

void PlanningTestBase::SetUpTestCase() {
  FLAGS_planning_config_file = "modules/planning/conf/planning_config.pb.txt";
  FLAGS_adapter_config_path = "modules/planning/testdata/conf/adapter.conf";
  FLAGS_map_dir = "modules/planning/testdata";
  FLAGS_test_localization_file = "garage_localization.pb.txt";
  FLAGS_test_chassis_file = "garage_chassis.pb.txt";
  FLAGS_test_prediction_file = "garage_prediction.pb.txt";
  FLAGS_align_prediction_time = false;
}

bool PlanningTestBase::SetUpAdapters() {
  if (!AdapterManager::Initialized()) {
    AdapterManager::Init(FLAGS_adapter_config_path);
  }
  if (!AdapterManager::GetRoutingResponse()) {
    AERROR << "routing is not registered in adapter manager, check adapter "
              "config file."
           << FLAGS_adapter_config_path;
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
  auto prediction_file = FLAGS_test_data_dir + "/" + FLAGS_test_prediction_file;
  if (!FLAGS_test_prediction_file.empty() &&
      !AdapterManager::FeedPredictionFile(prediction_file)) {
    AERROR << "Failed to load prediction file: " << prediction_file;
    return false;
  }
  AINFO << "Using Prediction file: " << prediction_file;
  return true;
}

void PlanningTestBase::SetUp() {
  planning_.Stop();
  CHECK(SetUpAdapters()) << "Failed to setup adapters";
  planning_.Init();
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

  adc_trajectory_ = *trajectory_pointer;
  TrimPlanning(&adc_trajectory_);
  if (FLAGS_test_update_golden_log) {
    AINFO << "The golden file is regenerated:" << full_golden_path;
    ::apollo::common::util::SetProtoToASCIIFile(adc_trajectory_,
                                                full_golden_path);
  } else {
    ADCTrajectory golden_result;
    bool load_success = ::apollo::common::util::GetProtoFromASCIIFile(
        full_golden_path, &golden_result);
    TrimPlanning(&golden_result);
    if (!load_success ||
        !::apollo::common::util::IsProtoEqual(golden_result, adc_trajectory_)) {
      char tmp_fname[100] = "/tmp/XXXXXX";
      int fd = mkstemp(tmp_fname);
      if (!::apollo::common::util::SetProtoToASCIIFile(adc_trajectory_, fd)) {
        AERROR << "Failed to write to file " << tmp_fname;
      }
      AERROR << "found\ndiff " << tmp_fname << " " << full_golden_path;
      return false;
    }
  }
  return true;
}

void PlanningTestBase::export_sl_points(
    const std::vector<std::vector<common::SLPoint>>& points,
    const std::string& filename) {
  AINFO << "Write sl_points to file " << filename;
  std::ofstream ofs(filename);
  ofs << "level, s, l" << std::endl;
  int level = 0;
  for (const auto& level_points : points) {
    for (const auto& point : level_points) {
      ofs << level << ", " << point.s() << ", " << point.l() << std::endl;
    }
    ++level;
  }
  ofs.close();
}

void PlanningTestBase::export_path_data(const PathData& path_data,
                                        const std::string& filename) {
  AINFO << "Write path_data to file " << filename;
  std::ofstream ofs(filename);
  ofs << "s, l, dl, ddl, x, y, z" << std::endl;
  const auto& frenet_path = path_data.frenet_frame_path();
  const auto& discrete_path = path_data.discretized_path();
  if (frenet_path.NumOfPoints() != discrete_path.NumOfPoints()) {
    AERROR << "frenet_path and discrete path have different number of points";
    return;
  }
  for (uint32_t i = 0; i < frenet_path.NumOfPoints(); ++i) {
    const auto& frenet_point = frenet_path.PointAt(i);
    const auto& discrete_point = discrete_path.PathPointAt(i);
    ofs << frenet_point.s() << ", " << frenet_point.l() << ", "
        << frenet_point.dl() << ", " << frenet_point.ddl() << discrete_point.x()
        << ", " << discrete_point.y() << ", " << discrete_point.z()
        << std::endl;
  }
  ofs.close();
}

}  // namespace planning
}  // namespace apollo
