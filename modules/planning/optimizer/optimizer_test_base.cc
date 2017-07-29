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

#include "modules/planning/optimizer/optimizer_test_base.h"

#include "modules/common/log.h"
#include "modules/common/vehicle_state/vehicle_state.h"

namespace apollo {
namespace planning {

using common::adapter::AdapterManager;

DEFINE_string(test_routing_result_file,
              "modules/planning/testdata/garage_routing.pb.txt",
              "The routing file used in test");
DEFINE_string(test_localization_file,
              "modules/planning/testdata/garage_localization.pb.txt",
              "The localization test file");
DEFINE_string(test_chassis_file,
              "modules/planning/testdata/garage_chassis.pb.txt",
              "The chassis test file");

void OptimizerTestBase::SetDataConfigs() {
  FLAGS_planning_config_file =
      "modules/planning/testdata/conf/planning_config.pb.txt";
  FLAGS_adapter_config_path = "modules/planning/testdata/conf/adapter.conf";
  FLAGS_map_filename = "modules/planning/testdata/base_map.txt";
  FLAGS_reference_line_smoother_config_file =
      "modules/planning/testdata/conf/reference_line_smoother_config.pb.txt";
  FLAGS_dp_poly_path_config_file =
      "modules/planning/testdata/conf/dp_poly_path_config.pb.txt";
  FLAGS_test_localization_file =
      "modules/planning/testdata/garage_localization.pb.txt";
  FLAGS_test_chassis_file = "modules/planning/testdata/garage_chassis.pb.txt",
  FLAGS_v = 4;
  FLAGS_alsologtostderr = true;
}

bool OptimizerTestBase::SetUpAdapters() {
  AdapterManager::Init(FLAGS_adapter_config_path);
  if (!AdapterManager::GetRoutingResult()) {
    AERROR << "routing is not registered in adapter manager, check adapter "
              "config file "
           << FLAGS_adapter_config_path;
    return false;
  }
  if (!AdapterManager::FeedRoutingResultFile(FLAGS_test_routing_result_file)) {
    AERROR << "failed to routing file: " << FLAGS_test_routing_result_file;
    return false;
  }
  if (!AdapterManager::FeedLocalizationFile(FLAGS_test_localization_file)) {
    AERROR << "Failed to load localization file: "
           << FLAGS_test_localization_file;
    return false;
  }
  if (!AdapterManager::FeedChassisFile(FLAGS_test_chassis_file)) {
    AERROR << "Failed to load chassis file: " << FLAGS_test_chassis_file;
    return false;
  }
  AdapterManager::Observe();
  return true;
}

void OptimizerTestBase::SetUp() {
  SetDataConfigs();
  if (!SetUpAdapters()) {
    AERROR << "Failed to setup adapters";
    return;
  }
  common::VehicleState::instance()->Update(
      AdapterManager::GetLocalization()->GetLatestObserved(),
      AdapterManager::GetChassis()->GetLatestObserved());

  auto* data_center = DataCenter::instance();
  if (!data_center->init_current_frame(0)) {
    AERROR << "Failed to init frame";
    return;
  }
  if (!common::util::GetProtoFromFile(FLAGS_dp_poly_path_config_file,
                                      &dp_poly_path_config_)) {
    AERROR << "Failed to load file " << FLAGS_dp_poly_path_config_file;
    return;
  }
  pose_ = common::VehicleState::instance()->pose();
  frame_ = data_center->current_frame();
  ASSERT_TRUE(frame_ != nullptr);
}

void OptimizerTestBase::export_sl_points(
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

void OptimizerTestBase::export_path_data(const PathData& path_data,
                                         const std::string& filename) {
  AINFO << "Write path_data to file " << filename;
  std::ofstream ofs(filename);
  ofs << "s, l, dl, ddl, x, y, z" << std::endl;
  const auto& frenet_path = path_data.frenet_frame_path();
  const auto& discrete_path = path_data.discretized_path();
  if (frenet_path.number_of_points() != discrete_path.num_of_points()) {
    AERROR << "frenet_path and discrete path have different number of points";
    return;
  }
  for (uint32_t i = 0; i < frenet_path.number_of_points(); ++i) {
    const auto& frenet_point = frenet_path.point_at(i);
    const auto& discrete_point = discrete_path.path_point_at(i);
    ofs << frenet_point.s() << ", " << frenet_point.l() << ", "
        << frenet_point.dl() << ", " << frenet_point.ddl() << discrete_point.x()
        << ", " << discrete_point.y() << ", " << discrete_point.z()
        << std::endl;
  }
  ofs.close();
}

}  // namespace planning
}  // namespace apollo
