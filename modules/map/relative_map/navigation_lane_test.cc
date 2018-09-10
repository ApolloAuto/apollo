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

/**
 * @file
 * @brief This file provides several unit tests for the class "NavigationLane".
 */

#include "modules/map/relative_map/navigation_lane.h"

#include <string>
#include <vector>
#include "gtest/gtest.h"
#include "third_party/json/json.hpp"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/relative_map/common/relative_map_gflags.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/map/relative_map/proto/relative_map_config.pb.h"

namespace apollo {
namespace relative_map {

using apollo::common::VehicleStateProvider;
using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::relative_map::NavigationInfo;
using apollo::relative_map::NavigationLane;
using apollo::relative_map::NavigationPath;
using nlohmann::json;

namespace {
bool GetNavigationPathFromFile(const std::string& filename,
                               NavigationPath* navigation_path) {
  CHECK_NOTNULL(navigation_path);

  std::ifstream ifs(filename, std::ios::in);
  if (!ifs.is_open()) {
    AERROR << "Failed to open " << filename;
    return false;
  }
  std::string line_str;
  while (std::getline(ifs, line_str)) {
    try {
      auto json_obj = json::parse(line_str);
      auto* point = navigation_path->mutable_path()->add_path_point();
      point->set_x(json_obj["x"]);
      point->set_y(json_obj["y"]);
      point->set_s(json_obj["s"]);
      point->set_theta(json_obj["theta"]);
      point->set_kappa(json_obj["kappa"]);
      point->set_dkappa(json_obj["dkappa"]);
    } catch (const std::exception& e) {
      AERROR << "Failed to parse JSON data: " << e.what();
      return false;
    }
  }

  return true;
}

bool GenerateNavigationInfo(
    const std::vector<std::string>& navigation_line_filenames,
    NavigationInfo* navigation_info) {
  CHECK_NOTNULL(navigation_info);

  int i = 0;
  for (const std::string& filename : navigation_line_filenames) {
    auto* navigation_path = navigation_info->add_navigation_path();
    if (!GetNavigationPathFromFile(filename, navigation_path)) {
      continue;
    }
    navigation_path->set_path_priority(i);
    navigation_path->mutable_path()->set_name("Navigation path " + i);
    ++i;
  }
  return navigation_info->navigation_path_size() > 0;
}
}  // namespace

class NavigationLaneTest : public testing::Test {
 public:
  virtual void SetUp() {
    common::adapter::AdapterManagerConfig adapter_conf;
    RelativeMapConfig config;

    EXPECT_TRUE(common::util::GetProtoFromFile(
        FLAGS_relative_map_adapter_config_filename, &adapter_conf));
    EXPECT_TRUE(common::util::GetProtoFromFile(
        FLAGS_relative_map_config_filename, &config));

    navigation_lane_.SetConfig(config.navigation_lane());
    map_param_ = config.map_param();
    navigation_lane_.SetDefaultWidth(map_param_.default_left_width(),
                                     map_param_.default_right_width());

    data_file_dir_ = "modules/map/relative_map/testdata/multi_lane_map/";

    localization::LocalizationEstimate localization;
    canbus::Chassis chassis;
    EXPECT_TRUE(common::util::GetProtoFromFile(
        data_file_dir_ + "localization_info.pb.txt", &localization));
    EXPECT_TRUE(common::util::GetProtoFromFile(
        data_file_dir_ + "chassis_info.pb.txt", &chassis));
    VehicleStateProvider::instance()->Update(localization, chassis);
  }

 protected:
  NavigationLane navigation_lane_;
  NavigationInfo navigation_info_;
  MapGenerationParam map_param_;
  std::vector<std::string> navigation_line_filenames_;
  std::string data_file_dir_;
};

TEST_F(NavigationLaneTest, GenerateOneLaneMap) {
  navigation_line_filenames_.clear();
  navigation_info_.Clear();
  navigation_line_filenames_.emplace_back(data_file_dir_ + "left.smoothed");
  EXPECT_TRUE(
      GenerateNavigationInfo(navigation_line_filenames_, &navigation_info_));
  navigation_lane_.UpdateNavigationInfo(navigation_info_);
  EXPECT_TRUE(navigation_lane_.GeneratePath());
  EXPECT_GT(navigation_lane_.Path().path().path_point_size(), 0);

  MapMsg map_msg;
  EXPECT_TRUE(navigation_lane_.CreateMap(map_param_, &map_msg));
  EXPECT_EQ(1, map_msg.hdmap().lane_size());
}

TEST_F(NavigationLaneTest, GenerateTwoLaneMap) {
  navigation_line_filenames_.clear();
  navigation_info_.Clear();
  navigation_line_filenames_.emplace_back(data_file_dir_ + "left.smoothed");
  navigation_line_filenames_.emplace_back(data_file_dir_ + "right.smoothed");
  EXPECT_TRUE(
      GenerateNavigationInfo(navigation_line_filenames_, &navigation_info_));
  navigation_lane_.UpdateNavigationInfo(navigation_info_);
  EXPECT_TRUE(navigation_lane_.GeneratePath());
  EXPECT_GT(navigation_lane_.Path().path().path_point_size(), 0);

  MapMsg map_msg;
  EXPECT_TRUE(navigation_lane_.CreateMap(map_param_, &map_msg));
  EXPECT_EQ(2, map_msg.hdmap().lane_size());
}

TEST_F(NavigationLaneTest, GenerateThreeLaneMap) {
  navigation_line_filenames_.clear();
  navigation_info_.Clear();
  navigation_line_filenames_.emplace_back(data_file_dir_ + "left.smoothed");
  navigation_line_filenames_.emplace_back(data_file_dir_ + "middle.smoothed");
  navigation_line_filenames_.emplace_back(data_file_dir_ + "right.smoothed");
  EXPECT_TRUE(
      GenerateNavigationInfo(navigation_line_filenames_, &navigation_info_));
  navigation_lane_.UpdateNavigationInfo(navigation_info_);
  EXPECT_TRUE(navigation_lane_.GeneratePath());
  EXPECT_GT(navigation_lane_.Path().path().path_point_size(), 0);

  MapMsg map_msg;
  EXPECT_TRUE(navigation_lane_.CreateMap(map_param_, &map_msg));
  EXPECT_EQ(3, map_msg.hdmap().lane_size());
}

}  // namespace relative_map
}  // namespace apollo
