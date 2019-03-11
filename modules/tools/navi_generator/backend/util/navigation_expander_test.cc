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
 * @brief This file provides several unit tests for the class
 * "NavigationExpander".
 */
#include "modules/tools/navi_generator/backend/util/navigation_expander.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "modules/common/log.h"
#include "modules/common/util/util.h"
#include "modules/planning/reference_line/reference_point.h"

using apollo::common::util::DistanceXY;
using apollo::planning::ReferencePoint;
namespace apollo {
namespace navi_generator {
namespace util {

class NavigationExpanderTest : public testing::Test {
 public:
  virtual void SetUp() {
    expand_lane_ = std::make_unique<NavigationExpander>();
    raw_filename_ =
        "modules/tools/navi_generator/backend/util/testdata/"
        "trajectory_expand_lane/smoothed.txt";
  }

 protected:
  std::unique_ptr<NavigationExpander> expand_lane_;
  std::list<LanePointsInfo> lane_points_list_;
  LanePoints lane_points_;
  std::string raw_filename_;

 protected:
  bool Import(const std::string& filename);
  bool Export(const std::string& filename);
};

bool NavigationExpanderTest::Import(const std::string& filename) {
  std::ifstream ifs(filename.c_str(), std::ifstream::in);
  if (!ifs.is_open()) {
    AERROR << "Can't open the reference point file: " << filename;
    return false;
  }
  std::string point_str;
  lane_points_.clear();
  while (std::getline(ifs, point_str)) {
    std::string::size_type n1[6];
    std::string::size_type n2[6];
    n1[0] = point_str.find(':');
    n2[0] = point_str.find(',');
    if ((n1[0] == std::string::npos) || (n2[0] == std::string::npos)) {
      continue;
    }
    auto kappa_str = point_str.substr(n1[0] + 1, (n2[0] - n1[0] - 1));
    n1[1] = point_str.find(':', n2[0] + 1);
    n2[1] = point_str.find(',', n2[0] + 1);
    auto s_str = point_str.substr(n1[1] + 1, (n2[1] - n1[1] - 1));
    n1[2] = point_str.find(':', n2[1] + 1);
    n2[2] = point_str.find(',', n2[1] + 1);
    auto theta_str = point_str.substr(n1[2] + 1, (n2[2] - n1[2] - 1));
    n1[3] = point_str.find(':', n2[2] + 1);
    n2[3] = point_str.find(',', n2[2] + 1);
    auto x_str = point_str.substr(n1[3] + 1, (n2[3] - n1[3] - 1));
    n1[4] = point_str.find(':', n2[3] + 1);
    n2[4] = point_str.find(',', n2[3] + 1);
    auto y_str = point_str.substr(n1[4] + 1, (n2[4] - n1[4] - 1));
    n1[5] = point_str.find(':', n2[4] + 1);
    n2[5] = point_str.find(',', n2[4] + 1);
    auto dkappa_str = point_str.substr(n1[5] + 1);

    auto filter_point_func = [this](double x, double y, double theta,
                                    double kappa, double dkappa) {
      ReferencePoint pt;
      pt.set_x(x);
      pt.set_y(y);
      pt.set_heading(theta);
      lane_points_.push_back(pt);
    };
    filter_point_func(std::stod(x_str), std::stod(y_str), std::stod(theta_str),
                      std::stod(kappa_str), std::stod(dkappa_str));
  }
  return true;
}
bool NavigationExpanderTest::Export(const std::string& filename) {
  if (lane_points_list_.empty()) {
    AERROR << "There aren't any lane points to output.";
    return false;
  }

  std::ofstream ofs(filename.c_str());
  if (!ofs.is_open()) {
    AERROR << "Failed to open the output file: " << filename;
    return false;
  }
  ofs.precision(6);
  double s = 0.0;
  AINFO << "Lane points list size " << lane_points_list_.size();

  for (const auto& lane_points_info : lane_points_list_) {
    AINFO << "Lane index " << lane_points_info.index;
    for (std::size_t i = 1; i + 1 < lane_points_info.points.size(); ++i) {
      const auto& ref_point = lane_points_info.points[i];
      ofs << std::fixed << "{\"kappa\": " << ref_point.kappa()
          << ", \"s\": " << s << ", \"theta\": " << ref_point.heading()
          << ", \"x\":" << ref_point.x() << ", \"y\":" << ref_point.y()
          << ", \"dkappa\":" << ref_point.dkappa() << "}" << std::endl;

      s += DistanceXY(ref_point, lane_points_info.points[i + 1]);

      AINFO << std::fixed << "{\"kappa\": " << ref_point.kappa()
            << ", \"s\": " << s << ", \"theta\": " << ref_point.heading()
            << ", \"x\":" << ref_point.x() << ", \"y\":" << ref_point.y()
            << ", \"dkappa\":" << ref_point.dkappa() << "}";
    }
  }
  ofs.close();
  AINFO << "The expand lane result is saved to the file: " << filename;

  return true;
}

TEST_F(NavigationExpanderTest, ExpandL1) {
  EXPECT_TRUE(Import(raw_filename_));
  EXPECT_GT(lane_points_.size(), 0);
  lane_points_list_.clear();
  EXPECT_TRUE(
      expand_lane_->ExpandLane(lane_points_, 1, 0, 3.75, &lane_points_list_));
  EXPECT_EQ(2, lane_points_list_.size());
  EXPECT_TRUE(
      Export("modules/tools/navi_generator/backend/util/testdata/"
             "trajectory_expand_lane/expand_l1.smoothed"));
}

TEST_F(NavigationExpanderTest, ExpandR1) {
  EXPECT_TRUE(Import(raw_filename_));
  EXPECT_GT(lane_points_.size(), 0);
  lane_points_list_.clear();
  EXPECT_TRUE(
      expand_lane_->ExpandLane(lane_points_, 0, 1, 3.75, &lane_points_list_));
  EXPECT_EQ(2, lane_points_list_.size());
  EXPECT_TRUE(
      Export("modules/tools/navi_generator/backend/util/testdata/"
             "trajectory_expand_lane/expand_r1.smoothed"));
}

TEST_F(NavigationExpanderTest, ExpandL1R1) {
  EXPECT_TRUE(Import(raw_filename_));
  EXPECT_GT(lane_points_.size(), 0);
  lane_points_list_.clear();
  EXPECT_TRUE(
      expand_lane_->ExpandLane(lane_points_, 1, 1, 3.75, &lane_points_list_));
  EXPECT_EQ(3, lane_points_list_.size());
  EXPECT_TRUE(
      Export("modules/tools/navi_generator/backend/util/testdata/"
             "trajectory_expand_lane/expand_l1r1.smoothed"));
}

TEST_F(NavigationExpanderTest, ExpandL1R2) {
  EXPECT_TRUE(Import(raw_filename_));
  EXPECT_GT(lane_points_.size(), 0);
  lane_points_list_.clear();
  EXPECT_TRUE(
      expand_lane_->ExpandLane(lane_points_, 1, 2, 3.75, &lane_points_list_));
  EXPECT_EQ(4, lane_points_list_.size());
  EXPECT_TRUE(
      Export("modules/tools/navi_generator/backend/util/testdata/"
             "trajectory_expand_lane/expand_l1r2.smoothed"));
}

TEST_F(NavigationExpanderTest, ExpandL2R2) {
  EXPECT_TRUE(Import(raw_filename_));
  EXPECT_GT(lane_points_.size(), 0);
  lane_points_list_.clear();
  EXPECT_TRUE(
      expand_lane_->ExpandLane(lane_points_, 2, 2, 3.75, &lane_points_list_));
  EXPECT_EQ(5, lane_points_list_.size());
  EXPECT_TRUE(
      Export("modules/tools/navi_generator/backend/util/testdata/"
             "trajectory_expand_lane/expand_l2r2.smoothed"));
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
