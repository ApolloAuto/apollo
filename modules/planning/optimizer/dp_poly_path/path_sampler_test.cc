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

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "ros/include/ros/ros.h"

#include "modules/planning/proto/dp_poly_path_config.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/planning/common/data_center.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/optimizer/dp_poly_path/path_sampler.h"
#include "modules/planning/optimizer/dp_poly_path/path_sampler.h"

namespace apollo {
namespace planning {

using common::adapter::AdapterManager;

class PathSamplerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    FLAGS_planning_config_file =
        "modules/planning/testdata/conf/planning_config.pb.txt";
    FLAGS_adapter_config_path = "modules/planning/testdata/conf/adapter.conf";
    FLAGS_map_filename = "modules/planning/testdata/base_map.txt";
    FLAGS_reference_line_smoother_config_file =
        "modules/planning/testdata/conf/reference_line_smoother_config.pb.txt";
    FLAGS_dp_poly_path_config_file =
        "modules/planning/testdata/conf/dp_poly_path_config.pb.txt";
    AdapterManager::Init(FLAGS_adapter_config_path);
    if (!AdapterManager::GetRoutingResult()) {
      AERROR << "routing is not registered in adapter manager, check adapter "
                "config file "
             << FLAGS_adapter_config_path;
      return;
    }
    AdapterManager::FeedRoutingResultFile(routing_file_);
    AdapterManager::Observe();
    auto* data_center = DataCenter::instance();
    if (!data_center->init_current_frame(0)) {
      AERROR << "Failed to init frame";
      return;
    }
    if (!common::util::GetProtoFromFile(FLAGS_dp_poly_path_config_file,
                                        &config_)) {
      AERROR << "Failed to load file " << FLAGS_dp_poly_path_config_file;
      return;
    }
    frame_ = data_center->current_frame();
  }

  void export_sl_points(std::vector<std::vector<common::SLPoint>>& points,
                        const std::string& filename) {
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

 protected:
  const std::string routing_file_ =
      "modules/planning/testdata/garage_routing.pb.txt";
  DpPolyPathConfig config_;
  Frame* frame_ = nullptr;
};

TEST_F(PathSamplerTest, sample_one_point) {
  ASSERT_TRUE(frame_ != nullptr);
  config_.set_sample_points_num_each_level(1);
  PathSampler sampler(config_);
  std::vector<std::vector<common::SLPoint>> sampled_points;
  common::TrajectoryPoint init_point;
  init_point.mutable_path_point()->set_x(586392.84003);
  init_point.mutable_path_point()->set_y(4140673.01232);
  common::SLPoint init_sl_point;
  init_sl_point.set_s(0.0);
  init_sl_point.set_l(0.0);

  const auto& reference_line = frame_->planning_data().reference_line();
  const double reference_line_length =
      reference_line.reference_map_line().length();
  EXPECT_FLOAT_EQ(70.1723, reference_line_length);
  bool sample_result = sampler.sample(reference_line, init_point, init_sl_point,
                                      &sampled_points);
  EXPECT_TRUE(sample_result);
  EXPECT_EQ(8, sampled_points.size());
  EXPECT_EQ(1, sampled_points[0].size());
  EXPECT_EQ(1, sampled_points[7].size());
  EXPECT_FLOAT_EQ(40, sampled_points[7][0].s());
  EXPECT_FLOAT_EQ(0, sampled_points[7][0].l());
  // export_sl_points(sampled_points, "/tmp/points.txt");
}

TEST_F(PathSamplerTest, sample_three_points) {
  ASSERT_TRUE(frame_ != nullptr);
  config_.set_sample_points_num_each_level(3);
  PathSampler sampler(config_);
  std::vector<std::vector<common::SLPoint>> sampled_points;
  common::TrajectoryPoint init_point;
  init_point.mutable_path_point()->set_x(586392.84003);
  init_point.mutable_path_point()->set_y(4140673.01232);
  common::SLPoint init_sl_point;
  init_sl_point.set_s(0.0);
  init_sl_point.set_l(0.0);

  const auto& reference_line = frame_->planning_data().reference_line();
  const double reference_line_length =
      reference_line.reference_map_line().length();
  EXPECT_FLOAT_EQ(70.1723, reference_line_length);
  bool sample_result = sampler.sample(reference_line, init_point, init_sl_point,
                                      &sampled_points);
  EXPECT_TRUE(sample_result);
  EXPECT_EQ(8, sampled_points.size());
  EXPECT_EQ(3, sampled_points[0].size());
  ASSERT_EQ(3, sampled_points[7].size());
  EXPECT_FLOAT_EQ(40, sampled_points[7][0].s());
  EXPECT_FLOAT_EQ(-0.5, sampled_points[7][0].l());
  EXPECT_FLOAT_EQ(40, sampled_points[7][1].s());
  EXPECT_FLOAT_EQ(0, sampled_points[7][1].l());
  EXPECT_FLOAT_EQ(40, sampled_points[7][2].s());
  EXPECT_FLOAT_EQ(0.5, sampled_points[7][2].l());
  // export_sl_points(sampled_points, "/tmp/points.txt");
}

}  // namespace planning
}  // namespace apollo
