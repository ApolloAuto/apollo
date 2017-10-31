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

/**
 * @file qp_spline_reference_line_smoother_test.cc
 **/
#include "modules/planning/reference_line/qp_spline_reference_line_smoother.h"

#include "gtest/gtest.h"

#include "modules/planning/proto/qp_spline_reference_line_smoother_config.pb.h"

#include "modules/common/math/vec2d.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_point.h"

namespace apollo {
namespace planning {

class QpSplineReferenceLineSmootherTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    hdmap_.LoadMapFromFile(map_file);
    const std::string lane_id = "1_-1";
    lane_info_ptr = hdmap_.GetLaneById(hdmap::MakeMapId(lane_id));
    if (!lane_info_ptr) {
      AERROR << "failed to find lane " << lane_id << " from map " << map_file;
      return;
    }
    QpSplineReferenceLineSmootherConfig config;
    smoother_.Init(config,
                   &spline_solver_);  // use the default value in config.

    std::vector<ReferencePoint> ref_points;
    const auto& points = lane_info_ptr->points();
    const auto& headings = lane_info_ptr->headings();
    const auto& accumulate_s = lane_info_ptr->accumulate_s();
    for (std::size_t i = 0; i < points.size(); ++i) {
      std::vector<hdmap::LaneWaypoint> waypoint;
      waypoint.emplace_back(lane_info_ptr, accumulate_s[i]);
      hdmap::MapPathPoint map_path_point(points[i], headings[i], waypoint);
      ref_points.emplace_back(map_path_point, 0.0, 0.0, -2.0, 2.0);
    }
    reference_line_.reset(new ReferenceLine(ref_points));
    vehicle_position_ = points[0];
  }

  const std::string map_file =
      "modules/planning/testdata/garage_map/base_map.txt";

  hdmap::HDMap hdmap_;
  QpSplineReferenceLineSmoother smoother_;
  common::math::Vec2d vehicle_position_;
  std::vector<double> t_knots_;
  Spline2dSolver spline_solver_{t_knots_, 5};
  std::unique_ptr<ReferenceLine> reference_line_;
  hdmap::LaneInfoConstPtr lane_info_ptr = nullptr;
};

TEST_F(QpSplineReferenceLineSmootherTest, smooth) {
  ReferenceLine smoothed_reference_line;
  EXPECT_FLOAT_EQ(153.87421, reference_line_->Length());
  EXPECT_TRUE(smoother_.Smooth(*reference_line_, &smoothed_reference_line));
  EXPECT_FLOAT_EQ(153.64156, smoothed_reference_line.Length());
}

}  // namespace planning
}  // namespace apollo
