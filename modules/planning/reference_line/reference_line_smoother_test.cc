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
 * @file reference_line_smooother_test.cpp
 **/
#include "gtest/gtest.h"

#include "modules/common/math/vec2d.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_line_smoother.h"
#include "modules/planning/reference_line/reference_point.h"

namespace apollo {
namespace planning {

class ReferenceLineSmootherTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    hdmap_.load_map_from_file(map_file);
    const std::string lane_id_str = "1_-1";
    hdmap::Id lane_id;
    lane_id.set_id(lane_id_str);
    lane_info_ptr = hdmap_.get_lane_by_id(lane_id);
    if (!lane_info_ptr) {
      AERROR << "failed to find lane " << lane_id_str << " from map "
             << map_file;
      return;
    }
    smoother_.SetConfig(config_);  // use the default value in config.

    std::vector<ReferencePoint> ref_points;
    const auto& points = lane_info_ptr->points();
    const auto& headings = lane_info_ptr->headings();
    for (std::size_t i = 0; i < points.size(); ++i) {
      ref_points.emplace_back(points[i], headings[i], 0.0, 0.0, -2.0, 2.0);
    }
    reference_line_.reset(new ReferenceLine(ref_points));
    vehicle_position_ = points[0];
  }

  const std::string map_file = "modules/planning/testdata/base_map.txt";
  ReferenceLineSmootherConfig config_;

  hdmap::HDMap hdmap_;
  ReferenceLineSmoother smoother_;
  common::math::Vec2d vehicle_position_;
  std::unique_ptr<ReferenceLine> reference_line_;
  hdmap::LaneInfoConstPtr lane_info_ptr = nullptr;
};

TEST_F(ReferenceLineSmootherTest, smooth) {
  std::vector<ReferencePoint> smoothed_ref_points;
  EXPECT_TRUE(smoother_.smooth(*reference_line_, vehicle_position_,
                               &smoothed_ref_points));
  EXPECT_FALSE(smoothed_ref_points.empty());
}

}  // namespace planning
}  // namespace apollo
