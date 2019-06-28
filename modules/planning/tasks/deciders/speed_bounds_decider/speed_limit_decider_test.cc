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

#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_limit_decider.h"

#include "gmock/gmock.h"

#include "cyber/common/log.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/reference_line/qp_spline_reference_line_smoother.h"

namespace apollo {
namespace planning {

class SpeedLimitDeciderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    hdmap_.LoadMapFromFile(map_file);
    const std::string lane_id = "1_-1";
    lane_info_ptr = hdmap_.GetLaneById(hdmap::MakeMapId(lane_id));
    if (!lane_info_ptr) {
      AERROR << "failed to find lane " << lane_id << " from map " << map_file;
      return;
    }
    ReferenceLineSmootherConfig config;

    std::vector<ReferencePoint> ref_points;
    const auto& points = lane_info_ptr->points();
    const auto& headings = lane_info_ptr->headings();
    const auto& accumulate_s = lane_info_ptr->accumulate_s();
    for (size_t i = 0; i < points.size(); ++i) {
      std::vector<hdmap::LaneWaypoint> waypoint;
      waypoint.emplace_back(lane_info_ptr, accumulate_s[i]);
      hdmap::MapPathPoint map_path_point(points[i], headings[i], waypoint);
      ref_points.emplace_back(map_path_point, 0.0, 0.0);
    }
    reference_line_.reset(new ReferenceLine(ref_points));
    vehicle_position_ = points[0];

    path_data_.SetReferenceLine(reference_line_.get());

    std::vector<common::FrenetFramePoint> ff_points;
    for (int i = 0; i < 100; ++i) {
      common::FrenetFramePoint ff_point;
      ff_point.set_s(i * 1.0);
      ff_point.set_l(0.1);
      ff_points.push_back(std::move(ff_point));
    }
    frenet_frame_path_ = FrenetFramePath(ff_points);
    path_data_.SetFrenetPath(frenet_frame_path_);
  }

 protected:
  const std::string map_file =
      "modules/planning/testdata/garage_map/base_map.txt";
  hdmap::HDMap hdmap_;
  common::math::Vec2d vehicle_position_;
  std::unique_ptr<ReferenceLine> reference_line_;
  hdmap::LaneInfoConstPtr lane_info_ptr = nullptr;
  PathData path_data_;
  FrenetFramePath frenet_frame_path_;
};

TEST_F(SpeedLimitDeciderTest, get_centric_acc_limit) {
  SpeedBoundsDeciderConfig config;
  SpeedLimitDecider speed_limit_decider(config, *reference_line_, path_data_);

  double kappa = 0.0001;
  while (kappa < 0.2) {
    const double acc_limit = speed_limit_decider.GetCentricAccLimit(kappa);
    const double v = std::sqrt(acc_limit / kappa);
    if (v > config.high_speed_threshold()) {
      EXPECT_DOUBLE_EQ(acc_limit,
                       config.high_speed_centric_acceleration_limit());
    } else if (v < config.low_speed_threshold()) {
      EXPECT_DOUBLE_EQ(acc_limit,
                       config.low_speed_centric_acceleration_limit());
    } else {
      EXPECT_LE(acc_limit, config.low_speed_centric_acceleration_limit());
      EXPECT_GE(acc_limit, config.high_speed_centric_acceleration_limit());
    }
    kappa += 0.0001;
  }
}
}  // namespace planning
}  // namespace apollo
