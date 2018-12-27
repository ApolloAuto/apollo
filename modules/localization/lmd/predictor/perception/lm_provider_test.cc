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

#include "modules/localization/lmd/predictor/perception/lm_provider.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {

TEST(LMProviderTest, GetLaneMarkerPackSize) {
  LMProvider* lm_provider_ = new LMProvider();
  EXPECT_EQ(2, lm_provider_->GetLaneMarkerPackSize());
  delete lm_provider_;
}

TEST(LMProviderTest, GetLaneMarkerSize) {
  LMProvider* lm_provider_ = new LMProvider();
  EXPECT_EQ(2641, lm_provider_->GetLaneMarkerSize(0));
  EXPECT_EQ(2641, lm_provider_->GetLaneMarkerSize(1));
  delete lm_provider_;
}

TEST(LMProviderTest, CalculateDistance) {
  apollo::common::PointENU position;
  position.set_x(682375.86553);
  position.set_y(3111283.00623);
  position.set_z(66.24156);
  apollo::common::PointENU position_destination;
  position_destination.set_x(682275.86553);
  position_destination.set_y(3111183.00623);
  position_destination.set_z(56.24156);
  LMProvider* lm_provider_ = new LMProvider();
  EXPECT_DOUBLE_EQ(141.42135623730951, lm_provider_->CalculateDistance(
                                           position, position_destination));
  delete lm_provider_;
}

TEST(LMProviderTest, FindNearestLaneMarkerIndex) {
  apollo::common::PointENU position;
  position.set_x(682375.86553);
  position.set_y(3111283.00623);
  position.set_z(66.24156);
  LMProvider* lm_provider_ = new LMProvider();
  const std::pair<int64_t, int64_t>& result_index =
      lm_provider_->FindNearestLaneMarkerIndex(position);
  EXPECT_EQ(1, result_index.first);
  EXPECT_EQ(514, result_index.second);
  delete lm_provider_;
}

TEST(LMProviderTest, GetPrevLaneMarkerIndex) {
  apollo::common::PointENU position;
  position.set_x(682375.86553);
  position.set_y(3111283.00623);
  position.set_z(66.24156);
  LMProvider* lm_provider_ = new LMProvider();
  const std::pair<int64_t, int64_t> nearest_index =
      lm_provider_->FindNearestLaneMarkerIndex(position);
  const std::pair<int64_t, int64_t> prev_index =
      lm_provider_->GetPrevLaneMarkerIndex(nearest_index);
  EXPECT_EQ(prev_index.first, nearest_index.first);
  EXPECT_EQ(prev_index.second, nearest_index.second - 1);
  EXPECT_EQ(1, prev_index.first);
  EXPECT_EQ(513, prev_index.second);
  delete lm_provider_;
}

TEST(LMProviderTest, GetNextLaneMarkerIndex) {
  apollo::common::PointENU position;
  position.set_x(682375.86553);
  position.set_y(3111283.00623);
  position.set_z(66.24156);
  LMProvider* lm_provider_ = new LMProvider();
  const std::pair<int64_t, int64_t> nearest_index =
      lm_provider_->FindNearestLaneMarkerIndex(position);
  const std::pair<int64_t, int64_t> next_index =
      lm_provider_->GetNextLaneMarkerIndex(nearest_index);
  EXPECT_EQ(next_index.first, nearest_index.first);
  EXPECT_EQ(next_index.second, nearest_index.second + 1);
  EXPECT_EQ(1, next_index.first);
  EXPECT_EQ(515, next_index.second);
  delete lm_provider_;
}

TEST(LMProviderTest, GetLeftLaneMarkerIndex) {
  apollo::common::PointENU position;
  position.set_x(682375.86553);
  position.set_y(3111283.00623);
  position.set_z(66.24156);
  LMProvider* lm_provider_ = new LMProvider();
  const std::pair<int64_t, int64_t> nearest_index =
      lm_provider_->FindNearestLaneMarkerIndex(position);
  const std::pair<int64_t, int64_t> left_index =
      lm_provider_->GetLeftLaneMarkerIndex(nearest_index);
  EXPECT_EQ(left_index.first, nearest_index.first - 1);
  EXPECT_EQ(left_index.second, nearest_index.second);
  EXPECT_EQ(0, left_index.first);
  EXPECT_EQ(514, left_index.second);
  delete lm_provider_;
}

TEST(LMProviderTest, GetRightLaneMarkerIndex) {
  apollo::common::PointENU position;
  position.set_x(682375.86553);
  position.set_y(3111283.00623);
  position.set_z(66.24156);
  LMProvider* lm_provider_ = new LMProvider();
  const std::pair<int64_t, int64_t> nearest_index =
      lm_provider_->FindNearestLaneMarkerIndex(position);
  const std::pair<int64_t, int64_t> index =
      lm_provider_->GetLeftLaneMarkerIndex(nearest_index);
  EXPECT_EQ(nearest_index.first - 1, index.first);
  EXPECT_EQ(nearest_index.second, index.second);
  const std::pair<int64_t, int64_t> right_index =
      lm_provider_->GetRightLaneMarkerIndex(index);
  EXPECT_EQ(index.first + 1, right_index.first);
  EXPECT_EQ(index.second, right_index.second);
  EXPECT_EQ(right_index.first, nearest_index.first);
  EXPECT_EQ(right_index.second, nearest_index.second);
  EXPECT_EQ(0, index.first);
  EXPECT_EQ(514, index.second);
  EXPECT_EQ(1, right_index.first);
  EXPECT_EQ(514, right_index.second);
  delete lm_provider_;
}

TEST(LMProviderTest, GetLaneMarker) {
  apollo::common::PointENU position;
  position.set_x(682375.86553);
  position.set_y(3111283.00623);
  position.set_z(66.24156);
  LMProvider* lm_provider_ = new LMProvider();
  const std::pair<int64_t, int64_t> nearest_index =
      lm_provider_->FindNearestLaneMarkerIndex(position);
  const apollo::localization::OdometryLaneMarker* lane_marker =
      lm_provider_->GetLaneMarker(nearest_index);
  EXPECT_EQ(10, lane_marker->points_size());
  for (int64_t index = 0; index < lane_marker->points_size(); index++) {
    EXPECT_NEAR(-0.653764, lane_marker->points(index).direct().x(), 0.001);
    EXPECT_NEAR(0.7566988, lane_marker->points(index).direct().y(), 0.01);
    EXPECT_NEAR(0.0, lane_marker->points(index).direct().z(), 0.001);
  }
  EXPECT_NEAR(682377.2518249487, lane_marker->points(0).position().x(), 0.01);
  EXPECT_NEAR(3111284.2039481895, lane_marker->points(0).position().y(), 0.01);
  EXPECT_NEAR(66.241558944806457, lane_marker->points(0).position().z(), 0.01);
  delete lm_provider_;
}

}  // namespace localization
}  // namespace apollo
