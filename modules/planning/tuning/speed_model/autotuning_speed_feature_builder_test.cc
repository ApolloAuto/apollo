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
 * @file: testing casese for mlp net model
 **/

#include "modules/planning/tuning/speed_model/autotuning_speed_feature_builder.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(AutotuningSpeedFeatureBuilder, test_case_two) {
  AutotuningSpeedFeatureBuilder feature_builder;
  autotuning::TrajectoryRawFeature raw_feature;
  autotuning::TrajectoryPointRawFeature raw_point_feature;
  autotuning::TrajectoryFeature input_feature;
  autotuning::TrajectoryPointwiseFeature point_feature;
  auto status = feature_builder.BuildFeature(raw_feature, &input_feature);
  EXPECT_TRUE(status.ok());
  status = feature_builder.BuildPointFeature(raw_point_feature, &point_feature);
  EXPECT_TRUE(status.ok());
}

}  // namespace planning
}  // namespace apollo
