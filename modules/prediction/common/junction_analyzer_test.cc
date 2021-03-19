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

#include "modules/prediction/common/junction_analyzer.h"

#include "modules/prediction/common/kml_map_based_test.h"

namespace apollo {
namespace prediction {

class JunctionAnalyzerTest : public KMLMapBasedTest {};

TEST_F(JunctionAnalyzerTest, SingleLane) {
  JunctionAnalyzer junction_analyzer;
  junction_analyzer.Init("j2");
  EXPECT_NEAR(junction_analyzer.ComputeJunctionRange(), 74.0306, 0.001);
  const JunctionFeature& junction_feature =
      junction_analyzer.GetJunctionFeature("l61");
  EXPECT_EQ(junction_feature.junction_id(), "j2");
  EXPECT_EQ(junction_feature.enter_lane().lane_id(), "l61");
  EXPECT_GT(junction_feature.junction_exit_size(), 0);
  junction_analyzer.Clear();
}

TEST_F(JunctionAnalyzerTest, MultiLane) {
  JunctionAnalyzer junction_analyzer;
  junction_analyzer.Init("j2");
  const JunctionFeature& merged_junction_feature =
      junction_analyzer.GetJunctionFeature(
          std::vector<std::string>{"l35", "l61", "l114", "l162"});
  EXPECT_EQ(merged_junction_feature.junction_id(), "j2");
  EXPECT_EQ(merged_junction_feature.junction_exit_size(), 3);
  junction_analyzer.Clear();
}

}  // namespace prediction
}  // namespace apollo
