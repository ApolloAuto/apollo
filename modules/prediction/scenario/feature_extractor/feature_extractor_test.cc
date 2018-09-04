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

#include "modules/prediction/scenario/feature_extractor/feature_extractor.h"

#include <memory>

#include "gtest/gtest.h"

#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/prediction/common/kml_map_based_test.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/container/container_manager.h"


namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

class FeatureExtractorTest : public KMLMapBasedTest {
 public:
  virtual void SetUp() {}

 protected:
  LocalizationEstimate localization_message_;
  ADCTrajectory adc_trajectory_;
};

TEST_F(FeatureExtractorTest, junction) {
  ContainerManager::instance()->RegisterContainers();
  std::unique_ptr<Container> adc_traj_container =
      ContainerManager::instance()->CreateContainer(
          AdapterConfig::PLANNING_TRAJECTORY);
  FeatureExtractor feature_extractor;
  feature_extractor.ExtractFrontJunctionFeatures();
  ScenarioFeature scenario_feature =
      feature_extractor.GetScenarioFeatures();
  EXPECT_TRUE(!scenario_feature.has_junction_id());
}

}  // namespace prediction
}  // namespace apollo
