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

#include "modules/prediction/common/kml_map_based_test.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;

class FeatureExtractorTest : public KMLMapBasedTest {};

TEST_F(FeatureExtractorTest, junction) {
  ContainerManager::Instance()->RegisterContainers();
  std::unique_ptr<Container> adc_traj_container =
      ContainerManager::Instance()->CreateContainer(
          AdapterConfig::PLANNING_TRAJECTORY);

  EnvironmentFeatures environment_features;
  FeatureExtractor::ExtractFrontJunctionFeatures(&environment_features);

  environment_features = FeatureExtractor::ExtractEnvironmentFeatures();
  EXPECT_FALSE(environment_features.has_front_junction());
}

}  // namespace prediction
}  // namespace apollo
