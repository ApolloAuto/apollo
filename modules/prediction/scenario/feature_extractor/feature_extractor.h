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
 */

#pragma once

#include <memory>

#include "modules/prediction/common/environment_features.h"
#include "modules/prediction/container/adc_trajectory/adc_trajectory_container.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/pose/pose_container.h"

namespace apollo {
namespace prediction {

class FeatureExtractor {
 public:
  /**
   * @brief Constructor
   */
  FeatureExtractor() = delete;

  /**
   * @brief Extract features for scenario analysis
   * @return Scenario features
   */
  static EnvironmentFeatures ExtractEnvironmentFeatures();

  FRIEND_TEST(FeatureExtractorTest, junction);

 private:
  static void ExtractEgoLaneFeatures(
      EnvironmentFeatures* ptr_environment_features,
      const std::shared_ptr<const hdmap::LaneInfo>& ptr_ego_lane,
      const common::math::Vec2d& ego_position);

  static void ExtractNeighborLaneFeatures(
      EnvironmentFeatures* ptr_environment_features,
      const std::shared_ptr<const hdmap::LaneInfo>& ptr_ego_lane,
      const common::math::Vec2d& ego_position);

  static void ExtractFrontJunctionFeatures(
      EnvironmentFeatures* ptr_environment_features);

  static std::shared_ptr<const hdmap::LaneInfo> GetEgoLane(
      const common::Point3D& position, const double heading);
};

}  // namespace prediction
}  // namespace apollo
