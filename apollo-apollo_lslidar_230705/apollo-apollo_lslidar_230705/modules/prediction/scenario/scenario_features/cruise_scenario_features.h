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

#include <string>
#include <unordered_set>

#include "modules/prediction/common/environment_features.h"
#include "modules/prediction/scenario/scenario_features/scenario_features.h"

namespace apollo {
namespace prediction {

class CruiseScenarioFeatures : public ScenarioFeatures {
 public:
  CruiseScenarioFeatures();

  virtual ~CruiseScenarioFeatures();

  bool IsLaneOfInterest(const std::string& lane_id) const;

  void InsertLaneOfInterest(const std::string& lane_id);

  void BuildCruiseScenarioFeatures(
      const EnvironmentFeatures& environment_features);

 private:
  void SearchForwardAndInsert(const std::string& lane_id,
                              const double start_lane_s, const double range);

  std::unordered_set<std::string> lane_ids_of_interest_;
};

}  // namespace prediction
}  // namespace apollo
