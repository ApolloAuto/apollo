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

#include "modules/prediction/scenario/scenario_features/cruise_scenario_features.h"

namespace apollo {
namespace prediction {

CruiseScenarioFeatures::CruiseScenarioFeatures() {}

CruiseScenarioFeatures::~CruiseScenarioFeatures() {}

bool CruiseScenarioFeatures::IsLaneOfInterest(const std::string lane_id) const {
  return lane_ids_of_interest_.find(lane_id) != lane_ids_of_interest_.end();
}

void CruiseScenarioFeatures::InsertLaneOfInterest(const std::string lane_id) {
  lane_ids_of_interest_.insert(lane_id);
}

}  // namespace prediction
}  // namespace apollo
