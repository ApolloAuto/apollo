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

#ifndef MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_CONTI_RADAR_ID_EXPANSION_H_
#define MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_CONTI_RADAR_ID_EXPANSION_H_

#include <map>

#include "modules/perception/obstacle/radar/interface/base_radar_detector.h"
#include "modules/perception/obstacle/radar/modest/radar_define.h"

namespace apollo {
namespace perception {

class ContiRadarIDExpansion {
 public:
  ContiRadarIDExpansion();

  ~ContiRadarIDExpansion();

  void ExpandIds(ContiRadar* radar_obs);

  void SkipOutdatedObjects(ContiRadar* radar_obs);

  void SetNeedRestart(const bool need_restart);

  void UpdateTimestamp(const double& timestamp);

 private:
  int GetNextId();

  int current_idx_;
  bool need_restart_;
  bool need_inner_restart_;
  double timestamp_;
  std::map<int, int> local2global_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_MODEST_RADAR_CONTI_RADAR_ID_EXPANSION_H_
