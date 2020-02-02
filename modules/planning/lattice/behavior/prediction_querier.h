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
 **/

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/planning/common/obstacle.h"

namespace apollo {
namespace planning {

class PredictionQuerier {
 public:
  PredictionQuerier(const std::vector<const Obstacle*>& obstacles,
                    const std::shared_ptr<std::vector<common::PathPoint>>&
                        ptr_reference_line);

  virtual ~PredictionQuerier() = default;

  std::vector<const Obstacle*> GetObstacles() const;

  double ProjectVelocityAlongReferenceLine(const std::string& obstacle_id,
                                           const double s,
                                           const double t) const;

 private:
  std::unordered_map<std::string, const Obstacle*> id_obstacle_map_;

  std::vector<const Obstacle*> obstacles_;

  std::shared_ptr<std::vector<common::PathPoint>> ptr_reference_line_;
};

}  // namespace planning
}  // namespace apollo
