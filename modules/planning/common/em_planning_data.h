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
 * @file
 **/

#ifndef MODULES_PLANNING_COMMON_EM_PLANNING_DATA_H
#define MODULES_PLANNING_COMMON_EM_PLANNING_DATA_H

#include <string>
#include <vector>

#include "modules/planning/common/planning_data.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/speed/speed_data.h"

namespace apollo {
namespace planning {

class EMPlanningData : public PlanningData {
 public:
  EMPlanningData() = default;
  virtual std::string type() const;

  void init(const std::size_t num_iter);

  std::size_t num_iter() const;
  const PathData& path_data(const std::size_t index) const;
  const SpeedData& speed_data(const std::size_t index) const;

  PathData* mutable_path_data(const std::size_t index);
  SpeedData* mutable_speed_data(const std::size_t index);

  // aggregate final result together by some configuration
  bool aggregate(const double time_resolution);

 private:
  std::size_t _num_iter = 0;
  std::vector<PathData> _path_data_vec;
  std::vector<SpeedData> _speed_data_vec;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_EM_PLANNING_DATA_H
