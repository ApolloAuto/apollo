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
 * @file discretized_path.h
 **/

#ifndef MODULES_PLANNING_COMMON_PATH_DISCRETIZED_PATH_H_
#define MODULES_PLANNING_COMMON_PATH_DISCRETIZED_PATH_H_

#include <vector>

#include "modules/planning/common/path/path.h"
#include "modules/planning/common/path/path_point_util.h"

namespace apollo {
namespace planning {

class DiscretizedPath : public Path {
 public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(std::vector<common::PathPoint> path_points);

  virtual ~DiscretizedPath() = default;

  common::PathPoint evaluate(const double param) const override;

  double param_length() const override;

  common::PathPoint start_point() const override;

  common::PathPoint end_point() const override;

  int query_closest_point(const double param) const;

  std::vector<common::PathPoint>* mutable_path_points();

  const std::vector<common::PathPoint>& path_points() const;

  std::size_t num_of_points() const;

  const common::PathPoint& path_point_at(const std::size_t index) const;

  common::PathPoint& path_point_at(const std::size_t index);

 private:
  std::vector<common::PathPoint>::const_iterator query_lower_bound(
      const double param) const;

  std::vector<common::PathPoint> path_points_;
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_COMMON_PATH_PATH_H_ */
