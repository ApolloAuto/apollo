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

#include "modules/common/proto/pnc_point.pb.h"

namespace apollo {
namespace planning {

class DiscretizedPath {
 public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(const std::vector<common::PathPoint>& path_points);

  virtual ~DiscretizedPath() = default;

  void set_path_points(const std::vector<common::PathPoint>& path_points);

  double Length() const;

  const common::PathPoint& StartPoint() const;

  const common::PathPoint& EndPoint() const;

  common::PathPoint Evaluate(const double path_s) const;

  const std::vector<common::PathPoint>& path_points() const;

  std::uint32_t NumOfPoints() const;

  virtual void Clear();

 protected:
  std::vector<common::PathPoint>::const_iterator QueryLowerBound(
      const double path_s) const;

  std::vector<common::PathPoint> path_points_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_PATH_PATH_H_
