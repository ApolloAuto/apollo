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

/** * @file path_data.h
 **/

#ifndef MODULES_PLANNING_COMMON_PATH_PATH_DATA_H_
#define MODULES_PLANNING_COMMON_PATH_PATH_DATA_H_

#include <string>
#include <vector>

#include "modules/planning/common/path/discretized_path.h"
#include "modules/planning/common/path/frenet_frame_path.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class PathData {
 public:
  PathData() = default;

  void set_discretized_path(const DiscretizedPath &path);

  void set_discretized_path(const std::vector<common::PathPoint> &path_points);

  void set_frenet_path(const FrenetFramePath &frenet_path);

  const DiscretizedPath &discretized_path() const;

  const FrenetFramePath &frenet_frame_path() const;

  bool get_path_point_with_path_s(const double s,
                                  common::PathPoint *const path_point) const;

  bool get_path_point_with_ref_s(const ReferenceLine &reference_line,
                                 const double ref_s,
                                 common::PathPoint *const path_point) const;

  void Clear();

  std::string DebugString() const;

 private:
  DiscretizedPath discretized_path_;

  FrenetFramePath frenet_path_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_PATH_PATH_DATA_H_
