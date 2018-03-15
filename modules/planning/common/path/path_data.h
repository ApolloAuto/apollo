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
 * @file path_data.h
 **/

#ifndef MODULES_PLANNING_COMMON_PATH_PATH_DATA_H_
#define MODULES_PLANNING_COMMON_PATH_PATH_DATA_H_

#include <list>
#include <string>
#include <utility>

#include "modules/planning/common/path/discretized_path.h"
#include "modules/planning/common/path/frenet_frame_path.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class PathData {
 public:
  PathData() = default;

  bool SetDiscretizedPath(const DiscretizedPath &path);

  bool SetFrenetPath(const FrenetFramePath &frenet_path);

  void SetReferenceLine(const ReferenceLine *reference_line);

  const DiscretizedPath &discretized_path() const;

  const FrenetFramePath &frenet_frame_path() const;

  bool GetPathPointWithPathS(const double s,
                             common::PathPoint *const path_point) const;

  std::list<std::pair<DiscretizedPath, FrenetFramePath>> &path_data_history();

  /*
   * brief: this function will find the path_point in discretized_path whose
   * projection to reference line has s value closest to ref_s.
   */
  bool GetPathPointWithRefS(const double ref_s,
                            common::PathPoint *const path_point) const;

  void Clear();

  bool Empty() const;

  std::string DebugString() const;

 private:
  /*
   * convert frenet path to cartesian path by reference line
   */
  bool SLToXY(const FrenetFramePath &frenet_path,
              DiscretizedPath *const discretized_path);
  bool XYToSL(const DiscretizedPath &discretized_path,
              FrenetFramePath *const frenet_path);
  const ReferenceLine *reference_line_ = nullptr;
  DiscretizedPath discretized_path_;
  FrenetFramePath frenet_path_;
  std::list<std::pair<DiscretizedPath, FrenetFramePath>> path_data_history_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_PATH_PATH_DATA_H_
