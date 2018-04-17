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
 * @file frenet_frame_path.h
 **/

#ifndef MODULES_PLANNING_COMMON_PATH_FRENET_FRAME_PATH_H_
#define MODULES_PLANNING_COMMON_PATH_FRENET_FRAME_PATH_H_

#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/sl_boundary.pb.h"

namespace apollo {
namespace planning {

class FrenetFramePath {
 public:
  FrenetFramePath() = default;
  explicit FrenetFramePath(
      const std::vector<common::FrenetFramePoint> &sl_points);
  virtual ~FrenetFramePath() = default;

  void set_points(const std::vector<common::FrenetFramePoint> &points);
  const std::vector<common::FrenetFramePoint> &points() const;
  std::uint32_t NumOfPoints() const;
  double Length() const;
  const common::FrenetFramePoint &PointAt(const std::uint32_t index) const;
  common::FrenetFramePoint EvaluateByS(const double s) const;

  /**
   * @brief Get the FrenetFramePoint that is within SLBoundary, or the one with
   * smallest l() in SLBoundary's s range [start_s(), end_s()]
   */
  common::FrenetFramePoint GetNearestPoint(const SLBoundary &sl) const;

  virtual void Clear();

 private:
  static bool LowerBoundComparator(const common::FrenetFramePoint &p,
                                   const double s) {
    return p.s() < s;
  }
  static bool UpperBoundComparator(const double s,
                                   const common::FrenetFramePoint &p) {
    return s < p.s();
  }

  std::vector<common::FrenetFramePoint> points_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_PATH_FRENET_FRAME_PATH_H_
