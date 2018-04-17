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

#ifndef MODULES_PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_
#define MODULES_PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_

#include <vector>

#include "modules/planning/proto/planning.pb.h"

#include "modules/common/math/vec2d.h"
#include "modules/planning/common/trajectory/trajectory.h"

namespace apollo {
namespace planning {

class DiscretizedTrajectory : public Trajectory {
 public:
  DiscretizedTrajectory() = default;

  /**
   * Create a DiscretizedTrajectory based on protobuf message
   */
  explicit DiscretizedTrajectory(const ADCTrajectory& trajectory);

  explicit DiscretizedTrajectory(
      const std::vector<common::TrajectoryPoint>& trajectory_points);

  void SetTrajectoryPoints(
      const std::vector<common::TrajectoryPoint>& trajectory_points);

  virtual ~DiscretizedTrajectory() = default;

  common::TrajectoryPoint StartPoint() const override;

  double GetTemporalLength() const override;

  double GetSpatialLength() const override;

  common::TrajectoryPoint Evaluate(const double relative_time) const override;

  virtual uint32_t QueryNearestPoint(const double relative_time) const;

  virtual uint32_t QueryNearestPoint(const common::math::Vec2d& position) const;

  virtual void AppendTrajectoryPoint(
      const common::TrajectoryPoint& trajectory_point);

  template <typename Iter>
  void PrependTrajectoryPoints(Iter begin, Iter end) {
    if (!trajectory_points_.empty() && begin != end) {
      CHECK((end - 1)->relative_time() <
            trajectory_points_.front().relative_time());
    }
    trajectory_points_.insert(trajectory_points_.begin(), begin, end);
  }

  const common::TrajectoryPoint& TrajectoryPointAt(
      const std::uint32_t index) const;

  uint32_t NumOfPoints() const;

  const std::vector<common::TrajectoryPoint>& trajectory_points() const;

  virtual void Clear();

 protected:
  std::vector<common::TrajectoryPoint> trajectory_points_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_
