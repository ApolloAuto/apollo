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
 * @file discretized_trajectory.cc
 **/

#include "modules/planning/planning_base/common/trajectory/discretized_trajectory.h"

#include <limits>

#include "cyber/common/log.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/planning/planning_base/common/planning_context.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

DiscretizedTrajectory::DiscretizedTrajectory(
    const std::vector<TrajectoryPoint>& trajectory_points)
    : std::vector<TrajectoryPoint>(trajectory_points) {
  ACHECK(!trajectory_points.empty())
      << "trajectory_points should NOT be empty()";
}

DiscretizedTrajectory::DiscretizedTrajectory(const ADCTrajectory& trajectory) {
  assign(trajectory.trajectory_point().begin(),
         trajectory.trajectory_point().end());
}

TrajectoryPoint DiscretizedTrajectory::Evaluate(
    const double relative_time) const {
  auto comp = [](const TrajectoryPoint& p, const double relative_time) {
    return p.relative_time() < relative_time;
  };

  auto it_lower = std::lower_bound(begin(), end(), relative_time, comp);

  if (it_lower == begin()) {
    return front();
  } else if (it_lower == end()) {
    AWARN << "When evaluate trajectory, relative_time(" << relative_time
          << ") is too large";
    return back();
  }
  return common::math::InterpolateUsingLinearApproximation(
      *(it_lower - 1), *it_lower, relative_time);
}
/// @brief 
/// @param relative_time 表示查询的相对时间
/// @param epsilon 表示容忍的误差，用于调整比较的精度
/// @return 
size_t DiscretizedTrajectory::QueryLowerBoundPoint(const double relative_time,
                                                   const double epsilon) const {
  // 确保DiscretizedTrajectory对象不为空
  ACHECK(!empty());

  if (relative_time >= back().relative_time()) {
    // 返回最后一个点的索引
    return size() - 1;
  }
  // 判断该轨迹点的相对时间加上epsilon是否小于查询的relative_time
  auto func = [&epsilon](const TrajectoryPoint& tp,
                         const double relative_time) {
    return tp.relative_time() + epsilon < relative_time;
  };
  // 返回一个迭代器，指向第一个满足func条件的位置
  auto it_lower = std::lower_bound(begin(), end(), relative_time, func);
  // 返回从容器的开始位置到it_lower迭代器之间的距离。这个距离就是查询点relative_time的下界（lower bound）索引
  return std::distance(begin(), it_lower);
}

size_t DiscretizedTrajectory::QueryNearestPoint(
    const common::math::Vec2d& position) const {
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  for (size_t i = 0; i < size(); ++i) {
    const common::math::Vec2d curr_point(data()[i].path_point().x(),
                                         data()[i].path_point().y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

size_t DiscretizedTrajectory::QueryNearestPointWithBuffer(
    const common::math::Vec2d& position, const double buffer) const {
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  for (size_t i = 0; i < size(); ++i) {
    const common::math::Vec2d curr_point(data()[i].path_point().x(),
                                         data()[i].path_point().y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min + buffer) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

void DiscretizedTrajectory::AppendTrajectoryPoint(
    const TrajectoryPoint& trajectory_point) {
  if (!empty()) {
    CHECK_GT(trajectory_point.relative_time(), back().relative_time());
  }
  push_back(trajectory_point);
}
/// @brief 常量引用意味着该函数返回的对象不能被修改，并且通过引用返回的对象不会被复制
// 从离散轨迹中获取某个特定点的函数
/// @param index 
/// @return 
const TrajectoryPoint& DiscretizedTrajectory::TrajectoryPointAt(
    const size_t index) const {
  // 检查 index 是否小于 NumOfPoints()
  CHECK_LT(index, NumOfPoints());
  // 一个成员函数，它返回 DiscretizedTrajectory 类的一个数据结构（通常是一个数组或类似容器），其中存储了轨迹的所有点
  return data()[index];
}

TrajectoryPoint DiscretizedTrajectory::StartPoint() const {
  ACHECK(!empty());
  return front();
}

double DiscretizedTrajectory::GetTemporalLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().relative_time() - front().relative_time();
}

double DiscretizedTrajectory::GetSpatialLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().path_point().s() - front().path_point().s();
}

}  // namespace planning
}  // namespace apollo
