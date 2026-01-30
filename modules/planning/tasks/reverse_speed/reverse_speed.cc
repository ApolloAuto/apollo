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

#include <algorithm>
#include <memory>
#include <vector>
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/tasks/reverse_speed/reverse_speed.h"

#include "modules/common/util/util.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

bool ReverseSpeed::Init(const std::string &config_dir, const std::string &name,
                        const std::shared_ptr<DependencyInjector> &injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  return Task::LoadConfig<ReverseSpeedConfig>(&config_);
}

Status ReverseSpeed::Execute(Frame *frame,
                             ReferenceLineInfo *reference_line_info) {
  Task::Execute(frame, reference_line_info);
  std::vector<const STBoundary *> boundaries;
  const TrajectoryPoint init_point = frame->PlanningStartPoint();
  const auto &path_data = reference_line_info->path_data();
  GetSTboundaries(path_data.discretized_path(), frame, &boundaries);
  SpeedLimit speed_limit;
  speed_limit.AppendSpeedLimit(0, config_.speed_limit());
  speed_limit.AppendSpeedLimit(config_.total_time(), config_.speed_limit());
  StGraphData *st_graph_data = reference_line_info->mutable_st_graph_data();
  auto *debug = reference_line_info_->mutable_debug();
  auto *st_graph_debug = debug->mutable_planning_data()->add_st_graph();
  st_graph_data->LoadData(boundaries, 0.0, init_point, speed_limit,
                          config_.speed_limit(),
                          path_data.discretized_path().Length(),
                          config_.total_time(), st_graph_debug);
  return Status::OK();
}

void ReverseSpeed::GetSTboundaries(
    const DiscretizedPath &path_data, Frame *frame,
    std::vector<const STBoundary *> *boundaries) {
  CHECK_NOTNULL(boundaries);
  CHECK_NOTNULL(frame);
  static constexpr double kEpsilon = 1e-2;
  auto obstacles_by_frame = frame->GetObstacleList();

  for (auto *obstacle : obstacles_by_frame->Items()) {
    STBoundary st_boundary;
    std::vector<STPoint> lower_points;
    std::vector<STPoint> upper_points;
    for (const auto &curr_point_on_path : path_data) {
      if (CheckOverlap(curr_point_on_path, obstacle->PerceptionPolygon(),
                       config_.l_buffer())) {
        const double backward_distance = -vehicle_param_.back_edge_to_center();
        const double forward_distance =
            obstacle->PerceptionBoundingBox().length();
        double low_s =
            std::fmax(0.0, curr_point_on_path.s() + backward_distance);
        double high_s = std::fmax(low_s + kEpsilon,
                            std::fmin(path_data.Length(),
                                  curr_point_on_path.s() + forward_distance));
        // It is an unrotated rectangle appearing on the ST-graph.
        // TODO(jiacheng): reconsider the backward_distance, it might be
        // unnecessary, but forward_distance is indeed meaningful though.
        lower_points.emplace_back(low_s, 0.0);
        lower_points.emplace_back(low_s, config_.total_time());
        upper_points.emplace_back(high_s, 0.0);
        upper_points.emplace_back(high_s, config_.total_time());
        STBoundary st_boundary =
            STBoundary::CreateInstance(lower_points, upper_points);
        st_boundary.ExpandByS(config_.s_buffer());
        st_boundary.SetCharacteristicLength(config_.s_buffer());
        st_boundary.SetBoundaryType(STBoundary::BoundaryType::STOP);
        auto *origin_obstacle = frame->Find(obstacle->Id());
        origin_obstacle->set_path_st_boundary(st_boundary);
        boundaries->push_back(&origin_obstacle->path_st_boundary());
        break;
      }
    }
  }
}

bool ReverseSpeed::CheckOverlap(const PathPoint &path_point,
                                const Polygon2d &obs_polygon,
                                const double l_buffer) {
  // Convert reference point from center of rear axis to center of ADC.
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(path_point.theta());
  ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

  // Compute the ADC bounding box.
  Box2d adc_box(ego_center_map_frame, path_point.theta(),
                vehicle_param_.length(), vehicle_param_.width() + l_buffer * 2);

  // Check whether ADC polygon overlaps with obstacle polygon.
  Polygon2d adc_polygon(adc_box);
  return obs_polygon.HasOverlap(adc_polygon);
}

}  // namespace planning
}  // namespace apollo
