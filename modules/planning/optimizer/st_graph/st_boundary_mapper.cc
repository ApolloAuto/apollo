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
 *   @file: st_boundary_mapper.cc
 **/

#include "modules/planning/optimizer/st_graph/st_boundary_mapper.h"

#include <limits>
#include <memory>
#include <string>

#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::hdmap::HDMap;
using apollo::localization::Pose;

StBoundaryMapper::StBoundaryMapper(
    const StBoundaryConfig& st_boundary_config,
    const apollo::common::config::VehicleParam& veh_param)
    : _st_boundary_config(st_boundary_config), _veh_param(veh_param) {}

double StBoundaryMapper::get_area(
    const std::vector<STPoint>& boundary_points) const {
  if (boundary_points.size() < 3) {
    return 0.0;
  }

  double area = 0.0;

  for (uint32_t i = 1; i < boundary_points.size(); ++i) {
    const double ds1 = boundary_points[i - 1].s() - boundary_points[0].s();
    const double dt1 = boundary_points[i].t() - boundary_points[0].t();
    const double ds2 = boundary_points[i].s() - boundary_points[0].s();
    const double dt2 = boundary_points[i - 1].t() - boundary_points[0].t();
    area += (ds1 * dt1 - ds2 * dt2);
  }

  return fabs(area);
}

bool StBoundaryMapper::check_overlap(
    const PathPoint& path_point,
    const apollo::common::config::VehicleParam& params,
    const apollo::common::math::Box2d& obs_box, const double buffer) const {
  const double mid_to_rear_center =
      params.length() / 2.0 - params.front_edge_to_center();
  const double x =
      path_point.x() - mid_to_rear_center * std::cos(path_point.theta());
  const double y =
      path_point.y() - mid_to_rear_center * std::sin(path_point.theta());
  const apollo::common::math::Box2d adc_box = apollo::common::math::Box2d(
      {x, y}, path_point.theta(), params.length() + 2 * buffer,
      params.width() + 2 * buffer);
  return obs_box.HasOverlap(adc_box);
}

Status StBoundaryMapper::get_speed_limits(
    const Pose& pose, const HDMap& map, const PathData& path_data,
    const double planning_distance, const std::uint32_t matrix_dimension_s,
    const double default_speed_limit, SpeedLimit* const speed_limit_data) {
  const auto& adc_position = pose.position();

  apollo::hdmap::Point adc_point;
  adc_point.set_x(adc_position.x());
  adc_point.set_y(adc_position.y());
  adc_point.set_z(adc_position.z());
  // std::vector<const apollo::hdmap::LaneInfo*> lanes;
  std::vector<std::shared_ptr<const apollo::hdmap::LaneInfo>> lanes;
  int ret = map.get_lanes(adc_point, 1.0, &lanes);
  if (ret != 0) {
    AERROR << "Fail to get lanes for point [" << adc_position.x() << ", "
           << adc_position.y() << "].";
    return Status(ErrorCode::PLANNING_ERROR_FAILED,
                  "StBoundaryMapper::get_speed_limits");
  }

  std::vector<double> speed_limits;
  // TODO(zhangliangliang): waiting for hdmap to support is_on_lane.
  // for (const auto& point : path_data.path().path_points()) {
  //   speed_limits.push_back(_st_boundary_config.maximal_speed());
  //   for (auto& lane : lanes) {
  //     if (lane->is_on_lane({point.x(), point.y()})) {
  //       speed_limits.back() =
  //           std::fmin(speed_limits.back(), lane->lane().speed_limit());
  //       if (lane->lane().turn() == apollo::hdmap::Lane::U_TURN) {
  //         speed_limits.back() = std::fmin(
  //             speed_limits.back(),
  //             _st_boundary_config.speed_limit_on_u_turn());
  //       }
  //     }
  //   }
  // }

  if (planning_distance > path_data.path().path_points().back().s()) {
    const std::string msg = "path length cannot be less than planning_distance";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR_FAILED, msg);
  }

  double s = 0.0;
  const double unit_s = planning_distance / matrix_dimension_s;
  std::uint32_t i = 0;
  std::uint32_t j = 1;
  const auto& path_points = path_data.path().path_points();
  while (i < matrix_dimension_s && j < path_points.size()) {
    const auto& point = path_points[j];
    const auto& pre_point = path_points[j - 1];
    double curr_speed_limit = default_speed_limit;

    if (Double::compare(s, point.s()) > 0) {
      ++j;
    } else {
      double r = 0.0;
      if (Double::compare(point.s(), pre_point.s()) != 0) {
        r = (s - pre_point.s()) / (point.s() - pre_point.s());
      }
      const double decision_speed =
          speed_limits[j - 1] + r * (speed_limits[j + 1] - speed_limits[j]);
      curr_speed_limit = std::fmin(curr_speed_limit, decision_speed);
      const double kappa = std::fabs(pre_point.kappa() +
                                     r * (point.kappa() - pre_point.kappa()));
      if (kappa > _st_boundary_config.kappa_threshold()) {
        const double speed =
            std::sqrt(_st_boundary_config.centric_acceleration_limit() /
                      std::fmax(kappa, _st_boundary_config.minimal_kappa()));
        curr_speed_limit =
            std::fmin(curr_speed_limit,
                      std::fmin(_st_boundary_config.maximal_speed(), speed));
      }

      curr_speed_limit *= _st_boundary_config.speed_multiply_buffer();
      curr_speed_limit =
          std::fmax(curr_speed_limit, _st_boundary_config.lowest_speed());
      auto* mutable_speed_limits_data =
          speed_limit_data->mutable_speed_limits();

      SpeedPoint speed_point;
      speed_point.set_s(s);
      speed_point.set_t(0.0);
      speed_point.set_v(curr_speed_limit);
      speed_point.set_a(0.0);
      speed_point.set_da(0.0);
      mutable_speed_limits_data->push_back(speed_point);

      s += unit_s;
      ++i;
    }
  }

  return Status::OK();
}

const StBoundaryConfig& StBoundaryMapper::st_boundary_config() const {
  return _st_boundary_config;
}

const apollo::common::config::VehicleParam& StBoundaryMapper::vehicle_param()
    const {
  return _veh_param;
}

}  // namespace planning
}  // namespace apollo
