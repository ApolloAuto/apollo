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

#include "optimizer/st_graph/st_boundary_mapper.h"

#include <limits>

#include "common/houston_gflags.h"
//#include "map/pnc_map.h"
#include "math/double.h"
#include "math/segment2d.h"
#include "math/vec2d.h"
#include "util/data_center.h"

namespace adu {
namespace planning {

STBoundaryMapper::STBoundaryMapper(
    const STBoundaryConfig& st_boundary_config,
    const ::adu::common::config::VehicleParam& veh_param)
    : _st_boundary_config(st_boundary_config), _veh_param(veh_param) {}

double STBoundaryMapper::get_area(
    const std::vector<STPoint>& boundary_points) const {
  if (boundary_points.size() < 3) {
    return 0.0;
  }

  double area = 0.0;

  for (size_t i = 1; i < boundary_points.size(); ++i) {
    const double dx1 = boundary_points[i - 1].x() - boundary_points[0].x();
    const double dy1 = boundary_points[i].y() - boundary_points[0].y();
    const double dx2 = boundary_points[i].x() - boundary_points[0].x();
    const double dy2 = boundary_points[i - 1].y() - boundary_points[0].y();
    area += (dx1 * dy1 - dx2 * dy2);
  }

  return fabs(area);
}

bool STBoundaryMapper::check_overlap(
    const PathPoint& path_point,
    const ::adu::common::config::VehicleParam& params,
    const ::adu::common::math::Box2d& obs_box, const double buffer) const {
  const double mid_to_rear_center =
      params.length() / 2.0 - params.front_edge_to_center();
  const double x =
      path_point.x() - mid_to_rear_center * std::cos(path_point.theta());
  const double y =
      path_point.y() - mid_to_rear_center * std::sin(path_point.theta());
  const ::adu::common::math::Box2d adc_box = ::adu::common::math::Box2d(
      ::adu::common::math::Vec2d(x, y), path_point.theta(),
      params.length() + 2 * buffer, params.width() + 2 * buffer);
  return obs_box.has_overlap(adc_box);
}

ErrorCode STBoundaryMapper::get_speed_limits(
    const Environment& environment, const LogicMap& map,
    const PathData& path_data, const double planning_distance,
    const std::size_t matrix_dimension_s, const double default_speed_limit,
    SpeedLimit* const speed_limit_data) const {
  const auto& adc_position =
      environment.vehicle_state_proxy().vehicle_state().pose().position();

  ::adu::common::hdmap::Point adc_point;
  adc_point.set_x(adc_position.x());
  adc_point.set_y(adc_position.y());
  adc_point.set_z(adc_position.z());
  std::vector<const ::adu::hdmap::LaneInfo*> lanes;
  int ret = map.get_lanes(adc_point, 1.0, &lanes);
  QUIT_IF(ret != 0, ErrorCode::PLANNING_ERROR_FAILED, Level::ERROR,
          "Fail to get lanes for point [%lf, %lf]", adc_position.x(),
          adc_position.y());

  std::vector<double> speed_limits;
  for (const auto& point : path_data.path().path_points()) {
    speed_limits.push_back(_st_boundary_config.maximal_speed());
    for (const auto* lane : lanes) {
      if (lane->is_on_lane({point.x(), point.y()})) {
        speed_limits.back() =
            std::fmin(speed_limits.back(), lane->lane().speed_limit());
        if (lane->lane().turn() == ::adu::common::hdmap::Lane::U_TURN) {
          speed_limits.back() = std::fmin(
              speed_limits.back(), _st_boundary_config.speed_limit_on_u_turn());
        }
      }
    }
  }

  QUIT_IF(planning_distance > path_data.path().path_points().back().s(),
          ErrorCode::PLANNING_ERROR_FAILED, Level::ERROR,
          "path length cannot be less than planning_distance");

  double s = 0.0;
  const double unit_s = planning_distance / matrix_dimension_s;
  std::size_t i = 0;
  std::size_t j = 1;
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
      mutable_speed_limits_data->push_back(
          SpeedPoint(s, 0.0, curr_speed_limit, 0.0, 0.0));

      IDG_LOG(Level::INFO, "speed_limit i:%u, j:%u, v[%lf], kappa[%lf]", i, j,
              curr_speed_limit, kappa);
      s += unit_s;
      ++i;
    }
  }

  return ErrorCode::PLANNING_OK;
}
const STBoundaryConfig& STBoundaryMapper::st_boundary_config() const {
  return _st_boundary_config;
}

const ::adu::common::config::VehicleParam& STBoundaryMapper::vehicle_param()
    const {
  return _veh_param;
}

std::string STBoundaryMapper::to_json(
    std::vector<STGraphBoundary>* const obs_boundary) const {
  std::ostringstream sout;
  sout << "[" << std::endl;

  for (std::size_t i = 0; i < obs_boundary->size(); ++i) {
    const auto& boundary = obs_boundary->at(i);

    if (i > 0) {
      sout << ", " << std::endl;
    }

    sout << boundary.to_json().c_str() << std::endl;
  }

  sout << "]" << std::endl;
  sout.flush();
  return sout.str();
}

}  // namespace planning
}  // namespace adu

/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */
