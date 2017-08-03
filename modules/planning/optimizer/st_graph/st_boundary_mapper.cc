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
#include "modules/common/util/file.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::hdmap::PncMap;
using apollo::localization::Pose;
using apollo::common::VehicleParam;
using apollo::common::math::Vec2d;
using apollo::common::math::Box2d;

bool StBoundaryMapper::Init(const std::string& config_file) {
  if (!common::util::GetProtoFromFile(config_file, &st_boundary_config_)) {
    AERROR << "Failed to load config file " << config_file;
    return false;
  }
  return true;
}

double StBoundaryMapper::GetArea(
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
    // cross product
    area += (ds1 * dt1 - ds2 * dt2);
  }
  return fabs(area);
}

bool StBoundaryMapper::CheckOverlap(const PathPoint& path_point,
                                    const VehicleParam& params,
                                    const Box2d& obs_box,
                                    const double buffer) const {
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

Status StBoundaryMapper::GetSpeedLimits(
    const ReferenceLine& reference_line, const PathData& path_data,
    SpeedLimit* const speed_limit_data) const {
  std::vector<double> speed_limits;
  for (const auto& path_point : path_data.discretized_path().points()) {
    // speed limit from reference line
    double speed_limit_on_reference_line =
        std::fmin(st_boundary_config_.maximal_speed(),
                  reference_line.GetSpeedLimitFromPoint(
                      Vec2d(path_point.x(), path_point.y())));

    // speed limit from path curvature
    double speed_limit_on_path = std::sqrt(
        st_boundary_config_.centric_acceleration_limit() /
        std::fmax(path_point.kappa(), st_boundary_config_.minimal_kappa()));

    const double curr_speed_limit = std::fmax(
        st_boundary_config_.lowest_speed(),
        std::fmin(speed_limit_on_path, speed_limit_on_reference_line));

    SpeedPoint speed_point;
    speed_point.set_s(path_point.s());
    speed_point.set_v(curr_speed_limit);
    speed_limit_data->add_speed_limit(speed_point);
  }
  return Status::OK();
}

const StBoundaryConfig& StBoundaryMapper::st_boundary_config() const {
  return st_boundary_config_;
}

}  // namespace planning
}  // namespace apollo
