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

#include "modules/common/util/util.h"

#include <cmath>

namespace apollo {
namespace common {
namespace util {

using SLPoint = apollo::common::SLPoint;
using PathPoint = apollo::common::PathPoint;
using TrajectoryPoint = apollo::common::TrajectoryPoint;

SLPoint MakeSLPoint(const double s, const double l) {
  SLPoint sl;
  sl.set_s(s);
  sl.set_l(l);
  return sl;
}

PathPoint MakePathPoint(const double x, const double y, const double z,
                        const double theta, const double kappa,
                        const double dkappa, const double ddkappa) {
  PathPoint path_point;
  path_point.set_x(x);
  path_point.set_y(y);
  path_point.set_z(z);
  path_point.set_theta(theta);
  path_point.set_kappa(kappa);
  path_point.set_dkappa(dkappa);
  path_point.set_ddkappa(ddkappa);
  return path_point;
}
TrajectoryPoint MakeTrajectoryPoint(const apollo::common::PathPoint& path_point,
                                    const double v, const double a,
                                    const double relative_time) {
  TrajectoryPoint point;
  point.mutable_path_point()->CopyFrom(path_point);
  point.set_v(v);
  point.set_a(a);
  point.set_relative_time(relative_time);
  return point;
}

double Distance2D(const PathPoint& a, const PathPoint& b) {
  return std::hypot(a.x() - b.x(), a.y() - b.y());
}

apollo::hdmap::Id MakeMapId(const std::string& id) {
  apollo::hdmap::Id map_id;
  map_id.set_id(id);
  return map_id;
}

}  // namespace util
}  // namespace common
}  // namespace apollo
