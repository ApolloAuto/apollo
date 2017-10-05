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
#include <vector>

namespace apollo {
namespace common {
namespace util {

SLPoint MakeSLPoint(const double s, const double l) {
  SLPoint sl;
  sl.set_s(s);
  sl.set_l(l);
  return sl;
}

PointENU MakePointENU(const double x, const double y, const double z) {
  PointENU point_enu;
  point_enu.set_x(x);
  point_enu.set_y(y);
  point_enu.set_z(z);
  return point_enu;
}

PointENU MakePointENU(const math::Vec2d& xy) {
  PointENU point_enu;
  point_enu.set_x(xy.x());
  point_enu.set_y(xy.y());
  point_enu.set_z(0.0);
  return point_enu;
}

apollo::perception::Point MakePerceptionPoint(const double x, const double y,
                                              const double z) {
  perception::Point point3d;
  point3d.set_x(x);
  point3d.set_y(y);
  point3d.set_z(z);
  return point3d;
}

SpeedPoint MakeSpeedPoint(const double s, const double t, const double v,
                          const double a, const double da) {
  SpeedPoint speed_point;
  speed_point.set_s(s);
  speed_point.set_t(t);
  speed_point.set_v(v);
  speed_point.set_a(a);
  speed_point.set_da(da);
  return speed_point;
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

void uniform_slice(double start, double end, uint32_t num,
                   std::vector<double>* sliced) {
  if (!sliced || num == 0) {
    return;
  }
  const double delta = (end - start) / num;
  sliced->resize(num + 1);
  double s = start;
  for (uint32_t i = 0; i < num; ++i, s += delta) {
    sliced->at(i) = s;
  }
  sliced->at(num) = end;
}

}  // namespace util
}  // namespace common
}  // namespace apollo
