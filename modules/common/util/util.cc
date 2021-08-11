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

PointENU operator+(const PointENU enu, const math::Vec2d& xy) {
  PointENU point;
  point.set_x(enu.x() + xy.x());
  point.set_y(enu.y() + xy.y());
  point.set_z(enu.z());
  return point;
}

PathPoint GetWeightedAverageOfTwoPathPoints(const PathPoint& p1,
                                            const PathPoint& p2,
                                            const double w1, const double w2) {
  PathPoint p;
  p.set_x(p1.x() * w1 + p2.x() * w2);
  p.set_y(p1.y() * w1 + p2.y() * w2);
  p.set_z(p1.z() * w1 + p2.z() * w2);
  p.set_theta(p1.theta() * w1 + p2.theta() * w2);
  p.set_kappa(p1.kappa() * w1 + p2.kappa() * w2);
  p.set_dkappa(p1.dkappa() * w1 + p2.dkappa() * w2);
  p.set_ddkappa(p1.ddkappa() * w1 + p2.ddkappa() * w2);
  p.set_s(p1.s() * w1 + p2.s() * w2);
  return p;
}

}  // namespace util
}  // namespace common
}  // namespace apollo
