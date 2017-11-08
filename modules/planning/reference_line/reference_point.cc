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
 * @file reference_point.cc
 **/

#include "modules/planning/reference_line/reference_point.h"

#include <string>
#include <vector>

#include "modules/common/util/string_util.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::util::StrCat;

namespace {
// Minimum distance to remove duplicated points.
const double kDuplicatedPointsEpsilon = 1e-7;
}

ReferencePoint::ReferencePoint(const MapPathPoint& map_path_point,
                               const double kappa, const double dkappa,
                               const double lower_bound,
                               const double upper_bound)
    : hdmap::MapPathPoint(map_path_point),
      kappa_(kappa),
      dkappa_(dkappa),
      lower_bound_(lower_bound),
      upper_bound_(upper_bound) {}

common::PathPoint ReferencePoint::ToPathPoint(double s) const {
  common::PathPoint path_point;
  path_point.set_x(x());
  path_point.set_y(y());
  path_point.set_z(0.0);
  path_point.set_theta(heading());
  path_point.set_s(s);
  path_point.set_kappa(kappa_);
  path_point.set_dkappa(dkappa_);
  path_point.set_ddkappa(0.0);
  return path_point;
}

double ReferencePoint::kappa() const { return kappa_; }

double ReferencePoint::dkappa() const { return dkappa_; }

double ReferencePoint::lower_bound() const { return lower_bound_; }

double ReferencePoint::upper_bound() const { return upper_bound_; }

std::string ReferencePoint::DebugString() const {
  // StrCat supports 9 arguments at most.
  return StrCat(
      StrCat("{x: ", x(), ", "
             "y: ", y(), ", "
             "theta: ", heading(), ", "
             "kappa: ", kappa(), ", "),
      StrCat("dkappa: ", dkappa(), ", "
             "upper_bound: ", upper_bound(), ", "
             "lower_bound: ", lower_bound(), "}"));
}

void ReferencePoint::RemoveDuplicates(std::vector<ReferencePoint>* points) {
  CHECK_NOTNULL(points);
  int count = 0;
  const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  for (size_t i = 0; i < points->size(); ++i) {
    if (count == 0 ||
        (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = (*points)[i];
    } else {
      (*points)[count - 1].add_lane_waypoints((*points)[i].lane_waypoints());
    }
  }
  points->resize(count);
}

}  // namespace planning
}  // namespace apollo
