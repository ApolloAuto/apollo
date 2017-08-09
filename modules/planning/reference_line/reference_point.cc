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

ReferencePoint::ReferencePoint(const MapPathPoint& map_path_point,
                               const double kappa, const double dkappa,
                               const double lower_bound,
                               const double upper_bound)
    : hdmap::MapPathPoint(map_path_point),
      kappa_(kappa),
      dkappa_(dkappa),
      lower_bound_(lower_bound),
      upper_bound_(upper_bound) {}

ReferencePoint::operator PathPoint() const {
  PathPoint result;
  result.set_x(x());
  result.set_y(y());
  result.set_theta(heading());
  result.set_kappa(kappa());
  result.set_dkappa(dkappa());
  return result;
}

void ReferencePoint::set_kappa(const double kappa) { kappa_ = kappa; }

void ReferencePoint::set_dkappa(const double dkappa) { dkappa_ = dkappa; }

void ReferencePoint::set_lower_bound(const double lower_bound) {
  lower_bound_ = lower_bound;
}

void ReferencePoint::set_upper_bound(const double upper_bound) {
  upper_bound_ = upper_bound;
}

double ReferencePoint::kappa() const { return kappa_; }

double ReferencePoint::dkappa() const { return dkappa_; }

double ReferencePoint::lower_bound() const { return lower_bound_; }

double ReferencePoint::upper_bound() const { return upper_bound_; }

const std::string ReferencePoint::DebugString() const {
  // clang-format off
  return apollo::common::util::StrCat("{x: ", std::fixed, x(),
         ", " "y: ", y(), ", " "theta: ", heading(), ", " "kappa: ",
         kappa(), ", " "dkappa: ", dkappa(), ", " "upper_bound: ",
         upper_bound(), ", " "lower_bound: ", lower_bound(), "}");
  // clang-format on
}

}  // namespace planning
}  // namespace apollo
