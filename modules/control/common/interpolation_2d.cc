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

#include "modules/control/common/interpolation_2d.h"

#include <cmath>

#include "cyber/common/log.h"

namespace {

const double kDoubleEpsilon = 1.0e-6;

}  // namespace

namespace apollo {
namespace control {

bool Interpolation2D::Init(const DataType &xyz) {
  if (xyz.empty()) {
    AERROR << "empty input.";
    return false;
  }
  for (const auto &t : xyz) {
    xyz_[std::get<0>(t)][std::get<1>(t)] = std::get<2>(t);
  }
  return true;
}

double Interpolation2D::Interpolate(const KeyType &xy) const {
  double max_x = xyz_.rbegin()->first;
  double min_x = xyz_.begin()->first;
  if (xy.first >= max_x - kDoubleEpsilon) {
    return InterpolateYz(xyz_.rbegin()->second, xy.second);
  }
  if (xy.first <= min_x + kDoubleEpsilon) {
    return InterpolateYz(xyz_.begin()->second, xy.second);
  }

  auto itr_after = xyz_.lower_bound(xy.first);
  auto itr_before = itr_after;
  if (itr_before != xyz_.begin()) {
    --itr_before;
  }

  double x_before = itr_before->first;
  double z_before = InterpolateYz(itr_before->second, xy.second);
  double x_after = itr_after->first;
  double z_after = InterpolateYz(itr_after->second, xy.second);

  double x_diff_before = std::fabs(xy.first - x_before);
  double x_diff_after = std::fabs(xy.first - x_after);

  return InterpolateValue(z_before, x_diff_before, z_after, x_diff_after);
}

double Interpolation2D::InterpolateYz(const std::map<double, double> &yz_table,
                                      double y) const {
  if (yz_table.empty()) {
    AERROR << "Unable to interpolateYz because yz_table is empty.";
    return y;
  }
  double max_y = yz_table.rbegin()->first;
  double min_y = yz_table.begin()->first;
  if (y >= max_y - kDoubleEpsilon) {
    return yz_table.rbegin()->second;
  }
  if (y <= min_y + kDoubleEpsilon) {
    return yz_table.begin()->second;
  }

  auto itr_after = yz_table.lower_bound(y);
  auto itr_before = itr_after;

  if (itr_before != yz_table.begin()) {
    --itr_before;
  }

  double y_before = itr_before->first;
  double z_before = itr_before->second;
  double y_after = itr_after->first;
  double z_after = itr_after->second;

  double y_diff_before = std::fabs(y - y_before);
  double y_diff_after = std::fabs(y - y_after);

  return InterpolateValue(z_before, y_diff_before, z_after, y_diff_after);
}

double Interpolation2D::InterpolateValue(const double value_before,
                                         const double dist_before,
                                         const double value_after,
                                         const double dist_after) const {
  if (dist_before < kDoubleEpsilon) {
    return value_before;
  }
  if (dist_after < kDoubleEpsilon) {
    return value_after;
  }
  double value_gap = value_after - value_before;
  double value_buff = value_gap * dist_before / (dist_before + dist_after);
  return value_before + value_buff;
}

}  // namespace control
}  // namespace apollo
