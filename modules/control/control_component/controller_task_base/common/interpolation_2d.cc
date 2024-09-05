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

#include "modules/control/control_component/controller_task_base/common/interpolation_2d.h"

#include <cmath>

#include "cyber/common/log.h"
#include "modules/control/control_component/common/control_gflags.h"

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

  if (!CheckMap()) {
    AERROR << "calibration map is not correct.";
    return false;
  }

  return true;
}

bool Interpolation2D::CheckMap() const {
  double keysize = xyz_.begin()->second.size();
  auto itr_ = xyz_.begin();
  for (; itr_ != xyz_.end(); itr_++) {
    if (FLAGS_use_calibration_dimension_equal_check) {
      if (keysize != itr_->second.size()) {
        AERROR << "calibration map dimension is not equal.";
        AERROR << "first value: " << keysize
               << ", second value: " << itr_->second.size();
        return false;
      }
    }

    int pos_count = 0;
    int nag_count = 0;
    auto inner_itr_ = itr_->second.begin();
    for (; inner_itr_ != itr_->second.end(); inner_itr_++) {
      if (inner_itr_->first > 0) {
        pos_count++;
      } else if (inner_itr_->first < 0) {
        nag_count++;
      }
    }
    if (nag_count == 0) {
      AERROR << "calibration has all pos map";
      return false;
    }
    if (pos_count == 0) {
      AERROR << "calibration has all nag map";
      return false;
    }
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
