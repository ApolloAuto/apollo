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
 * @file speed_data.cc
 **/

#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/planning_gflags.h"

#include <algorithm>
#include <sstream>

#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

namespace {
bool speed_time_comp(const double t, const SpeedPoint& speed_point) {
  return t < speed_point.t();
}
}

SpeedData::SpeedData(std::vector<SpeedPoint> speed_points) {
  speed_vector_ = std::move(speed_points);
}

std::vector<SpeedPoint>* SpeedData::mutable_speed_vector() {
  return &speed_vector_;
}

const std::vector<SpeedPoint>& SpeedData::speed_vector() const {
  return speed_vector_;
}

void SpeedData::set_speed_vector(const std::vector<SpeedPoint>& speed_points) {
  speed_vector_ = std::move(speed_points);
}

bool SpeedData::get_speed_point_with_time(const double t,
                                          SpeedPoint* const speed_point) const {
  if (speed_vector_.size() < 2) {
    return false;
  }
  std::uint32_t index = find_index(t);
  if (Double::compare(t, speed_vector_[index].t()) < 0 ||
      index + 1 >= speed_vector_.size()) {
    return false;
  }

  // index index + 1
  double weight = 0.0;
  if (Double::compare(speed_vector_[index + 1].t(), speed_vector_[index].t()) >
      0) {
    weight = (t - speed_vector_[index].t()) /
             (speed_vector_[index + 1].t() - speed_vector_[index].t());
  }

  *speed_point =
      interpolate(speed_vector_[index], speed_vector_[index + 1], weight);
  return true;
}

double SpeedData::total_time() const {
  if (speed_vector_.empty()) {
    return 0.0;
  }
  return speed_vector_.back().t() - speed_vector_.front().t();
}

std::string SpeedData::DebugString() const {
  std::ostringstream sout;
  sout << "[" << std::endl;
  for (int i = 0; i < static_cast<int>(speed_vector_.size()) &&
         i < FLAGS_trajectory_point_num_for_debug; ++i) {
    if (i > 0) {
      sout << "," << std::endl;
    }
    sout << speed_vector_[i].DebugString();
  }
  sout << "]" << std::endl;
  sout.flush();
  return sout.str();
}

std::uint32_t SpeedData::find_index(const double t) const {
  auto upper_bound = std::upper_bound(speed_vector_.begin() + 1,
                                      speed_vector_.end(), t, speed_time_comp);
  return std::min(
             static_cast<std::uint32_t>(speed_vector_.size() - 1),
             static_cast<std::uint32_t>(upper_bound - speed_vector_.begin())) -
         1;
}

SpeedPoint SpeedData::interpolate(const SpeedPoint& left,
                                  const SpeedPoint& right,
                                  const double weight) const {
  double s = (1 - weight) * left.s() + weight * right.s();
  double t = (1 - weight) * left.t() + weight * right.t();
  double v = (1 - weight) * left.v() + weight * right.v();
  double a = (1 - weight) * left.a() + weight * right.a();
  double da = (1 - weight) * left.da() + weight * right.da();
  SpeedPoint speed_point;
  speed_point.set_s(s);
  speed_point.set_t(t);
  speed_point.set_v(v);
  speed_point.set_a(a);
  speed_point.set_da(da);

  return speed_point;
}

}  // namespace planning
}  // namespace apollo
