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
 * @file
 **/

#include "modules/planning/common/speed/speed_data.h"

#include <algorithm>
#include <utility>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::SpeedPoint;

SpeedData::SpeedData(std::vector<SpeedPoint> speed_points)
    : std::vector<SpeedPoint>(std::move(speed_points)) {
  std::sort(begin(), end(), [](const SpeedPoint& p1, const SpeedPoint& p2) {
    return p1.t() < p2.t();
  });
}

void SpeedData::AppendSpeedPoint(const double s, const double time,
                                 const double v, const double a,
                                 const double da) {
  if (!empty()) {
    ACHECK(back().t() < time);
  }
  push_back(common::util::PointFactory::ToSpeedPoint(s, time, v, a, da));
}

bool SpeedData::EvaluateByTime(const double t,
                               common::SpeedPoint* const speed_point) const {
  if (size() < 2) {
    return false;
  }
  if (!(front().t() < t + 1.0e-6 && t - 1.0e-6 < back().t())) {
    return false;
  }

  auto comp = [](const common::SpeedPoint& sp, const double t) {
    return sp.t() < t;
  };

  auto it_lower = std::lower_bound(begin(), end(), t, comp);
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    double t0 = p0.t();
    double t1 = p1.t();

    speed_point->Clear();
    speed_point->set_s(common::math::lerp(p0.s(), t0, p1.s(), t1, t));
    speed_point->set_t(t);
    if (p0.has_v() && p1.has_v()) {
      speed_point->set_v(common::math::lerp(p0.v(), t0, p1.v(), t1, t));
    }
    if (p0.has_a() && p1.has_a()) {
      speed_point->set_a(common::math::lerp(p0.a(), t0, p1.a(), t1, t));
    }
    if (p0.has_da() && p1.has_da()) {
      speed_point->set_da(common::math::lerp(p0.da(), t0, p1.da(), t1, t));
    }
  }
  return true;
}

bool SpeedData::EvaluateByS(const double s,
                            common::SpeedPoint* const speed_point) const {
  if (size() < 2) {
    return false;
  }
  if (!(front().s() < s + 1.0e-6 && s - 1.0e-6 < back().s())) {
    return false;
  }

  auto comp = [](const common::SpeedPoint& sp, const double s) {
    return sp.s() < s;
  };

  auto it_lower = std::lower_bound(begin(), end(), s, comp);
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    double s0 = p0.s();
    double s1 = p1.s();

    speed_point->Clear();
    speed_point->set_s(s);
    speed_point->set_t(common::math::lerp(p0.t(), s0, p1.t(), s1, s));
    if (p0.has_v() && p1.has_v()) {
      speed_point->set_v(common::math::lerp(p0.v(), s0, p1.v(), s1, s));
    }
    if (p0.has_a() && p1.has_a()) {
      speed_point->set_a(common::math::lerp(p0.a(), s0, p1.a(), s1, s));
    }
    if (p0.has_da() && p1.has_da()) {
      speed_point->set_da(common::math::lerp(p0.da(), s0, p1.da(), s1, s));
    }
  }
  return true;
}

double SpeedData::TotalTime() const {
  if (empty()) {
    return 0.0;
  }
  return back().t() - front().t();
}

double SpeedData::TotalLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().s() - front().s();
}

std::string SpeedData::DebugString() const {
  const auto limit = std::min(
      size(), static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));
  return absl::StrCat(
      "[\n",
      absl::StrJoin(begin(), begin() + limit, ",\n",
                    apollo::common::util::DebugStringFormatter()),
      "]\n");
}

}  // namespace planning
}  // namespace apollo
