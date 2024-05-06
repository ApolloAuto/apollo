/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file discrete_points_math.cc
 **/

#include "modules/planning/planning_base/math/discrete_points_math.h"

#include <cmath>

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

bool DiscretePointsMath::ComputePathProfile(
    const std::vector<std::pair<double, double>>& xy_points,
    std::vector<double>* headings, std::vector<double>* accumulated_s,
    std::vector<double>* kappas, std::vector<double>* dkappas) {
  CHECK_NOTNULL(headings);
  CHECK_NOTNULL(kappas);
  CHECK_NOTNULL(dkappas);
  headings->clear();
  kappas->clear();
  dkappas->clear();

  if (xy_points.size() < 2) {
    return false;
  }
  std::vector<double> dxs;
  std::vector<double> dys;
  std::vector<double> y_over_s_first_derivatives;
  std::vector<double> x_over_s_first_derivatives;
  std::vector<double> y_over_s_second_derivatives;
  std::vector<double> x_over_s_second_derivatives;

  // Get finite difference approximated dx and dy for heading and kappa
  // calculation
  std::size_t points_size = xy_points.size();
  for (std::size_t i = 0; i < points_size; ++i) {
    double x_delta = 0.0;
    double y_delta = 0.0;
    if (i == 0) {
      x_delta = (xy_points[i + 1].first - xy_points[i].first);
      y_delta = (xy_points[i + 1].second - xy_points[i].second);
    } else if (i == points_size - 1) {
      x_delta = (xy_points[i].first - xy_points[i - 1].first);
      y_delta = (xy_points[i].second - xy_points[i - 1].second);
    } else {
      x_delta = 0.5 * (xy_points[i + 1].first - xy_points[i - 1].first);
      y_delta = 0.5 * (xy_points[i + 1].second - xy_points[i - 1].second);
    }
    dxs.push_back(x_delta);
    dys.push_back(y_delta);
  }

  // Heading calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    headings->push_back(std::atan2(dys[i], dxs[i]));
  }

  // Get linear interpolated s for dkappa calculation
  double distance = 0.0;
  accumulated_s->push_back(distance);
  double fx = xy_points[0].first;
  double fy = xy_points[0].second;
  double nx = 0.0;
  double ny = 0.0;
  for (std::size_t i = 1; i < points_size; ++i) {
    nx = xy_points[i].first;
    ny = xy_points[i].second;
    double end_segment_s =
        std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
    accumulated_s->push_back(end_segment_s + distance);
    distance += end_segment_s;
    fx = nx;
    fy = ny;
  }

  // Get finite difference approximated first derivative of y and x respective
  // to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = 0.0;
    double yds = 0.0;
    if (i == 0) {
      xds = (xy_points[i + 1].first - xy_points[i].first) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i));
      yds = (xy_points[i + 1].second - xy_points[i].second) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      xds = (xy_points[i].first - xy_points[i - 1].first) /
            (accumulated_s->at(i) - accumulated_s->at(i - 1));
      yds = (xy_points[i].second - xy_points[i - 1].second) /
            (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      xds = (xy_points[i + 1].first - xy_points[i - 1].first) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      yds = (xy_points[i + 1].second - xy_points[i - 1].second) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_first_derivatives.push_back(xds);
    y_over_s_first_derivatives.push_back(yds);
  }

  // Get finite difference approximated second derivative of y and x respective
  // to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double xdds = 0.0;
    double ydds = 0.0;
    if (i == 0) {
      xdds =
          (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
          (accumulated_s->at(i + 1) - accumulated_s->at(i));
      ydds =
          (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
          (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      xdds =
          (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
          (accumulated_s->at(i) - accumulated_s->at(i - 1));
      ydds =
          (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
          (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      xdds = (x_over_s_first_derivatives[i + 1] -
              x_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      ydds = (y_over_s_first_derivatives[i + 1] -
              y_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_second_derivatives.push_back(xdds);
    y_over_s_second_derivatives.push_back(ydds);
  }

  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = x_over_s_first_derivatives[i];
    double yds = y_over_s_first_derivatives[i];
    double xdds = x_over_s_second_derivatives[i];
    double ydds = y_over_s_second_derivatives[i];
    double kappa =
        (xds * ydds - yds * xdds) /
        (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
    kappas->push_back(kappa);
  }

  // Dkappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double dkappa = 0.0;
    if (i == 0) {
      dkappa = (kappas->at(i + 1) - kappas->at(i)) /
               (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      dkappa = (kappas->at(i) - kappas->at(i - 1)) /
               (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      dkappa = (kappas->at(i + 1) - kappas->at(i - 1)) /
               (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    dkappas->push_back(dkappa);
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
