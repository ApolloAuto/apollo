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
 * @file piecewise_poly_speed_curve.h
 **/

#include "modules/planning/tasks/optimizers/poly_vt_speed/piecewise_poly_speed_curve.h"

#include <cmath>

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

namespace {
constexpr double kMinTimeRes = 0.0001;
}

PiecewisePolySpeedCurve::PiecewisePolySpeedCurve(
    const common::SpeedPoint& init_point,
    const common::SpeedPoint& connect_point, const double end_t)
    : v_curve_(QuarticPolynomialCurve1d().FitWithEndPointSecondOrder(
          init_point.v(), init_point.a(), connect_point.v(), connect_point.a(),
          connect_point.da(), connect_point.t())),
      extended_v_curve_(QuarticPolynomialCurve1d().FitWithEndPointSecondOrder(
          connect_point.v(), connect_point.a(), connect_point.v(),
          connect_point.a(), 0.0,
          std::fmax(kMinTimeRes, end_t - connect_point.t()))),
      init_point_(init_point),
      connect_point_(connect_point),
      end_point_(connect_point),
      param_t_(end_t) {
  s_curve_.IntegratedFromQuarticCurve(v_curve_, 0.0);
  a_curve_.DerivedFromQuarticCurve(v_curve_);
  connect_point_.set_s(s_curve_.Evaluate(0, connect_point.t()));
  extended_s_curve_.IntegratedFromQuarticCurve(extended_v_curve_,
                                               connect_point_.s());
  extended_a_curve_.DerivedFromQuarticCurve(extended_v_curve_);
  end_point_.set_t(end_t);
  end_point_.set_s(extended_s_curve_.Evaluate(0, end_point_.t()));
}

PiecewisePolySpeedCurve::PiecewisePolySpeedCurve(
    const common::SpeedPoint& init_point,
    const common::SpeedPoint& connect_point,
    const common::SpeedPoint& end_point)
    : v_curve_(QuarticPolynomialCurve1d().FitWithEndPointSecondOrder(
          init_point.v(), init_point.a(), connect_point.v(), connect_point.a(),
          connect_point.da(), connect_point.t())),
      extended_v_curve_(QuarticPolynomialCurve1d().FitWithEndPointSecondOrder(
          connect_point.v(), connect_point.a(), end_point.v(), end_point.a(),
          end_point.da(),
          std::fmax(kMinTimeRes, end_point.t() - connect_point.t()))),
      init_point_(init_point),
      connect_point_(connect_point),
      end_point_(end_point),
      param_t_(end_point.t()) {
  s_curve_.IntegratedFromQuarticCurve(v_curve_, 0.0);
  a_curve_.DerivedFromQuarticCurve(v_curve_);
  connect_point_.set_s(s_curve_.Evaluate(0, connect_point.t()));
  extended_s_curve_.IntegratedFromQuarticCurve(extended_v_curve_,
                                               connect_point_.s());
  end_point_.set_s(extended_s_curve_.Evaluate(0, end_point.t()));
  extended_a_curve_.DerivedFromQuarticCurve(extended_v_curve_);
}

PiecewisePolySpeedCurve::PiecewisePolySpeedCurve(
    const double init_v, const double init_a, const double connect_v,
    const double connect_a, const double connect_j, const double connect_t,
    const double end_t)
    : v_curve_(QuarticPolynomialCurve1d().FitWithEndPointSecondOrder(
          init_v, init_a, connect_v, connect_a, connect_j, connect_t)),
      extended_v_curve_(QuarticPolynomialCurve1d().FitWithEndPointSecondOrder(
          connect_v, connect_a, connect_v, connect_a, 0.0,
          std::fmax(kMinTimeRes, end_t - connect_t))),
      param_t_(end_t) {
  init_point_.set_s(0.0);
  init_point_.set_t(0.0);
  init_point_.set_v(init_v);
  init_point_.set_a(init_a);
  init_point_.set_da(0.0);

  connect_point_.set_s(0.0);
  connect_point_.set_t(connect_t);
  connect_point_.set_v(connect_v);
  connect_point_.set_a(connect_a);
  connect_point_.set_da(connect_j);

  end_point_.set_s(0.0);
  end_point_.set_t(end_t);
  end_point_.set_v(connect_v);
  end_point_.set_a(connect_a);
  end_point_.set_da(0.0);
  s_curve_.IntegratedFromQuarticCurve(v_curve_, 0.0);
  a_curve_.DerivedFromQuarticCurve(v_curve_);
  connect_point_.set_s(s_curve_.Evaluate(0, connect_t));
  extended_s_curve_.IntegratedFromQuarticCurve(extended_v_curve_,
                                               connect_point_.s());
  extended_a_curve_.DerivedFromQuarticCurve(extended_v_curve_);
  end_point_.set_s(extended_s_curve_.Evaluate(0, end_point_.t()));
}

void PiecewisePolySpeedCurve::InitWithStopPoint(
    const common::SpeedPoint& init_point, const common::SpeedPoint& stop_point,
    const double end_t) {
  init_point_ = init_point;
  connect_point_ = stop_point;
  param_t_ = end_t;
  s_curve_.SetParam(init_point.s(), init_point.v(), init_point.a(),
                    stop_point.s(), stop_point.v(), stop_point.a(),
                    stop_point.t());
  extended_s_curve_.SetParam(stop_point.s(), stop_point.v(), stop_point.a(),
                             stop_point.s(), 0.0, 0.0,
                             std::fmax(kMinTimeRes, end_t - stop_point.t()));
  v_curve_.DerivedFromQuinticCurve(s_curve_);
  extended_v_curve_.DerivedFromQuinticCurve(extended_s_curve_);
  a_curve_.DerivedFromQuarticCurve(v_curve_);
  extended_a_curve_.DerivedFromQuarticCurve(extended_v_curve_);
}

void PiecewisePolySpeedCurve::Evaluate(
    const double t, common::SpeedPoint* const speed_point) const {
  if (t <= connect_point_.t()) {
    speed_point->set_s(std::fmax(0.0, s_curve_.Evaluate(0, t)));
    speed_point->set_t(t);
    speed_point->set_v(std::fmax(0.0, v_curve_.Evaluate(0, t)));
    speed_point->set_a(a_curve_.Evaluate(0, t));
    speed_point->set_da(v_curve_.Evaluate(2, t));
  } else {
    double delt_t = t - connect_point_.t();
    speed_point->set_s(std::fmax(0.0, extended_s_curve_.Evaluate(0, delt_t)));
    speed_point->set_t(t);
    speed_point->set_v(std::fmax(0.0, extended_v_curve_.Evaluate(0, delt_t)));
    speed_point->set_a(extended_a_curve_.Evaluate(0, delt_t));
    speed_point->set_da(extended_v_curve_.Evaluate(2, delt_t));
  }
}

void PiecewisePolySpeedCurve::SampleSpeedPoints(
    const size_t num_points,
    std::vector<common::SpeedPoint>* const speed_points) const {
  if (num_points == 0) {
    return;
  }
  speed_points->clear();
  speed_points->reserve(num_points);
  double pre_s = 0.0;
  if (num_points <= 1) {
    speed_points->emplace_back(init_point_);
  } else {
    double unit_t = param_t_ / (static_cast<double>(num_points) - 1);
    for (size_t i = 0; i < num_points; ++i) {
      speed_points->emplace_back();
      Evaluate(static_cast<double>(i) * unit_t, &speed_points->back());
      speed_points->back().set_s(std::fmax(pre_s, speed_points->back().s()));
      pre_s = speed_points->back().s();
    }
  }
}

void PiecewisePolySpeedCurve::SampleSpeedPointsWithTime(
    const double unit_t,
    std::vector<common::SpeedPoint>* const speed_points) const {
  double t_res = std::fmax(unit_t, kMinTimeRes);
  speed_points->clear();
  double pre_s = 0.0;
  for (double t = 0; t < param_t_ + t_res; t += t_res) {
    speed_points->emplace_back();
    Evaluate(t, &speed_points->back());
    speed_points->back().set_s(std::fmax(pre_s, speed_points->back().s()));
    pre_s = speed_points->back().s();
  }
}

double PiecewisePolySpeedCurve::param_t() const { return param_t_; }

const common::SpeedPoint& PiecewisePolySpeedCurve::connect_point() const {
  return connect_point_;
}

const common::SpeedPoint& PiecewisePolySpeedCurve::init_point() const {
  return init_point_;
}

const common::SpeedPoint& PiecewisePolySpeedCurve::end_point() const {
  return end_point_;
}

}  // namespace planning
}  // namespace apollo
