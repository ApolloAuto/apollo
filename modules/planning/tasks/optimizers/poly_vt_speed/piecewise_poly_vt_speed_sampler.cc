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
 * file : piecewise_poly_vt_speed_sampler.h
 */

#include "modules/planning/tasks/optimizers/poly_vt_speed/piecewise_poly_vt_speed_sampler.h"

#include <cmath>
#include <string>

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

namespace {
constexpr double kMinTimeRange = 0.0001;
}

using apollo::common::SpeedPoint;
using apollo::common::TrajectoryPoint;

PiecewisePolyVTSpeedSampler::PiecewisePolyVTSpeedSampler(
    const PolyVTSpeedConfig& config)
    : config_(config) {}

bool PiecewisePolyVTSpeedSampler::Sample(
    const TrajectoryPoint& init_point, const double path_length,
    std::vector<PiecewisePolySpeedProfile>* const samples) const {
  CHECK_NOTNULL(samples);
  samples->clear();

  if (config_.num_t_layers() < 1) {
    const std::string msg = "Config's number of time layers less than 1";
    AERROR << msg;
    return false;
  }

  if (config_.total_time() <= kMinTimeRange) {
    const std::string msg = "Config's total time is less than 0.0001";
    AERROR << msg;
    return false;
  }

  const double unit_t = config_.total_time() / config_.num_t_layers();

  // extract init speed point status
  SpeedPoint init_speed_point;
  init_speed_point.set_s(0.0);
  init_speed_point.set_t(0.0);
  init_speed_point.set_v(init_point.v());
  init_speed_point.set_a(init_point.a());
  init_speed_point.set_da(0.0);

  const double unit_v = config_.sampling_unit_v();

  for (int i = 1; i <= config_.num_t_layers(); ++i) {
    double layer_t = i * unit_t;
    double upper_v =
        std::fmin(config_.online_max_speed(),
                  init_point.v() + config_.online_max_acc() * layer_t);
    double lower_v =
        std::fmax(0.0, init_point.v() + config_.online_max_dec() * layer_t);
    SpeedPoint connect_point;
    connect_point.set_t(layer_t);

    for (double sampling_v = lower_v; sampling_v < upper_v + unit_v;
         sampling_v += unit_v) {
      // make sure sampling v is not too small when init speed point v
      if (i > 1 && init_speed_point.v() < 0.1 && sampling_v < 1e-3) {
        continue;
      }
      connect_point.set_v(sampling_v);
      SpeedPoint end_point;
      end_point.set_s(0.0);
      end_point.set_t(config_.total_time());
      end_point.set_v(sampling_v);
      end_point.set_a(0.0);
      end_point.set_da(0.0);
      PiecewisePolySpeedCurve curve(init_speed_point, connect_point, end_point);
      samples->emplace_back(curve, config_.num_evaluated_points());

      // resampling
      if (sampling_v > init_point.v() - unit_v && i != config_.num_t_layers()) {
        double dec_lower_v =
            std::fmax(0.0, sampling_v +
                               0.5 * config_.online_max_dec() *
                                   (config_.num_t_layers() - i));
        for (double end_v = dec_lower_v; end_v < sampling_v; end_v += unit_v) {
          end_point.set_v(end_v);
          PiecewisePolySpeedCurve curve(init_speed_point, connect_point,
                                        end_point);
          samples->emplace_back(curve, config_.num_evaluated_points());
        }
      }
    }
  }
  return !samples->empty();
}

}  // namespace planning
}  // namespace apollo
