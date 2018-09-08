/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file waypoint_sampler.cc
 **/

#include "modules/planning/toolkits/optimizers/road_graph/waypoint_sampler.h"

#include <algorithm>
#include <utility>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/path/frenet_frame_path.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::SLPoint;
using apollo::common::Status;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::util::MakeSLPoint;

WaypointSampler::WaypointSampler(
    const WaypointSamplerConfig &config,
    const ReferenceLineInfo &reference_line_info,
    const common::SLPoint &init_sl_point,
    const common::FrenetFramePoint &init_frenet_frame_point)
    : config_(config),
      reference_line_info_(reference_line_info),
      reference_line_(reference_line_info.reference_line()),
      init_sl_point_(init_sl_point),
      init_frenet_frame_point_(init_frenet_frame_point) {}

bool WaypointSampler::SamplePathWaypoints(
    const common::TrajectoryPoint &init_point,
    std::vector<std::vector<common::SLPoint>> *const points) {
  CHECK_NOTNULL(points);

  const float kMinSampleDistance = 40.0;
  const float total_length = std::fmin(
      init_sl_point_.s() + std::fmax(init_point.v() * 8.0, kMinSampleDistance),
      reference_line_.Length());
  const auto &vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();
  const float half_adc_width = vehicle_config.vehicle_param().width() / 2.0;
  const size_t num_sample_per_level =
      FLAGS_use_navigation_mode ? config_.navigator_sample_num_each_level()
                                : config_.sample_points_num_each_level();

  const bool has_sidepass = HasSidepass();

  constexpr float kSamplePointLookForwardTime = 4.0;
  const float step_length =
      common::math::Clamp(init_point.v() * kSamplePointLookForwardTime,
                          config_.step_length_min(), config_.step_length_max());

  const float level_distance =
      (init_point.v() > FLAGS_max_stop_speed) ? step_length : step_length / 2.0;
  float accumulated_s = init_sl_point_.s();
  float prev_s = accumulated_s;

  auto *status = GetPlanningStatus();
  if (!status->has_pull_over() && status->pull_over().in_pull_over()) {
    status->mutable_pull_over()->set_status(PullOverStatus::IN_OPERATION);
    const auto &start_point = status->pull_over().start_point();
    SLPoint start_point_sl;
    if (!reference_line_.XYToSL(start_point, &start_point_sl)) {
      AERROR << "Fail to change xy to sl.";
      return false;
    }

    if (init_sl_point_.s() > start_point_sl.s()) {
      const auto &stop_point = status->pull_over().stop_point();
      SLPoint stop_point_sl;
      if (!reference_line_.XYToSL(stop_point, &stop_point_sl)) {
        AERROR << "Fail to change xy to sl.";
        return false;
      }
      std::vector<common::SLPoint> level_points(1, stop_point_sl);
      points->emplace_back(level_points);
      return true;
    }
  }

  for (std::size_t i = 0; accumulated_s < total_length; ++i) {
    accumulated_s += level_distance;
    if (accumulated_s + level_distance / 2.0 > total_length) {
      accumulated_s = total_length;
    }
    const float s = std::fmin(accumulated_s, total_length);
    constexpr float kMinAllowedSampleStep = 1.0;
    if (std::fabs(s - prev_s) < kMinAllowedSampleStep) {
      continue;
    }
    prev_s = s;

    double left_width = 0.0;
    double right_width = 0.0;
    reference_line_.GetLaneWidth(s, &left_width, &right_width);

    constexpr float kBoundaryBuff = 0.20;
    const float eff_right_width = right_width - half_adc_width - kBoundaryBuff;
    const float eff_left_width = left_width - half_adc_width - kBoundaryBuff;

    // the heuristic shift of L for lane change scenarios
    const double delta_dl = 1.2 / 20.0;
    const double kChangeLaneDeltaL = common::math::Clamp(
        level_distance * (std::fabs(init_frenet_frame_point_.dl()) + delta_dl),
        1.2, 3.5);

    float kDefaultUnitL = kChangeLaneDeltaL / (num_sample_per_level - 1);
    if (reference_line_info_.IsChangeLanePath() &&
        !reference_line_info_.IsSafeToChangeLane()) {
      kDefaultUnitL = 1.0;
    }
    const float sample_l_range = kDefaultUnitL * (num_sample_per_level - 1);
    float sample_right_boundary = -eff_right_width;
    float sample_left_boundary = eff_left_width;

    const float kLargeDeviationL = 1.75;
    if (reference_line_info_.IsChangeLanePath() ||
        std::fabs(init_sl_point_.l()) > kLargeDeviationL) {
      sample_right_boundary = std::fmin(-eff_right_width, init_sl_point_.l());
      sample_left_boundary = std::fmax(eff_left_width, init_sl_point_.l());

      if (init_sl_point_.l() > eff_left_width) {
        sample_right_boundary = std::fmax(sample_right_boundary,
                                          init_sl_point_.l() - sample_l_range);
      }
      if (init_sl_point_.l() < eff_right_width) {
        sample_left_boundary = std::fmin(sample_left_boundary,
                                         init_sl_point_.l() + sample_l_range);
      }
    }

    std::vector<float> sample_l;
    if (reference_line_info_.IsChangeLanePath() &&
        !reference_line_info_.IsSafeToChangeLane()) {
      sample_l.push_back(reference_line_info_.OffsetToOtherReferenceLine());
    } else if (has_sidepass) {
      // currently only left nudge is supported. Need road hard boundary for
      // both sides
      switch (sidepass_.type()) {
        case ObjectSidePass::LEFT: {
          sample_l.push_back(eff_left_width + config_.sidepass_distance());
          break;
        }
        case ObjectSidePass::RIGHT: {
          sample_l.push_back(-eff_right_width - config_.sidepass_distance());
          break;
        }
        default:
          break;
      }
    } else {
      common::util::uniform_slice(sample_right_boundary, sample_left_boundary,
                                  num_sample_per_level - 1, &sample_l);
    }
    std::vector<common::SLPoint> level_points;
    planning_internal::SampleLayerDebug sample_layer_debug;
    for (size_t j = 0; j < sample_l.size(); ++j) {
      common::SLPoint sl = common::util::MakeSLPoint(s, sample_l[j]);
      sample_layer_debug.add_sl_point()->CopyFrom(sl);
      level_points.push_back(std::move(sl));
    }
    if (!reference_line_info_.IsChangeLanePath() && has_sidepass) {
      auto sl_zero = common::util::MakeSLPoint(s, 0.0);
      sample_layer_debug.add_sl_point()->CopyFrom(sl_zero);
      level_points.push_back(std::move(sl_zero));
    }

    if (!level_points.empty()) {
      planning_debug_->mutable_planning_data()
          ->mutable_dp_poly_graph()
          ->add_sample_layer()
          ->CopyFrom(sample_layer_debug);
      points->emplace_back(level_points);
    }
  }
  return true;
}

bool WaypointSampler::HasSidepass() {
  const auto &path_decision = reference_line_info_.path_decision();
  for (const auto &obstacle : path_decision.path_obstacles().Items()) {
    if (obstacle->LateralDecision().has_sidepass()) {
      sidepass_ = obstacle->LateralDecision().sidepass();
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
