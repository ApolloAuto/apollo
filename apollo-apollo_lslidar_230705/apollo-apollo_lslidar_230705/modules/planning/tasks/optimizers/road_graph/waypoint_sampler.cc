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

#include "modules/planning/tasks/optimizers/road_graph/waypoint_sampler.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/path/frenet_frame_path.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"

namespace apollo {
namespace planning {

void WaypointSampler::Init(
    const ReferenceLineInfo *reference_line_info,
    const common::SLPoint &init_sl_point,
    const common::FrenetFramePoint &init_frenet_frame_point) {
  reference_line_info_ = reference_line_info;
  init_sl_point_ = init_sl_point;
  init_frenet_frame_point_ = init_frenet_frame_point;
}

bool WaypointSampler::SamplePathWaypoints(
    const common::TrajectoryPoint &init_point,
    std::vector<std::vector<common::SLPoint>> *const points) {
  CHECK_NOTNULL(points);
  points->clear();
  points->insert(points->begin(), std::vector<common::SLPoint>{init_sl_point_});

  const double kMinSampleDistance = 40.0;
  const double total_length = std::fmin(
      init_sl_point_.s() + std::fmax(init_point.v() * 8.0, kMinSampleDistance),
      reference_line_info_->reference_line().Length());
  const auto &vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  const double half_adc_width = vehicle_config.vehicle_param().width() / 2.0;
  const double num_sample_per_level =
      FLAGS_use_navigation_mode ? config_.navigator_sample_num_each_level()
                                : config_.sample_points_num_each_level();

  static constexpr double kSamplePointLookForwardTime = 4.0;
  const double level_distance =
      common::math::Clamp(init_point.v() * kSamplePointLookForwardTime,
                          config_.step_length_min(), config_.step_length_max());

  double accumulated_s = init_sl_point_.s();
  double prev_s = accumulated_s;

  static constexpr size_t kNumLevel = 3;
  for (size_t i = 0; i < kNumLevel && accumulated_s < total_length; ++i) {
    accumulated_s += level_distance;
    if (accumulated_s + level_distance / 2.0 > total_length) {
      accumulated_s = total_length;
    }
    const double s = std::fmin(accumulated_s, total_length);
    static constexpr double kMinAllowedSampleStep = 1.0;
    if (std::fabs(s - prev_s) < kMinAllowedSampleStep) {
      continue;
    }
    prev_s = s;

    double left_width = 0.0;
    double right_width = 0.0;
    reference_line_info_->reference_line().GetLaneWidth(s, &left_width,
                                                        &right_width);

    static constexpr double kBoundaryBuff = 0.20;
    const double eff_right_width = right_width - half_adc_width - kBoundaryBuff;
    const double eff_left_width = left_width - half_adc_width - kBoundaryBuff;

    // the heuristic shift of L for lane change scenarios
    const double delta_dl = 1.2 / 20.0;
    const double kChangeLaneDeltaL = common::math::Clamp(
        level_distance * (std::fabs(init_frenet_frame_point_.dl()) + delta_dl),
        1.2, 3.5);

    double kDefaultUnitL = kChangeLaneDeltaL / (num_sample_per_level - 1);
    if (reference_line_info_->IsChangeLanePath() &&
        LaneChangeDecider::IsClearToChangeLane(reference_line_info_)) {
      kDefaultUnitL = 1.0;
    }
    const double sample_l_range = kDefaultUnitL * (num_sample_per_level - 1);
    double sample_right_boundary = -eff_right_width;
    double sample_left_boundary = eff_left_width;

    static constexpr double kLargeDeviationL = 1.75;
    static constexpr double kTwentyMilesPerHour = 8.94;
    if (reference_line_info_->IsChangeLanePath() ||
        std::fabs(init_sl_point_.l()) > kLargeDeviationL) {
      if (injector_->ego_info()->start_point().v() > kTwentyMilesPerHour) {
        sample_right_boundary = std::fmin(-eff_right_width, init_sl_point_.l());
        sample_left_boundary = std::fmax(eff_left_width, init_sl_point_.l());

        if (init_sl_point_.l() > eff_left_width) {
          sample_right_boundary = std::fmax(
              sample_right_boundary, init_sl_point_.l() - sample_l_range);
        }
        if (init_sl_point_.l() < eff_right_width) {
          sample_left_boundary = std::fmin(sample_left_boundary,
                                           init_sl_point_.l() + sample_l_range);
        }
      }
    }

    std::vector<double> sample_l;
    if (reference_line_info_->IsChangeLanePath() &&
        LaneChangeDecider::IsClearToChangeLane(reference_line_info_)) {
      sample_l.push_back(reference_line_info_->OffsetToOtherReferenceLine());
    } else {
      common::util::uniform_slice(
          sample_right_boundary, sample_left_boundary,
          static_cast<uint32_t>(num_sample_per_level - 1), &sample_l);
    }
    std::vector<common::SLPoint> level_points;
    planning_internal::SampleLayerDebug sample_layer_debug;
    for (size_t j = 0; j < sample_l.size(); ++j) {
      common::SLPoint sl =
          common::util::PointFactory::ToSLPoint(s, sample_l[j]);
      sample_layer_debug.add_sl_point()->CopyFrom(sl);
      level_points.push_back(std::move(sl));
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

}  // namespace planning
}  // namespace apollo
