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
 * @file: poly_vt_speed_optimizer.cc
 */
#include "modules/planning/tasks/optimizers/poly_vt_speed/poly_vt_speed_optimizer.h"

#include <string>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/util/file.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning/tasks/optimizers/poly_vt_speed/piecewise_poly_vt_speed_sampler.h"
#include "modules/planning/tasks/optimizers/st_graph/speed_limit_decider.h"
#include "modules/planning/tasks/optimizers/st_graph/st_boundary_mapper.h"
#include "modules/planning/tuning/autotuning_raw_feature_generator.h"
#include "modules/planning/tuning/speed_model/autotuning_speed_feature_builder.h"
#include "modules/planning/tuning/speed_model/autotuning_speed_mlp_model.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::planning_internal::StGraphBoundaryDebug;
using apollo::planning_internal::STGraphDebug;

PolyVTSpeedOptimizer::PolyVTSpeedOptimizer(const TaskConfig& config)
    : Task(config) {
  CHECK(config_.has_poly_vt_speed_config());
  SetName("PolyVTSpeedOptimizer");
  st_boundary_config_ = config_.poly_vt_speed_config().st_boundary_config();
}

apollo::common::Status PolyVTSpeedOptimizer::Execute(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  if (frame == nullptr) {
    AERROR << "Frame info is empty!";
    return Status(ErrorCode::PLANNING_ERROR, "No Frame info");
  }

  if (reference_line_info == nullptr) {
    AERROR << "Reference line info is empty!";
    return Status(ErrorCode::PLANNING_ERROR, "No reference line info!");
  }

  const auto& poly_vt_config = config_.poly_vt_speed_config();
  // extract infos
  const SLBoundary& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  const PathData& path_data = reference_line_info->path_data();
  const TrajectoryPoint& init_point = frame->PlanningStartPoint();
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  PathDecision* path_decision = reference_line_info->path_decision();
  SpeedData* speed_data = reference_line_info->mutable_speed_data();

  reference_line_info_ = reference_line_info;
  if (reference_line_info->ReachedDestination()) {
    return Status::OK();
  }

  if (path_data.discretized_path().empty()) {
    std::string msg("Empty path data");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  StBoundaryMapper boundary_mapper(
      adc_sl_boundary, st_boundary_config_, reference_line, path_data,
      poly_vt_config.total_s(), poly_vt_config.total_time(),
      reference_line_info->IsChangeLanePath());

  for (const auto* obstacle : path_decision->obstacles().Items()) {
    DCHECK(obstacle->HasLongitudinalDecision());
  }
  // step 1 : get boundaries
  path_decision->EraseStBoundaries();
  if (boundary_mapper.CreateStBoundary(path_decision).code() ==
      ErrorCode::PLANNING_ERROR) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Mapping obstacle for qp st speed optimizer failed!");
  }

  for (const auto* obstacle : path_decision->obstacles().Items()) {
    auto id = obstacle->Id();
    auto* mutable_obstacle = path_decision->Find(id);

    if (!obstacle->st_boundary().IsEmpty()) {
      mutable_obstacle->SetBlockingObstacle(true);
    } else {
      path_decision->SetStBoundary(
          id, path_decision->Find(id)->reference_line_st_boundary());
    }
  }

  SpeedLimitDecider speed_limit_decider(adc_sl_boundary, st_boundary_config_,
                                        reference_line, path_data);
  SpeedLimit speed_limits;
  if (speed_limit_decider.GetSpeedLimits(path_decision->obstacles(),
                                         &speed_limits) != Status::OK()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "GetSpeedLimits for qp st speed optimizer failed!");
  }

  // step 2 : sampling speed profile
  PiecewisePolyVTSpeedSampler sampler(poly_vt_config);
  std::vector<PiecewisePolySpeedProfile> sampled_speed_profile;
  sampler.Sample(init_point, poly_vt_config.total_s(), &sampled_speed_profile);
  // step 3 : autotuning feature generator, feature builder as well as speed
  // model
  AutotuningRawFeatureGenerator feature_generator(
      poly_vt_config.total_time(), poly_vt_config.num_evaluated_points(),
      *reference_line_info, *frame, speed_limits);
  AutotuningSpeedFeatureBuilder feature_builder;
  AutotuningSpeedMLPModel autotuning_speed_model;

  // step 4 : Evaluation and find optimal speed profile

  PiecewisePolySpeedProfile optimal_profile;
  bool has_valid_speed_profile = false;
  for (auto& speed_profile : sampled_speed_profile) {
    autotuning::TrajectoryRawFeature raw_feature;
    feature_generator.EvaluateSpeedProfile(speed_profile.eval_points(),
                                           &raw_feature);
    autotuning::TrajectoryFeature input_feature;
    feature_builder.BuildFeature(raw_feature, &input_feature);
    double cost = autotuning_speed_model.Evaluate(input_feature);
    speed_profile.set_cost(cost);
    speed_profile.set_collision(false);
    if ((!speed_profile.collision()) &&
        speed_profile.cost() < optimal_profile.cost()) {
      optimal_profile = speed_profile;
    }
  }

  if (!has_valid_speed_profile) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "No valid non-collision speed profile.");
  }

  // step 5 : write back
  constexpr double delta_t = 0.1;
  optimal_profile.GeneratePointsByTime(delta_t);
  *speed_data = SpeedData(optimal_profile.eval_points());

  return Status::OK();
}

void PolyVTSpeedOptimizer::RecordDebugInfo(const SpeedData& speed_data) {
  if (reference_line_info_ == nullptr) {
    return;
  }
  auto* debug = reference_line_info_->mutable_debug();
  auto ptr_speed_plan = debug->mutable_planning_data()->add_speed_plan();
  ptr_speed_plan->set_name(Name());
  ptr_speed_plan->mutable_speed_point()->CopyFrom(
      {speed_data.begin(), speed_data.end()});
}

void PolyVTSpeedOptimizer::RecordSTGraphDebug(
    const StGraphData& st_graph_data, STGraphDebug* st_graph_debug) const {
  if (!FLAGS_enable_record_debug || !st_graph_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }

  st_graph_debug->set_name(Name());
  for (const auto& boundary : st_graph_data.st_boundaries()) {
    auto boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name(boundary->id());
    switch (boundary->boundary_type()) {
      case StBoundary::BoundaryType::FOLLOW:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_FOLLOW);
        break;
      case StBoundary::BoundaryType::OVERTAKE:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_OVERTAKE);
        break;
      case StBoundary::BoundaryType::STOP:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_STOP);
        break;
      case StBoundary::BoundaryType::UNKNOWN:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);
        break;
      case StBoundary::BoundaryType::YIELD:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_YIELD);
        break;
      case StBoundary::BoundaryType::KEEP_CLEAR:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_KEEP_CLEAR);
        break;
    }

    for (const auto& point : boundary->points()) {
      auto point_debug = boundary_debug->add_point();
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }
  }

  for (const auto& point : st_graph_data.speed_limit().speed_limit_points()) {
    common::SpeedPoint speed_point;
    speed_point.set_s(point.first);
    speed_point.set_v(point.second);
    st_graph_debug->add_speed_limit()->CopyFrom(speed_point);
  }

  if (reference_line_info_ == nullptr) {
    return;
  }
  const auto& speed_data = reference_line_info_->speed_data();
  st_graph_debug->mutable_speed_profile()->CopyFrom(
      {speed_data.begin(), speed_data.end()});
}
}  // namespace planning
}  // namespace apollo
