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
 * @file frame_open_space.cc
 **/
#include "modules/planning/common/frame_open_space.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <list>
#include <string>
#include <utility>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::common::adapter::AdapterManager;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::common::monitor::MonitorLogBuffer;
using apollo::prediction::PredictionObstacles;

constexpr double kMathEpsilon = 1e-8;

FrameOpenSpaceHistory::FrameOpenSpaceHistory()
    : IndexedQueue<uint32_t, FrameOpenSpace>(FLAGS_max_history_frame_num) {}

FrameOpenSpace::FrameOpenSpace(
    uint32_t sequence_num, const common::TrajectoryPoint &planning_start_point,
    const double start_time, const common::VehicleState &vehicle_state)
    : sequence_num_(sequence_num),
      planning_start_point_(planning_start_point),
      start_time_(start_time),
      vehicle_state_(vehicle_state),
      monitor_logger_(common::monitor::MonitorMessageItem::PLANNING) {
  if (FLAGS_enable_lag_prediction) {
    lag_predictor_.reset(
        new LagPrediction(FLAGS_lag_prediction_min_appear_num,
                          FLAGS_lag_prediction_max_disappear_num));
  }
}

const common::TrajectoryPoint &FrameOpenSpace::PlanningStartPoint() const {
  return planning_start_point_;
}

const common::VehicleState &FrameOpenSpace::vehicle_state() const {
  return vehicle_state_;
}

Status FrameOpenSpace::Init() {
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
  vehicle_state_ = common::VehicleStateProvider::instance()->vehicle_state();
  const auto &point = common::util::MakePointENU(
      vehicle_state_.x(), vehicle_state_.y(), vehicle_state_.z());
  if (std::isnan(point.x()) || std::isnan(point.y())) {
    AERROR << "init point is not set";
    return Status(ErrorCode::PLANNING_ERROR, "init point is not set");
  }
  ADEBUG << "Enabled align prediction time ? : " << std::boolalpha
         << FLAGS_align_prediction_time;

  // prediction
  if (AdapterManager::GetPrediction() &&
      !AdapterManager::GetPrediction()->Empty()) {
    if (FLAGS_enable_lag_prediction && lag_predictor_) {
      lag_predictor_->GetLaggedPrediction(&prediction_);
    } else {
      prediction_.CopyFrom(
          AdapterManager::GetPrediction()->GetLatestObserved());
    }
    if (FLAGS_align_prediction_time) {
      AlignPredictionTime(vehicle_state_.timestamp(), &prediction_);
    }
    for (auto &ptr : Obstacle::CreateObstacles(prediction_)) {
      AddObstacle(*ptr);
    }
  }

  if (FLAGS_enable_collision_detection) {
    const auto *collision_obstacle = FindCollisionObstacle();
    if (collision_obstacle) {
      std::string err_str =
          "Found collision with obstacle: " + collision_obstacle->Id();
      apollo::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
      buffer.ERROR(err_str);
      return Status(ErrorCode::PLANNING_ERROR, err_str);
    }
  }
  return Status::OK();
}

const Obstacle *FrameOpenSpace::FindCollisionObstacle() const {
  if (obstacles_.Items().empty()) {
    return nullptr;
  }
  const auto &param =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  Vec2d position(vehicle_state_.x(), vehicle_state_.y());
  Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);
  Vec2d center(position + vec_to_center.rotate(vehicle_state_.heading()));
  Box2d adc_box(center, vehicle_state_.heading(), param.length(),
                param.width());
  const double adc_half_diagnal = adc_box.diagonal() / 2.0;
  for (const auto &obstacle : obstacles_.Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }

    double center_dist =
        adc_box.center().DistanceTo(obstacle->PerceptionBoundingBox().center());
    if (center_dist > obstacle->PerceptionBoundingBox().diagonal() / 2.0 +
                          adc_half_diagnal + FLAGS_max_collision_distance) {
      ADEBUG << "Obstacle : " << obstacle->Id() << " is too far to collide";
      continue;
    }
    double distance = obstacle->PerceptionPolygon().DistanceTo(adc_box);
    if (FLAGS_ignore_overlapped_obstacle && distance < kMathEpsilon) {
      bool all_points_in = true;
      for (const auto &point : obstacle->PerceptionPolygon().points()) {
        if (!adc_box.IsPointIn(point)) {
          all_points_in = false;
          break;
        }
      }
      if (all_points_in) {
        ADEBUG << "Skip overlapped obstacle, which is often caused by lidar "
                  "calibration error";
        continue;
      }
    }
    if (distance < FLAGS_max_collision_distance) {
      AERROR << "Found collision with obstacle " << obstacle->Id();
      return obstacle;
    }
  }
  return nullptr;
}

uint32_t FrameOpenSpace::SequenceNum() const { return sequence_num_; }

std::string FrameOpenSpace::DebugString() const {
  return "FrameOpenSpace: " + std::to_string(sequence_num_);
}

void FrameOpenSpace::RecordInputDebug(planning_internal::Debug *debug) {
  if (!debug) {
    ADEBUG << "Skip record input into debug";
    return;
  }
  auto *planning_data = debug->mutable_planning_data();
  auto *adc_position = planning_data->mutable_adc_position();
  const auto &localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  adc_position->CopyFrom(localization);

  const auto &chassis = AdapterManager::GetChassis()->GetLatestObserved();
  auto debug_chassis = planning_data->mutable_chassis();
  debug_chassis->CopyFrom(chassis);

  planning_data->mutable_prediction_header()->CopyFrom(prediction_.header());
}

void FrameOpenSpace::AlignPredictionTime(
    const double planning_start_time,
    PredictionObstacles *prediction_obstacles) {
  if (!prediction_obstacles || !prediction_obstacles->has_header() ||
      !prediction_obstacles->header().has_timestamp_sec()) {
    return;
  }
  double prediction_header_time =
      prediction_obstacles->header().timestamp_sec();
  for (auto &obstacle : *prediction_obstacles->mutable_prediction_obstacle()) {
    for (auto &trajectory : *obstacle.mutable_trajectory()) {
      for (auto &point : *trajectory.mutable_trajectory_point()) {
        point.set_relative_time(prediction_header_time + point.relative_time() -
                                planning_start_time);
      }
      if (!trajectory.trajectory_point().empty() &&
          trajectory.trajectory_point().begin()->relative_time() < 0) {
        auto it = trajectory.trajectory_point().begin();
        while (it != trajectory.trajectory_point().end() &&
               it->relative_time() < 0) {
          ++it;
        }
        trajectory.mutable_trajectory_point()->erase(
            trajectory.trajectory_point().begin(), it);
      }
    }
  }
}

Obstacle *FrameOpenSpace::Find(const std::string &id) {
  return obstacles_.Find(id);
}

void FrameOpenSpace::AddObstacle(const Obstacle &obstacle) {
  obstacles_.Add(obstacle.Id(), obstacle);
}

const std::vector<const Obstacle *> FrameOpenSpace::obstacles() const {
  return obstacles_.Items();
}

}  // namespace planning
}  // namespace apollo
