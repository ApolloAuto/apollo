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
 * @file frame.cc
 **/
#include "modules/planning/common/frame.h"

#include <cmath>
#include <list>
#include <string>
#include <utility>

#include "modules/routing/proto/routing.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/reference_line_smoother.h"

namespace apollo {
namespace planning {

using apollo::common::adapter::AdapterManager;

const hdmap::PncMap *Frame::pnc_map_ = nullptr;

void Frame::SetMap(hdmap::PncMap *pnc_map) { pnc_map_ = pnc_map; }

FrameHistory::FrameHistory()
    : IndexedQueue<uint32_t, Frame>(FLAGS_max_history_frame_num) {}

Frame::Frame(const uint32_t sequence_num) : sequence_num_(sequence_num) {}

void Frame::SetVehicleInitPose(const localization::Pose &pose) {
  init_pose_ = pose;
}

void Frame::SetRoutingResponse(const routing::RoutingResponse &routing) {
  routing_response_ = routing;
}

void Frame::SetPlanningStartPoint(const common::TrajectoryPoint &start_point) {
  planning_start_point_ = start_point;
}

const hdmap::PncMap *Frame::PncMap() {
  DCHECK(pnc_map_) << "map is not setup in frame";
  return pnc_map_;
}

const common::TrajectoryPoint &Frame::PlanningStartPoint() const {
  return planning_start_point_;
}

void Frame::SetPrediction(const prediction::PredictionObstacles &prediction) {
  prediction_ = prediction;
}

void Frame::CreatePredictionObstacles(
    const prediction::PredictionObstacles &prediction) {
  auto obstacles = Obstacle::CreateObstacles(prediction);
  for (auto &ptr : obstacles) {
    auto id(ptr->Id());
    obstacles_.Add(id, std::move(ptr));
  }
}

const routing::RoutingResponse &Frame::routing_response() const {
  return routing_response_;
}

const ADCTrajectory &Frame::GetADCTrajectory() const { return trajectory_pb_; }

ADCTrajectory *Frame::MutableADCTrajectory() { return &trajectory_pb_; }

std::vector<ReferenceLineInfo> &Frame::reference_line_info() {
  return reference_line_info_;
}

bool Frame::InitReferenceLineInfo(
    const std::vector<ReferenceLine> &reference_lines) {
  reference_line_info_.clear();
  for (const auto &reference_line : reference_lines) {
    reference_line_info_.emplace_back(pnc_map_, reference_line);
    if (!reference_line_info_.back().Init()) {
      AERROR << "Failed to init reference line info";
      return false;
    }
  }
  for (auto &info : reference_line_info_) {
    if (!info.AddObstacles(obstacles_.Items())) {
      AERROR << "Failed to add obstacles to reference line";
      return false;
    }
  }
  return true;
}

bool Frame::Init(const PlanningConfig &config,
                 const double current_time_stamp) {
  if (!pnc_map_) {
    AERROR << "map is null, call SetMap() first";
    return false;
  }
  const auto &point = init_pose_.position();
  if (std::isnan(point.x()) || std::isnan(point.y())) {
    AERROR << "init point is not set";
    return false;
  }
  smoother_config_ = config.reference_line_smoother_config();
  std::vector<ReferenceLine> reference_lines;
  if (!CreateReferenceLineFromRouting(init_pose_.position(), routing_response_,
                                      &reference_lines)) {
    AERROR << "Failed to create reference line from position: "
           << init_pose_.DebugString();
    return false;
  }

  AINFO << "Enabled align prediction time ? : " << std::boolalpha
        << FLAGS_align_prediction_time;
  if (FLAGS_align_prediction_time) {
    AlignPredictionTime(current_time_stamp);
  }
  if (FLAGS_enable_prediction) {
    CreatePredictionObstacles(prediction_);
  }

  if (!InitReferenceLineInfo(reference_lines)) {
    AERROR << "Failed to init reference line info";
    return false;
  }

  return true;
}

uint32_t Frame::sequence_num() const { return sequence_num_; }

const std::vector<ReferenceLineInfo> &Frame::reference_line_info() const {
  return reference_line_info_;
}

const Obstacle *Frame::FindObstacle(const std::string &obstacle_id) {
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  return obstacles_.Find(obstacle_id);
}

bool Frame::CreateReferenceLineFromRouting(
    const common::PointENU &position, const routing::RoutingResponse &routing,
    std::vector<ReferenceLine> *reference_lines) {
  CHECK_NOTNULL(reference_lines);
  CHECK(reference_lines->empty());

  std::vector<std::vector<hdmap::LaneSegment>> route_segments;
  if (!pnc_map_->GetLaneSegmentsFromRouting(
          routing, position, FLAGS_look_backward_distance,
          FLAGS_look_forward_distance, &route_segments)) {
    AERROR << "Failed to extract segments from routing";
    return false;
  }

  ReferenceLineSmoother smoother;
  smoother.Init(smoother_config_);

  for (const auto &segments : route_segments) {
    hdmap::Path hdmap_path;
    pnc_map_->CreatePathFromLaneSegments(segments, &hdmap_path);
    if (FLAGS_enable_smooth_reference_line) {
      ReferenceLine reference_line;
      if (!smoother.Smooth(ReferenceLine(hdmap_path), &reference_line)) {
        AERROR << "Failed to smooth reference line";
        continue;
      }
      reference_lines->push_back(std::move(reference_line));
    } else {
      reference_lines->emplace_back(hdmap_path);
    }
  }

  if (reference_lines->empty()) {
    AERROR << "No smooth reference lines available";
    return false;
  }
  return true;
}

const IndexedObstacles &Frame::GetObstacles() const { return obstacles_; }

std::string Frame::DebugString() const {
  return "Frame: " + std::to_string(sequence_num_);
}

void Frame::RecordInputDebug() {
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record input into debug";
    return;
  }
  auto planning_data = trajectory_pb_.mutable_debug()->mutable_planning_data();
  auto adc_position = planning_data->mutable_adc_position();
  const auto &localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  adc_position->CopyFrom(localization);

  const auto &chassis = AdapterManager::GetChassis()->GetLatestObserved();
  auto debug_chassis = planning_data->mutable_chassis();
  debug_chassis->CopyFrom(chassis);

  const auto &routing_response =
      AdapterManager::GetRoutingResponse()->GetLatestObserved();

  auto debug_routing = planning_data->mutable_routing();
  debug_routing->CopyFrom(routing_response);
}

void Frame::AlignPredictionTime(const double trajectory_header_time) {
  ADEBUG << "planning header: " << std::to_string(trajectory_header_time);
  double prediction_header_time = prediction_.header().timestamp_sec();
  ADEBUG << "prediction header: " << std::to_string(prediction_header_time);

  for (int i = 0; i < prediction_.prediction_obstacle_size(); ++i) {
    prediction::PredictionObstacle *obstacle =
        prediction_.mutable_prediction_obstacle(i);
    for (int j = 0; j < obstacle->trajectory_size(); ++j) {
      prediction::Trajectory *trajectory = obstacle->mutable_trajectory(j);
      for (int k = 0; k < trajectory->trajectory_point_size(); ++k) {
        common::TrajectoryPoint *point =
            trajectory->mutable_trajectory_point(k);
        double abs_relative_time = point->relative_time();
        point->set_relative_time(prediction_header_time + abs_relative_time -
                                 trajectory_header_time);
      }
    }
  }
}

bool Frame::AddObstacle(std::unique_ptr<Obstacle> obstacle) {
  auto id(obstacle->Id());

  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  return obstacles_.Add(id, std::move(obstacle));
}

const ReferenceLineInfo *Frame::FindDriveReferenceLineInfo() {
  if (!drive_reference_line_info_) {
    double reference_line_cost = std::numeric_limits<double>::infinity();
    for (const auto &reference_line_info : reference_line_info_) {
      if (reference_line_info.Cost() < reference_line_cost) {
        drive_reference_line_info_ = &reference_line_info;
        reference_line_cost = reference_line_info.Cost();
      }
    }
  }
  return drive_reference_line_info_;
}

const ReferenceLineInfo *Frame::DriveReferenceLinfInfo() const {
  return drive_reference_line_info_;
}

}  // namespace planning
}  // namespace apollo
