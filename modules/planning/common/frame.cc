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
#include <string>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
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
  routing_result_ = routing;
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

void Frame::CreateObstacles(const prediction::PredictionObstacles &prediction) {
  std::list<std::unique_ptr<Obstacle> > obstacles;
  Obstacle::CreateObstacles(prediction, &obstacles);
  for (auto &ptr : obstacles) {
    mutable_planning_data()->mutable_decision_data()->AddObstacle(ptr.get());
    obstacles_.Add(ptr->Id(), std::move(ptr));
  }
}

bool Frame::AddDecision(const std::string &tag, const std::string &object_id,
                        const ObjectDecisionType &decision) {
  auto *path_obstacle = path_obstacles_.Find(object_id);
  if (!path_obstacle) {
    AERROR << "failed to find obstacle";
    return false;
  }
  path_obstacle->AddDecision(tag, decision);
  return true;
}

const ADCTrajectory& Frame::GetADCTrajectory() const {
  return trajectory_pb_;
}

ADCTrajectory* Frame::MutableADCTrajectory() {
  return &trajectory_pb_;
}

bool Frame::Init() {
  if (!pnc_map_) {
    AERROR << "map is null, call SetMap() first";
    return false;
  }
  const auto &point = init_pose_.position();
  if (std::isnan(point.x()) || std::isnan(point.y())) {
    AERROR << "init point is not set";
    return false;
  }
  if (!CreateReferenceLineFromRouting()) {
    AERROR << "Failed to create reference line from position: "
           << init_pose_.DebugString();
    return false;
  }
  if (FLAGS_enable_prediction) {
    CreateObstacles(prediction_);
  }
  return true;
}

uint32_t Frame::sequence_num() const { return sequence_num_; }

const PlanningData &Frame::planning_data() const { return _planning_data; }

PlanningData *Frame::mutable_planning_data() { return &_planning_data; }

void Frame::set_computed_trajectory(const PublishableTrajectory &trajectory) {
  _computed_trajectory = trajectory;
}

const PublishableTrajectory &Frame::computed_trajectory() const {
  return _computed_trajectory;
}

const ReferenceLine &Frame::reference_line() const { return *reference_line_; }

bool Frame::CreateReferenceLineFromRouting() {
  common::PointENU vehicle_position;
  vehicle_position.set_x(init_pose_.position().x());
  vehicle_position.set_y(init_pose_.position().y());
  vehicle_position.set_z(init_pose_.position().z());

  if (!pnc_map_->CreatePathFromRouting(
          routing_result_, vehicle_position, FLAGS_look_backward_distance,
          FLAGS_look_forward_distance, &hdmap_path_)) {
    AERROR << "Failed to get path from routing";
    return false;
  }
  if (!SmoothReferenceLine()) {
    AERROR << "Failed to smooth reference line";
    return false;
  }
  return true;
}

bool Frame::SmoothReferenceLine() {
  std::vector<ReferencePoint> ref_points;

  for (const auto &point : hdmap_path_.path_points()) {
    if (point.lane_waypoints().empty()) {
      AERROR << "path point has no lane_waypoint";
      return false;
    }
    const auto &lane_waypoint = point.lane_waypoints()[0];
    ref_points.emplace_back(point, point.heading(), 0.0, 0.0, lane_waypoint);
  }
  if (ref_points.empty()) {
    AERROR << "Found no reference points from map";
    return false;
  }

  std::unique_ptr<ReferenceLine> reference_line(new ReferenceLine(ref_points));
  std::vector<ReferencePoint> smoothed_ref_points;
  ReferenceLineSmoother smoother;
  if (!smoother.Init(FLAGS_reference_line_smoother_config_file)) {
    AERROR << "Failed to load file "
           << FLAGS_reference_line_smoother_config_file;
    return false;
  }
  if (!smoother.smooth(*reference_line, &smoothed_ref_points)) {
    AERROR << "Fail to smooth a reference line from map";
    return false;
  }
  ADEBUG << "smooth reference points num:" << smoothed_ref_points.size();
  reference_line_.reset(new ReferenceLine(smoothed_ref_points));
  return true;
}

const Obstacles &Frame::GetObstacles() const { return obstacles_; }
Obstacles *Frame::MutableObstacles() { return &obstacles_; }

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
  const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  adc_position->CopyFrom(localization);

  const auto& chassis = AdapterManager::GetChassis()->GetLatestObserved();
  auto debug_chassis = planning_data->mutable_chassis();
  debug_chassis->CopyFrom(chassis);

  const auto& routing_result =
      AdapterManager::GetRoutingResponse()->GetLatestObserved();

  auto debug_routing = planning_data->mutable_routing();
  debug_routing->CopyFrom(routing_result);
}


}  // namespace planning
}  // namespace apollo
