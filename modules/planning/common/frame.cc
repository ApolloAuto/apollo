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

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <list>
#include <string>
#include <utility>

#include "modules/routing/proto/routing.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/reference_line_provider.h"
#include "modules/planning/reference_line/reference_line_smoother.h"

namespace apollo {
namespace planning {

using apollo::common::adapter::AdapterManager;
using apollo::common::ErrorCode;
using apollo::common::Status;

std::unique_ptr<hdmap::PncMap> Frame::pnc_map_;

void Frame::SetMap(const hdmap::HDMap *hdmap) {
  pnc_map_.reset(new hdmap::PncMap(hdmap));
}

FrameHistory::FrameHistory()
    : IndexedQueue<uint32_t, Frame>(FLAGS_max_history_frame_num) {}

Frame::Frame(const uint32_t sequence_num) : sequence_num_(sequence_num) {}

void Frame::SetVehicleInitPose(const localization::Pose &pose) {
  init_pose_ = pose;
}

void Frame::SetPlanningStartPoint(const common::TrajectoryPoint &start_point) {
  planning_start_point_ = start_point;
}

const common::TrajectoryPoint &Frame::PlanningStartPoint() const {
  return planning_start_point_;
}

void Frame::SetPrediction(const prediction::PredictionObstacles &prediction) {
  prediction_ = prediction;
}

void Frame::CreatePredictionObstacles(
    const prediction::PredictionObstacles &prediction) {
  for (auto &ptr : Obstacle::CreateObstacles(prediction)) {
    AddObstacle(*ptr);
  }
}

bool Frame::Rerouting() {
  auto *adapter_manager = AdapterManager::instance();
  const auto *vehicle_state = common::VehicleState::instance();
  if (adapter_manager->GetRoutingResponse()->Empty()) {
    AERROR << "No previous routing available";
    return false;
  }
  auto request = adapter_manager->GetRoutingResponse()
                     ->GetLatestObserved()
                     .routing_request();
  request.clear_header();
  AdapterManager::FillRoutingRequestHeader("planning", &request);
  auto point = common::util::MakePointENU(
      vehicle_state->x(), vehicle_state->y(), vehicle_state->z());
  double s = 0.0;
  double l = 0.0;
  hdmap::LaneInfoConstPtr lane;
  if (pnc_map_->hdmap()->GetNearestLaneWithHeading(
          point, 5.0, vehicle_state->heading(), M_PI / 3.0, &lane, &s, &l) !=
      0) {
    AERROR << "Failed to find nearest lane from map at position: "
           << point.DebugString() << ", heading:" << vehicle_state->heading();
    return false;
  }
  auto *start_point = request.mutable_waypoint(0);
  start_point->set_id(lane->id().id());
  start_point->set_s(s);
  start_point->mutable_pose()->CopyFrom(point);
  AdapterManager::PublishRoutingRequest(request);
  return true;
}

void Frame::UpdateRoutingResponse(const routing::RoutingResponse &routing) {
  pnc_map_->UpdateRoutingResponse(routing);
}

const routing::RoutingResponse &Frame::routing_response() const {
  return pnc_map_->routing_response();
}

std::list<ReferenceLineInfo> &Frame::reference_line_info() {
  return reference_line_info_;
}

bool Frame::InitReferenceLineInfo() {
  std::list<ReferenceLine> reference_lines;
  std::list<hdmap::RouteSegments> segments;
  bool has_ref_line = false;
  if (FLAGS_enable_reference_line_provider_thread) {
    has_ref_line = ReferenceLineProvider::instance()->GetReferenceLines(
        &reference_lines, &segments);
  } else {
    has_ref_line = CreateReferenceLineFromRouting(init_pose_.position(),
                                                  &reference_lines, &segments);
  }
  if (!has_ref_line) {
    AERROR << "Failed to create reference line from position: "
           << init_pose_.DebugString();
    return false;
  }
  reference_line_info_.clear();
  auto ref_line_iter = reference_lines.begin();
  auto segments_iter = segments.begin();
  while (ref_line_iter != reference_lines.end()) {
    reference_line_info_.emplace_back(pnc_map_.get(), *ref_line_iter,
                                      *segments_iter, planning_start_point_);
    ++ref_line_iter;
    ++segments_iter;
  }

  for (auto &info : reference_line_info_) {
    if (!info.Init()) {
      AERROR << "Failed to init adc sl boundary";
      return false;
    }
    if (!info.AddObstacles(obstacles_.Items())) {
      AERROR << "Failed to add obstacles to reference line";
      return false;
    }
  }
  return true;
}

const Obstacle *Frame::AddStaticVirtualObstacle(
    const std::string &id, const common::math::Box2d &box) {
  const auto *object = obstacles_.Find(id);
  if (object) {
    AWARN << "obstacle " << id << " already exist.";
    return object;
  }
  auto *ptr =
      obstacles_.Add(id, *Obstacle::CreateStaticVirtualObstacles(id, box));
  if (!ptr) {
    AERROR << "Failed to create virtual obstacle " << id;
  }
  return ptr;
}

const Obstacle *Frame::CreateDestinationObstacle() {
  const auto &routing = pnc_map_->routing_response();
  if (routing.routing_request().waypoint_size() < 2) {
    ADEBUG << "routing_request has no end";
    return nullptr;
  }
  const auto &routing_end = *routing.routing_request().waypoint().rbegin();
  const auto lane =
      pnc_map_->hdmap()->GetLaneById(hdmap::MakeMapId(routing_end.id()));
  if (!lane) {
    AERROR << "Failed to find lane for destination : "
           << routing_end.DebugString();
    return nullptr;
  }

  double dest_lane_s =
      std::max(0.0, routing_end.s() - FLAGS_virtual_stop_wall_length -
                        FLAGS_stop_distance_destination);
  auto dest_point = lane->GetSmoothPoint(dest_lane_s);
  // check if destination point is in planning range
  common::math::Box2d destination_box{{dest_point.x(), dest_point.y()},
                                      lane->Heading(dest_lane_s),
                                      FLAGS_virtual_stop_wall_length,
                                      FLAGS_virtual_stop_wall_width};
  return AddStaticVirtualObstacle(FLAGS_destination_obstacle_id,
                                  destination_box);
}

Status Frame::Init(const PlanningConfig &config,
                   const double current_time_stamp) {
  if (!pnc_map_) {
    AERROR << "map is null, call SetMap() first";
    return Status(ErrorCode::PLANNING_ERROR, "map is empty");
  }
  const auto &point = init_pose_.position();
  if (std::isnan(point.x()) || std::isnan(point.y())) {
    AERROR << "init point is not set";
    return Status(ErrorCode::PLANNING_ERROR, "init point is not set");
  }
  smoother_config_ = config.reference_line_smoother_config();

  ADEBUG << "Enabled align prediction time ? : " << std::boolalpha
         << FLAGS_align_prediction_time;
  if (FLAGS_align_prediction_time) {
    AlignPredictionTime(current_time_stamp);
  }
  if (FLAGS_enable_prediction) {
    CreatePredictionObstacles(prediction_);
  }

  if (!CreateDestinationObstacle()) {
    AERROR << "Failed to create the destination obstacle";
    return Status(ErrorCode::PLANNING_ERROR, "failed to find destination");
  }

  if (CheckCollision()) {
    AERROR << "Found collision with obstacle: " << collision_obstacle_id_;
    return Status(ErrorCode::PLANNING_ERROR,
                  "Collision found with " + collision_obstacle_id_);
  }
  if (!InitReferenceLineInfo()) {
    AERROR << "Failed to init reference line info";
    return Status(ErrorCode::PLANNING_ERROR,
                  "failed to init reference line info");
  }
  return Status::OK();
}

bool Frame::CheckCollision() {
  const auto &adc_box = common::VehicleState::instance()->AdcBoundingBox();
  common::math::Polygon2d adc_polygon(adc_box);
  const double adc_half_diagnal = adc_box.diagonal() / 2.0;
  for (const auto &obstacle : obstacles_.Items()) {
    double center_dist =
        adc_box.center().DistanceTo(obstacle->PerceptionBoundingBox().center());
    if (center_dist > obstacle->PerceptionBoundingBox().diagonal() / 2.0 +
                          adc_half_diagnal + FLAGS_max_collision_distance) {
      ADEBUG << "Obstacle : " << obstacle->Id() << " is too far to collide";
      continue;
    }
    if (adc_polygon.DistanceTo(obstacle->PerceptionPolygon()) <
        FLAGS_max_collision_distance) {
      AERROR << "Found collision with obstacle " << obstacle->Id();
      collision_obstacle_id_ = obstacle->Id();
      return true;
    }
  }
  return false;
}

uint32_t Frame::SequenceNum() const { return sequence_num_; }

bool Frame::CreateReferenceLineFromRouting(
    const common::PointENU &position, std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
  DCHECK_NOTNULL(reference_lines);
  DCHECK_NOTNULL(segments);
  std::vector<hdmap::RouteSegments> route_segments;
  const auto &adc_speed = common::VehicleState::instance()->linear_velocity();
  double look_forward_distance = (adc_speed * FLAGS_look_forward_time_sec >
                                  FLAGS_look_forward_min_distance)
                                     ? FLAGS_look_forward_distance
                                     : FLAGS_look_forward_min_distance;

  if (!pnc_map_->UpdatePosition(position)) {
    AERROR << "Failed to update position: " << position.ShortDebugString()
           << " in pnc map.";
    return false;
  }
  if (!pnc_map_->GetRouteSegments(FLAGS_look_backward_distance,
                                  look_forward_distance, &route_segments)) {
    AERROR << "Failed to extract segments from routing";
    return false;
  }

  ReferenceLineSmoother smoother;
  smoother.Init(smoother_config_);

  SpiralReferenceLineSmoother spiral_smoother;
  double max_spiral_smoother_dev = 0.1;
  spiral_smoother.set_max_point_deviation(max_spiral_smoother_dev);

  for (const auto &each_segments : route_segments) {
    hdmap::Path hdmap_path;
    hdmap::PncMap::CreatePathFromLaneSegments(each_segments, &hdmap_path);
    if (FLAGS_enable_smooth_reference_line) {
      ReferenceLine reference_line;
      if (FLAGS_enable_spiral_reference_line) {
        if (!spiral_smoother.Smooth(ReferenceLine(hdmap_path),
                                    &reference_line)) {
          AERROR << "Failed to smooth reference_line with spiral smoother";
        }
      } else {
        std::vector<double> init_t_knots;
        Spline2dSolver spline_solver(init_t_knots, 5);
        if (!smoother.Smooth(ReferenceLine(hdmap_path), &reference_line,
                             &spline_solver)) {
          AERROR << "Failed to smooth reference line";
          continue;
        }
      }
      reference_lines->emplace_back(std::move(reference_line));
      segments->emplace_back(each_segments);
    } else {
      reference_lines->emplace_back(hdmap_path);
      segments->emplace_back(each_segments);
    }
  }
  return !reference_lines->empty();
}

std::string Frame::DebugString() const {
  return "Frame: " + std::to_string(sequence_num_);
}

void Frame::RecordInputDebug(planning_internal::Debug *debug) {
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

  auto debug_routing = planning_data->mutable_routing();
  debug_routing->CopyFrom(routing_response());

  planning_data->mutable_prediction_header()->CopyFrom(prediction_.header());
}

void Frame::AlignPredictionTime(const double trajectory_header_time) {
  ADEBUG << "planning header: " << std::to_string(trajectory_header_time);
  double prediction_header_time = prediction_.header().timestamp_sec();
  ADEBUG << "prediction header: " << std::to_string(prediction_header_time);

  for (auto &obstacle : *prediction_.mutable_prediction_obstacle()) {
    for (auto &trajectory : *obstacle.mutable_trajectory()) {
      for (auto &point : *trajectory.mutable_trajectory_point()) {
        point.set_relative_time(prediction_header_time + point.relative_time() -
                                trajectory_header_time);
      }
    }
  }
}

void Frame::AddObstacle(const Obstacle &obstacle) {
  obstacles_.Add(obstacle.Id(), obstacle);
}

const ReferenceLineInfo *Frame::FindDriveReferenceLineInfo() {
  drive_reference_line_info_ = &reference_line_info_.front();
  double reference_line_cost = drive_reference_line_info_->Cost();
  for (const auto &reference_line_info : reference_line_info_) {
    if (reference_line_info.Cost() < reference_line_cost) {
      drive_reference_line_info_ = &reference_line_info;
      reference_line_cost = reference_line_info.Cost();
    }
  }
  return drive_reference_line_info_;
}

const ReferenceLineInfo *Frame::DriveReferenceLinfInfo() const {
  return drive_reference_line_info_;
}

const std::vector<const Obstacle *> Frame::obstacles() const {
  return obstacles_.Items();
}

apollo::common::PointENU Frame::GetRoutingDestination() {
  const auto &routing = pnc_map_->routing_response();
  const auto &routing_end = *routing.routing_request().waypoint().rbegin();
  return routing_end.pose();
}

}  // namespace planning
}  // namespace apollo
