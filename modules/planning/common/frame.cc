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
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/reference_line_provider.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::common::adapter::AdapterManager;

FrameHistory::FrameHistory()
    : IndexedQueue<uint32_t, Frame>(FLAGS_max_history_frame_num) {}

Frame::Frame(uint32_t sequence_num,
             const common::TrajectoryPoint &planning_start_point,
             const common::VehicleState &vehicle_state)
    : sequence_num_(sequence_num),
      planning_start_point_(planning_start_point),
      vehicle_state_(vehicle_state) {}

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
      vehicle_state_.x(), vehicle_state_.y(), vehicle_state_.z());
  double s = 0.0;
  double l = 0.0;
  hdmap::LaneInfoConstPtr lane;
  if (hdmap_->GetNearestLaneWithHeading(point, 5.0, vehicle_state_.heading(),
                                        M_PI / 3.0, &lane, &s, &l) != 0) {
    AERROR << "Failed to find nearest lane from map at position: "
           << point.DebugString() << ", heading:" << vehicle_state_.heading();
    return false;
  }
  auto *start_point = request.mutable_waypoint(0);
  start_point->set_id(lane->id().id());
  start_point->set_s(s);
  start_point->mutable_pose()->CopyFrom(point);
  AdapterManager::PublishRoutingRequest(request);
  return true;
}

std::list<ReferenceLineInfo> &Frame::reference_line_info() {
  return reference_line_info_;
}

bool Frame::InitReferenceLineInfo() {
  std::list<ReferenceLine> reference_lines;
  std::list<hdmap::RouteSegments> segments;
  // Update reference line provider
  if (!ReferenceLineProvider::instance()->UpdateRoutingResponse(
          AdapterManager::GetRoutingResponse()->GetLatestObserved())) {
    AERROR << "Failed to update routing in reference line provider";
    return false;
  }
  ReferenceLineProvider::instance()->UpdateVehicleState(vehicle_state_);
  if (!ReferenceLineProvider::instance()->GetReferenceLines(&reference_lines,
                                                            &segments)) {
    AERROR << "Failed to create reference line";
    return false;
  }
  reference_line_info_.clear();
  auto ref_line_iter = reference_lines.begin();
  auto segments_iter = segments.begin();
  while (ref_line_iter != reference_lines.end()) {
    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,
                                      *ref_line_iter, *segments_iter);
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
  const auto &routing =
      AdapterManager::GetRoutingResponse()->GetLatestObserved();
  if (routing.routing_request().waypoint_size() < 2) {
    ADEBUG << "routing_request has no end";
    return nullptr;
  }
  const auto &routing_end = *routing.routing_request().waypoint().rbegin();
  const auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(routing_end.id()));
  if (!lane) {
    AERROR << "Failed to find lane for destination : "
           << routing_end.ShortDebugString();
    return nullptr;
  }

  double dest_lane_s =
      std::max(0.0, routing_end.s() - FLAGS_virtual_stop_wall_length -
                        FLAGS_stop_distance_destination);
  auto dest_point = lane->GetSmoothPoint(dest_lane_s);
  double left_width = 0.0;
  double right_width = 0.0;
  lane->GetWidth(dest_lane_s, &left_width, &right_width);
  // check if destination point is in planning range
  common::math::Box2d destination_box{{dest_point.x(), dest_point.y()},
                                      lane->Heading(dest_lane_s),
                                      FLAGS_virtual_stop_wall_length,
                                      left_width + right_width};
  return AddStaticVirtualObstacle(FLAGS_destination_obstacle_id,
                                  destination_box);
}

Status Frame::Init() {
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  vehicle_state_ = common::VehicleStateProvider::instance()->vehicle_state();
  const auto &point = common::util::MakePointENU(
      vehicle_state_.x(), vehicle_state_.y(), vehicle_state_.z());
  if (std::isnan(point.x()) || std::isnan(point.y())) {
    AERROR << "init point is not set";
    return Status(ErrorCode::PLANNING_ERROR, "init point is not set");
  }
  ADEBUG << "Enabled align prediction time ? : " << std::boolalpha
         << FLAGS_align_prediction_time;
  if (FLAGS_align_prediction_time) {
    AlignPredictionTime(vehicle_state_.timestamp());
  }
  if (FLAGS_enable_prediction) {
    CreatePredictionObstacles(prediction_);
  }

  if (!CreateDestinationObstacle()) {
    AERROR << "Failed to create the destination obstacle";
    return Status(ErrorCode::PLANNING_ERROR, "failed to find destination");
  }

  const auto *collision_obstacle = FindCollisionObstacle();
  if (collision_obstacle) {
    AERROR << "Found collision with obstacle: " << collision_obstacle->Id();
    return Status(ErrorCode::PLANNING_ERROR,
                  "Collision found with " + collision_obstacle->Id());
  }
  if (!InitReferenceLineInfo()) {
    AERROR << "Failed to init reference line info";
    return Status(ErrorCode::PLANNING_ERROR,
                  "failed to init reference line info");
  }
  return Status::OK();
}

const Obstacle *Frame::FindCollisionObstacle() const {
  if (obstacles_.Items().empty()) {
    return nullptr;
  }
  const auto &param =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  common::math::Vec2d position(vehicle_state_.x(), vehicle_state_.y());
  common::math::Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);
  common::math::Vec2d center(position +
                             vec_to_center.rotate(vehicle_state_.heading()));
  common::math::Box2d adc_box(center, vehicle_state_.heading(), param.length(),
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
    if (obstacle->PerceptionPolygon().DistanceTo(adc_box) <
        FLAGS_max_collision_distance) {
      AERROR << "Found collision with obstacle " << obstacle->Id();
      return obstacle;
    }
  }
  return nullptr;
}

uint32_t Frame::SequenceNum() const { return sequence_num_; }

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
  debug_routing->CopyFrom(
      AdapterManager::GetRoutingResponse()->GetLatestObserved());

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
  double min_cost = std::numeric_limits<double>::infinity();
  for (const auto &reference_line_info : reference_line_info_) {
    if (reference_line_info.IsDrivable() &&
        reference_line_info.Cost() < min_cost) {
      drive_reference_line_info_ = &reference_line_info;
      min_cost = reference_line_info.Cost();
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

}  // namespace planning
}  // namespace apollo
