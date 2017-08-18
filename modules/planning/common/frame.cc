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

const localization::Pose &Frame::VehicleInitPose() const { return init_pose_; }

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
  std::list<std::unique_ptr<Obstacle> > obstacles;
  Obstacle::CreateObstacles(prediction, &obstacles);
  for (auto &ptr : obstacles) {
    auto id(ptr->Id());
    obstacles_.Add(id, std::move(ptr));
  }
}

bool Frame::CreateDestinationObstacle() {
  // set destination point
  if (!routing_response_.routing_request().has_end()) {
    ADEBUG << "routing_request has no end";
    return false;
  }
  common::math::Vec2d destination;
  destination.set_x(routing_response_.routing_request().end().pose().x());
  destination.set_y(routing_response_.routing_request().end().pose().y());

  // check if destination point is in planning range
  common::SLPoint destination_sl;
  reference_line_.get_point_in_frenet_frame(destination, &destination_sl);
  double destination_s = destination_sl.s();
  if (destination_s < 0 || destination_s > reference_line_.length()) {
    AINFO << "destination(s[:" << destination_sl.s()
          << "]) out of planning range. Skip";
    return true;
  }

  // adjust destination based on adc_front_s
  common::SLPoint adc_sl;
  auto& adc_position = common::VehicleState::instance()->pose().position();
  reference_line_.get_point_in_frenet_frame(
      {adc_position.x(), adc_position.y()}, &adc_sl);
  const auto& vehicle_param =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  double adc_front_s = adc_sl.s() + vehicle_param.front_edge_to_center();
  if (destination_sl.s() <= adc_front_s) {
    destination_s = adc_front_s + FLAGS_destination_adjust_distance_buffer;
  }

  std::unique_ptr<Obstacle> obstacle_ptr = CreateVirtualObstacle(
      FLAGS_destination_obstacle_id,
      destination_s,
      FLAGS_virtual_stop_wall_length,
      FLAGS_virtual_stop_wall_width,
      FLAGS_virtual_stop_wall_height);

  obstacles_.Add(FLAGS_destination_obstacle_id, std::move(obstacle_ptr));

  return true;
}

const ADCTrajectory &Frame::GetADCTrajectory() const { return trajectory_pb_; }

ADCTrajectory *Frame::MutableADCTrajectory() { return &trajectory_pb_; }

PathDecision *Frame::path_decision() { return path_decision_; }

std::vector<ReferenceLineInfo> &Frame::reference_line_info() {
  return reference_line_info_;
}

bool Frame::InitReferenceLineInfo(
    const std::vector<ReferenceLine> &reference_lines) {
  reference_line_info_.clear();
  for (const auto &reference_line : reference_lines) {
    reference_line_info_.emplace_back(reference_line);
    auto &info = reference_line_info_.back();
    if (!info.AddObstacles(obstacles_.Items())) {
      AERROR << "Failed to add obstacles to reference line";
      return false;
    }
  }
  return true;
}

bool Frame::Init(const PlanningConfig &config) {
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

  if (FLAGS_enable_prediction) {
    CreatePredictionObstacles(prediction_);
  }

  InitReferenceLineInfo(reference_lines);
  reference_line_ = reference_lines.front();

  CreateDestinationObstacle();

  // FIXME(all) remove path decision from Frame.
  path_decision_ = reference_line_info_[0].path_decision();
  CHECK(path_decision_->path_obstacles().Items().size() > 0);

  if (FLAGS_enable_traffic_decision) {
    MakeTrafficDecision(routing_response_, reference_line_);
  }

  return true;
}

uint32_t Frame::sequence_num() const { return sequence_num_; }

const PlanningData &Frame::planning_data() const { return _planning_data; }

PlanningData *Frame::mutable_planning_data() { return &_planning_data; }

const ReferenceLine &Frame::reference_line() const { return reference_line_; }

bool Frame::MakeTrafficDecision(const routing::RoutingResponse &routing,
                                const ReferenceLine &reference_line) {
  const auto &path_obstacles = path_decision_->path_obstacles();
  for (const auto path_obstacle : path_obstacles.Items()) {
    const auto &obstacle = path_obstacle->Obstacle();

    // destination stop
    if (obstacle->Id() == FLAGS_destination_obstacle_id) {
      // check stop_posision on reference line
      auto stop_position = obstacle->Perception().position();
      common::SLPoint stop_line_sl;
      reference_line.get_point_in_frenet_frame(
          { stop_position.x(),  stop_position.x()}, &stop_line_sl);
      if (!reference_line.is_on_road(stop_line_sl)) {
        continue;
      }

      // check stop_line_s vs adc_s. stop_line_s must be ahead of adc_front_s
      common::SLPoint adc_sl;
      auto& adc_position = common::VehicleState::instance()->pose().position();
      reference_line.get_point_in_frenet_frame(
          {adc_position.x(), adc_position.y()}, &adc_sl);
      const auto& vehicle_param =
          common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();
      double adc_front_s = adc_sl.s() + vehicle_param.front_edge_to_center();
      if (stop_line_sl.s() <= adc_front_s) {
        ADEBUG << "skip: object:" << obstacle->Id() << " fence route_s["
               << stop_line_sl.s() << "] behind adc_front_s["
               << adc_front_s << "]";
        continue;
      }

      ObjectDecisionType object_stop;
      ObjectStop *object_stop_ptr = object_stop.mutable_stop();
      object_stop_ptr->set_distance_s(FLAGS_stop_line_min_distance);
      object_stop_ptr->set_reason_code(
          StopReasonCode::STOP_REASON_DESTINATION);

      auto stop_ref_point = reference_line_.get_reference_point(
          stop_position.x(), stop_position.y());
      object_stop_ptr->mutable_stop_point()->set_x(stop_ref_point.x());
      object_stop_ptr->mutable_stop_point()->set_y(stop_ref_point.y());
      object_stop_ptr->set_stop_heading(stop_ref_point.heading());

      path_decision_->AddLongitudinalDecision("TBD", obstacle->Id(),
                                              object_stop);
    }
  }

  return true;
}

bool Frame::CreateReferenceLineFromRouting(
    const common::PointENU &position, const routing::RoutingResponse &routing,
    std::vector<ReferenceLine> *reference_lines) {
  CHECK_NOTNULL(reference_lines);

  hdmap::Path hdmap_path;
  if (!pnc_map_->CreatePathFromRouting(
          routing, position, FLAGS_look_backward_distance,
          FLAGS_look_forward_distance, &hdmap_path)) {
    AERROR << "Failed to get path from routing";
    return false;
  }
  reference_lines->emplace_back(ReferenceLine());
  ReferenceLineSmoother smoother;
  smoother.Init(smoother_config_);
  if (!smoother.smooth(ReferenceLine(hdmap_path), &reference_lines->back())) {
    AERROR << "Failed to smooth reference line";
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
  double prediction_header_time = prediction_.header().timestamp_sec();

  for (int i = 0; i < prediction_.prediction_obstacle_size(); i++) {
    prediction::PredictionObstacle *obstacle =
        prediction_.mutable_prediction_obstacle(i);
    for (int j = 0; j < obstacle->trajectory_size(); j++) {
      prediction::Trajectory *trajectory = obstacle->mutable_trajectory(j);
      for (int k = 0; k < trajectory->trajectory_point_size(); k++) {
        common::TrajectoryPoint *point =
            trajectory->mutable_trajectory_point(k);
        double abs_relative_time = point->relative_time();
        point->set_relative_time(prediction_header_time + abs_relative_time -
                                 trajectory_header_time);
      }
    }
  }
}

std::unique_ptr<Obstacle> Frame::CreateVirtualObstacle(
    const std::string &obstacle_id,
    const double route_s,
    const double length, const double width, const double height) {
  std::unique_ptr<Obstacle> obstacle_ptr(nullptr);

  // create a "virtual" perception_obstacle
  perception::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_id(-1);  // simulator needs a valid integer
  auto dest_ref_point =
      reference_line_.get_reference_point(route_s);
  perception_obstacle.mutable_position()->set_x(dest_ref_point.x());
  perception_obstacle.mutable_position()->set_y(dest_ref_point.y());
  perception_obstacle.set_theta(dest_ref_point.heading());
  perception_obstacle.mutable_velocity()->set_x(0);
  perception_obstacle.mutable_velocity()->set_y(0);
  perception_obstacle.set_length(length);
  perception_obstacle.set_width(length);
  perception_obstacle.set_height(length);
  perception_obstacle.set_type(
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE);
  perception_obstacle.set_tracking_time(1.0);

  obstacle_ptr.reset(new Obstacle(obstacle_id, perception_obstacle));
  return obstacle_ptr;
}

}  // namespace planning
}  // namespace apollo
