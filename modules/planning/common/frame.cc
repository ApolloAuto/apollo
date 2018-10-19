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

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/reference_line_provider.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::PathOverlap;
using apollo::prediction::PredictionObstacles;

constexpr double kMathEpsilon = 1e-8;

FrameHistory::FrameHistory()
    : IndexedQueue<uint32_t, Frame>(FLAGS_max_history_frame_num) {}

Frame::Frame(uint32_t sequence_num, const LocalView &local_view,
             const common::TrajectoryPoint &planning_start_point,
             const double start_time, const common::VehicleState &vehicle_state,
             ReferenceLineProvider *reference_line_provider)
    : sequence_num_(sequence_num),
      local_view_(local_view),
      planning_start_point_(planning_start_point),
      start_time_(start_time),
      vehicle_state_(vehicle_state),
      reference_line_provider_(reference_line_provider),
      monitor_logger_buffer_(common::monitor::MonitorMessageItem::PLANNING) {
  // if (FLAGS_enable_lag_prediction) {
  //   lag_predictor_.reset(
  //       new LagPrediction(FLAGS_lag_prediction_min_appear_num,
  //                         FLAGS_lag_prediction_max_disappear_num));
  // }
}

const common::TrajectoryPoint &Frame::PlanningStartPoint() const {
  return planning_start_point_;
}

const common::VehicleState &Frame::vehicle_state() const {
  return vehicle_state_;
}

bool Frame::Rerouting() {
  if (FLAGS_use_navigation_mode) {
    AERROR << "Rerouting not supported in navigation mode";
    return false;
  }
  if (local_view_.routing == nullptr) {
    AERROR << "No previous routing available";
    return false;
  }
  if (!hdmap_) {
    AERROR << "Invalid HD Map.";
    return false;
  }
  auto request = local_view_.routing->routing_request();
  request.clear_header();
  // AdapterManager::FillRoutingRequestHeader("planning", &request);

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
  request.clear_waypoint();
  auto *start_point = request.add_waypoint();
  start_point->set_id(lane->id().id());
  start_point->set_s(s);
  start_point->mutable_pose()->CopyFrom(point);
  for (const auto &waypoint : future_route_waypoints_) {
    // reference_line_provider_->FutureRouteWaypoints()) {
    request.add_waypoint()->CopyFrom(waypoint);
  }
  if (request.waypoint_size() <= 1) {
    AERROR << "Failed to find future waypoints";
    return false;
  }

  PlanningContext::MutablePlanningStatus()
      ->mutable_rerouting()
      ->set_need_rerouting(true);

  PlanningContext::MutablePlanningStatus()
      ->mutable_rerouting()
      ->mutable_routing_request()
      ->CopyFrom(request);

  monitor_logger_buffer_.INFO("Planning send Rerouting request");
  return true;
}

const std::list<ReferenceLineInfo> &Frame::reference_line_info() const {
  return reference_line_info_;
}

std::list<ReferenceLineInfo> *Frame::mutable_reference_line_info() {
  return &reference_line_info_;
}

void Frame::UpdateReferenceLinePriority(
    const std::map<std::string, uint32_t> &id_to_priority) {
  for (const auto &pair : id_to_priority) {
    const auto id = pair.first;
    const auto priority = pair.second;
    auto ref_line_info_itr =
        std::find_if(reference_line_info_.begin(), reference_line_info_.end(),
                     [&id](const ReferenceLineInfo &ref_line_info) {
                       return ref_line_info.Lanes().Id() == id;
                     });
    if (ref_line_info_itr != reference_line_info_.end()) {
      ref_line_info_itr->SetPriority(priority);
    }
  }
}

bool Frame::CreateReferenceLineInfo(
    const std::list<ReferenceLine> &reference_lines,
    const std::list<hdmap::RouteSegments> &segments) {
  reference_line_info_.clear();
  auto ref_line_iter = reference_lines.begin();
  auto segments_iter = segments.begin();
  while (ref_line_iter != reference_lines.end()) {
    if (segments_iter->StopForDestination()) {
      is_near_destination_ = true;
    }
    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,
                                      *ref_line_iter, *segments_iter);
    ++ref_line_iter;
    ++segments_iter;
  }

  if (FLAGS_enable_change_lane_decider &&
      !change_lane_decider_.Apply(&reference_line_info_)) {
    AERROR << "Failed to apply change lane decider";
    return false;
  }

  if (reference_line_info_.size() == 2) {
    common::math::Vec2d xy_point(vehicle_state_.x(), vehicle_state_.y());
    common::SLPoint first_sl;
    if (!reference_line_info_.front().reference_line().XYToSL(xy_point,
                                                              &first_sl)) {
      return false;
    }
    common::SLPoint second_sl;
    if (!reference_line_info_.back().reference_line().XYToSL(xy_point,
                                                             &second_sl)) {
      return false;
    }
    const double offset = first_sl.l() - second_sl.l();
    reference_line_info_.front().SetOffsetToOtherReferenceLine(offset);
    reference_line_info_.back().SetOffsetToOtherReferenceLine(-offset);
  }

  bool has_valid_reference_line = false;
  for (auto &ref_info : reference_line_info_) {
    if (!ref_info.Init(obstacles())) {
      AERROR << "Failed to init reference line";
      continue;
    } else {
      has_valid_reference_line = true;
    }
  }
  return has_valid_reference_line;
}

/**
 * @brief: create static virtual object with lane width,
 *         mainly used for virtual stop wall
 */
const Obstacle *Frame::CreateStopObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_s) {
  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  const auto &reference_line = reference_line_info->reference_line();
  const double box_center_s = obstacle_s + FLAGS_virtual_stop_wall_length / 2.0;
  auto box_center = reference_line.GetReferencePoint(box_center_s);
  double heading = reference_line.GetReferencePoint(obstacle_s).heading();
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  reference_line.GetLaneWidth(obstacle_s, &lane_left_width, &lane_right_width);
  Box2d stop_wall_box{box_center, heading, FLAGS_virtual_stop_wall_length,
                      lane_left_width + lane_right_width};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

/**
 * @brief: create static virtual object with lane width,
 *         mainly used for virtual stop wall
 */
const Obstacle *Frame::CreateStopObstacle(const std::string &obstacle_id,
                                          const std::string &lane_id,
                                          const double lane_s) {
  if (!hdmap_) {
    AERROR << "Invalid HD Map.";
    return nullptr;
  }
  const auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(lane_id));
  if (!lane) {
    AERROR << "Failed to find lane[" << lane_id << "]";
    return nullptr;
  }

  double dest_lane_s = std::max(0.0, lane_s);
  auto dest_point = lane->GetSmoothPoint(dest_lane_s);

  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  lane->GetWidth(dest_lane_s, &lane_left_width, &lane_right_width);

  Box2d stop_wall_box{{dest_point.x(), dest_point.y()},
                      lane->Heading(dest_lane_s),
                      FLAGS_virtual_stop_wall_length,
                      lane_left_width + lane_right_width};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

/**
 * @brief: create static virtual object with lane width,
 */
const Obstacle *Frame::CreateStaticObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_start_s,
    const double obstacle_end_s) {
  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  const auto &reference_line = reference_line_info->reference_line();

  // start_xy
  common::SLPoint sl_point;
  sl_point.set_s(obstacle_start_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_start_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_start_xy)) {
    AERROR << "Failed to get start_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  // end_xy
  sl_point.set_s(obstacle_end_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_end_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_end_xy)) {
    AERROR << "Failed to get end_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  if (!reference_line.GetLaneWidth(obstacle_start_s, &left_lane_width,
                                   &right_lane_width)) {
    AERROR << "Failed to get lane width at s[" << obstacle_start_s << "]";
    return nullptr;
  }

  common::math::Box2d obstacle_box{
      common::math::LineSegment2d(obstacle_start_xy, obstacle_end_xy),
      left_lane_width + right_lane_width};

  return CreateStaticVirtualObstacle(obstacle_id, obstacle_box);
}

const Obstacle *Frame::CreateStaticVirtualObstacle(const std::string &id,
                                                   const Box2d &box) {
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

Status Frame::Init(
    const std::list<ReferenceLine> &reference_lines,
    const std::list<hdmap::RouteSegments> &segments,
    const std::vector<routing::LaneWaypoint> &future_route_waypoints) {
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
  vehicle_state_ = common::VehicleStateProvider::Instance()->vehicle_state();
  const auto &point = common::util::MakePointENU(
      vehicle_state_.x(), vehicle_state_.y(), vehicle_state_.z());
  if (std::isnan(point.x()) || std::isnan(point.y())) {
    AERROR << "init point is not set";
    return Status(ErrorCode::PLANNING_ERROR, "init point is not set");
  }
  ADEBUG << "Enabled align prediction time ? : " << std::boolalpha
         << FLAGS_align_prediction_time;

  // if (FLAGS_enable_lag_prediction && lag_predictor_) {
  // lag_predictor_->GetLaggedPrediction(
  //    local_view_.prediction_obstacles.get());
  //}
  auto prediction = *(local_view_.prediction_obstacles);

  if (FLAGS_align_prediction_time) {
    AlignPredictionTime(vehicle_state_.timestamp(), &prediction);
    local_view_.prediction_obstacles->CopyFrom(prediction);
  }
  for (auto &ptr :
       Obstacle::CreateObstacles(*local_view_.prediction_obstacles)) {
    AddObstacle(*ptr);
  }
  if (FLAGS_enable_collision_detection) {
    const auto *collision_obstacle = FindCollisionObstacle();
    if (collision_obstacle) {
      std::string err_str =
          "Found collision with obstacle: " + collision_obstacle->Id();
      monitor_logger_buffer_.ERROR(err_str);
      return Status(ErrorCode::PLANNING_ERROR, err_str);
    }
  }
  if (!CreateReferenceLineInfo(reference_lines, segments)) {
    AERROR << "Failed to init reference line info";
    return Status(ErrorCode::PLANNING_ERROR,
                  "failed to init reference line info");
  }
  future_route_waypoints_ = future_route_waypoints;

  return Status::OK();
}

const Obstacle *Frame::FindCollisionObstacle() const {
  if (obstacles_.Items().empty()) {
    return nullptr;
  }
  const auto &param =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
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

uint32_t Frame::SequenceNum() const { return sequence_num_; }

std::string Frame::DebugString() const {
  return "Frame: " + std::to_string(sequence_num_);
}

void Frame::RecordInputDebug(planning_internal::Debug *debug) {
  if (!debug) {
    ADEBUG << "Skip record input into debug";
    return;
  }
  auto *planning_debug_data = debug->mutable_planning_data();
  auto *adc_position = planning_debug_data->mutable_adc_position();
  adc_position->CopyFrom(*local_view_.localization_estimate);

  auto debug_chassis = planning_debug_data->mutable_chassis();
  debug_chassis->CopyFrom(*local_view_.chassis);

  if (!FLAGS_use_navigation_mode) {
    auto debug_routing = planning_debug_data->mutable_routing();
    debug_routing->CopyFrom(*local_view_.routing);
  }

  planning_debug_data->mutable_prediction_header()->CopyFrom(
      local_view_.prediction_obstacles->header());

  /*
  auto relative_map = AdapterManager::GetRelativeMap();
  if (!relative_map->Empty()) {
    planning_debug_data->mutable_relative_map()->mutable_header()->CopyFrom(
        relative_map->GetLatestObserved().header());
  }
  */
}

void Frame::AlignPredictionTime(const double planning_start_time,
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

Obstacle *Frame::Find(const std::string &id) { return obstacles_.Find(id); }

void Frame::AddObstacle(const Obstacle &obstacle) {
  obstacles_.Add(obstacle.Id(), obstacle);
}

const ReferenceLineInfo *Frame::FindDriveReferenceLineInfo() {
  double min_cost = std::numeric_limits<double>::infinity();
  drive_reference_line_info_ = nullptr;
  for (const auto &reference_line_info : reference_line_info_) {
    if (reference_line_info.IsDrivable() &&
        reference_line_info.Cost() < min_cost) {
      drive_reference_line_info_ = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  return drive_reference_line_info_;
}

const ReferenceLineInfo *Frame::DriveReferenceLineInfo() const {
  return drive_reference_line_info_;
}

const std::vector<const Obstacle *> Frame::obstacles() const {
  return obstacles_.Items();
}

bool Frame::VPresentationObstacle() {
  // load info from pnc map
  if (!OpenSpaceROI()) {
    AINFO << "fail at ROI()";
    return false;
  }

  std::size_t perception_obstacles_num = obstacles_.Items().size();
  std::size_t parking_boundaries_num = ROI_parking_boundary_.size();
  obstacles_num_ = perception_obstacles_num + parking_boundaries_num;
  if (perception_obstacles_num == 0) {
    AINFO << "no obstacle given by percption";
  }
  if (parking_boundaries_num != 4) {
    AERROR << "parking boundary obstacles size not right";
    return false;
  }
  // load percption obstacle list for warm start
  for (std::size_t i = 0; i < perception_obstacles_num; i++) {
    Box2d original_box = obstacles_.Items().at(i)->PerceptionBoundingBox();
    original_box.Shift(-1 * origin_point_);
    original_box.RotateFromCenter(-origin_heading_);
    std::unique_ptr<Obstacle> obstacle = Obstacle::CreateStaticVirtualObstacles(
        obstacles_.Items().at(i)->Id(), original_box);
    openspace_warmstart_obstacles_.Add(obstacle->Id(), *obstacle);
  }

  // load boundary obstacle for warm start
  Vec2d left_boundary_center(
      (ROI_parking_boundary_[0][0].x() + ROI_parking_boundary_[0][1].x()) / 2,
      (ROI_parking_boundary_[0][1].y() + ROI_parking_boundary_[0][2].y()) / 2);
  double left_boundary_heading = std::atan2(
      ROI_parking_boundary_[0][1].y() - ROI_parking_boundary_[0][0].y(),
      ROI_parking_boundary_[0][1].x() - ROI_parking_boundary_[0][0].x());
  double left_boundary_length = std::abs(-ROI_parking_boundary_[0][0].x() +
                                         ROI_parking_boundary_[0][1].x());
  double left_boundary_width = std::abs(ROI_parking_boundary_[0][1].y() -
                                        ROI_parking_boundary_[0][2].y());
  Box2d left_boundary_box(left_boundary_center, left_boundary_heading,
                          left_boundary_length, left_boundary_width);
  // check the heading of the parking spot is facing up or down
  Vec2d down_boundary_center(
      (ROI_parking_boundary_[1][0].x() + ROI_parking_boundary_[1][1].x()) / 2,
      ROI_parking_boundary_[1][1].y() +
          (open_space_end_pose_[2] > kMathEpsilon ? 0.5 : -0.5));
  double down_boundary_heading = std::atan2(
      ROI_parking_boundary_[1][1].y() - ROI_parking_boundary_[1][0].y(),
      ROI_parking_boundary_[1][1].x() - ROI_parking_boundary_[1][0].x());
  double down_boundary_length = std::abs(-ROI_parking_boundary_[1][0].x() +
                                         ROI_parking_boundary_[1][1].x());
  double down_boundary_width = 1.0;
  Box2d down_boundary_box(down_boundary_center, down_boundary_heading,
                          down_boundary_length, down_boundary_width);

  Vec2d right_boundary_center(
      (ROI_parking_boundary_[2][1].x() + ROI_parking_boundary_[2][2].x()) / 2,
      (ROI_parking_boundary_[2][0].y() + ROI_parking_boundary_[2][1].y()) / 2);
  double right_boundary_heading = std::atan2(
      ROI_parking_boundary_[2][2].y() - ROI_parking_boundary_[2][1].y(),
      ROI_parking_boundary_[2][2].x() - ROI_parking_boundary_[2][1].x());
  double right_boundary_length = std::abs(-ROI_parking_boundary_[2][1].x() +
                                          ROI_parking_boundary_[2][2].x());
  double right_boundary_width = std::abs(ROI_parking_boundary_[2][1].y() -
                                         ROI_parking_boundary_[2][0].y());
  Box2d right_boundary_box(right_boundary_center, right_boundary_heading,
                           right_boundary_length, right_boundary_width);

  Vec2d up_boundary_center(
      (ROI_parking_boundary_[3][0].x() + ROI_parking_boundary_[3][1].x()) / 2,
      ROI_parking_boundary_[3][0].y() +
          (open_space_end_pose_[2] > kMathEpsilon ? -0.5 : 0.5));
  double up_boundary_heading = std::atan2(
      ROI_parking_boundary_[3][1].y() - ROI_parking_boundary_[3][0].y(),
      ROI_parking_boundary_[3][1].x() - ROI_parking_boundary_[3][0].x());
  double up_boundary_length = std::abs(-ROI_parking_boundary_[3][0].x() +
                                       ROI_parking_boundary_[3][1].x());
  double up_boundary_width = 1.0;
  Box2d up_boundary_box(up_boundary_center, up_boundary_heading,
                        up_boundary_length, up_boundary_width);

  std::unique_ptr<Obstacle> left_boundary_obstacle =
      Obstacle::CreateStaticVirtualObstacles("left_boundary",
                                             left_boundary_box);

  openspace_warmstart_obstacles_.Add(left_boundary_obstacle->Id(),
                                     *left_boundary_obstacle);

  std::unique_ptr<Obstacle> down_boundary_obstacle =
      Obstacle::CreateStaticVirtualObstacles("down_boundary",
                                             down_boundary_box);

  openspace_warmstart_obstacles_.Add(down_boundary_obstacle->Id(),
                                     *down_boundary_obstacle);

  std::unique_ptr<Obstacle> right_boundary_obstacle =
      Obstacle::CreateStaticVirtualObstacles("right_boundary",
                                             right_boundary_box);

  openspace_warmstart_obstacles_.Add(right_boundary_obstacle->Id(),
                                     *right_boundary_obstacle);

  std::unique_ptr<Obstacle> up_boundary_obstacle =
      Obstacle::CreateStaticVirtualObstacles("up_boundary", up_boundary_box);

  openspace_warmstart_obstacles_.Add(up_boundary_obstacle->Id(),
                                     *up_boundary_obstacle);

  // load vertice vector for distance approach
  Eigen::MatrixXd perception_obstacles_edges_num_ =
      4 * Eigen::MatrixXd::Ones(perception_obstacles_num, 1);
  Eigen::MatrixXd parking_boundaries_obstacles_edges_num(4, 1);
  // the order is decided by the ROI()
  parking_boundaries_obstacles_edges_num << 2, 1, 2, 1;
  obstacles_edges_num_.resize(perception_obstacles_edges_num_.rows() +
                                  parking_boundaries_obstacles_edges_num.rows(),
                              1);
  obstacles_edges_num_ << perception_obstacles_edges_num_,
      parking_boundaries_obstacles_edges_num;

  // load vertices for perception obstacles(repeat the first vertice at the
  // last to form closed convex hull)
  for (const auto &obstacle : obstacles_.Items()) {
    Box2d original_box = obstacle->PerceptionBoundingBox();
    original_box.Shift(-1 * origin_point_);
    original_box.RotateFromCenter(-origin_heading_);
    std::vector<Vec2d> vertices_ccw = original_box.GetAllCorners();
    std::vector<Vec2d> vertices_cw;
    while (!vertices_ccw.empty()) {
      vertices_cw.emplace_back(vertices_ccw.back());
      vertices_ccw.pop_back();
    }
    // As the perception obstacle is a closed convex set, the first vertice is
    // repeated at the end of the vector to help transform all four edges to
    // inequality constraint
    vertices_cw.push_back(vertices_cw.front());
    obstacles_vertices_vec_.emplace_back(vertices_cw);
  }
  // load vertices for parking boundary (not need to repeat the first vertice to
  // get close hull)
  for (std::size_t i = 0; i < parking_boundaries_num; i++) {
    // directly load the ROI_distance_approach_parking_boundary_ into
    // obstacles_vertices_vec_
    obstacles_vertices_vec_.emplace_back(ROI_parking_boundary_[i]);
  }
  return true;
}

bool Frame::HPresentationObstacle() {
  obstacles_A_ = Eigen::MatrixXd::Zero(obstacles_edges_num_.sum(), 2);
  obstacles_b_ = Eigen::MatrixXd::Zero(obstacles_edges_num_.sum(), 1);
  // vertices using H-represetntation
  if (!ObsHRep(obstacles_num_, obstacles_edges_num_, obstacles_vertices_vec_,
               &obstacles_A_, &obstacles_b_)) {
    AINFO << "Fail to present obstacle in hyperplane";
    return false;
  }
  return true;
}

bool Frame::ObsHRep(
    const std::size_t &obstacles_num,
    const Eigen::MatrixXd &obstacles_edges_num,
    const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
    Eigen::MatrixXd *A_all, Eigen::MatrixXd *b_all) {
  if (obstacles_num != obstacles_vertices_vec.size()) {
    AINFO << "obstacles_num != obstacles_vertices_vec.size()";
    return false;
  }

  A_all->resize(obstacles_edges_num.sum(), 2);
  b_all->resize(obstacles_edges_num.sum(), 1);

  int counter = 0;
  double kEpsilon = 1.0e-5;
  // start building H representation
  for (std::size_t i = 0; i < obstacles_num; ++i) {
    std::size_t current_vertice_num = obstacles_edges_num(i, 0);
    Eigen::MatrixXd A_i(current_vertice_num, 2);
    Eigen::MatrixXd b_i(current_vertice_num, 1);

    // take two subsequent vertices, and computer hyperplane
    for (std::size_t j = 0; j < current_vertice_num; ++j) {
      Vec2d v1 = obstacles_vertices_vec[i][j];
      Vec2d v2 = obstacles_vertices_vec[i][j + 1];

      Eigen::MatrixXd A_tmp(2, 1), b_tmp(1, 1), ab(2, 1);
      // find hyperplane passing through v1 and v2
      if (std::abs(v1.x() - v2.x()) < kEpsilon) {
        if (v2.y() < v1.y()) {
          A_tmp << 1, 0;
          b_tmp << v1.x();
        } else {
          A_tmp << -1, 0;
          b_tmp << -v1.x();
        }
      } else if (std::abs(v1.y() - v2.y()) < kEpsilon) {
        if (v1.x() < v2.x()) {
          A_tmp << 0, 1;
          b_tmp << v1.y();
        } else {
          A_tmp << 0, -1;
          b_tmp << -v1.y();
        }
      } else {
        Eigen::MatrixXd tmp1(2, 2);
        tmp1 << v1.x(), 1, v2.x(), 1;
        Eigen::MatrixXd tmp2(2, 1);
        tmp2 << v1.y(), v2.y();
        ab = tmp1.inverse() * tmp2;
        double a = ab(0, 0);
        double b = ab(1, 0);

        if (v1.x() < v2.x()) {
          A_tmp << -a, 1;
          b_tmp << b;
        } else {
          A_tmp << a, -1;
          b_tmp << -b;
        }
      }

      // store vertices
      A_i.block(j, 0, 1, 2) = A_tmp.transpose();
      b_i.block(j, 0, 1, 1) = b_tmp;
    }

    A_all->block(counter, 0, A_i.rows(), 2) = A_i;
    b_all->block(counter, 0, b_i.rows(), 1) = b_i;
    counter += current_vertice_num;
  }
  return true;
}

bool Frame::OpenSpaceROI() {
  const ReferenceLineInfo *best_ref_info = FindDriveReferenceLineInfo();
  if (!best_ref_info) {
    std::string msg("fail to get the best reference line");
    AERROR << msg;
    return false;
  }
  ParkingSpaceInfoConstPtr target_parking_spot;
  const hdmap::Path &path = best_ref_info->reference_line().GetMapPath();
  const auto &parking_space_overlaps = path.parking_space_overlaps();
  for (const PathOverlap &parking_overlap : parking_space_overlaps) {
    if (parking_overlap.object_id == FLAGS_target_parking_spot_id) {
      hdmap::Id id;
      id.set_id(parking_overlap.object_id);
      target_parking_spot = hdmap_->GetParkingSpaceById(id);
    }
  }
  // left or right of the parking lot is decided when viewing the parking spot
  // open upward
  Vec2d left_top = target_parking_spot->polygon().points().at(0);
  Vec2d left_down = target_parking_spot->polygon().points().at(3);
  Vec2d right_top = target_parking_spot->polygon().points().at(1);
  Vec2d right_down = target_parking_spot->polygon().points().at(2);
  double left_top_s = 0.0;
  double left_top_l = 0.0;
  double right_top_s = 0.0;
  double right_top_l = 0.0;
  if (!(path.GetProjection(left_top, &left_top_s, &left_top_l) &&
        path.GetProjection(right_top, &right_top_s, &right_top_l))) {
    std::string msg(
        "fail to get parking spot points' projections on reference line");
    AERROR << msg;
    return false;
  }
  // start or end, left or right is decided by the vehicle's heading
  double center_line_s = (left_top_s + right_top_s) / 2;
  double start_s = center_line_s - FLAGS_parking_longitudinal_range;
  double end_s = center_line_s + FLAGS_parking_longitudinal_range;
  hdmap::MapPathPoint end_point = path.GetSmoothPoint(end_s);
  hdmap::MapPathPoint start_point = path.GetSmoothPoint(start_s);
  double start_left_width = path.GetRoadLeftWidth(start_s);
  double start_right_width = path.GetRoadRightWidth(start_s);
  double end_left_width = path.GetRoadLeftWidth(end_s);
  double end_right_width = path.GetRoadRightWidth(end_s);
  double start_right_vec_cos = std::cos(start_point.heading() - M_PI / 2);
  double start_right_vec_sin = std::sin(start_point.heading() - M_PI / 2);
  double start_left_vec_cos = std::cos(start_point.heading() + M_PI / 2);
  double start_left_vec_sin = std::sin(start_point.heading() + M_PI / 2);
  double end_right_vec_cos = std::cos(end_point.heading() - M_PI / 2);
  double end_right_vec_sin = std::sin(end_point.heading() - M_PI / 2);
  double end_left_vec_cos = std::cos(end_point.heading() + M_PI / 2);
  double end_left_vec_sin = std::sin(end_point.heading() + M_PI / 2);

  Vec2d start_right = Vec2d(start_right_width * start_right_vec_cos,
                            start_right_width * start_right_vec_sin);
  Vec2d start_left = Vec2d(start_left_width * start_left_vec_cos,
                           start_left_width * start_left_vec_sin);
  Vec2d end_right = Vec2d(end_right_width * end_right_vec_cos,
                          end_right_width * end_right_vec_cos);
  Vec2d end_left = Vec2d(end_left_width * end_left_vec_cos,
                         end_left_width * end_left_vec_sin);

  // rotate the points to have the lane to be horizontal to x axis and scale
  // them base on the origin point
  origin_heading_ = path.GetSmoothPoint(center_line_s).heading();
  origin_point_.set_x(left_top.x());
  origin_point_.set_y(left_top.y());
  left_top -= left_top;
  left_down -= left_top;
  right_top -= left_top;
  right_down -= left_top;
  start_right -= left_top;
  start_left -= left_top;
  end_right -= left_top;
  end_left -= left_top;
  left_top.SelfRotate(-origin_heading_);
  left_down.SelfRotate(-origin_heading_);
  right_top.SelfRotate(-origin_heading_);
  right_down.SelfRotate(-origin_heading_);
  start_right.SelfRotate(-origin_heading_);
  start_left.SelfRotate(-origin_heading_);
  end_right.SelfRotate(-origin_heading_);
  end_left.SelfRotate(-origin_heading_);

  // get end_pose of the parking spot
  double heading = (left_down - left_top).Angle();
  double x = (left_top.x() + right_top.x()) / 2;
  double y = 0.0;
  if (heading > kMathEpsilon) {
    y = left_top.y() + (-left_top.y() + left_down.y()) / 4;
  } else {
    y = left_down.y() + 3 * (left_top.y() - left_down.y()) / 4;
  }
  open_space_end_pose_.emplace_back(x);
  open_space_end_pose_.emplace_back(y);
  open_space_end_pose_.emplace_back(heading);
  open_space_end_pose_.emplace_back(0.0);

  // get xy boundary of the ROI
  double x_min = start_left.x();
  double x_max = end_left.x();
  double y_min = 0.0;
  double y_max = 0.0;
  if (left_down.y() > start_left.y()) {
    y_max = left_down.y();
    y_min = start_right.y();
  } else {
    y_max = start_left.y();
    y_min = left_down.y();
  }
  ROI_xy_boundary_.emplace_back(x_min);
  ROI_xy_boundary_.emplace_back(x_max);
  ROI_xy_boundary_.emplace_back(y_min);
  ROI_xy_boundary_.emplace_back(y_max);

  // If smaller than zero, the parking spot is on the right of the lane
  // Left, right, down or up of the boundary is decided when viewing the
  // parking spot upward
  std::vector<Vec2d> left_boundary;
  std::vector<Vec2d> down_boundary;
  std::vector<Vec2d> right_boundary;
  std::vector<Vec2d> up_boundary;

  if (left_top_l < 0) {
    start_right.set_x(-left_top_l * start_right_vec_cos);
    start_right.set_y(-left_top_l * start_right_vec_sin);
    end_right.set_x(-left_top_l * end_right_vec_cos);
    end_right.set_y(-left_top_l * end_right_vec_sin);
    start_right -= left_top;
    end_right -= left_top;
    start_right.SelfRotate(-origin_heading_);
    end_right.SelfRotate(-origin_heading_);
    left_boundary.push_back(start_right);
    left_boundary.push_back(left_top);
    left_boundary.push_back(left_down);
    down_boundary.push_back(left_down);
    down_boundary.push_back(right_down);
    right_boundary.push_back(right_down);
    right_boundary.push_back(right_top);
    right_boundary.push_back(end_right);
    up_boundary.push_back(end_left);
    up_boundary.push_back(start_left);
  } else {
    start_left.set_x(left_top_l * start_left_vec_cos);
    start_left.set_y(left_top_l * start_left_vec_sin);
    end_left.set_x(left_top_l * end_left_vec_cos);
    end_left.set_y(left_top_l * end_left_vec_sin);
    start_left -= left_top;
    end_left -= left_top;
    start_left.SelfRotate(-origin_heading_);
    end_left.SelfRotate(-origin_heading_);
    left_boundary.push_back(end_left);
    left_boundary.push_back(left_top);
    left_boundary.push_back(left_down);
    down_boundary.push_back(left_down);
    down_boundary.push_back(right_down);
    right_boundary.push_back(right_down);
    right_boundary.push_back(right_top);
    right_boundary.push_back(start_left);
    up_boundary.push_back(start_right);
    up_boundary.push_back(end_right);
  }
  ROI_parking_boundary_.emplace_back(left_boundary);
  ROI_parking_boundary_.emplace_back(down_boundary);
  ROI_parking_boundary_.emplace_back(right_boundary);
  ROI_parking_boundary_.emplace_back(up_boundary);

  return true;
}

}  // namespace planning
}  // namespace apollo
