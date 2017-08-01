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

#include "modules/common/log.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/reference_line_smoother.h"

namespace apollo {
namespace planning {

Frame::Frame(const uint32_t sequence_num, const hdmap::PncMap *pnc_map)
    : sequence_num_(sequence_num), pnc_map_(pnc_map) {}

void Frame::SetInitPose(const localization::Pose &pose) { init_pose_ = pose; }

void Frame::SetRouting(const hdmap::RoutingResult &routing) {
  routing_result_ = routing;
}

void Frame::SetDecisionDataFromPrediction(
    const prediction::PredictionObstacles &prediction_obstacles) {
  for (const auto &prediction_obstacle :
       prediction_obstacles.prediction_obstacle()) {
    Obstacle obstacle;
    auto &perception_obstacle = prediction_obstacle.perception_obstacle();
    obstacle.SetId(perception_obstacle.id());
    obstacle.SetType(perception_obstacle.type());
    obstacle.SetHeight(perception_obstacle.height());
    obstacle.SetWidth(perception_obstacle.width());
    obstacle.SetLength(perception_obstacle.length());
    double theta = perception_obstacle.theta();
    obstacle.SetHeading(theta);
    double speed = perception_obstacle.velocity().x() * cos(theta) +
                   perception_obstacle.velocity().y() * sin(theta);
    obstacle.SetSpeed(speed);

    for (const auto &trajectory : prediction_obstacle.trajectory()) {
      PredictionTrajectory prediction_trajectory;
      prediction_trajectory.set_probability(trajectory.probability());
      prediction_trajectory.set_start_timestamp(
          prediction_obstacle.time_stamp());
      for (const auto &trajectory_point : trajectory.trajectory_point()) {
        prediction_trajectory.add_trajectory_point(trajectory_point);
      }
      obstacle.add_prediction_trajectory(prediction_trajectory);
    }

    mutable_planning_data()->mutable_decision_data()->AddObstacle(obstacle);
  }
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
  if (!SmoothReferenceLine()) {
    AERROR << "Failed to smooth reference line";
    return false;
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

bool Frame::CreateReferenceLineFromRouting() {
  common::PointENU vehicle_position;
  vehicle_position.set_x(init_pose_.position().x());
  vehicle_position.set_y(init_pose_.position().y());
  vehicle_position.set_z(init_pose_.position().z());

  return pnc_map_->CreatePathFromRouting(
      routing_result_, vehicle_position, FLAGS_look_backward_distance,
      FLAGS_look_forward_distance, &hdmap_path_);
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
  _planning_data.set_reference_line(smoothed_ref_points);
  return true;
}

std::string Frame::DebugString() const {
  return "Frame: " + std::to_string(sequence_num_);
}

}  // namespace planning
}  // namespace apollo
