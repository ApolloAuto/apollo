/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file
 **/

#include "modules/planning/tasks/deciders/open_space_roi_decider.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::LaneInfoConstPtr;
using apollo::hdmap::LaneSegment;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;

constexpr double kMathEpsilon = 1e-8;

OpenSpaceRoiDecider::OpenSpaceRoiDecider(const TaskConfig &config)
    : Decider(config) {
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
  vehicle_params_ =
      apollo::common::VehicleConfigHelper::GetConfig().vehicle_param();
}

Status OpenSpaceRoiDecider::Process(Frame *frame) {
  // TODO(Jinyun) only run GetOpenSpaceROI() at the first frame of
  // open space planner to save computation effort
  vehicle_state_ = frame->vehicle_state();
  ROI_parking_boundary_.clear();
  obstacles_by_frame_ = frame->GetObstacleList();
  const auto &routing_request = frame->local_view().routing->routing_request();
  if (routing_request.has_parking_space() &&
      routing_request.parking_space().has_id()) {
    target_parking_spot_id_ = routing_request.parking_space().id().id();
  } else {
    const std::string msg = "Failed to get parking space id from routing";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!(GetOpenSpaceROI(frame) && GetOpenSpaceInfo())) {
    const std::string msg = "Fail to get open space roi";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  return Status::OK();
}

bool OpenSpaceRoiDecider::VPresentationObstacle() {
  size_t parking_boundaries_num = ROI_parking_boundary_.size();
  if (parking_boundaries_num != 4) {
    AERROR << "parking boundary obstacles size not right";
    return false;
  }

  if (config_.open_space_roi_decider_config().enable_perception_obstacles()) {
    size_t perception_obstacles_num = obstacles_by_frame_->Items().size();

    frame_->mutable_open_space_info()->set_obstacles_num(
        perception_obstacles_num + parking_boundaries_num);
    if (perception_obstacles_num == 0) {
      AERROR << "no obstacle given by percption";
    }
    // load vertice vector for distance approach
    Eigen::MatrixXi perception_obstacles_edges_num_ =
        4 * Eigen::MatrixXi::Ones(perception_obstacles_num, 1);
    Eigen::MatrixXi parking_boundaries_obstacles_edges_num(4, 1);
    // the order is decided by the ROI()
    parking_boundaries_obstacles_edges_num << 2, 1, 2, 1;
    frame_->mutable_open_space_info()->mutable_obstacles_edges_num()->resize(
        perception_obstacles_edges_num_.rows() +
            parking_boundaries_obstacles_edges_num.rows(),
        1);
    *(frame_->mutable_open_space_info()->mutable_obstacles_edges_num())
        << perception_obstacles_edges_num_,
        parking_boundaries_obstacles_edges_num;
    // load vertices for perception obstacles(repeat the first vertice at the
    // last to form closed convex hull)
    const auto &origin_point = frame_->open_space_info().origin_point();
    const auto &origin_heading = frame_->open_space_info().origin_heading();
    for (const auto &obstacle : obstacles_by_frame_->Items()) {
      Box2d original_box = obstacle->PerceptionBoundingBox();
      original_box.Shift(-1.0 * origin_point);
      original_box.RotateFromCenter(-1.0 * origin_heading);
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
      frame_->mutable_open_space_info()
          ->mutable_obstacles_vertices_vec()
          ->emplace_back(vertices_cw);
    }
  } else {
    frame_->mutable_open_space_info()->set_obstacles_num(
        parking_boundaries_num);
    // load vertice vector for distance approach
    Eigen::MatrixXi parking_boundaries_obstacles_edges_num(4, 1);
    // the order is decided by the ROI()
    parking_boundaries_obstacles_edges_num << 2, 1, 2, 1;
    frame_->mutable_open_space_info()->mutable_obstacles_edges_num()->resize(
        parking_boundaries_obstacles_edges_num.rows(), 1);
    *(frame_->mutable_open_space_info()->mutable_obstacles_edges_num())
        << parking_boundaries_obstacles_edges_num;
  }

  // load vertices for parking boundary (not need to repeat the first vertice to
  // get close hull)
  for (size_t i = 0; i < parking_boundaries_num; ++i) {
    // directly load the ROI_distance_approach_parking_boundary_ into
    // obstacles_vertices_vec_
    frame_->mutable_open_space_info()
        ->mutable_obstacles_vertices_vec()
        ->emplace_back(ROI_parking_boundary_[i]);
  }
  return true;
}

bool OpenSpaceRoiDecider::HPresentationObstacle() {
  *(frame_->mutable_open_space_info()->mutable_obstacles_A()) =
      Eigen::MatrixXd::Zero(
          frame_->open_space_info().obstacles_edges_num().sum(), 2);
  *(frame_->mutable_open_space_info()->mutable_obstacles_b()) =
      Eigen::MatrixXd::Zero(
          frame_->open_space_info().obstacles_edges_num().sum(), 1);
  // vertices using H-represetntation
  if (!ObsHRep(frame_->open_space_info().obstacles_num(),
               frame_->open_space_info().obstacles_edges_num(),
               frame_->open_space_info().obstacles_vertices_vec(),
               frame_->mutable_open_space_info()->mutable_obstacles_A(),
               frame_->mutable_open_space_info()->mutable_obstacles_b())) {
    AERROR << "Fail to present obstacle in hyperplane";
    return false;
  }
  return true;
}

bool OpenSpaceRoiDecider::ObsHRep(
    const size_t &obstacles_num, const Eigen::MatrixXi &obstacles_edges_num,
    const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
    Eigen::MatrixXd *A_all, Eigen::MatrixXd *b_all) {
  if (obstacles_num != obstacles_vertices_vec.size()) {
    AERROR << "obstacles_num != obstacles_vertices_vec.size()";
    return false;
  }

  A_all->resize(obstacles_edges_num.sum(), 2);
  b_all->resize(obstacles_edges_num.sum(), 1);

  int counter = 0;
  double kEpsilon = 1.0e-5;
  // start building H representation
  for (size_t i = 0; i < obstacles_num; ++i) {
    size_t current_vertice_num = obstacles_edges_num(i, 0);
    Eigen::MatrixXd A_i(current_vertice_num, 2);
    Eigen::MatrixXd b_i(current_vertice_num, 1);

    // take two subsequent vertices, and computer hyperplane
    for (size_t j = 0; j < current_vertice_num; ++j) {
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
    counter += static_cast<int>(current_vertice_num);
  }
  return true;
}

bool OpenSpaceRoiDecider::GetOpenSpaceROI(Frame *frame) {
  // Find parking spot by getting nearestlane
  ParkingSpaceInfoConstPtr target_parking_spot = nullptr;
  std::shared_ptr<Path> nearby_path = nullptr;
  if (!GetMapInfo(frame, &target_parking_spot, &nearby_path)) {
    AERROR << "fail to get map info in open space planner";
    return false;
  }

  // left or right of the parking lot is decided when viewing the parking spot
  // open upward
  Vec2d left_top = target_parking_spot->polygon().points().at(3);
  Vec2d left_down = target_parking_spot->polygon().points().at(0);
  Vec2d right_top = target_parking_spot->polygon().points().at(2);
  Vec2d right_down = target_parking_spot->polygon().points().at(1);
  double left_top_s = 0.0;
  double left_top_l = 0.0;
  double right_top_s = 0.0;
  double right_top_l = 0.0;
  if (!(nearby_path->GetProjection(left_top, &left_top_s, &left_top_l) &&
        nearby_path->GetProjection(right_top, &right_top_s, &right_top_l))) {
    AERROR << "fail to get parking spot points' projections on reference line";
    return false;
  }
  // start or end, left or right is decided by the vehicle's heading
  double center_line_s = (left_top_s + right_top_s) / 2;
  double start_s =
      center_line_s -
      config_.open_space_roi_decider_config().roi_longitudinal_range();
  double end_s =
      center_line_s +
      config_.open_space_roi_decider_config().roi_longitudinal_range();
  hdmap::MapPathPoint end_point = nearby_path->GetSmoothPoint(end_s);
  hdmap::MapPathPoint start_point = nearby_path->GetSmoothPoint(start_s);
  double start_left_width = nearby_path->GetRoadLeftWidth(start_s);
  double start_right_width = nearby_path->GetRoadRightWidth(start_s);
  double end_left_width = nearby_path->GetRoadLeftWidth(end_s);
  double end_right_width = nearby_path->GetRoadRightWidth(end_s);
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
  start_right = start_right + start_point;
  Vec2d start_left = Vec2d(start_left_width * start_left_vec_cos,
                           start_left_width * start_left_vec_sin);
  start_left = start_left + start_point;
  Vec2d end_right = Vec2d(end_right_width * end_right_vec_cos,
                          end_right_width * end_right_vec_sin);
  end_right = end_right + end_point;
  Vec2d end_left = Vec2d(end_left_width * end_left_vec_cos,
                         end_left_width * end_left_vec_sin);
  end_left = end_left + end_point;

  // rotate the points to have the lane to be horizontal to x axis and scale
  // them base on the origin point
  frame_->mutable_open_space_info()->set_origin_heading(
      nearby_path->GetSmoothPoint(center_line_s).heading());
  frame_->mutable_open_space_info()->mutable_origin_point()->set_x(
      left_top.x());
  frame_->mutable_open_space_info()->mutable_origin_point()->set_y(
      left_top.y());
  left_top -= frame_->open_space_info().origin_point();
  left_down -= frame_->open_space_info().origin_point();
  right_top -= frame_->open_space_info().origin_point();
  right_down -= frame_->open_space_info().origin_point();
  start_right -= frame_->open_space_info().origin_point();
  start_left -= frame_->open_space_info().origin_point();
  end_right -= frame_->open_space_info().origin_point();
  end_left -= frame_->open_space_info().origin_point();
  left_top.SelfRotate(-1.0 * frame_->open_space_info().origin_heading());
  left_down.SelfRotate(-1.0 * frame_->open_space_info().origin_heading());
  right_top.SelfRotate(-1.0 * frame_->open_space_info().origin_heading());
  right_down.SelfRotate(-1.0 * frame_->open_space_info().origin_heading());
  start_right.SelfRotate(-1.0 * frame_->open_space_info().origin_heading());
  start_left.SelfRotate(-1.0 * frame_->open_space_info().origin_heading());
  end_right.SelfRotate(-1.0 * frame_->open_space_info().origin_heading());
  end_left.SelfRotate(-1.0 * frame_->open_space_info().origin_heading());

  // get end_pose of the parking spot
  parking_spot_heading_ = (left_down - left_top).Angle();
  double end_x = (left_top.x() + right_top.x()) / 2.0;
  double end_y = 0.0;
  const double parking_depth_buffer =
      config_.open_space_roi_decider_config().parking_depth_buffer();
  CHECK_GE(parking_depth_buffer, 0.0);
  const bool parking_inwards =
      config_.open_space_roi_decider_config().parking_inwards();
  const double top_to_down_distance = left_top.y() - left_down.y();
  if (parking_spot_heading_ > kMathEpsilon) {
    if (parking_inwards) {
      end_y =
          left_down.y() - (std::max(3.0 * -top_to_down_distance / 4.0,
                                    vehicle_params_.front_edge_to_center()) +
                           parking_depth_buffer);

    } else {
      end_y = left_down.y() - (std::max(-top_to_down_distance / 4.0,
                                        vehicle_params_.back_edge_to_center()) +
                               parking_depth_buffer);
    }
  } else {
    if (parking_inwards) {
      end_y =
          left_down.y() + (std::max(3.0 * top_to_down_distance / 4.0,
                                    vehicle_params_.front_edge_to_center()) +
                           parking_depth_buffer);
    } else {
      end_y = left_down.y() + (std::max(top_to_down_distance / 4.0,
                                        vehicle_params_.back_edge_to_center()) +
                               parking_depth_buffer);
    }
  }

  auto *end_pose =
      frame_->mutable_open_space_info()->mutable_open_space_end_pose();
  end_pose->emplace_back(end_x);
  end_pose->emplace_back(end_y);
  if (parking_inwards) {
    end_pose->emplace_back(parking_spot_heading_);
  } else {
    end_pose->emplace_back(
        common::math::NormalizeAngle(parking_spot_heading_ + M_PI));
  }
  // end_pose velocity is set to be zero
  end_pose->emplace_back(0.0);

  // get xy boundary of the ROI
  double x_min = std::min({start_left.x(), start_right.x()});
  double x_max = std::max({end_left.x(), end_right.x()});
  double y_min = std::min({left_down.y(), start_right.y(), start_left.y()});
  double y_max = std::max({left_down.y(), start_right.y(), start_left.y()});
  auto *xy_boundary =
      frame_->mutable_open_space_info()->mutable_ROI_xy_boundary();
  xy_boundary->emplace_back(x_min);
  xy_boundary->emplace_back(x_max);
  xy_boundary->emplace_back(y_min);
  xy_boundary->emplace_back(y_max);

  // check if vehicle in range of xy_boundary
  Vec2d vehicle_xy = Vec2d(vehicle_state_.x(), vehicle_state_.y());
  vehicle_xy -= frame_->open_space_info().origin_point();
  vehicle_xy.SelfRotate(-1.0 * frame_->open_space_info().origin_heading());
  if (vehicle_xy.x() > x_max || vehicle_xy.x() < x_min ||
      vehicle_xy.y() > y_max || vehicle_xy.y() < y_min) {
    AERROR << "vehicle pose outside of xy boundary of parking ROI";
    return false;
  }
  // If smaller than zero, the parking spot is on the right of the lane
  // Left, right, down or up of the boundary is decided when viewing the
  // parking spot upward
  std::vector<Vec2d> left_boundary;
  std::vector<Vec2d> down_boundary;
  std::vector<Vec2d> right_boundary;
  std::vector<Vec2d> up_boundary;

  if (left_top_l < 0) {
    start_right.set_x(-1.0 * left_top_l * start_right_vec_cos);
    start_right.set_y(-1.0 * left_top_l * start_right_vec_sin);
    start_right = start_right + start_point;
    end_right.set_x(-1.0 * left_top_l * end_right_vec_cos);
    end_right.set_y(-1.0 * left_top_l * end_right_vec_sin);
    end_right = end_right + end_point;
    start_right -= frame_->open_space_info().origin_point();
    end_right -= frame_->open_space_info().origin_point();
    start_right.SelfRotate(-1.0 * frame_->open_space_info().origin_heading());
    end_right.SelfRotate(-1.0 * frame_->open_space_info().origin_heading());
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
    start_left = start_left + start_point;
    end_left.set_x(left_top_l * end_left_vec_cos);
    end_left.set_y(left_top_l * end_left_vec_sin);
    end_left = end_left + end_point;
    start_left -= frame_->open_space_info().origin_point();
    end_left -= frame_->open_space_info().origin_point();
    start_left.SelfRotate(-1.0 * frame_->open_space_info().origin_heading());
    end_left.SelfRotate(-1.0 * frame_->open_space_info().origin_heading());
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

void OpenSpaceRoiDecider::SearchTargetParkingSpotOnPath(
    std::shared_ptr<Path> *nearby_path,
    ParkingSpaceInfoConstPtr *target_parking_spot) {
  const auto &parking_space_overlaps = (*nearby_path)->parking_space_overlaps();
  for (const auto &parking_overlap : parking_space_overlaps) {
    if (parking_overlap.object_id == target_parking_spot_id_) {
      hdmap::Id id;
      id.set_id(parking_overlap.object_id);
      *target_parking_spot = hdmap_->GetParkingSpaceById(id);
    }
  }
}

// TODO(Jinyun) Deprecate because of duplicate code in valet parking scenario
bool OpenSpaceRoiDecider::GetMapInfo(
    Frame *frame, ParkingSpaceInfoConstPtr *target_parking_spot,
    std::shared_ptr<Path> *nearby_path) {
  auto point = common::util::MakePointENU(
      vehicle_state_.x(), vehicle_state_.y(), vehicle_state_.z());
  LaneInfoConstPtr nearest_lane;
  double vehicle_lane_s = 0.0;
  double vehicle_lane_l = 0.0;
  // Check if last frame lane is avaiable
  const auto &previous_open_space_info =
      FrameHistory::Instance()->Latest()->open_space_info();
  if (previous_open_space_info.target_parking_lane() != nullptr &&
      previous_open_space_info.target_parking_spot_id() ==
          frame->open_space_info().target_parking_spot_id()) {
    nearest_lane = previous_open_space_info.target_parking_lane();
  } else {
    int status = HDMapUtil::BaseMap().GetNearestLaneWithHeading(
        point, 10.0, vehicle_state_.heading(), M_PI / 2.0, &nearest_lane,
        &vehicle_lane_s, &vehicle_lane_l);
    if (status != 0) {
      AERROR << "Getlane failed at OpenSpaceRoiDecider::GetOpenSpaceROI()";
      return false;
    }
  }
  frame->mutable_open_space_info()->set_target_parking_lane(nearest_lane);
  LaneSegment nearest_lanesegment =
      LaneSegment(nearest_lane, nearest_lane->accumulate_s().front(),
                  nearest_lane->accumulate_s().back());
  std::vector<LaneSegment> segments_vector;
  int next_lanes_num = nearest_lane->lane().successor_id_size();
  if (next_lanes_num != 0) {
    for (int i = 0; i < next_lanes_num; ++i) {
      auto next_lane_id = nearest_lane->lane().successor_id(i);
      segments_vector.push_back(nearest_lanesegment);
      auto next_lane = hdmap_->GetLaneById(next_lane_id);
      LaneSegment next_lanesegment =
          LaneSegment(next_lane, next_lane->accumulate_s().front(),
                      next_lane->accumulate_s().back());
      segments_vector.emplace_back(next_lanesegment);
      (*nearby_path).reset(new Path(segments_vector));
      SearchTargetParkingSpotOnPath(nearby_path, target_parking_spot);
      if (*target_parking_spot != nullptr) {
        break;
      }
    }
  } else {
    segments_vector.push_back(nearest_lanesegment);
    (*nearby_path).reset(new Path(segments_vector));
    SearchTargetParkingSpotOnPath(nearby_path, target_parking_spot);
  }

  if (*target_parking_spot == nullptr) {
    AERROR << "No such parking spot found after searching all path forward "
              "possible";
    return false;
  }

  if (!CheckDistanceToParkingSpot(nearby_path, target_parking_spot)) {
    AERROR << "target parking spot found, but too far, distance larger than "
              "pre-defined distance";
    return false;
  }

  return true;
}

bool OpenSpaceRoiDecider::CheckDistanceToParkingSpot(
    std::shared_ptr<Path> *nearby_path,
    ParkingSpaceInfoConstPtr *target_parking_spot) {
  Vec2d left_bottom_point = (*target_parking_spot)->polygon().points().at(0);
  Vec2d right_bottom_point = (*target_parking_spot)->polygon().points().at(1);
  double left_bottom_point_s = 0.0;
  double left_bottom_point_l = 0.0;
  double right_bottom_point_s = 0.0;
  double right_bottom_point_l = 0.0;
  double vehicle_point_s = 0.0;
  double vehicle_point_l = 0.0;
  (*nearby_path)
      ->GetNearestPoint(left_bottom_point, &left_bottom_point_s,
                        &left_bottom_point_l);
  (*nearby_path)
      ->GetNearestPoint(right_bottom_point, &right_bottom_point_s,
                        &right_bottom_point_l);
  Vec2d vehicle_vec(vehicle_state_.x(), vehicle_state_.y());
  (*nearby_path)
      ->GetNearestPoint(vehicle_vec, &vehicle_point_s, &vehicle_point_l);
  if (std::abs((left_bottom_point_s + right_bottom_point_s) / 2 -
               vehicle_point_s) <
      config_.open_space_roi_decider_config().parking_start_range()) {
    return true;
  } else {
    return false;
  }
}

bool OpenSpaceRoiDecider::GetOpenSpaceInfo() {
  // Gather vertice needed by warm start and distance approach
  if (!VPresentationObstacle()) {
    AERROR << "fail at VPresentationObstacle()";
    return false;
  }
  // Transform vertices into the form of Ax>b
  if (!HPresentationObstacle()) {
    AERROR << "fail at HPresentationObstacle()";
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
