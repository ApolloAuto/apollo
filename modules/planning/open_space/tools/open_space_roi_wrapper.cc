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
 * @file
 **/

#include "Eigen/Dense"
#include "cyber/common/file.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planner_open_space_config.pb.h"

namespace apollo {
namespace planning {

using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::LaneSegment;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;

constexpr double kMathEpsilon = 1e-10;

class OpenSpaceROITest {
 public:
  bool OpenSpaceROI() {
    // left or right of the parking lot is decided when viewing the parking spot
    // open upward
    Vec2d left_top = target_parking_spot_->polygon().points().at(3);
    Vec2d left_down = target_parking_spot_->polygon().points().at(0);
    Vec2d right_top = target_parking_spot_->polygon().points().at(2);
    Vec2d right_down = target_parking_spot_->polygon().points().at(1);
    double left_top_s = 0.0;
    double left_top_l = 0.0;
    double right_top_s = 0.0;
    double right_top_l = 0.0;
    if (!(nearby_path_->GetProjection(left_top, &left_top_s, &left_top_l) &&
          nearby_path_->GetProjection(right_top, &right_top_s, &right_top_l))) {
      AERROR <<  "fail to get parking spot points' projections "
                 "on reference line";
      return false;
    }
    // start or end, left or right is decided by the vehicle's heading
    double center_line_s = (left_top_s + right_top_s) / 2;
    double start_s =
        center_line_s -
        planner_open_space_config_.roi_config().roi_longitudinal_range_start();
    double end_s =
        center_line_s +
        planner_open_space_config_.roi_config().roi_longitudinal_range_end();
    hdmap::MapPathPoint end_point = nearby_path_->GetSmoothPoint(end_s);
    hdmap::MapPathPoint start_point = nearby_path_->GetSmoothPoint(start_s);
    double start_left_width = nearby_path_->GetRoadLeftWidth(start_s);
    double start_right_width = nearby_path_->GetRoadRightWidth(start_s);
    double end_left_width = nearby_path_->GetRoadLeftWidth(end_s);
    double end_right_width = nearby_path_->GetRoadRightWidth(end_s);
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
    origin_heading_ = nearby_path_->GetSmoothPoint(center_line_s).heading();
    origin_point_.set_x(left_top.x());
    origin_point_.set_y(left_top.y());
    left_top -= origin_point_;
    left_down -= origin_point_;
    right_top -= origin_point_;
    right_down -= origin_point_;
    start_right -= origin_point_;
    start_left -= origin_point_;
    end_right -= origin_point_;
    end_left -= origin_point_;
    AINFO << "left_down x " << left_down.x();
    AINFO << "right_down x " << right_down.x();
    AINFO << "left_top x " << left_top.x();
    left_top.SelfRotate(-origin_heading_);
    left_down.SelfRotate(-origin_heading_);
    right_top.SelfRotate(-origin_heading_);
    right_down.SelfRotate(-origin_heading_);
    start_right.SelfRotate(-origin_heading_);
    start_left.SelfRotate(-origin_heading_);
    end_right.SelfRotate(-origin_heading_);
    end_left.SelfRotate(-origin_heading_);

    // get end_pose of the parking spot
    parking_spot_heading_ = (left_down - left_top).Angle();
    double end_x = (left_top.x() + right_top.x()) / 2;
    double end_y = 0.0;
    if (parking_spot_heading_ > kMathEpsilon) {
      if (planner_open_space_config_.roi_config().parking_inwards()) {
        end_y = left_top.y() + (left_down.y() - left_top.y()) / 4;
      } else {
        end_y = left_top.y() + 3 * (left_down.y() - left_top.y()) / 4;
      }
    } else {
      if (planner_open_space_config_.roi_config().parking_inwards()) {
        end_y = left_down.y() + 3 * (left_top.y() - left_down.y()) / 4;
      } else {
        end_y = left_down.y() + (left_top.y() - left_down.y()) / 4;
      }
    }
    open_space_end_pose_.emplace_back(end_x);
    open_space_end_pose_.emplace_back(end_y);
    if (planner_open_space_config_.roi_config().parking_inwards()) {
      open_space_end_pose_.emplace_back(parking_spot_heading_);
    } else {
      open_space_end_pose_.emplace_back(
          common::math::NormalizeAngle(parking_spot_heading_ + M_PI));
    }
    open_space_end_pose_.emplace_back(0.0);

    // get xy boundary of the ROI
    double x_min = std::min({start_left.x(), start_right.x()});
    double x_max = std::max({end_left.x(), end_right.x()});
    double y_min = std::min({left_down.y(), start_right.y(), start_left.y()});
    double y_max = std::max({left_down.y(), start_right.y(), start_left.y()});
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
      start_right = start_right + start_point;
      end_right.set_x(-left_top_l * end_right_vec_cos);
      end_right.set_y(-left_top_l * end_right_vec_sin);
      end_right = end_right + end_point;
      start_right -= origin_point_;
      end_right -= origin_point_;
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
      start_left = start_left + start_point;
      end_left.set_x(left_top_l * end_left_vec_cos);
      end_left.set_y(left_top_l * end_left_vec_sin);
      end_left = end_left + end_point;
      start_left -= origin_point_;
      end_left -= origin_point_;
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

  bool NoRotateOpenSpaceROI() {
    // left or right of the parking lot is decided when viewing the parking spot
    // open upward
    Vec2d left_top = target_parking_spot_->polygon().points().at(3);
    Vec2d left_down = target_parking_spot_->polygon().points().at(0);
    Vec2d right_top = target_parking_spot_->polygon().points().at(2);
    Vec2d right_down = target_parking_spot_->polygon().points().at(1);
    double left_top_s = 0.0;
    double left_top_l = 0.0;
    double right_top_s = 0.0;
    double right_top_l = 0.0;
    if (!(nearby_path_->GetProjection(left_top, &left_top_s, &left_top_l) &&
          nearby_path_->GetProjection(right_top, &right_top_s, &right_top_l))) {
      AERROR <<  "fail to get parking spot points' projections "
                 "on reference line";
      return false;
    }
    // start or end, left or right is decided by the vehicle's heading
    double center_line_s = (left_top_s + right_top_s) / 2;
    double start_s =
        center_line_s -
        planner_open_space_config_.roi_config().roi_longitudinal_range_start();
    double end_s =
        center_line_s +
        planner_open_space_config_.roi_config().roi_longitudinal_range_end();
    hdmap::MapPathPoint end_point = nearby_path_->GetSmoothPoint(end_s);
    hdmap::MapPathPoint start_point = nearby_path_->GetSmoothPoint(start_s);
    double start_left_width = nearby_path_->GetRoadLeftWidth(start_s);
    double start_right_width = nearby_path_->GetRoadRightWidth(start_s);
    double end_left_width = nearby_path_->GetRoadLeftWidth(end_s);
    double end_right_width = nearby_path_->GetRoadRightWidth(end_s);

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
    double x_min = std::min({start_left.x(), start_right.x()});
    double x_max = std::max({end_left.x(), end_right.x()});
    double y_min = std::min({left_down.y(), start_right.y(), start_left.y()});
    double y_max = std::max({left_down.y(), start_right.y(), start_left.y()});
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
      start_right = start_right + start_point;
      end_right.set_x(-left_top_l * end_right_vec_cos);
      end_right.set_y(-left_top_l * end_right_vec_sin);
      end_right = end_right + end_point;
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
    No_rotate_ROI_parking_boundary_.emplace_back(left_boundary);
    No_rotate_ROI_parking_boundary_.emplace_back(down_boundary);
    No_rotate_ROI_parking_boundary_.emplace_back(right_boundary);
    No_rotate_ROI_parking_boundary_.emplace_back(up_boundary);

    return true;
  }

  bool VPresentationObstacle(const std::string& lane_id,
                             const std::string& parking_id) {
    if (!LoadMap(lane_id, parking_id)) {
      AINFO << "fail at loading map";
      return false;
    }

    ACHECK(cyber::common::GetProtoFromFile(
        FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
        << "Failed to load open space config file "
        << FLAGS_planner_open_space_config_filename;

    // load info from pnc map
    if (!OpenSpaceROI()) {
      AINFO << "fail at ROI()";
      return false;
    }
    if (!NoRotateOpenSpaceROI()) {
      AINFO << "fail at ROI()";
      return false;
    }

    size_t parking_boundaries_num = ROI_parking_boundary_.size();

    if (parking_boundaries_num != 4) {
      AERROR << "parking boundary obstacles size not right";
      return false;
    }

    obstacles_num_ = parking_boundaries_num;
    // load vertice vector for distance approach
    Eigen::MatrixXd parking_boundaries_obstacles_edges_num(4, 1);
    // the order is decided by the ROI()
    parking_boundaries_obstacles_edges_num << 2, 1, 2, 1;
    obstacles_edges_num_.resize(parking_boundaries_obstacles_edges_num.rows(),
                                1);
    obstacles_edges_num_ << parking_boundaries_obstacles_edges_num;
    return true;
  }

  bool LoadPathBoundary() { return true; }

  bool LoadMap(const std::string& lane_id, const std::string& parking_id) {
    std::cout << lane_id << std::endl;
    std::cout << parking_id << std::endl;
    auto map_ptr = HDMapUtil::BaseMapPtr();
    hdmap::Id nearby_lane_id;
    nearby_lane_id.set_id(lane_id);
    hdmap::Id target_lane_id;
    target_lane_id.set_id(parking_id);
    auto nearby_lane = map_ptr->GetLaneById(nearby_lane_id);
    if (nearby_lane == nullptr) {
      std::cout << "No such lane found " << lane_id << std::endl;
      return false;
    }
    std::cout << "the lane found is " << nearby_lane->id().id() << std::endl;
    LaneSegment nearby_lanesegment =
        LaneSegment(nearby_lane, nearby_lane->accumulate_s().front(),
                    nearby_lane->accumulate_s().back());
    std::vector<LaneSegment> segments_vector;
    segments_vector.push_back(nearby_lanesegment);
    nearby_path_ = std::unique_ptr<Path>(new Path(segments_vector));
    const auto& parking_space_overlaps = nearby_path_->parking_space_overlaps();
    if (parking_space_overlaps.empty()) {
      std::cout << "No parking overlaps found on the lane requested"
                << std::endl;
      return false;
    }
    for (const auto parking_overlap : parking_space_overlaps) {
      if (parking_overlap.object_id != parking_id) {
        target_parking_spot_ = map_ptr->GetParkingSpaceById(target_lane_id);
        std::cout << "parking_overlap.object_id is " << target_lane_id.id()
                  << std::endl;
      }
    }
    if (target_parking_spot_ == nullptr) {
      std::cout << "No such parking spot found " << parking_id << std::endl;
      return false;
    }
    parking_spot_box_ = target_parking_spot_->polygon();
    return true;
  }

  std::vector<double>* GetROIXYBoundary() { return &ROI_xy_boundary_; }
  std::vector<std::vector<Vec2d>>* GetROIParkingBoundary() {
    return &ROI_parking_boundary_;
  }
  std::vector<std::vector<Vec2d>>* GetNoRotateROIParkingBoundary() {
    return &No_rotate_ROI_parking_boundary_;
  }
  std::vector<double>* GetEndPose() { return &open_space_end_pose_; }
  double GetOriginHeading() { return origin_heading_; }
  Vec2d GetOriginPose() { return origin_point_; }
  Polygon2d GetParkingSpotBox() { return parking_spot_box_; }

 private:
  apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;
  ParkingSpaceInfoConstPtr target_parking_spot_ = nullptr;
  Polygon2d parking_spot_box_;
  std::unique_ptr<Path> nearby_path_ = nullptr;
  size_t obstacles_num_ = 0;
  Eigen::MatrixXd obstacles_edges_num_;
  std::vector<double> ROI_xy_boundary_;
  std::vector<std::vector<Vec2d>> ROI_parking_boundary_;
  std::vector<std::vector<Vec2d>> No_rotate_ROI_parking_boundary_;
  std::vector<double> open_space_end_pose_;
  double origin_heading_;
  Vec2d origin_point_;
  double parking_spot_heading_;
};

extern "C" {
OpenSpaceROITest* CreateROITestPtr() { return new OpenSpaceROITest(); }
// all data in form of array
bool ROITest(OpenSpaceROITest* test_ptr, char* lane_id, char* parking_id,
             double* unrotated_roi_boundary_x, double* unrotated_roi_boundary_y,
             double* roi_boundary_x, double* roi_boundary_y,
             double* parking_spot_x, double* parking_spot_y, double* end_pose,
             double* xy_boundary, double* origin_pose) {
  std::string lane_id_str(lane_id);
  std::string parking_id_str(parking_id);
  if (!test_ptr->VPresentationObstacle(lane_id_str, parking_id_str)) {
    AINFO << "VPresentationObstacle fail";
    return false;
  }
  std::vector<std::vector<Vec2d>>* unrotated_roi_boundary_ =
      test_ptr->GetNoRotateROIParkingBoundary();
  std::vector<std::vector<Vec2d>>* roi_boundary_ =
      test_ptr->GetROIParkingBoundary();
  Polygon2d parking_spot_ = test_ptr->GetParkingSpotBox();
  std::vector<double>* end_pose_ = test_ptr->GetEndPose();
  std::vector<double>* xy_boundary_ = test_ptr->GetROIXYBoundary();
  double origin_heading_ = test_ptr->GetOriginHeading();
  Vec2d origin_point_ = test_ptr->GetOriginPose();

  // load all into array
  size_t index = 0;
  for (size_t i = 0; i < unrotated_roi_boundary_->size(); i++) {
    std::vector<Vec2d> boundary = unrotated_roi_boundary_->at(i);
    for (size_t j = 0; j < boundary.size(); j++) {
      unrotated_roi_boundary_x[index] = boundary[j].x();
      unrotated_roi_boundary_y[index] = boundary[j].y();
      index++;
    }
  }

  index = 0;
  for (size_t i = 0; i < roi_boundary_->size(); i++) {
    std::vector<Vec2d> boundary = roi_boundary_->at(i);
    for (size_t j = 0; j < boundary.size(); j++) {
      roi_boundary_x[index] = boundary[j].x();
      roi_boundary_y[index] = boundary[j].y();
      index++;
    }
  }

  index = 0;
  std::vector<Vec2d> parking_spot_vec_ = parking_spot_.points();
  for (size_t i = 0; i < parking_spot_vec_.size(); i++) {
    parking_spot_x[index] = parking_spot_vec_[i].x();
    parking_spot_y[index] = parking_spot_vec_[i].y();
    index++;
  }

  for (size_t i = 0; i < end_pose_->size(); i++) {
    end_pose[i] = end_pose_->at(i);
  }

  for (size_t i = 0; i < xy_boundary_->size(); i++) {
    xy_boundary[i] = xy_boundary_->at(i);
  }

  // x, y, heading
  origin_pose[0] = origin_point_.x();
  origin_pose[1] = origin_point_.y();
  origin_pose[2] = origin_heading_;

  return true;
}
};

}  // namespace planning
}  // namespace apollo
