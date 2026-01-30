/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planning_open_space/utils/open_space_roi_util.h"

#include <limits>
#include <memory>
#include <numeric>
#include <utility>

#include "modules/common/util/point_factory.h"

namespace apollo {
namespace planning {

using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

bool OpenSpaceRoiUtil::FormulateBoundaryConstraints(OpenSpaceInfo *const open_space_info) {
    auto &obstacles_vertices_vec = *open_space_info->mutable_obstacles_vertices_vec();
    auto &obstacles_edges_num_vec = *open_space_info->mutable_obstacles_edges_num();
    size_t obstacles_vertices_vec_num = obstacles_vertices_vec.size();
    obstacles_edges_num_vec.resize(obstacles_vertices_vec_num, 1);
    for (size_t i = 0; i < obstacles_vertices_vec_num; i++) {
        CHECK_GT(obstacles_vertices_vec[i].size(), 1U);
        obstacles_edges_num_vec(i, 0) = static_cast<int>(obstacles_vertices_vec[i].size()) - 1;
    }
    open_space_info->set_obstacles_num(obstacles_vertices_vec.size());
    // Transform vertices into the form of Ax>b
    if (!LoadObstacleInHyperPlanes(open_space_info)) {
        AERROR << "fail at LoadObstacleInHyperPlanes()";
        return false;
    }
    return true;
}

void OpenSpaceRoiUtil::GetRoiXYBoundary(
        const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
        std::vector<double> *const XYBoundary) {
    XYBoundary->clear();
    XYBoundary->resize(4);  // x_min x_max y_min y_max
    *XYBoundary
            = {std::numeric_limits<double>::max(),
               std::numeric_limits<double>::lowest(),
               std::numeric_limits<double>::max(),
               std::numeric_limits<double>::lowest()};
    for (auto &bound_vec : roi_parking_boundary) {
        for (auto &pt : bound_vec) {
            XYBoundary->at(0) = std::min<double>(XYBoundary->at(0), pt.x());
            XYBoundary->at(1) = std::max<double>(XYBoundary->at(1), pt.x());
            XYBoundary->at(2) = std::min<double>(XYBoundary->at(2), pt.y());
            XYBoundary->at(3) = std::max<double>(XYBoundary->at(3), pt.y());
        }
    }
}

void OpenSpaceRoiUtil::TransformByOriginPoint(
        const common::math::Vec2d &origin_point,
        const double heading,
        std::vector<std::vector<common::math::Vec2d>> *roi_parking_boundary) {
    for (auto &bound_vec : *roi_parking_boundary) {
        TransformByOriginPoint(origin_point, heading, &bound_vec);
    }
}

void OpenSpaceRoiUtil::TransformByOriginPoint(
        const common::math::Vec2d &origin_point,
        const double heading,
        std::vector<common::math::Vec2d> *roi_parking_boundary) {
    for (auto &point : *roi_parking_boundary) {
        TransformByOriginPoint(origin_point, heading, &point);
    }
}

void OpenSpaceRoiUtil::TransformByOriginPoint(
        const common::math::Vec2d &origin_point,
        const double heading,
        common::math::Vec2d *point) {
    *point -= origin_point;
    point->SelfRotate(-heading);
}

bool OpenSpaceRoiUtil::LoadObstacles(
        const double filtering_distance,
        const double perception_obstacle_buffer,
        const std::vector<double> &xy_boundary,
        Frame *const frame,
        std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
    auto obstacles_by_frame = frame->GetObstacleList();
    size_t perception_obstacles_num = 0;
    // load vertices for perception obstacles(repeat the first vertice at the
    // last to form closed convex hull)
    for (const auto &obstacle : obstacles_by_frame->Items()) {
        if (FilterOutObstacle(filtering_distance, *frame, xy_boundary, *obstacle)) {
            continue;
        }
        ++perception_obstacles_num;

        Box2d original_box = obstacle->PerceptionBoundingBox();
        original_box.LongitudinalExtend(perception_obstacle_buffer);
        original_box.LateralExtend(perception_obstacle_buffer);

        // TODO(Jinyun): Check correctness of ExpandByDistance() in polygon
        // Polygon2d buffered_box(original_box);
        // buffered_box = buffered_box.ExpandByDistance(
        //    config_.perception_obstacle_buffer());
        // TODO(Runxin): Rotate from origin instead
        // original_box.RotateFromCenter(-1.0 * origin_heading);
        std::vector<Vec2d> vertices_ccw = original_box.GetAllCorners();
        std::vector<Vec2d> vertices_cw;
        while (!vertices_ccw.empty()) {
            auto current_corner_pt = vertices_ccw.back();
            vertices_cw.push_back(current_corner_pt);
            vertices_ccw.pop_back();
        }
        // As the perception obstacle is a closed convex set, the first vertice
        // is repeated at the end of the vector to help transform all four edges
        // to inequality constraint
        vertices_cw.push_back(vertices_cw.front());
        roi_parking_boundary->push_back(vertices_cw);
    }

    return true;
}

bool OpenSpaceRoiUtil::FilterOutObstacle(
        const double filtering_distance,
        const Frame &frame,
        const std::vector<double> &xy_boundary,
        const Obstacle &obstacle) {
    if (obstacle.IsVirtual() || !obstacle.IsStatic()) {
        return true;
    }

    const auto &open_space_info = frame.open_space_info();
    const auto &origin_point = open_space_info.origin_point();
    const auto &origin_heading = open_space_info.origin_heading();
    const auto &obstacle_box = obstacle.PerceptionBoundingBox();
    auto obstacle_center_xy = obstacle_box.center();

    // xy_boundary in xmin, xmax, ymin, ymax.
    obstacle_center_xy -= origin_point;
    obstacle_center_xy.SelfRotate(-origin_heading);
    if (obstacle_center_xy.x() < xy_boundary[0] || obstacle_center_xy.x() > xy_boundary[1]
        || obstacle_center_xy.y() < xy_boundary[2] || obstacle_center_xy.y() > xy_boundary[3]) {
        return true;
    }

    // Translate the end pose back to world frame with endpose in x, y, phi, v
    const auto &end_pose = open_space_info.open_space_end_pose();
    Vec2d end_pose_x_y(end_pose[0], end_pose[1]);
    end_pose_x_y.SelfRotate(origin_heading);
    end_pose_x_y += origin_point;
    const auto &vehicle_state = frame.vehicle_state();
    // Get vehicle state
    Vec2d vehicle_x_y(vehicle_state.x(), vehicle_state.y());

    // Use vehicle position and end position to filter out obstacle
    const double vehicle_center_to_obstacle = obstacle_box.DistanceTo(vehicle_x_y);
    const double end_pose_center_to_obstacle = obstacle_box.DistanceTo(end_pose_x_y);
    if (vehicle_center_to_obstacle > filtering_distance && end_pose_center_to_obstacle > filtering_distance) {
        return true;
    }
    return false;
}

bool OpenSpaceRoiUtil::LoadObstacleInHyperPlanes(OpenSpaceInfo *const open_space_info) {
    *(open_space_info->mutable_obstacles_A()) = Eigen::MatrixXd::Zero(open_space_info->obstacles_edges_num().sum(), 2);
    *(open_space_info->mutable_obstacles_b()) = Eigen::MatrixXd::Zero(open_space_info->obstacles_edges_num().sum(), 1);
    // vertices using H-representation
    if (!GetHyperPlanes(
                open_space_info->obstacles_num(),
                open_space_info->obstacles_edges_num(),
                open_space_info->obstacles_vertices_vec(),
                open_space_info->mutable_obstacles_A(),
                open_space_info->mutable_obstacles_b())) {
        AERROR << "Fail to present obstacle in hyperplane";
        return false;
    }
    return true;
}

bool OpenSpaceRoiUtil::GetHyperPlanes(
        const size_t &obstacles_num,
        const Eigen::MatrixXi &obstacles_edges_num,
        const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
        Eigen::MatrixXd *A_all,
        Eigen::MatrixXd *b_all) {
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

void OpenSpaceRoiUtil::TransformByOriginPoint(
        const Vec2d &origin_point,
        const double &origin_heading,
        OpenSpaceInfo *open_space_info) {
    open_space_info->set_origin_heading(origin_heading);
    *open_space_info->mutable_origin_point() = origin_point;
    auto obstacles_vertices_vec = open_space_info->mutable_obstacles_vertices_vec();
    TransformByOriginPoint(origin_point, origin_heading, obstacles_vertices_vec);
    auto end_pose_vec = open_space_info->mutable_open_space_end_pose();
    Vec2d end_pose(end_pose_vec->at(0), end_pose_vec->at(1));
    TransformByOriginPoint(origin_point, origin_heading, &end_pose);
    end_pose_vec->at(0) = end_pose.x();
    end_pose_vec->at(1) = end_pose.y();
    end_pose_vec->at(2) -= origin_heading;
    auto xy_boundary = open_space_info->mutable_ROI_xy_boundary();
    GetRoiXYBoundary(*obstacles_vertices_vec, xy_boundary);
}
bool OpenSpaceRoiUtil::IsPolygonClockwise(const std::vector<Vec2d> &polygon) {
    double s = 0;
    for (size_t i = 0; i < polygon.size(); i++) {
        if (i + 1 < polygon.size()) {
            s += polygon.at(i).x() * polygon.at(i + 1).y() - polygon.at(i).y() * polygon.at(i + 1).x();
        } else {
            s += polygon.at(i).x() * polygon.at(0).y() - polygon.at(i).y() * polygon.at(0).x();
        }
    }
    if (s < 0) {
        return true;
    } else {
        return false;
    }
}

bool OpenSpaceRoiUtil::AdjustPointsOrderToClockwise(std::vector<Vec2d> *polygon) {
    if (!IsPolygonClockwise(*polygon)) {
        // counter clockwise reverse it
        ADEBUG << "point is anticlockwise,reverse";
        std::reverse(polygon->begin(), polygon->end());
        return true;
    } else {
        return false;
    }
}
// make points order as left top ,right top, right bottom, left bottom
bool OpenSpaceRoiUtil::UpdateParkingPointsOrder(const apollo::hdmap::Path &nearby_path, std::vector<Vec2d> *points) {
    AdjustPointsOrderToClockwise(points);
    double min_dist = std::numeric_limits<double>::max();
    size_t min_index = 0;
    double sum_l = 0;
    for (size_t i = 0; i < points->size(); i++) {
        double s = 0.0, l = 0.0;
        nearby_path.GetProjection(points->at(i), &s, &l);
        sum_l += l;
        ADEBUG << std::fixed << points->at(i).x() << "," << points->at(i).y() << "sl" << s << "," << l;
        if (std::fabs(s) + 2.0 * std::fabs(l) < min_dist) {
            min_dist = std::fabs(s) + 2.0 * std::fabs(l);
            min_index = i;
        }
    }
    std::vector<Vec2d> tmp_points(*points);
    for (size_t i = 0; i < points->size(); i++) {
        tmp_points[i] = points->at((i + min_index) % points->size());
    }
    if (sum_l < 0) {
        for (size_t i = 0; i < points->size(); i++) {
            points->at(i) = tmp_points[i];
        }
    } else {
        points->at(0) = tmp_points[0];
        for (size_t i = points->size() - 1; i > 0; i--) {
            points->at(points->size() - i) = tmp_points[i];
        }
    }
    return true;
}

}  // namespace planning
}  // namespace apollo
