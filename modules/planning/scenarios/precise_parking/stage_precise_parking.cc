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

#include "modules/planning/scenarios/precise_parking/stage_precise_parking.h"

#include <string>
#include <vector>

#include "modules/planning/planning_open_space/utils/open_space_roi_util.h"

namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;

bool StagePreciseParking::Init(
        const StagePipeline &config,
        const std::shared_ptr<DependencyInjector> &injector,
        const std::string &config_dir,
        void *context) {
    CHECK_NOTNULL(context);
    bool ret = Stage::Init(config, injector, config_dir, context);
    if (!ret) {
        AERROR << Name() << "init failed!";
        return false;
    }
    precise_parking_context_ = GetContextAs<PreciseParkingContext>();
    return ret;
}

StageResult StagePreciseParking::Process(const common::TrajectoryPoint &planning_init_point, Frame *frame) {
    auto *mutable_open_space_info = frame->mutable_open_space_info();

    GetOpenspaceRoi(frame);
    frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
    StageResult result = ExecuteTaskOnOpenSpace(frame);
    if (result.HasError()) {
        AERROR << "StagePreceseParking planning error";
        return result.SetStageStatus(StageStatusType::ERROR);
    }

    if (frame->open_space_info().destination_reached()) {
        AINFO << "StagePreceseParking destination reached";
        return FinishStage(frame);
    }

    return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult StagePreciseParking::FinishStage(Frame *frame) {
    next_stage_ = "ARRIVE_PARKING_SPOT";
    frame->mutable_open_space_info()->mutable_optimizer_trajectory_data()->clear();
    frame->mutable_open_space_info()->mutable_path_planning_trajectory_result()->clear();
    frame->mutable_open_space_info()->mutable_interpolated_trajectory_result()->clear();
    frame->mutable_open_space_info()->mutable_partitioned_trajectories()->clear();
    frame->mutable_open_space_info()->mutable_chosen_partitioned_trajectory()->first.clear();
    return StageResult(StageStatusType::FINISHED);
}

bool StagePreciseParking::GetOpenspaceRoi(Frame *frame) {
    const auto &hdmap = hdmap::HDMapUtil::BaseMapPtr();
    // const auto &junction
    //         =
    //         hdmap->GetJunctionById(hdmap::MakeMapId(precise_parking_context_->precise_parking_command.junction_id()));
    common::PointENU point;
    std::vector<hdmap::AreaInfoConstPtr> areas;
    point.set_x(precise_parking_context_->precise_parking_command.parking_spot_pose().x());
    point.set_y(precise_parking_context_->precise_parking_command.parking_spot_pose().y());
    if (hdmap::HDMapUtil::BaseMapPtr()->GetAreas(point, 1e-1, &areas) < 0) {
        AERROR << "can not find any junction near parking spot";
        return false;
    };
    const auto &area_polygon = areas.front()->polygon();

    double ego_x = injector_->ego_info()->vehicle_state().x();
    double ego_y = injector_->ego_info()->vehicle_state().y();
    Vec2d ego_pos(ego_x, ego_y);

    auto points = area_polygon.points();
    OpenSpaceRoiUtil::AdjustPointsOrderToClockwise(&points);
    std::reverse(points.begin(), points.end());
    AINFO << "Junction polygon is anticlockwise";

    SetOrigin(points, ego_pos, frame);

    // get target pose
    GetEndPose(frame);

    // get roi boundary
    std::vector<std::vector<common::math::Vec2d>> roi_parking_boundary;
    std::vector<common::math::Vec2d> parking_point;
    for (const auto point : points) {
        parking_point.emplace_back(point.x(), point.y());
        parking_point.back() -= frame->open_space_info().origin_point();
        parking_point.back().SelfRotate(-frame->open_space_info().origin_heading());
    }
    parking_point.push_back(parking_point.front());

    for (int i = 0; i < parking_point.size() - 1; ++i) {
        roi_parking_boundary.emplace_back(std::vector<Vec2d>{parking_point.at(i), parking_point.at(i + 1)});
    }

    // Fuse line segments into convex contraints
    if (!FuseLineSegments(roi_parking_boundary)) {
        AERROR << "FuseLineSegments failed in parking ROI";
        return false;
    }

    // Get xy boundary
    auto xminmax = std::minmax_element(
            parking_point.begin(), parking_point.end(), [](const Vec2d &a, const Vec2d &b) { return a.x() < b.x(); });
    auto yminmax = std::minmax_element(
            parking_point.begin(), parking_point.end(), [](const Vec2d &a, const Vec2d &b) { return a.y() < b.y(); });
    std::vector<double> ROI_xy_boundary{
            xminmax.first->x(), xminmax.second->x(), yminmax.first->y(), yminmax.second->y()};
    auto *xy_boundary = frame->mutable_open_space_info()->mutable_ROI_xy_boundary();
    xy_boundary->assign(ROI_xy_boundary.begin(), ROI_xy_boundary.end());

    ego_pos -= frame->open_space_info().origin_point();
    ego_pos.SelfRotate(-frame->open_space_info().origin_heading());
    if (ego_pos.x() < ROI_xy_boundary[0] || ego_pos.x() > ROI_xy_boundary[1] || ego_pos.y() < ROI_xy_boundary[2]
        || ego_pos.y() > ROI_xy_boundary[3]) {
        AERROR << "vehicle outside of xy boundary of parking ROI";
        return false;
    }
    if (!FormulateBoundaryConstraints(roi_parking_boundary, frame)) {
        const std::string msg = "Fail to formulate boundary constraints";
        AERROR << msg;
        return false;
    }
    return true;
}

void StagePreciseParking::SetOrigin(const std::vector<Vec2d> &points, const Vec2d &ego_pos, Frame *frame) {
    auto *previous_frame = injector_->frame_history()->Latest();
    if (previous_frame->open_space_info().origin_point().x() != 0
        && previous_frame->open_space_info().origin_point().y() != 0
        && previous_frame->open_space_info().origin_heading() != 0) {
        frame->mutable_open_space_info()->set_origin_heading(previous_frame->open_space_info().origin_heading());
        frame->mutable_open_space_info()->mutable_origin_point()->set_x(
                previous_frame->open_space_info().origin_point().x());
        frame->mutable_open_space_info()->mutable_origin_point()->set_y(
                previous_frame->open_space_info().origin_point().y());
    } else {
        double distance = std::numeric_limits<double>::max();
        int min_index = 0;
        for (int index = 0; index < points.size(); index++) {
            const auto &point = points[index];
            if ((point - ego_pos).Length() < distance) {
                distance = (point - ego_pos).Length();
                min_index = index;
            }
        }
        double angle = 0;
        if (min_index == points.size() - 1) {
            angle = (points[0] - points[min_index]).Angle();
        } else {
            angle = (points[min_index + 1] - points[min_index]).Angle();
        }
        frame->mutable_open_space_info()->set_origin_heading(angle);
        frame->mutable_open_space_info()->mutable_origin_point()->set_x(points[min_index].x());
        frame->mutable_open_space_info()->mutable_origin_point()->set_y(points[min_index].y());
    }
}

void StagePreciseParking::GetEndPose(Frame *frame) {
    auto parking_spot_pose = precise_parking_context_->precise_parking_command.parking_spot_pose();
    Vec2d end_pt = Vec2d(parking_spot_pose.x(), parking_spot_pose.y());
    Vec2d vec = precise_parking_context_->precise_parking_command.parking_inwards()
            ? precise_parking_context_->scenario_config.precise_trajectory_length()
                    * Vec2d(std::cos(parking_spot_pose.heading()), std::sin(parking_spot_pose.heading())) * -1.0
            : precise_parking_context_->scenario_config.precise_trajectory_length()
                    * Vec2d(std::cos(parking_spot_pose.heading()), std::sin(parking_spot_pose.heading()));
    end_pt += vec;
    const auto &origin_point = frame->open_space_info().origin_point();
    const auto &origin_heading = frame->open_space_info().origin_heading();
    end_pt -= origin_point;
    end_pt.SelfRotate(-origin_heading);

    auto *end_pose = frame->mutable_open_space_info()->mutable_open_space_end_pose();
    end_pose->push_back(end_pt.x());
    end_pose->push_back(end_pt.y());
    end_pose->push_back(parking_spot_pose.heading() - origin_heading);
    end_pose->push_back(0.0);
}

bool StagePreciseParking::FuseLineSegments(std::vector<std::vector<common::math::Vec2d>> &line_segments_vec) {
    static constexpr double kEpsilon = 1.0e-8;
    auto cur_segment = line_segments_vec.begin();
    while (cur_segment != line_segments_vec.end() - 1) {
        auto next_segment = cur_segment + 1;
        auto cur_last_point = cur_segment->back();
        auto next_first_point = next_segment->front();
        // Check if they are the same points
        if (cur_last_point.DistanceTo(next_first_point) > kEpsilon) {
            ++cur_segment;
            continue;
        }
        if (cur_segment->size() < 2 || next_segment->size() < 2) {
            AERROR << "Single point line_segments vec not expected";
            return false;
        }
        size_t cur_segments_size = cur_segment->size();
        auto cur_second_to_last_point = cur_segment->at(cur_segments_size - 2);
        auto next_second_point = next_segment->at(1);
        if (common::math::CrossProd(cur_second_to_last_point, cur_last_point, next_second_point) < 0.0) {
            cur_segment->push_back(next_second_point);
            next_segment->erase(next_segment->begin(), next_segment->begin() + 2);
            if (next_segment->empty()) {
                line_segments_vec.erase(next_segment);
            }
        } else {
            ++cur_segment;
        }
    }
    return true;
}

bool StagePreciseParking::FormulateBoundaryConstraints(
        const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
        Frame *const frame) {
    // Gather vertice needed by warm start and distance approach
    if (!LoadObstacleInVertices(roi_parking_boundary, frame)) {
        AERROR << "fail at LoadObstacleInVertices()";
        return false;
    }
    // Transform vertices into the form of Ax>b
    if (!OpenSpaceRoiUtil::LoadObstacleInHyperPlanes(frame->mutable_open_space_info())) {
        AERROR << "fail at LoadObstacleInHyperPlanes()";
        return false;
    }
    return true;
}

bool StagePreciseParking::LoadObstacleInVertices(
        const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
        Frame *const frame) {
    auto *mutable_open_space_info = frame->mutable_open_space_info();
    const auto &open_space_info = frame->open_space_info();
    auto *obstacles_vertices_vec = mutable_open_space_info->mutable_obstacles_vertices_vec();
    auto *obstacles_edges_num_vec = mutable_open_space_info->mutable_obstacles_edges_num();

    // load vertices for parking boundary (not need to repeat the first
    // vertice to get close hull)
    size_t parking_boundaries_num = roi_parking_boundary.size();
    size_t perception_obstacles_num = 0;

    for (size_t i = 0; i < parking_boundaries_num; ++i) {
        obstacles_vertices_vec->push_back(roi_parking_boundary[i]);
    }

    Eigen::MatrixXi parking_boundaries_obstacles_edges_num(parking_boundaries_num, 1);
    for (size_t i = 0; i < parking_boundaries_num; i++) {
        if (roi_parking_boundary[i].size() <= 1U) {
            AERROR << "Roi parking boundary is invalid: " << roi_parking_boundary[i].size();
            return false;
        }
        parking_boundaries_obstacles_edges_num(i, 0) = static_cast<int>(roi_parking_boundary[i].size()) - 1;
    }

    AINFO << precise_parking_context_->scenario_config.enable_perception_obstacles();
    if (precise_parking_context_->scenario_config.enable_perception_obstacles()) {
        if (perception_obstacles_num == 0) {
            ADEBUG << "no obstacle given by perception";
        }
        // load vertices for perception obstacles(repeat the first vertice at the
        // last to form closed convex hull)
        const auto &origin_point = open_space_info.origin_point();
        const auto &origin_heading = open_space_info.origin_heading();
        ThreadSafeIndexedObstacles *obstacles_by_frame = frame->GetObstacleList();
        AINFO << obstacles_by_frame->Items().size();
        for (const auto &obstacle : obstacles_by_frame->Items()) {
            if (OpenSpaceRoiUtil::FilterOutObstacle(100, *frame, frame->open_space_info().ROI_xy_boundary(), *obstacle)) {
                AINFO << "obstacle is filtered out";
                continue;
            }
            ++perception_obstacles_num;

            std::vector<Vec2d> vertices_ccw;
            if (precise_parking_context_->scenario_config.expand_polygon_of_obstacle_by_distance()) {
                common::math::Polygon2d original_polygon = obstacle->PerceptionPolygon();
                original_polygon.ExpandByDistance(
                        precise_parking_context_->scenario_config.perception_obstacle_buffer());
                original_polygon.CalculateVertices(-1.0 * origin_point);
                vertices_ccw = original_polygon.GetAllVertices();
            } else {
                Box2d original_box = obstacle->PerceptionBoundingBox();
                original_box.Shift(-1.0 * origin_point);
                original_box.LongitudinalExtend(precise_parking_context_->scenario_config.perception_obstacle_buffer());
                original_box.LateralExtend(precise_parking_context_->scenario_config.perception_obstacle_buffer());
                vertices_ccw = original_box.GetAllCorners();
            }

            // TODO(Jinyun): Check correctness of ExpandByDistance() in polygon
            // Polygon2d buffered_box(original_box);
            // buffered_box = buffered_box.ExpandByDistance(
            //    config_.perception_obstacle_buffer());
            // TODO(Runxin): Rotate from origin instead
            // original_box.RotateFromCenter(-1.0 * origin_heading);

            std::vector<Vec2d> vertices_cw;
            while (!vertices_ccw.empty()) {
                auto current_corner_pt = vertices_ccw.back();
                current_corner_pt.SelfRotate(-1.0 * origin_heading);
                vertices_cw.push_back(current_corner_pt);
                vertices_ccw.pop_back();
            }
            // As the perception obstacle is a closed convex set, the first vertice
            // is repeated at the end of the vector to help transform all four edges
            // to inequality constraint
            vertices_cw.push_back(vertices_cw.front());
            AINFO << "obstacle vertices: " << vertices_cw.size();
            obstacles_vertices_vec->push_back(vertices_cw);
        }

        // obstacle boundary box is used, thus the edges are set to be 4
        Eigen::MatrixXi perception_obstacles_edges_num = 4 * Eigen::MatrixXi::Ones(perception_obstacles_num, 1);

        obstacles_edges_num_vec->resize(
                parking_boundaries_obstacles_edges_num.rows() + perception_obstacles_edges_num.rows(), 1);
        *(obstacles_edges_num_vec) << parking_boundaries_obstacles_edges_num, perception_obstacles_edges_num;

    } else {
        obstacles_edges_num_vec->resize(parking_boundaries_obstacles_edges_num.rows(), 1);
        *(obstacles_edges_num_vec) << parking_boundaries_obstacles_edges_num;
    }

    mutable_open_space_info->set_obstacles_num(parking_boundaries_num + perception_obstacles_num);
    return true;
}

}  // namespace planning
}  // namespace apollo
