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

#include "modules/planning/tasks/open_space_roi_decider_park/open_space_roi_decider_park.h"

#include <limits>
#include <memory>
#include <utility>

#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning/planning_open_space/utils/open_space_roi_util.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
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

bool OpenSpaceRoiDeciderPark::Init(
        const std::string &config_dir,
        const std::string &name,
        const std::shared_ptr<DependencyInjector> &injector) {
    if (!Decider::Init(config_dir, name, injector)) {
        return false;
    }
    hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
    CHECK_NOTNULL(hdmap_);
    vehicle_params_ = apollo::common::VehicleConfigHelper::GetConfig().vehicle_param();
    // Load the config this task.
    bool res = Decider::LoadConfig<OpenSpaceRoiDeciderParkConfig>(&config_);
    AINFO << config_.DebugString();
    return res;
}

Status OpenSpaceRoiDeciderPark::Process(Frame *frame) {
    if (frame == nullptr) {
        const std::string msg = "Invalid frame, fail to process the OpenSpaceRoiDeciderPark.";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    vehicle_state_ = frame->vehicle_state();
    obstacles_by_frame_ = frame->GetObstacleList();

    std::array<Vec2d, 4> spot_vertices;
    Path nearby_path;
    // @brief vector of different obstacle consisting of vertice points.The
    // obstacle and the vertices order are in counter-clockwise order
    std::vector<std::vector<common::math::Vec2d>> roi_boundary;
    std::vector<std::vector<common::math::Vec2d>> soft_boundary;

    const auto &roi_type = config_.roi_type();
    if (roi_type == OpenSpaceRoiDeciderParkConfig::PARKING) {
        target_parking_spot_id_ = frame->open_space_info().target_parking_spot_id();
        ParkingInfo parking_info;
        if (!GetParkingSpot(frame, &parking_info)) {
            const std::string msg = "Fail to get parking boundary from map";
            AERROR << msg;
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }

        frame->mutable_open_space_info()->set_parking_type(parking_info.parking_type);

        SetOrigin(parking_info, frame);

        SetParkingSpotEndPose(parking_info, frame);

        if (!GetParkingBoundary(parking_info, *nearby_path_, frame, &roi_boundary, &soft_boundary)) {
            const std::string msg = "Fail to get parking boundary from map";
            AERROR << msg;
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }
    } else if (roi_type == OpenSpaceRoiDeciderParkConfig::PULL_OVER) {
        if (!GetPullOverSpot(frame, &spot_vertices, &nearby_path)) {
            const std::string msg = "Fail to get parking boundary from map";
            AERROR << msg;
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }

        SetOrigin(frame, spot_vertices);

        SetPullOverSpotEndPose(frame);

        if (!GetPullOverBoundary(frame, spot_vertices, nearby_path, &roi_boundary)) {
            const std::string msg = "Fail to get parking boundary from map";
            AERROR << msg;
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }
    } else if (roi_type == OpenSpaceRoiDeciderParkConfig::PARK_AND_GO) {
        ADEBUG << "in Park_and_Go";
        nearby_path = frame->reference_line_info().front().reference_line().GetMapPath();

        ADEBUG << "nearby_path: " << nearby_path.DebugString();
        ADEBUG << "found nearby_path";
        if (!injector_->planning_context()->planning_status().park_and_go().has_adc_init_position()) {
            const std::string msg = "ADC initial position is unavailable";
            AERROR << msg;
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }
        SetOriginFromADC(frame, nearby_path);
        ADEBUG << "SetOrigin";
        auto adc_point = common::util::PointFactory::ToPointENU(vehicle_state_);
        hdmap::LaneInfoConstPtr lane;
        double s = 0.0;
        double l = 0.0;
        if (!is_parking_out) {
            is_parking_out = HDMapUtil::BaseMap().GetNearestLaneWithHeading(
                                     adc_point, 2.0, vehicle_state_.heading(), M_PI / 3.0, &lane, &s, &l)
                    == -1;
        }
        if (is_parking_out) {
            AINFO << "GetParkingOutBoundary!!";
            if (!GetParkingOutBoundary(nearby_path, frame, &roi_boundary)) {
                const std::string msg = "Fail to get park and go boundary from map";
                AERROR << msg;
                return Status(ErrorCode::PLANNING_ERROR, msg);
            }
        } else {
            AINFO << "GetParkAndGoBoundary!!!";
            if (!GetParkAndGoBoundary(frame, nearby_path, &roi_boundary)) {
                const std::string msg = "Fail to get park and go boundary from map";
                AERROR << msg;
                return Status(ErrorCode::PLANNING_ERROR, msg);
            }
        }

        SetParkAndGoEndPose(frame);
        ADEBUG << "SetEndPose";
    } else if (roi_type == OpenSpaceRoiDeciderParkConfig::LARGE_CURVATURE) {
        ADEBUG << "in LARGE_CURVATURE";
        nearby_path = frame->reference_line_info().front().reference_line().GetMapPath();
        SetOriginFromADC(frame, nearby_path);
        // get plan boundary
        Vec2d uturn_end_pos;
        double uturn_end_heading;
        if (!GetUTurnPath(frame, &nearby_path_) || !GetLargeCurvatureBoundary(frame, *nearby_path_, &roi_boundary)) {
            const std::string msg = "Fail to get large curvature boundary from map";
            AERROR << msg;
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }
        GetLargeCurvatureEndPose(frame, uturn_end_pos, uturn_end_heading);
    } else {
        const std::string msg = "chosen open space roi secenario type not implemented";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    if (!FormulateBoundaryConstraints(roi_boundary, frame)) {
        const std::string msg = "Fail to formulate boundary constraints";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    *(frame->mutable_open_space_info()->mutable_soft_boundary_vertices_vec()) = soft_boundary;

    return Status::OK();
}

// get origin from ADC
void OpenSpaceRoiDeciderPark::SetOriginFromADC(Frame *const frame, const hdmap::Path &nearby_path) {
    const double adc_init_x = vehicle_state_.x();
    const double adc_init_y = vehicle_state_.y();
    const double adc_init_heading = vehicle_state_.heading();
    common::math::Vec2d adc_init_position = {adc_init_x, adc_init_y};
    const double adc_length = vehicle_params_.length();
    const double adc_width = vehicle_params_.width();
    // ADC box
    Box2d adc_box(adc_init_position, adc_init_heading, adc_length + 2.0, adc_width + 2.0);
    // get vertices from ADC box
    std::vector<common::math::Vec2d> adc_corners;
    adc_box.GetAllCorners(&adc_corners);
    for (size_t i = 0; i < adc_corners.size(); ++i) {
        AINFO << "ADC [" << i << "]x: " << std::setprecision(9) << adc_corners[i].x();
        AINFO << "ADC [" << i << "]y: " << std::setprecision(9) << adc_corners[i].y();
    }
    auto left_top = adc_corners[1];

    ADEBUG << "left_top x: " << std::setprecision(9) << left_top.x();
    ADEBUG << "left_top y: " << std::setprecision(9) << left_top.y();

    // rotate the points to have the lane to be horizontal to x axis positive
    // direction and scale them base on the origin point
    // heading angle
    double heading;
    if (!nearby_path.GetHeadingAlongPath(left_top, &heading)) {
        AERROR << "fail to get heading on reference line";
        return;
    }

    frame->mutable_open_space_info()->set_origin_heading(common::math::NormalizeAngle(heading));
    ADEBUG << "heading: " << heading;
    frame->mutable_open_space_info()->mutable_origin_point()->set_x(left_top.x());
    frame->mutable_open_space_info()->mutable_origin_point()->set_y(left_top.y());
}

void OpenSpaceRoiDeciderPark::SetOrigin(Frame *const frame, const std::array<common::math::Vec2d, 4> &vertices) {
    auto left_top = vertices[0];
    auto right_top = vertices[3];
    // rotate the points to have the lane to be horizontal to x axis positive
    // direction and scale them base on the origin point
    Vec2d heading_vec = right_top - left_top;
    frame->mutable_open_space_info()->set_origin_heading(heading_vec.Angle());
    frame->mutable_open_space_info()->mutable_origin_point()->set_x(left_top.x());
    frame->mutable_open_space_info()->mutable_origin_point()->set_y(left_top.y());
}

void OpenSpaceRoiDeciderPark::SetOrigin(const ParkingInfo &parking_info, Frame *const frame) {
    auto left_top = parking_info.corner_points[0];
    auto right_top = parking_info.corner_points[1];
    // rotate the points to have the lane to be horizontal to x axis positive
    // direction and scale them base on the origin point
    Vec2d heading_vec = right_top - left_top;
    frame->mutable_open_space_info()->set_origin_heading(heading_vec.Angle());
    frame->mutable_open_space_info()->mutable_origin_point()->set_x(left_top.x());
    frame->mutable_open_space_info()->mutable_origin_point()->set_y(left_top.y());
}

void OpenSpaceRoiDeciderPark::SetParkingSpotEndPose(const ParkingInfo &parking_info, Frame *const frame) {
    auto left_top = parking_info.corner_points[0];
    auto left_down = parking_info.corner_points[3];
    auto right_down = parking_info.corner_points[2];
    auto right_top = parking_info.corner_points[1];

    const auto &origin_point = frame->open_space_info().origin_point();
    const auto &origin_heading = frame->open_space_info().origin_heading();

    // End pose is set in normalized boundary
    left_top -= origin_point;
    left_top.SelfRotate(-origin_heading);
    left_down -= origin_point;
    left_down.SelfRotate(-origin_heading);
    right_top -= origin_point;
    right_top.SelfRotate(-origin_heading);
    right_down -= origin_point;
    right_down.SelfRotate(-origin_heading);
    Vec2d end_pt;
    double parking_heading = 0;
    // now only support parking space at right road side
    const double parking_depth_buffer = config_.parking_depth_buffer();
    if (parking_info.parking_type == ParkingType::VERTICAL_PARKING) {
        const bool parking_inwards = config_.parking_inwards();
        if (parking_inwards) {
            parking_heading = (left_down - left_top).Angle();
            Vec2d middle_top = (left_top + right_top) / 2.0;
            end_pt = middle_top
                    + Vec2d::CreateUnitVec2d(parking_heading)
                            * (vehicle_params_.front_edge_to_center() + parking_depth_buffer);
        } else {
            parking_heading = (left_top - left_down).Angle();
            Vec2d middle_down = (left_down + right_down) / 2.0;
            end_pt = middle_down
                    + Vec2d::CreateUnitVec2d(parking_heading)
                            * (vehicle_params_.back_edge_to_center() + parking_depth_buffer);
        }
    } else {
        parking_heading = (right_top - left_top).Angle();
        Vec2d middle_left = (left_top + left_down) / 2.0;
        end_pt = middle_left
                + Vec2d::CreateUnitVec2d(parking_heading)
                        * (vehicle_params_.back_edge_to_center() + parking_depth_buffer);
    }
    auto *end_pose = frame->mutable_open_space_info()->mutable_open_space_end_pose();
    end_pose->push_back(end_pt.x());
    end_pose->push_back(end_pt.y());
    end_pose->push_back(parking_heading);
    end_pose->push_back(0.0);
}

void OpenSpaceRoiDeciderPark::SetPullOverSpotEndPose(Frame *const frame) {
    const auto &pull_over_status = injector_->planning_context()->planning_status().pull_over();
    const double pull_over_x = pull_over_status.position().x();
    const double pull_over_y = pull_over_status.position().y();
    double pull_over_theta = pull_over_status.theta();

    // Normalize according to origin_point and origin_heading
    const auto &origin_point = frame->open_space_info().origin_point();
    const auto &origin_heading = frame->open_space_info().origin_heading();
    Vec2d center(pull_over_x, pull_over_y);
    center -= origin_point;
    center.SelfRotate(-origin_heading);
    pull_over_theta = common::math::NormalizeAngle(pull_over_theta - origin_heading);

    auto *end_pose = frame->mutable_open_space_info()->mutable_open_space_end_pose();
    end_pose->push_back(center.x());
    end_pose->push_back(center.y());
    end_pose->push_back(pull_over_theta);
    // end pose velocity set to be zero
    end_pose->push_back(0.0);
}

void OpenSpaceRoiDeciderPark::SetParkAndGoEndPose(Frame *const frame) {
    const double kSTargetBuffer = config_.end_pose_s_distance();
    const double kSpeedRatio = 0.1;  // after adjust speed is 10% of speed limit
    // get vehicle current location
    // get vehicle s,l info
    auto park_and_go_status = injector_->planning_context()->mutable_planning_status()->mutable_park_and_go();

    const double adc_init_x = park_and_go_status->adc_init_position().x();
    const double adc_init_y = park_and_go_status->adc_init_position().y();

    ADEBUG << "ADC position (x): " << std::setprecision(9) << adc_init_x;
    ADEBUG << "ADC position (y): " << std::setprecision(9) << adc_init_y;

    const common::math::Vec2d adc_position = {adc_init_x, adc_init_y};
    common::SLPoint adc_position_sl;

    // get nearest reference line
    const auto &reference_line_list = frame->reference_line_info();
    ADEBUG << reference_line_list.size();
    const auto reference_line_info = std::min_element(
            reference_line_list.begin(),
            reference_line_list.end(),
            [&](const ReferenceLineInfo &ref_a, const ReferenceLineInfo &ref_b) {
                common::SLPoint adc_position_sl_a;
                common::SLPoint adc_position_sl_b;
                ref_a.reference_line().XYToSL(adc_position, &adc_position_sl_a);
                ref_b.reference_line().XYToSL(adc_position, &adc_position_sl_b);
                return std::fabs(adc_position_sl_a.l()) < std::fabs(adc_position_sl_b.l());
            });

    const auto &reference_line = reference_line_info->reference_line();
    reference_line.XYToSL(adc_position, &adc_position_sl);

    // target is at reference line
    const double target_s = adc_position_sl.s() + kSTargetBuffer;
    const auto reference_point = reference_line.GetReferencePoint(target_s);
    const double target_x = reference_point.x();
    const double target_y = reference_point.y();
    double target_theta = reference_point.heading();

    park_and_go_status->mutable_adc_adjust_end_pose()->set_x(target_x);
    park_and_go_status->mutable_adc_adjust_end_pose()->set_y(target_y);

    ADEBUG << "center.x(): " << std::setprecision(9) << target_x;
    ADEBUG << "center.y(): " << std::setprecision(9) << target_y;
    ADEBUG << "target_theta: " << std::setprecision(9) << target_theta;

    // Normalize according to origin_point and origin_heading
    const auto &origin_point = frame->open_space_info().origin_point();
    const auto &origin_heading = frame->open_space_info().origin_heading();
    Vec2d center(target_x, target_y);
    center -= origin_point;
    center.SelfRotate(-origin_heading);
    target_theta = common::math::NormalizeAngle(target_theta - origin_heading);

    auto *end_pose = frame->mutable_open_space_info()->mutable_open_space_end_pose();

    end_pose->push_back(center.x());
    end_pose->push_back(center.y());
    end_pose->push_back(target_theta);

    ADEBUG << "ADC position (x): " << std::setprecision(9) << (*end_pose)[0];
    ADEBUG << "ADC position (y): " << std::setprecision(9) << (*end_pose)[1];
    ADEBUG << "reference_line ID: " << reference_line_info->Lanes().Id();

    // end pose velocity set to be speed limit
    double target_speed = reference_line.GetSpeedLimitFromS(target_s);
    end_pose->push_back(kSpeedRatio * target_speed);
}

void OpenSpaceRoiDeciderPark::GetLargeCurvatureEndPose(Frame *const frame, Vec2d uturn_end_pos, double uturn_heading) {
    const auto &origin_point = frame->open_space_info().origin_point();
    double ego_s = 0.0;
    double ego_l = 0.0;
    nearby_path_->GetProjection(origin_point, &ego_s, &ego_l);

    double check_s = ego_s;
    hdmap::MapPathPoint cur_point;
    hdmap::MapPathPoint pre_point;
    hdmap::MapPathPoint pre_pre_point;
    double xds = 0.0;
    double yds = 0.0;
    double pre_xds = 0.0;
    double pre_yds = 0.0;
    double pre_pre_xds = 0.0;
    double pre_pre_yds = 0.0;
    double xdds = 0.0;
    double ydds = 0.0;
    double delta_s = 0.1;
    double delta_s_s = delta_s * 2;
    double kappa = 0.0;
    int low_curvature_count = 0;
    while (check_s < nearby_path_->accumulated_s().back()) {
        pre_pre_point = pre_point;
        pre_point = cur_point;
        cur_point = nearby_path_->GetSmoothPoint(check_s);
        pre_pre_xds = pre_xds;
        pre_pre_yds = pre_yds;
        pre_xds = xds;
        pre_yds = yds;
        xds = (cur_point.x() - pre_pre_point.x()) / delta_s_s;
        yds = (cur_point.y() - pre_pre_point.y()) / delta_s_s;
        xdds = (xds - pre_pre_xds) / delta_s_s;
        ydds = (yds - pre_pre_yds) / delta_s_s;
        kappa = (xds * ydds - yds * xdds) / (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
        AINFO << "check_s: " << check_s << ", kappa: " << kappa << " ,pre_pre_xds: " << pre_pre_xds;
        if (pre_pre_xds != 0 && std::abs(kappa) < 0.05) {
            AINFO << "check_s: " << check_s;
            low_curvature_count++;
        } else {
            low_curvature_count = 0;
        }
        if (low_curvature_count > 10) {
            AINFO << "check_s: " << check_s;
            break;
        }
        check_s += delta_s;
    }
    uturn_end_pos = pre_pre_point;
    uturn_heading = pre_pre_point.heading();

    // Normalize according to origin_point and origin_heading
    const auto &origin_heading = frame->open_space_info().origin_heading();
    uturn_end_pos -= origin_point;
    uturn_end_pos.SelfRotate(-origin_heading);
    uturn_heading = common::math::NormalizeAngle(uturn_heading - origin_heading);

    auto *end_pose = frame->mutable_open_space_info()->mutable_open_space_end_pose();

    end_pose->push_back(uturn_end_pos.x());
    end_pose->push_back(uturn_end_pos.y());
    end_pose->push_back(uturn_heading);

    ADEBUG << "ADC position (x): " << std::setprecision(9) << (*end_pose)[0];
    ADEBUG << "ADC position (y): " << std::setprecision(9) << (*end_pose)[1];
    end_pose->push_back(0.0);
}

void OpenSpaceRoiDeciderPark::GetRoadBoundary(
        const hdmap::Path &nearby_path,
        const double center_line_s,
        const common::math::Vec2d &origin_point,
        const double origin_heading,
        std::vector<Vec2d> *left_lane_boundary,
        std::vector<Vec2d> *right_lane_boundary,
        std::vector<Vec2d> *center_lane_boundary_left,
        std::vector<Vec2d> *center_lane_boundary_right,
        std::vector<double> *center_lane_s_left,
        std::vector<double> *center_lane_s_right,
        std::vector<double> *left_lane_road_width,
        std::vector<double> *right_lane_road_width) {
    double start_s = center_line_s - config_.roi_longitudinal_range_start();
    double end_s = center_line_s + config_.roi_longitudinal_range_end();

    hdmap::MapPathPoint start_point = nearby_path.GetSmoothPoint(start_s);
    double last_check_point_heading = start_point.heading();
    double index = 0.0;
    double check_point_s = start_s;

    // For the road boundary, add key points to left/right side boundary
    // separately. Iterate s_value to check key points at a step of
    // roi_line_segment_length. Key points include: start_point, end_point, points
    // where path curvature is large, points near left/right road-curb corners
    while (check_point_s <= end_s) {
        hdmap::MapPathPoint check_point = nearby_path.GetSmoothPoint(check_point_s);
        double check_point_heading = check_point.heading();
        bool is_center_lane_heading_change
                = std::abs(common::math::NormalizeAngle(check_point_heading - last_check_point_heading))
                > config_.roi_line_segment_min_angle();
        last_check_point_heading = check_point_heading;

        ADEBUG << "is is_center_lane_heading_change: " << is_center_lane_heading_change;
        ADEBUG << "check_point_s: " << check_point_s << " check_point_heading: " << check_point_heading;
        ADEBUG << "check_point: " << std::setprecision(9) << check_point.x() << " " << check_point.y();
        // Check if the current center-lane checking-point is start point || end
        // point || or point with larger curvature. If yes, mark it as an anchor
        // point.
        bool is_anchor_point = check_point_s == start_s || check_point_s == end_s || is_center_lane_heading_change;
        // Add key points to the left-half boundary
        AddBoundaryKeyPoint(
                nearby_path,
                check_point_s,
                start_s,
                end_s,
                is_anchor_point,
                true,
                center_lane_boundary_left,
                left_lane_boundary,
                center_lane_s_left,
                left_lane_road_width);
        // Add key points to the right-half boundary
        AddBoundaryKeyPoint(
                nearby_path,
                check_point_s,
                start_s,
                end_s,
                is_anchor_point,
                false,
                center_lane_boundary_right,
                right_lane_boundary,
                center_lane_s_right,
                right_lane_road_width);
        if (check_point_s == end_s) {
            break;
        }
        index += 1.0;
        check_point_s = start_s + index * config_.roi_line_segment_length();
        check_point_s = check_point_s >= end_s ? end_s : check_point_s;
    }

    size_t left_point_size = left_lane_boundary->size();
    size_t right_point_size = right_lane_boundary->size();
    for (size_t i = 0; i < left_point_size; i++) {
        AINFO << "left_lane_boundary: " << std::setprecision(9) << left_lane_boundary->at(i).x() << " "
              << left_lane_boundary->at(i).y();
        left_lane_boundary->at(i) -= origin_point;
        left_lane_boundary->at(i).SelfRotate(-origin_heading);
    }
    for (size_t i = 0; i < right_point_size; i++) {
        AINFO << "right_lane_boundary: " << std::setprecision(9) << right_lane_boundary->at(i).x() << " "
              << right_lane_boundary->at(i).y();
        right_lane_boundary->at(i) -= origin_point;
        right_lane_boundary->at(i).SelfRotate(-origin_heading);
    }
}

void OpenSpaceRoiDeciderPark::GetParkingSoftBoundary(
        common::math::Vec2d left_top,
        common::math::Vec2d left_down,
        common::math::Vec2d right_top,
        common::math::Vec2d right_down,
        std::vector<std::vector<common::math::Vec2d>> *const parking_soft_boundary) {
    std::vector<common::math::Vec2d> left_boundary{left_top, left_down};
    std::vector<common::math::Vec2d> right_boundary{right_down, right_top};
    parking_soft_boundary->emplace_back(left_boundary);
    parking_soft_boundary->emplace_back(right_boundary);
}

void OpenSpaceRoiDeciderPark::GetRoadBoundaryFromMap(
        const hdmap::Path &nearby_path,
        const double center_line_s,
        const Vec2d &origin_point,
        const double origin_heading,
        std::vector<Vec2d> *left_lane_boundary,
        std::vector<Vec2d> *right_lane_boundary,
        std::vector<Vec2d> *center_lane_boundary_left,
        std::vector<Vec2d> *center_lane_boundary_right,
        std::vector<double> *center_lane_s_left,
        std::vector<double> *center_lane_s_right,
        std::vector<double> *left_lane_road_width,
        std::vector<double> *right_lane_road_width) {
    // Longitudinal range can be asymmetric.
    double start_s = center_line_s - config_.roi_longitudinal_range_start();
    double end_s = center_line_s + config_.roi_longitudinal_range_end();
    hdmap::MapPathPoint start_point = nearby_path.GetSmoothPoint(start_s);

    double check_point_s = start_s;

    while (check_point_s <= end_s) {
        hdmap::MapPathPoint check_point = nearby_path.GetSmoothPoint(check_point_s);

        // get road boundaries
        double left_road_width = nearby_path.GetRoadLeftWidth(check_point_s);
        double right_road_width = nearby_path.GetRoadRightWidth(check_point_s);

        double current_road_width = std::max(left_road_width, right_road_width);

        // get road boundaries at current location
        common::PointENU check_point_xy;
        std::vector<hdmap::RoadRoiPtr> road_boundaries;
        std::vector<hdmap::JunctionInfoConstPtr> junctions;
        check_point_xy.set_x(check_point.x());
        check_point_xy.set_y(check_point.y());
        hdmap_->GetRoadBoundaries(check_point_xy, current_road_width, &road_boundaries, &junctions);

        if (check_point_s < center_line_s) {
            for (size_t i = 0; i < (*road_boundaries.at(0)).left_boundary.line_points.size(); i++) {
                right_lane_boundary->emplace_back(
                        Vec2d((*road_boundaries.at(0)).left_boundary.line_points[i].x(),
                              (*road_boundaries.at(0)).left_boundary.line_points[i].y()));
            }
            for (size_t i = 0; i < (*road_boundaries.at(0)).right_boundary.line_points.size(); i++) {
                left_lane_boundary->emplace_back(
                        Vec2d((*road_boundaries.at(0)).right_boundary.line_points[i].x(),
                              (*road_boundaries.at(0)).right_boundary.line_points[i].y()));
            }
        } else {
            for (size_t i = 0; i < (*road_boundaries.at(0)).left_boundary.line_points.size(); i++) {
                left_lane_boundary->emplace_back(
                        Vec2d((*road_boundaries.at(0)).left_boundary.line_points[i].x(),
                              (*road_boundaries.at(0)).left_boundary.line_points[i].y()));
            }
            for (size_t i = 0; i < (*road_boundaries.at(0)).right_boundary.line_points.size(); i++) {
                right_lane_boundary->emplace_back(
                        Vec2d((*road_boundaries.at(0)).right_boundary.line_points[i].x(),
                              (*road_boundaries.at(0)).right_boundary.line_points[i].y()));
            }
        }

        center_lane_boundary_right->emplace_back(check_point);
        center_lane_boundary_left->emplace_back(check_point);
        center_lane_s_left->emplace_back(check_point_s);
        center_lane_s_right->emplace_back(check_point_s);
        left_lane_road_width->emplace_back(left_road_width);
        right_lane_road_width->emplace_back(right_road_width);

        check_point_s = check_point_s + config_.roi_line_segment_length_from_map();
    }

    size_t left_point_size = left_lane_boundary->size();
    size_t right_point_size = right_lane_boundary->size();
    ADEBUG << "right_road_boundary size: " << right_lane_boundary->size();
    ADEBUG << "left_road_boundary size: " << left_lane_boundary->size();
    for (size_t i = 0; i < left_point_size; i++) {
        left_lane_boundary->at(i) -= origin_point;
        left_lane_boundary->at(i).SelfRotate(-origin_heading);
        ADEBUG << "left_road_boundary: [" << std::setprecision(9) << left_lane_boundary->at(i).x() << ", "
               << left_lane_boundary->at(i).y() << "]";
    }
    for (size_t i = 0; i < right_point_size; i++) {
        right_lane_boundary->at(i) -= origin_point;
        right_lane_boundary->at(i).SelfRotate(-origin_heading);
        ADEBUG << "right_road_boundary: [" << std::setprecision(9) << right_lane_boundary->at(i).x() << ", "
               << right_lane_boundary->at(i).y() << "]";
    }
    if (!left_lane_boundary->empty()) {
        sort(left_lane_boundary->begin(), left_lane_boundary->end(), [](const Vec2d &first_pt, const Vec2d &second_pt) {
            return first_pt.x() < second_pt.x() || (first_pt.x() == second_pt.x() && first_pt.y() < second_pt.y());
        });
        auto unique_end = std::unique(left_lane_boundary->begin(), left_lane_boundary->end());
        left_lane_boundary->erase(unique_end, left_lane_boundary->end());
    }
    if (!right_lane_boundary->empty()) {
        sort(right_lane_boundary->begin(),
             right_lane_boundary->end(),
             [](const Vec2d &first_pt, const Vec2d &second_pt) {
                 return first_pt.x() < second_pt.x() || (first_pt.x() == second_pt.x() && first_pt.y() < second_pt.y());
             });
        auto unique_end = std::unique(right_lane_boundary->begin(), right_lane_boundary->end());
        right_lane_boundary->erase(unique_end, right_lane_boundary->end());
    }
}

void OpenSpaceRoiDeciderPark::AddBoundaryKeyPoint(
        const hdmap::Path &nearby_path,
        const double check_point_s,
        const double start_s,
        const double end_s,
        const bool is_anchor_point,
        const bool is_left_curb,
        std::vector<Vec2d> *center_lane_boundary,
        std::vector<Vec2d> *curb_lane_boundary,
        std::vector<double> *center_lane_s,
        std::vector<double> *road_width) {
    // Check if current central-lane checking point's mapping on the left/right
    // road boundary is a key point. The road boundary point is a key point if
    // one of the following two confitions is satisfied:
    // 1. the current central-lane point is an anchor point: (a start/end point
    // or the point on path with large curvatures)
    // 2. the point on the left/right lane boundary is close to a curb corner
    // As indicated below:
    // (#) Key Point Type 1: Lane anchor points
    // (*) Key Point Type 2: Curb-corner points
    //                                                         #
    // Path Direction -->                                     /    /   #
    // Left Lane Boundary   #--------------------------------#    /   /
    //                                                           /   /
    // Center Lane          - - - - - - - - - - - - - - - - - - /   /
    //                                                             /
    // Right Lane Boundary  #--------*                 *----------#
    //                                \               /
    //                                 *-------------*

    // road width changes slightly at the turning point of a path
    // TODO(SHU): 1. consider distortion introduced by curvy road; 2. use both
    // round boundaries for single-track road; 3. longitudinal range may not be
    // symmetric
    const double previous_distance_s = std::min(config_.roi_line_segment_length(), check_point_s - start_s);
    const double next_distance_s = std::min(config_.roi_line_segment_length(), end_s - check_point_s);

    hdmap::MapPathPoint current_check_point = nearby_path.GetSmoothPoint(check_point_s);

    double current_check_point_heading = current_check_point.heading();
    double current_road_width
            = is_left_curb ? nearby_path.GetLaneLeftWidth(check_point_s) : nearby_path.GetLaneLeftWidth(check_point_s);
    // If the current center-lane checking point is an anchor point, then add
    // current left/right curb boundary point as a key point
    if (is_anchor_point) {
        double point_vec_cos = is_left_curb ? std::cos(current_check_point_heading + M_PI / 2.0)
                                            : std::cos(current_check_point_heading - M_PI / 2.0);
        double point_vec_sin = is_left_curb ? std::sin(current_check_point_heading + M_PI / 2.0)
                                            : std::sin(current_check_point_heading - M_PI / 2.0);
        Vec2d curb_lane_point = Vec2d(current_road_width * point_vec_cos, current_road_width * point_vec_sin);
        curb_lane_point = curb_lane_point + current_check_point;
        center_lane_boundary->push_back(current_check_point);
        curb_lane_boundary->push_back(curb_lane_point);
        center_lane_s->push_back(check_point_s);
        road_width->push_back(current_road_width);
        return;
    }
    double previous_road_width = is_left_curb ? nearby_path.GetRoadLeftWidth(check_point_s - previous_distance_s)
                                              : nearby_path.GetRoadRightWidth(check_point_s - previous_distance_s);
    double next_road_width = is_left_curb ? nearby_path.GetRoadLeftWidth(check_point_s + next_distance_s)
                                          : nearby_path.GetRoadRightWidth(check_point_s + next_distance_s);
    double previous_curb_segment_angle = (current_road_width - previous_road_width) / previous_distance_s;
    double next_segment_angle = (next_road_width - current_road_width) / next_distance_s;
    double current_curb_point_delta_theta = next_segment_angle - previous_curb_segment_angle;
    // If the delta angle between the previous curb segment and the next curb
    // segment is large (near a curb corner), then add current curb_lane_point
    // as a key point.
    if (std::abs(current_curb_point_delta_theta) > config_.curb_heading_tangent_change_upper_limit()) {
        double point_vec_cos = is_left_curb ? std::cos(current_check_point_heading + M_PI / 2.0)
                                            : std::cos(current_check_point_heading - M_PI / 2.0);
        double point_vec_sin = is_left_curb ? std::sin(current_check_point_heading + M_PI / 2.0)
                                            : std::sin(current_check_point_heading - M_PI / 2.0);
        Vec2d curb_lane_point = Vec2d(current_road_width * point_vec_cos, current_road_width * point_vec_sin);
        curb_lane_point = curb_lane_point + current_check_point;
        center_lane_boundary->push_back(current_check_point);
        curb_lane_boundary->push_back(curb_lane_point);
        center_lane_s->push_back(check_point_s);
        road_width->push_back(current_road_width);
    }
}

bool OpenSpaceRoiDeciderPark::GetParkingBoundary(
        const ParkingInfo &parking_info,
        const hdmap::Path &nearby_path,
        Frame *const frame,
        std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary,
        std::vector<std::vector<common::math::Vec2d>> *const parking_soft_boundary) {
    auto left_top = parking_info.corner_points[0];
    ADEBUG << std::fixed << "left_top: " << left_top.x() << ", " << left_top.y();
    auto left_down = parking_info.corner_points[3];
    ADEBUG << std::fixed << "left_down: " << left_down.x() << ", " << left_down.y();
    auto right_down = parking_info.corner_points[2];
    ADEBUG << std::fixed << "right_down: " << right_down.x() << ", " << right_down.y();
    auto right_top = parking_info.corner_points[1];
    ADEBUG << std::fixed << "right_top: " << right_top.x() << ", " << right_top.y();

    const auto &origin_point = frame->open_space_info().origin_point();
    ADEBUG << std::fixed << "origin_point: " << origin_point.x() << ", " << origin_point.y();
    const auto &origin_heading = frame->open_space_info().origin_heading();

    double left_top_s = 0.0;
    double left_top_l = 0.0;
    double right_top_s = 0.0;
    double right_top_l = 0.0;
    if (!(nearby_path.GetProjection(left_top, &left_top_s, &left_top_l)
          && nearby_path.GetProjection(right_top, &right_top_s, &right_top_l))) {
        AERROR << "fail to get parking spot points' projections on reference line";
        return false;
    }

    left_top -= origin_point;
    left_top.SelfRotate(-origin_heading);
    left_down -= origin_point;
    left_down.SelfRotate(-origin_heading);
    right_top -= origin_point;
    right_top.SelfRotate(-origin_heading);
    right_down -= origin_point;
    right_down.SelfRotate(-origin_heading);

    const double center_line_s = (left_top_s + right_top_s) / 2.0;
    std::vector<Vec2d> left_lane_boundary;
    std::vector<Vec2d> right_lane_boundary;
    // The pivot points on the central lane, mapping with the key points on
    // the left lane boundary.
    std::vector<Vec2d> center_lane_boundary_left;
    // The pivot points on the central lane, mapping with the key points on
    // the right lane boundary.
    std::vector<Vec2d> center_lane_boundary_right;
    // The s-value for the anchor points on the center_lane_boundary_left.
    std::vector<double> center_lane_s_left;
    // The s-value for the anchor points on the center_lane_boundary_right.
    std::vector<double> center_lane_s_right;
    // The left-half road width between the pivot points on the
    // center_lane_boundary_left and key points on the
    // left_lane_boundary.
    std::vector<double> left_lane_road_width;
    // The right-half road width between the pivot points on the
    // center_lane_boundary_right and key points on the
    // right_lane_boundary.
    std::vector<double> right_lane_road_width;

    GetRoadBoundary(
            nearby_path,
            center_line_s,
            origin_point,
            origin_heading,
            &left_lane_boundary,
            &right_lane_boundary,
            &center_lane_boundary_left,
            &center_lane_boundary_right,
            &center_lane_s_left,
            &center_lane_s_right,
            &left_lane_road_width,
            &right_lane_road_width);

    // If smaller than zero, the parking spot is on the right of the lane
    // Left, right, down or opposite of the boundary is decided when viewing the
    // parking spot upward
    const double average_l = (left_top_l + right_top_l) / 2.0;
    std::vector<Vec2d> boundary_points;

    // TODO(jiaxuan): Write a half-boundary formation function and call it twice
    // to avoid duplicated manipulations on the left and right sides
    if (average_l < 0) {
        // if average_l is lower than zero, the parking spot is on the right
        // lane boundary and assume that the lane half width is average_l
        ADEBUG << "average_l is less than 0 in OpenSpaceROI";
        size_t point_size = right_lane_boundary.size();
        for (size_t i = 0; i < point_size; i++) {
            right_lane_boundary[i].SelfRotate(origin_heading);
            right_lane_boundary[i] += origin_point;
            right_lane_boundary[i] -= center_lane_boundary_right[i];
            right_lane_boundary[i] /= right_lane_road_width[i];
            right_lane_boundary[i] *= (-average_l);
            right_lane_boundary[i] += center_lane_boundary_right[i];
            right_lane_boundary[i] -= origin_point;
            right_lane_boundary[i].SelfRotate(-origin_heading);
        }

        auto point_left_to_left_top_connor_s
                = std::lower_bound(center_lane_s_right.begin(), center_lane_s_right.end(), left_top_s);
        size_t point_left_to_left_top_connor_index
                = std::distance(center_lane_s_right.begin(), point_left_to_left_top_connor_s);
        point_left_to_left_top_connor_index = point_left_to_left_top_connor_index == 0
                ? point_left_to_left_top_connor_index
                : point_left_to_left_top_connor_index - 1;
        auto point_left_to_left_top_connor_itr = right_lane_boundary.begin() + point_left_to_left_top_connor_index;
        auto point_right_to_right_top_connor_s
                = std::upper_bound(center_lane_s_right.begin(), center_lane_s_right.end(), right_top_s);
        size_t point_right_to_right_top_connor_index
                = std::distance(center_lane_s_right.begin(), point_right_to_right_top_connor_s);
        auto point_right_to_right_top_connor_itr = right_lane_boundary.begin() + point_right_to_right_top_connor_index;

        std::copy(right_lane_boundary.begin(), point_left_to_left_top_connor_itr, std::back_inserter(boundary_points));

        std::vector<Vec2d> parking_spot_boundary{left_top, left_down, right_down, right_top};

        std::copy(parking_spot_boundary.begin(), parking_spot_boundary.end(), std::back_inserter(boundary_points));

        std::copy(point_right_to_right_top_connor_itr, right_lane_boundary.end(), std::back_inserter(boundary_points));

        std::reverse_copy(left_lane_boundary.begin(), left_lane_boundary.end(), std::back_inserter(boundary_points));

        // reinsert the initial point to the back to from closed loop
        boundary_points.push_back(right_lane_boundary.front());

        // disassemble line into line2d segments
        for (size_t i = 0; i < point_left_to_left_top_connor_index; i++) {
            std::vector<Vec2d> segment{right_lane_boundary[i], right_lane_boundary[i + 1]};
            roi_parking_boundary->push_back(segment);
        }

        std::vector<Vec2d> left_stitching_segment{right_lane_boundary[point_left_to_left_top_connor_index], left_top};
        roi_parking_boundary->push_back(left_stitching_segment);

        std::vector<Vec2d> left_parking_spot_segment{left_top, left_down};
        std::vector<Vec2d> down_parking_spot_segment{left_down, right_down};
        std::vector<Vec2d> right_parking_spot_segment{right_down, right_top};
        roi_parking_boundary->push_back(left_parking_spot_segment);
        roi_parking_boundary->push_back(down_parking_spot_segment);
        roi_parking_boundary->push_back(right_parking_spot_segment);

        std::vector<Vec2d> right_stitching_segment{
                right_top, right_lane_boundary[point_right_to_right_top_connor_index]};
        roi_parking_boundary->push_back(right_stitching_segment);

        size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
        for (size_t i = point_right_to_right_top_connor_index; i < right_lane_boundary_last_index; i++) {
            std::vector<Vec2d> segment{right_lane_boundary[i], right_lane_boundary[i + 1]};
            roi_parking_boundary->push_back(segment);
        }

        size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
        for (size_t i = left_lane_boundary_last_index; i > 0; i--) {
            std::vector<Vec2d> segment{left_lane_boundary[i], left_lane_boundary[i - 1]};
            roi_parking_boundary->push_back(segment);
        }

    } else {
        // if average_l is higher than zero, the parking spot is on the left
        // lane boundary and assume that the lane half width is average_l
        ADEBUG << "average_l is greater than 0 in OpenSpaceROI";
        size_t point_size = left_lane_boundary.size();
        for (size_t i = 0; i < point_size; i++) {
            left_lane_boundary[i].SelfRotate(origin_heading);
            left_lane_boundary[i] += origin_point;
            left_lane_boundary[i] -= center_lane_boundary_left[i];
            left_lane_boundary[i] /= left_lane_road_width[i];
            left_lane_boundary[i] *= average_l;
            left_lane_boundary[i] += center_lane_boundary_left[i];
            left_lane_boundary[i] -= origin_point;
            left_lane_boundary[i].SelfRotate(-origin_heading);
            ADEBUG << "left_lane_boundary[" << i << "]: " << left_lane_boundary[i].x() << ", "
                   << left_lane_boundary[i].y();
        }

        auto point_right_to_right_top_connor_s
                = std::lower_bound(center_lane_s_left.begin(), center_lane_s_left.end(), right_top_s);
        size_t point_right_to_right_top_connor_index
                = std::distance(center_lane_s_left.begin(), point_right_to_right_top_connor_s);
        auto point_right_to_right_top_connor_itr = left_lane_boundary.begin() + point_right_to_right_top_connor_index;

        auto point_left_to_left_top_connor_s
                = std::upper_bound(center_lane_s_left.begin(), center_lane_s_left.end(), left_top_s);
        size_t point_left_to_left_top_connor_index
                = std::distance(center_lane_s_left.begin(), point_left_to_left_top_connor_s);
        if (point_left_to_left_top_connor_index > 0) {
            --point_left_to_left_top_connor_index;
        }
        auto point_left_to_left_top_connor_itr = left_lane_boundary.begin() + point_left_to_left_top_connor_index;

        std::copy(right_lane_boundary.begin(), right_lane_boundary.end(), std::back_inserter(boundary_points));

        std::reverse_copy(
                point_left_to_left_top_connor_itr, left_lane_boundary.end(), std::back_inserter(boundary_points));

        std::vector<Vec2d> parking_spot_boundary{left_top, left_down, right_down, right_top};
        std::copy(parking_spot_boundary.begin(), parking_spot_boundary.end(), std::back_inserter(boundary_points));

        std::reverse_copy(
                left_lane_boundary.begin(), point_right_to_right_top_connor_itr, std::back_inserter(boundary_points));

        // reinsert the initial point to the back to from closed loop
        boundary_points.push_back(right_lane_boundary.front());

        // disassemble line into line2d segments
        size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
        for (size_t i = 0; i < right_lane_boundary_last_index; i++) {
            std::vector<Vec2d> segment{right_lane_boundary[i], right_lane_boundary[i + 1]};
            roi_parking_boundary->push_back(segment);
        }

        size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
        for (size_t i = left_lane_boundary_last_index; i > point_right_to_right_top_connor_index; i--) {
            std::vector<Vec2d> segment{left_lane_boundary[i], left_lane_boundary[i - 1]};
            roi_parking_boundary->push_back(segment);
        }

        std::vector<Vec2d> left_stitching_segment{left_lane_boundary[point_right_to_right_top_connor_index], right_top};
        roi_parking_boundary->push_back(left_stitching_segment);

        std::vector<Vec2d> left_parking_spot_segment{left_down, left_top};
        std::vector<Vec2d> down_parking_spot_segment{right_down, left_down};
        std::vector<Vec2d> right_parking_spot_segment{right_top, right_down};
        roi_parking_boundary->push_back(right_parking_spot_segment);
        roi_parking_boundary->push_back(down_parking_spot_segment);
        roi_parking_boundary->push_back(left_parking_spot_segment);

        std::vector<Vec2d> right_stitching_segment{
                left_top, left_lane_boundary[point_left_to_left_top_connor_index]};
        roi_parking_boundary->push_back(right_stitching_segment);

        for (size_t i = point_left_to_left_top_connor_index; i > 0; --i) {
            std::vector<Vec2d> segment{left_lane_boundary[i], left_lane_boundary[i - 1]};
            roi_parking_boundary->push_back(segment);
        }
    }

    // Fuse line segments into convex contraints
    if (!FuseLineSegments(roi_parking_boundary)) {
        AERROR << "FuseLineSegments failed in parking ROI";
        return false;
    }

    // Get soft boundary
    GetParkingSoftBoundary(left_top, left_down, right_down, right_top, parking_soft_boundary);

    // Get xy boundary
    auto xminmax
            = std::minmax_element(boundary_points.begin(), boundary_points.end(), [](const Vec2d &a, const Vec2d &b) {
                  return a.x() < b.x();
              });
    auto yminmax
            = std::minmax_element(boundary_points.begin(), boundary_points.end(), [](const Vec2d &a, const Vec2d &b) {
                  return a.y() < b.y();
              });
    std::vector<double> ROI_xy_boundary{
            xminmax.first->x(), xminmax.second->x(), yminmax.first->y(), yminmax.second->y()};
    auto *xy_boundary = frame->mutable_open_space_info()->mutable_ROI_xy_boundary();
    xy_boundary->assign(ROI_xy_boundary.begin(), ROI_xy_boundary.end());

    Vec2d vehicle_xy = Vec2d(vehicle_state_.x(), vehicle_state_.y());
    vehicle_xy -= origin_point;
    vehicle_xy.SelfRotate(-origin_heading);
    if (vehicle_xy.x() < ROI_xy_boundary[0] || vehicle_xy.x() > ROI_xy_boundary[1]
        || vehicle_xy.y() < ROI_xy_boundary[2] || vehicle_xy.y() > ROI_xy_boundary[3]) {
        AERROR << "vehicle outside of xy boundary of parking ROI";
        return false;
    }
    return true;
}

bool OpenSpaceRoiDeciderPark::GetPullOverBoundary(
        Frame *const frame,
        const std::array<common::math::Vec2d, 4> &vertices,
        const hdmap::Path &nearby_path,
        std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
    auto left_top = vertices[0];
    auto left_down = vertices[1];
    auto right_down = vertices[2];
    auto right_top = vertices[3];

    const auto &origin_point = frame->open_space_info().origin_point();
    const auto &origin_heading = frame->open_space_info().origin_heading();

    double left_top_s = 0.0;
    double left_top_l = 0.0;
    double right_top_s = 0.0;
    double right_top_l = 0.0;
    if (!(nearby_path.GetProjection(left_top, &left_top_s, &left_top_l)
          && nearby_path.GetProjection(right_top, &right_top_s, &right_top_l))) {
        AERROR << "fail to get parking spot points' projections on reference line";
        return false;
    }

    left_top -= origin_point;
    left_top.SelfRotate(-origin_heading);
    left_down -= origin_point;
    left_down.SelfRotate(-origin_heading);
    right_top -= origin_point;
    right_top.SelfRotate(-origin_heading);
    right_down -= origin_point;
    right_down.SelfRotate(-origin_heading);

    const double center_line_s = (left_top_s + right_top_s) / 2.0;
    std::vector<Vec2d> left_lane_boundary;
    std::vector<Vec2d> right_lane_boundary;
    std::vector<Vec2d> center_lane_boundary_left;
    std::vector<Vec2d> center_lane_boundary_right;
    std::vector<double> center_lane_s_left;
    std::vector<double> center_lane_s_right;
    std::vector<double> left_lane_road_width;
    std::vector<double> right_lane_road_width;

    GetRoadBoundary(
            nearby_path,
            center_line_s,
            origin_point,
            origin_heading,
            &left_lane_boundary,
            &right_lane_boundary,
            &center_lane_boundary_left,
            &center_lane_boundary_right,
            &center_lane_s_left,
            &center_lane_s_right,
            &left_lane_road_width,
            &right_lane_road_width);

    // Load boundary as line segments in counter-clockwise order
    std::reverse(left_lane_boundary.begin(), left_lane_boundary.end());

    std::vector<Vec2d> boundary_points;
    std::copy(right_lane_boundary.begin(), right_lane_boundary.end(), std::back_inserter(boundary_points));
    std::copy(left_lane_boundary.begin(), left_lane_boundary.end(), std::back_inserter(boundary_points));

    size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
    for (size_t i = 0; i < right_lane_boundary_last_index; i++) {
        std::vector<Vec2d> segment{right_lane_boundary[i], right_lane_boundary[i + 1]};
        roi_parking_boundary->push_back(segment);
    }

    size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
    for (size_t i = left_lane_boundary_last_index; i > 0; i--) {
        std::vector<Vec2d> segment{left_lane_boundary[i], left_lane_boundary[i - 1]};
        roi_parking_boundary->push_back(segment);
    }

    // Fuse line segments into convex contraints
    if (!FuseLineSegments(roi_parking_boundary)) {
        return false;
    }
    // Get xy boundary
    auto xminmax
            = std::minmax_element(boundary_points.begin(), boundary_points.end(), [](const Vec2d &a, const Vec2d &b) {
                  return a.x() < b.x();
              });
    auto yminmax
            = std::minmax_element(boundary_points.begin(), boundary_points.end(), [](const Vec2d &a, const Vec2d &b) {
                  return a.y() < b.y();
              });
    std::vector<double> ROI_xy_boundary{
            xminmax.first->x(), xminmax.second->x(), yminmax.first->y(), yminmax.second->y()};
    auto *xy_boundary = frame->mutable_open_space_info()->mutable_ROI_xy_boundary();
    xy_boundary->assign(ROI_xy_boundary.begin(), ROI_xy_boundary.end());

    Vec2d vehicle_xy = Vec2d(vehicle_state_.x(), vehicle_state_.y());
    vehicle_xy -= origin_point;
    vehicle_xy.SelfRotate(-origin_heading);
    if (vehicle_xy.x() < ROI_xy_boundary[0] || vehicle_xy.x() > ROI_xy_boundary[1]
        || vehicle_xy.y() < ROI_xy_boundary[2] || vehicle_xy.y() > ROI_xy_boundary[3]) {
        AERROR << "vehicle outside of xy boundary of parking ROI";
        return false;
    }
    return true;
}

bool OpenSpaceRoiDeciderPark::GetParkAndGoBoundary(
        Frame *const frame,
        const hdmap::Path &nearby_path,
        std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
    const auto &park_and_go_status = injector_->planning_context()->planning_status().park_and_go();
    const double adc_init_x = park_and_go_status.adc_init_position().x();
    const double adc_init_y = park_and_go_status.adc_init_position().y();
    const double adc_init_heading = park_and_go_status.adc_init_heading();
    common::math::Vec2d adc_init_position = {adc_init_x, adc_init_y};
    const double adc_length = vehicle_params_.length();
    const double adc_width = vehicle_params_.width();
    // ADC box
    Box2d adc_box(adc_init_position, adc_init_heading, adc_length, adc_width);
    // get vertices from ADC box
    std::vector<common::math::Vec2d> adc_corners;
    adc_box.GetAllCorners(&adc_corners);
    auto left_top = adc_corners[1];
    auto right_top = adc_corners[0];

    const auto &origin_point = frame->open_space_info().origin_point();
    const auto &origin_heading = frame->open_space_info().origin_heading();

    double left_top_s = 0.0;
    double left_top_l = 0.0;
    double right_top_s = 0.0;
    double right_top_l = 0.0;
    if (!(nearby_path.GetProjection(left_top, &left_top_s, &left_top_l)
          && nearby_path.GetProjection(right_top, &right_top_s, &right_top_l))) {
        AERROR << "fail to get parking spot points' projections on reference line";
        return false;
    }
    left_top -= origin_point;
    left_top.SelfRotate(-origin_heading);
    right_top -= origin_point;
    right_top.SelfRotate(-origin_heading);

    const double center_line_s = (left_top_s + right_top_s) / 2.0;
    std::vector<Vec2d> left_lane_boundary;
    std::vector<Vec2d> right_lane_boundary;
    std::vector<Vec2d> center_lane_boundary_left;
    std::vector<Vec2d> center_lane_boundary_right;
    std::vector<double> center_lane_s_left;
    std::vector<double> center_lane_s_right;
    std::vector<double> left_lane_road_width;
    std::vector<double> right_lane_road_width;

    if (config_.use_road_boundary_from_map()) {
        GetRoadBoundaryFromMap(
                nearby_path,
                center_line_s,
                origin_point,
                origin_heading,
                &left_lane_boundary,
                &right_lane_boundary,
                &center_lane_boundary_left,
                &center_lane_boundary_right,
                &center_lane_s_left,
                &center_lane_s_right,
                &left_lane_road_width,
                &right_lane_road_width);
    } else {
        GetRoadBoundary(
                nearby_path,
                center_line_s,
                origin_point,
                origin_heading,
                &left_lane_boundary,
                &right_lane_boundary,
                &center_lane_boundary_left,
                &center_lane_boundary_right,
                &center_lane_s_left,
                &center_lane_s_right,
                &left_lane_road_width,
                &right_lane_road_width);
    }

    // Load boundary as line segments in counter-clockwise order
    std::reverse(left_lane_boundary.begin(), left_lane_boundary.end());

    std::vector<Vec2d> boundary_points;
    std::copy(right_lane_boundary.begin(), right_lane_boundary.end(), std::back_inserter(boundary_points));
    std::copy(left_lane_boundary.begin(), left_lane_boundary.end(), std::back_inserter(boundary_points));

    size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
    for (size_t i = 0; i < right_lane_boundary_last_index; i++) {
        std::vector<Vec2d> segment{right_lane_boundary[i], right_lane_boundary[i + 1]};
        ADEBUG << "right segment";
        ADEBUG << "right_road_boundary: [" << std::setprecision(9) << right_lane_boundary[i].x() << ", "
               << right_lane_boundary[i].y() << "]";
        ADEBUG << "right_road_boundary: [" << std::setprecision(9) << right_lane_boundary[i + 1].x() << ", "
               << right_lane_boundary[i + 1].y() << "]";
        roi_parking_boundary->push_back(segment);
    }

    size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
    for (size_t i = left_lane_boundary_last_index; i > 0; i--) {
        std::vector<Vec2d> segment{left_lane_boundary[i], left_lane_boundary[i - 1]};
        roi_parking_boundary->push_back(segment);
    }

    PrintCurves print_curves;
    for (auto it : *roi_parking_boundary) {
        for (auto pt : it) {
            pt.SelfRotate(origin_heading);
            pt += origin_point;
            print_curves.AddPoint("roi_parking_boundary", pt);
        }
    }
    print_curves.PrintToLog();

    ADEBUG << "roi_parking_boundary size: [" << roi_parking_boundary->size() << "]";

    // Fuse line segments into convex contraints
    if (!FuseLineSegments(roi_parking_boundary)) {
        return false;
    }

    ADEBUG << "roi_parking_boundary size: [" << roi_parking_boundary->size() << "]";
    // Get xy boundary
    auto xminmax
            = std::minmax_element(boundary_points.begin(), boundary_points.end(), [](const Vec2d &a, const Vec2d &b) {
                  return a.x() < b.x();
              });
    auto yminmax
            = std::minmax_element(boundary_points.begin(), boundary_points.end(), [](const Vec2d &a, const Vec2d &b) {
                  return a.y() < b.y();
              });
    std::vector<double> ROI_xy_boundary{
            xminmax.first->x(), xminmax.second->x(), yminmax.first->y(), yminmax.second->y()};
    auto *xy_boundary = frame->mutable_open_space_info()->mutable_ROI_xy_boundary();
    xy_boundary->assign(ROI_xy_boundary.begin(), ROI_xy_boundary.end());

    Vec2d vehicle_xy = Vec2d(vehicle_state_.x(), vehicle_state_.y());
    vehicle_xy -= origin_point;
    vehicle_xy.SelfRotate(-origin_heading);
    if (vehicle_xy.x() < ROI_xy_boundary[0] || vehicle_xy.x() > ROI_xy_boundary[1]
        || vehicle_xy.y() < ROI_xy_boundary[2] || vehicle_xy.y() > ROI_xy_boundary[3]) {
        AERROR << "vehicle outside of xy boundary of parking ROI";
        return false;
    }
    return true;
}

bool OpenSpaceRoiDeciderPark::GetLargeCurvatureBoundary(
        Frame *const frame,
        const hdmap::Path &nearby_path,
        std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
    const auto &origin_point = frame->open_space_info().origin_point();
    const auto &origin_heading = frame->open_space_info().origin_heading();

    double ego_s = 0.0;
    double ego_l = 0.0;
    nearby_path.GetProjection(origin_point, &ego_s, &ego_l);
    std::vector<Vec2d> left_lane_boundary;
    std::vector<Vec2d> right_lane_boundary;
    std::vector<Vec2d> center_lane_boundary_left;
    std::vector<Vec2d> center_lane_boundary_right;
    std::vector<double> center_lane_s_left;
    std::vector<double> center_lane_s_right;
    std::vector<double> left_lane_road_width;
    std::vector<double> right_lane_road_width;

    if (config_.use_road_boundary_from_map()) {
        GetRoadBoundaryFromMap(
                nearby_path,
                ego_s,
                origin_point,
                origin_heading,
                &left_lane_boundary,
                &right_lane_boundary,
                &center_lane_boundary_left,
                &center_lane_boundary_right,
                &center_lane_s_left,
                &center_lane_s_right,
                &left_lane_road_width,
                &right_lane_road_width);
    } else {
        GetRoadBoundary(
                nearby_path,
                ego_s,
                origin_point,
                origin_heading,
                &left_lane_boundary,
                &right_lane_boundary,
                &center_lane_boundary_left,
                &center_lane_boundary_right,
                &center_lane_s_left,
                &center_lane_s_right,
                &left_lane_road_width,
                &right_lane_road_width);
    }

    // Load boundary as line segments in counter-clockwise order
    std::reverse(left_lane_boundary.begin(), left_lane_boundary.end());

    std::vector<Vec2d> boundary_points;
    std::copy(right_lane_boundary.begin(), right_lane_boundary.end(), std::back_inserter(boundary_points));
    std::copy(left_lane_boundary.begin(), left_lane_boundary.end(), std::back_inserter(boundary_points));

    size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
    for (size_t i = 0; i < right_lane_boundary_last_index; i++) {
        std::vector<Vec2d> segment{right_lane_boundary[i], right_lane_boundary[i + 1]};
        ADEBUG << "right segment";
        ADEBUG << "right_road_boundary: [" << std::setprecision(9) << right_lane_boundary[i].x() << ", "
               << right_lane_boundary[i].y() << "]";
        ADEBUG << "right_road_boundary: [" << std::setprecision(9) << right_lane_boundary[i + 1].x() << ", "
               << right_lane_boundary[i + 1].y() << "]";
        roi_parking_boundary->push_back(segment);
    }

    size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
    for (size_t i = left_lane_boundary_last_index; i > 0; i--) {
        std::vector<Vec2d> segment{left_lane_boundary[i], left_lane_boundary[i - 1]};
        ADEBUG << "left segment";
        ADEBUG << "left_road_boundary: [" << std::setprecision(9) << left_lane_boundary[i].x() << ", "
               << left_lane_boundary[i].y() << "]";
        ADEBUG << "left_road_boundary: [" << std::setprecision(9) << left_lane_boundary[i + 1].x() << ", "
               << left_lane_boundary[i + 1].y() << "]";
        roi_parking_boundary->push_back(segment);
    }

    PrintCurves print_curves;
    for (auto it : *roi_parking_boundary) {
        for (auto pt : it) {
            pt.SelfRotate(origin_heading);
            pt += origin_point;
            print_curves.AddPoint("roi_large_curvature_boundary", pt);
        }
    }
    print_curves.PrintToLog();

    ADEBUG << "roi_parking_boundary size: [" << roi_parking_boundary->size() << "]";

    // Fuse line segments into convex contraints
    if (!FuseLineSegments(roi_parking_boundary)) {
        return false;
    }

    ADEBUG << "roi_parking_boundary size: [" << roi_parking_boundary->size() << "]";
    // Get xy boundary
    auto xminmax
            = std::minmax_element(boundary_points.begin(), boundary_points.end(), [](const Vec2d &a, const Vec2d &b) {
                  return a.x() < b.x();
              });
    auto yminmax
            = std::minmax_element(boundary_points.begin(), boundary_points.end(), [](const Vec2d &a, const Vec2d &b) {
                  return a.y() < b.y();
              });
    std::vector<double> ROI_xy_boundary{
            xminmax.first->x(), xminmax.second->x(), yminmax.first->y(), yminmax.second->y()};
    auto *xy_boundary = frame->mutable_open_space_info()->mutable_ROI_xy_boundary();
    xy_boundary->assign(ROI_xy_boundary.begin(), ROI_xy_boundary.end());

    Vec2d vehicle_xy = Vec2d(vehicle_state_.x(), vehicle_state_.y());
    vehicle_xy -= origin_point;
    vehicle_xy.SelfRotate(-origin_heading);
    if (vehicle_xy.x() < ROI_xy_boundary[0] || vehicle_xy.x() > ROI_xy_boundary[1]
        || vehicle_xy.y() < ROI_xy_boundary[2] || vehicle_xy.y() > ROI_xy_boundary[3]) {
        AERROR << "vehicle outside of xy boundary of parking ROI";
        return false;
    }
    return true;
}

bool OpenSpaceRoiDeciderPark::GetParkingSpot(Frame *const frame, ParkingInfo *parking_info) {
    if (frame == nullptr) {
        AERROR << "Invalid frame, fail to GetParkingSpotFromMap from frame. ";
        return false;
    }
    const auto &parking_spot_id_string = frame->open_space_info().target_parking_spot_id();
    hdmap::Id parking_spot_id = hdmap::MakeMapId(parking_spot_id_string);
    auto parking_spot = hdmap_->GetParkingSpaceById(parking_spot_id);
    if (!nearby_path_) {
        GetNearbyPath(frame->local_view().planning_command->lane_follow_command(), parking_spot, &nearby_path_);
    }
    // points in polygon is always clockwise
    auto points = parking_spot->polygon().points();
    OpenSpaceRoiUtil::UpdateParkingPointsOrder(*nearby_path_, &points);
    Vec2d center_point(0, 0);
    for (size_t i = 0; i < points.size(); i++) {
        center_point += points[i];
    }
    center_point /= 4.0;
    double lane_heading = 0;
    parking_info->center_point = center_point;
    nearby_path_->GetHeadingAlongPath(center_point, &lane_heading);
    double s, l;
    nearby_path_->GetProjection(center_point, &s, &l);
    if (l > 0) {
        parking_info->is_on_left = true;
    } else {
        parking_info->is_on_left = false;
    }
    double diff_angle = common::math::AngleDiff(lane_heading, parking_spot->parking_space().heading());
    if (std::fabs(diff_angle) < M_PI / 3.0) {
        parking_info->parking_type = ParkingType::PARALLEL_PARKING;
    } else {
        parking_info->parking_type = ParkingType::VERTICAL_PARKING;
    }
    parking_info->corner_points = points;
    double parallel_dist = parking_info->corner_points[0].DistanceTo(parking_info->corner_points[1]);
    double verticle_dist = parking_info->corner_points[0].DistanceTo(parking_info->corner_points[3]);
    if (parallel_dist > verticle_dist) {
        parking_info->parking_type = ParkingType::PARALLEL_PARKING;
    } else {
        parking_info->parking_type = ParkingType::VERTICAL_PARKING;
    }
    return true;
}

bool OpenSpaceRoiDeciderPark::GetPullOverSpot(
        Frame *const frame,
        std::array<common::math::Vec2d, 4> *vertices,
        hdmap::Path *nearby_path) {
    const auto &pull_over_status = injector_->planning_context()->planning_status().pull_over();
    if (!pull_over_status.has_position() || !pull_over_status.position().has_x() || !pull_over_status.position().has_y()
        || !pull_over_status.has_theta()) {
        AERROR << "Pull over position not set in planning context";
        return false;
    }

    if (frame->reference_line_info().size() > 1) {
        AERROR << "Should not be in pull over when changing lane in open space "
                  "planning";
        return false;
    }

    *nearby_path = frame->reference_line_info().front().reference_line().GetMapPath();

    // Construct left_top, left_down, right_down, right_top points
    double pull_over_x = pull_over_status.position().x();
    double pull_over_y = pull_over_status.position().y();
    const double pull_over_theta = pull_over_status.theta();
    const double pull_over_length_front = pull_over_status.length_front();
    const double pull_over_length_back = pull_over_status.length_back();
    const double pull_over_width_left = pull_over_status.width_left();
    const double pull_over_width_right = pull_over_status.width_right();

    Vec2d center_shift_vec(
            (pull_over_length_front - pull_over_length_back) * 0.5,
            (pull_over_width_left - pull_over_width_right) * 0.5);
    center_shift_vec.SelfRotate(pull_over_theta);
    pull_over_x += center_shift_vec.x();
    pull_over_y += center_shift_vec.y();

    const double half_length = (pull_over_length_front + pull_over_length_back) / 2.0;
    const double half_width = (pull_over_width_left + pull_over_width_right) / 2.0;

    const double cos_heading = std::cos(pull_over_theta);
    const double sin_heading = std::sin(pull_over_theta);

    const double dx1 = cos_heading * half_length;
    const double dy1 = sin_heading * half_length;
    const double dx2 = sin_heading * half_width;
    const double dy2 = -cos_heading * half_width;

    Vec2d left_top(pull_over_x - dx1 + dx2, pull_over_y - dy1 + dy2);
    Vec2d left_down(pull_over_x - dx1 - dx2, pull_over_y - dy1 - dy2);
    Vec2d right_down(pull_over_x + dx1 - dx2, pull_over_y + dy1 - dy2);
    Vec2d right_top(pull_over_x + dx1 + dx2, pull_over_y + dy1 + dy2);

    std::array<Vec2d, 4> pull_over_vertices{left_top, left_down, right_down, right_top};
    *vertices = std::move(pull_over_vertices);

    return true;
}

void OpenSpaceRoiDeciderPark::SearchTargetParkingSpotOnPath(
        const hdmap::Path &nearby_path,
        ParkingSpaceInfoConstPtr *target_parking_spot) {
    const auto &parking_space_overlaps = nearby_path.parking_space_overlaps();
    for (const auto &parking_overlap : parking_space_overlaps) {
        if (parking_overlap.object_id == target_parking_spot_id_) {
            hdmap::Id id;
            id.set_id(parking_overlap.object_id);
            *target_parking_spot = hdmap_->GetParkingSpaceById(id);
        }
    }
}

bool OpenSpaceRoiDeciderPark::FuseLineSegments(std::vector<std::vector<common::math::Vec2d>> *line_segments_vec) {
    static constexpr double kEpsilon = 1.0e-8;
    auto cur_segment = line_segments_vec->begin();
    while (cur_segment != line_segments_vec->end() - 1) {
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
        if (CrossProd(cur_second_to_last_point, cur_last_point, next_second_point) < 0.0) {
            cur_segment->push_back(next_second_point);
            next_segment->erase(next_segment->begin(), next_segment->begin() + 2);
            if (next_segment->empty()) {
                line_segments_vec->erase(next_segment);
            }
        } else {
            ++cur_segment;
        }
    }
    return true;
}

bool OpenSpaceRoiDeciderPark::FormulateBoundaryConstraints(
        const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
        Frame *const frame) {
    // Gather vertice needed by warm start and distance approach
    if (!LoadObstacleInVertices(roi_parking_boundary, frame)) {
        AERROR << "fail at LoadObstacleInVertices()";
        return false;
    }
    // Transform vertices into the form of Ax>b
    if (!LoadObstacleInHyperPlanes(frame)) {
        AERROR << "fail at LoadObstacleInHyperPlanes()";
        return false;
    }
    return true;
}

bool OpenSpaceRoiDeciderPark::LoadObstacleInVertices(
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

    if (config_.enable_perception_obstacles()) {
        if (perception_obstacles_num == 0) {
            ADEBUG << "no obstacle given by perception";
        }

        // load vertices for perception obstacles(repeat the first vertice at the
        // last to form closed convex hull)
        const auto &origin_point = open_space_info.origin_point();
        const auto &origin_heading = open_space_info.origin_heading();
        for (const auto &obstacle : obstacles_by_frame_->Items()) {
            if (FilterOutObstacle(*frame, *obstacle)) {
                continue;
            }
            ++perception_obstacles_num;

            std::vector<Vec2d> vertices_ccw;
            if (config_.expand_polygon_of_obstacle_by_distance()) {
                common::math::Polygon2d original_polygon = obstacle->PerceptionPolygon();
                original_polygon.ExpandByDistance(config_.perception_obstacle_buffer());
                original_polygon.CalculateVertices(-1.0 * origin_point);
                vertices_ccw = original_polygon.GetAllVertices();
            } else {
                Box2d original_box = obstacle->PerceptionBoundingBox();
                original_box.Shift(-1.0 * origin_point);
                original_box.LongitudinalExtend(config_.perception_obstacle_buffer());
                original_box.LateralExtend(config_.perception_obstacle_buffer());
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

bool OpenSpaceRoiDeciderPark::FilterOutObstacle(const Frame &frame, const Obstacle &obstacle) {
    if (obstacle.IsVirtual() || !obstacle.IsStatic()) {
        return true;
    }

    const auto &open_space_info = frame.open_space_info();
    const auto &origin_point = open_space_info.origin_point();
    const auto &origin_heading = open_space_info.origin_heading();
    const auto &obstacle_box = obstacle.PerceptionBoundingBox();
    auto obstacle_center_xy = obstacle_box.center();

    // xy_boundary in xmin, xmax, ymin, ymax.
    const auto &roi_xy_boundary = open_space_info.ROI_xy_boundary();
    obstacle_center_xy -= origin_point;
    obstacle_center_xy.SelfRotate(-origin_heading);
    if (obstacle_center_xy.x() < roi_xy_boundary[0] || obstacle_center_xy.x() > roi_xy_boundary[1]
        || obstacle_center_xy.y() < roi_xy_boundary[2] || obstacle_center_xy.y() > roi_xy_boundary[3]) {
        return true;
    }

    // Translate the end pose back to world frame with endpose in x, y, phi, v
    const auto &end_pose = open_space_info.open_space_end_pose();
    Vec2d end_pose_x_y(end_pose[0], end_pose[1]);
    end_pose_x_y.SelfRotate(origin_heading);
    end_pose_x_y += origin_point;

    // Get vehicle state
    Vec2d vehicle_x_y(vehicle_state_.x(), vehicle_state_.y());

    // Use vehicle position and end position to filter out obstacle
    const double vehicle_center_to_obstacle = obstacle_box.DistanceTo(vehicle_x_y);
    const double end_pose_center_to_obstacle = obstacle_box.DistanceTo(end_pose_x_y);
    const double filtering_distance = config_.perception_obstacle_filtering_distance();
    if (vehicle_center_to_obstacle > filtering_distance && end_pose_center_to_obstacle > filtering_distance) {
        return true;
    }
    return false;
}

bool OpenSpaceRoiDeciderPark::LoadObstacleInHyperPlanes(Frame *const frame) {
    *(frame->mutable_open_space_info()->mutable_obstacles_A())
            = Eigen::MatrixXd::Zero(frame->open_space_info().obstacles_edges_num().sum(), 2);
    *(frame->mutable_open_space_info()->mutable_obstacles_b())
            = Eigen::MatrixXd::Zero(frame->open_space_info().obstacles_edges_num().sum(), 1);
    // vertices using H-representation
    if (!GetHyperPlanes(
                frame->open_space_info().obstacles_num(),
                frame->open_space_info().obstacles_edges_num(),
                frame->open_space_info().obstacles_vertices_vec(),
                frame->mutable_open_space_info()->mutable_obstacles_A(),
                frame->mutable_open_space_info()->mutable_obstacles_b())) {
        AERROR << "Fail to present obstacle in hyperplane";
        return false;
    }
    return true;
}

bool OpenSpaceRoiDeciderPark::GetHyperPlanes(
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

bool OpenSpaceRoiDeciderPark::IsInParkingLot(
        const double adc_init_x,
        const double adc_init_y,
        const double adc_init_heading,
        std::array<Vec2d, 4> *parking_lot_vertices) {
    std::vector<ParkingSpaceInfoConstPtr> parking_lots;
    // make sure there is only one parking lot in search range
    const double kDistance = 1.0;
    auto adc_parking_spot = common::util::PointFactory::ToPointENU(adc_init_x, adc_init_y, 0);
    ADEBUG << "IsInParkingLot";
    ADEBUG << hdmap_;
    ADEBUG << hdmap_->GetParkingSpaces(adc_parking_spot, kDistance, &parking_lots);
    if (hdmap_->GetParkingSpaces(adc_parking_spot, kDistance, &parking_lots) == 0) {
        GetParkSpotFromMap(parking_lots.front(), parking_lot_vertices);
        AINFO << "Get park lot from map!!";
        return true;
    }
    return false;
}

void OpenSpaceRoiDeciderPark::GetParkSpotFromMap(ParkingSpaceInfoConstPtr parking_lot, std::array<Vec2d, 4> *vertices) {
    // left or right of the parking lot is decided when viewing the parking spot
    // open upward
    Vec2d left_top = parking_lot->polygon().points().at(3);
    Vec2d left_down = parking_lot->polygon().points().at(0);
    Vec2d right_down = parking_lot->polygon().points().at(1);
    Vec2d right_top = parking_lot->polygon().points().at(2);

    std::array<Vec2d, 4> parking_vertices{left_top, left_down, right_down, right_top};

    *vertices = std::move(parking_vertices);
    Vec2d tmp = (*vertices)[0];
    ADEBUG << "Parking Lot";
    ADEBUG << "parking_lot_vertices: (" << tmp.x() << ", " << tmp.y() << ")";
}

void OpenSpaceRoiDeciderPark::GetAllLaneSegments(
        const routing::RoutingResponse &routing_response,
        std::vector<routing::LaneSegment> *routing_segments) {
    routing_segments->clear();
    for (const auto &road : routing_response.road()) {
        for (const auto &passage : road.passage()) {
            for (const auto &segment : passage.segment()) {
                routing_segments->emplace_back(segment);
            }
        }
    }
}

bool OpenSpaceRoiDeciderPark::GetNearbyPath(
        const apollo::routing::RoutingResponse &routing_response,
        const ParkingSpaceInfoConstPtr &parking_spot,
        std::shared_ptr<hdmap::Path> *nearby_path) {
    LaneInfoConstPtr nearest_lane;
    if (nullptr == parking_spot) {
        AERROR << "The parking spot id is invalid!" << parking_spot->id().id();
        return false;
    }
    auto parking_space = parking_spot->parking_space();
    auto overlap_ids = parking_space.overlap_id();
    if (overlap_ids.empty()) {
        AERROR << "There is no lane overlaps with the parking spot: " << parking_spot->id().id();
        return false;
    }
    std::vector<routing::LaneSegment> lane_segments;
    GetAllLaneSegments(routing_response, &lane_segments);
    bool has_found_nearest_lane = false;
    size_t nearest_lane_index = 0;
    for (auto id : overlap_ids) {
        auto overlaps = hdmap_->GetOverlapById(id)->overlap();
        for (auto object : overlaps.object()) {
            if (!object.has_lane_overlap_info()) {
                continue;
            }
            nearest_lane = hdmap_->GetLaneById(object.id());
            if (nearest_lane == nullptr) {
                continue;
            }
            // Check if the lane is contained in the routing response.
            for (auto &segment : lane_segments) {
                if (segment.id() == nearest_lane->id().id()) {
                    has_found_nearest_lane = true;
                    break;
                }
                ++nearest_lane_index;
            }
            if (has_found_nearest_lane) {
                break;
            }
        }
    }
    if (!has_found_nearest_lane) {
        AERROR << "Cannot find the lane nearest to the parking spot when "
                  "GetParkingSpot!";
    }

    // Get the lane nearest to the current position of the vehicle. If the
    // vehicle has not reached the nearest lane to the parking spot, set the
    // lane nearest to the vehicle as "nearest_lane".
    LaneInfoConstPtr nearest_lane_to_vehicle;
    auto point = common::util::PointFactory::ToPointENU(vehicle_state_);
    double vehicle_lane_s = 0.0;
    double vehicle_lane_l = 0.0;
    int status = hdmap_->GetNearestLaneWithHeading(
            point,
            10.0,
            vehicle_state_.heading(),
            M_PI / 2.0,
            &nearest_lane_to_vehicle,
            &vehicle_lane_s,
            &vehicle_lane_l);
    if (status == 0) {
        size_t nearest_lane_to_vehicle_index = 0;
        bool has_found_nearest_lane_to_vehicle = false;
        for (auto &segment : lane_segments) {
            if (segment.id() == nearest_lane_to_vehicle->id().id()) {
                has_found_nearest_lane_to_vehicle = true;
                break;
            }
            ++nearest_lane_to_vehicle_index;
        }
        // The vehicle has not reached the nearest lane to the parking spot。
        if (has_found_nearest_lane_to_vehicle && nearest_lane_to_vehicle_index < nearest_lane_index) {
            nearest_lane = nearest_lane_to_vehicle;
        }
    }

    // Find parking spot by getting nearestlane
    ParkingSpaceInfoConstPtr target_parking_spot = nullptr;
    LaneSegment nearest_lanesegment
            = LaneSegment(nearest_lane, nearest_lane->accumulate_s().front(), nearest_lane->accumulate_s().back());
    std::vector<LaneSegment> segments_vector;
    int next_lanes_num = nearest_lane->lane().successor_id_size();
    if (next_lanes_num != 0) {
        auto next_lane_id = nearest_lane->lane().successor_id(0);
        segments_vector.push_back(nearest_lanesegment);
        auto next_lane = hdmap_->GetLaneById(next_lane_id);
        LaneSegment next_lanesegment
                = LaneSegment(next_lane, next_lane->accumulate_s().front(), next_lane->accumulate_s().back());
        segments_vector.push_back(next_lanesegment);
        size_t succeed_lanes_num = next_lane->lane().successor_id_size();
        if (succeed_lanes_num != 0) {
            auto succeed_lane_id = next_lane->lane().successor_id(0);
            auto succeed_lane = hdmap_->GetLaneById(succeed_lane_id);
            LaneSegment succeed_lanesegment = LaneSegment(
                    succeed_lane, succeed_lane->accumulate_s().front(), succeed_lane->accumulate_s().back());
            segments_vector.push_back(succeed_lanesegment);
        }
        *nearby_path = std::make_shared<Path>(segments_vector);
    } else {
        segments_vector.push_back(nearest_lanesegment);
        *nearby_path = std::make_shared<Path>(segments_vector);
    }
    return true;
}

bool OpenSpaceRoiDeciderPark::GetUTurnPath(Frame *const frame, std::shared_ptr<hdmap::Path> *nearby_path) {
    if (frame == nullptr) {
        AERROR << "Frame is nullptr";
        return false;
    }

    std::vector<LaneSegment> segments_vector;

    const apollo::routing::RoutingResponse &routing_response
            = frame->local_view().planning_command->lane_follow_command();
    std::vector<routing::LaneSegment> lane_segments;
    GetAllLaneSegments(routing_response, &lane_segments);

    std::pair<double, std::string> min_lane = std::make_pair(std::numeric_limits<double>::max(), "");
    for (auto &seg : lane_segments) {
        auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(seg.id()));
        Vec2d proj_pt(0.0, 0.0);
        double s_offset = 0.0;
        int s_offset_index = 0;
        double dis = lane->DistanceTo({vehicle_state_.x(), vehicle_state_.y()}, &proj_pt, &s_offset, &s_offset_index);
        if (dis < min_lane.first) {
            min_lane = std::make_pair(dis, seg.id());
        }
    }

    bool has_found_nearest_lane = false;
    static constexpr int add_lane_num = 3;
    int lane_num = 0;
    hdmap::LaneInfoConstPtr previous_lane;
    for (auto &segment : lane_segments) {
        AINFO << "lane id: " << segment.id();
        if (lane_num == add_lane_num) {
            break;
        }
        if (segment.id() == min_lane.second) {
            has_found_nearest_lane = true;
        }
        if (has_found_nearest_lane) {
            AINFO << "routing lane id: " << segment.id();
            auto next_lane = hdmap_->GetLaneById(hdmap::MakeMapId(segment.id()));
            AINFO << "next lane id: " << next_lane->id().id();
            LaneSegment next_lanesegment
                    = LaneSegment(next_lane, next_lane->accumulate_s().front(), next_lane->accumulate_s().back());
            AINFO << "next lane s: " << next_lanesegment.start_s << " end s: " << next_lanesegment.end_s;
            segments_vector.push_back(next_lanesegment);
            lane_num++;
            if (lane_num >= add_lane_num && previous_lane != nullptr
                && fabs(previous_lane->Curvature(previous_lane->accumulate_s().back())) < 0.1) {
                AINFO << fabs(previous_lane->Curvature(previous_lane->accumulate_s().back()));
                AINFO << "previous lane is u turn";
                break;
            }
            previous_lane = next_lane;
        }
    }
    if (segments_vector.empty()) {
        AINFO << "No lane found";
        return false;
    }
    *nearby_path = std::make_shared<Path>(segments_vector);
    return true;
}

bool OpenSpaceRoiDeciderPark::AdjustPointsOrderToClockwise(std::vector<Vec2d> *polygon) {
    if (!OpenSpaceRoiUtil::IsPolygonClockwise(*polygon)) {
        // counter clockwise reverse it
        ADEBUG << "point is anticlockwise,reverse";
        std::reverse(polygon->begin(), polygon->end());
        return true;
    } else {
        return false;
    }
}

bool OpenSpaceRoiDeciderPark::GetParkingOutBoundary(
        const hdmap::Path &nearby_path,
        Frame *const frame,
        std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
    const auto &park_and_go_status = injector_->planning_context()->planning_status().park_and_go();
    const double adc_init_x = park_and_go_status.adc_init_position().x();
    const double adc_init_y = park_and_go_status.adc_init_position().y();
    const double adc_init_heading = park_and_go_status.adc_init_heading();
    common::math::Vec2d adc_init_position = {adc_init_x, adc_init_y};
    const double adc_length = vehicle_params_.length();
    const double adc_width = vehicle_params_.width();
    AINFO << std::fixed << "adc_init_x is " << adc_init_x << "adc_init_y is " << adc_init_y << "adc_init_heading is "
          << adc_init_heading;
    // Current localization position is not in the center of vehicle
    double shift_distance = vehicle_params_.front_edge_to_center() - 0.5 * adc_length;
    adc_init_position = adc_init_position + Vec2d::CreateUnitVec2d(adc_init_heading) * shift_distance;
    // ADC box of Vehcile
    Box2d adc_box(adc_init_position, adc_init_heading, adc_length, adc_width);
    // get vertices from ADC box
    std::vector<common::math::Vec2d> adc_corners;
    adc_box.GetAllCorners(&adc_corners);
    // Get the parking spot points where the vehicle is currently located
    std::vector<ParkingSpaceInfoConstPtr> parking_lots;
    auto adc_parking_spot = common::util::PointFactory::ToPointENU(adc_init_x, adc_init_y, 0);
    const double kDistance = 1.0;
    if (hdmap_->GetParkingSpaces(adc_parking_spot, kDistance, &parking_lots) != 0) {
        AINFO << "Failed to get the parking spot!!!";
        return false;
    } else {
        AINFO << "Get " << parking_lots.size() << " parking spots";
    }
    std::vector<Vec2d> parking_boundary;
    for (const auto &parking_overlap : parking_lots) {
        const auto parking_polygon = parking_overlap->polygon();
        AINFO << "parking_polygon: " << parking_polygon.DebugString();
        bool is_in_parking_spot = true;
        for (const auto &corner : adc_corners) {
            if (!parking_polygon.IsPointIn(corner)) {
                is_in_parking_spot = false;
                AINFO << "Vehicle is out of parking spot!";
                break;
            }
        }
        if (is_in_parking_spot) {
            auto points = parking_polygon.points();
            OpenSpaceRoiUtil::UpdateParkingPointsOrder(nearby_path, &points);
            for (size_t i = 0; i < points.size(); i++) {
                int t = static_cast<int>(i + 1) % static_cast<int>(points.size());
                parking_boundary.emplace_back(points.at(t).x(), points.at(t).y());
            }
        }
    }
    if (parking_boundary.size() < 4) {
        AINFO << " Current parking spot is invalid!";
        return false;
    }
    auto left_top = parking_boundary[3];
    auto right_top = parking_boundary[0];
    auto right_down = parking_boundary[1];
    auto left_down = parking_boundary[2];

    double left_top_s = 0.0;
    double left_top_l = 0.0;
    double right_top_s = 0.0;
    double right_top_l = 0.0;
    double left_down_s = 0.0;
    double left_down_l = 0.0;
    double right_down_s = 0.0;
    double right_down_l = 0.0;
    if (!(nearby_path.GetProjection(left_top, &left_top_s, &left_top_l)
          && nearby_path.GetProjection(right_top, &right_top_s, &right_top_l))) {
        AERROR << "fail to get parking spot points' projections on reference line";
        return false;
    }

    if (!(nearby_path.GetProjection(left_down, &left_down_s, &left_down_l)
          && nearby_path.GetProjection(right_down, &right_down_s, &right_down_l))) {
        AERROR << "fail to get parking spot points' projections on reference line";
        return false;
    }
    if (fabs(left_top_l + right_top_l) > fabs(left_down_l + right_down_l)) {
        frame->mutable_open_space_info()->mutable_origin_point()->set_x(right_down.x());
        frame->mutable_open_space_info()->mutable_origin_point()->set_y(right_down.y());
        double heading;
        if (!nearby_path.GetHeadingAlongPath(right_down, &heading)) {
            AERROR << "fail to get heading on reference line";
            return false;
        }
        frame->mutable_open_space_info()->set_origin_heading(common::math::NormalizeAngle(heading));
        std::swap(right_down, left_top);
        std::swap(right_top, left_down);
        std::swap(right_down_s, left_top_s);
        std::swap(right_down_l, left_top_l);
        std::swap(right_top_s, left_down_s);
        std::swap(right_top_l, left_down_l);
    }

    const double center_line_s = (left_top_s + right_top_s) / 2.0;
    std::vector<Vec2d> left_lane_boundary;
    std::vector<Vec2d> right_lane_boundary;
    // The pivot points on the central lane, mapping with the key points on
    // the left lane boundary.
    std::vector<Vec2d> center_lane_boundary_left;
    // The pivot points on the central lane, mapping with the key points on
    // the right lane boundary.
    std::vector<Vec2d> center_lane_boundary_right;
    // The s-value for the anchor points on the center_lane_boundary_left.
    std::vector<double> center_lane_s_left;
    // The s-value for the anchor points on the center_lane_boundary_right.
    std::vector<double> center_lane_s_right;
    // The left-half road width between the pivot points on the
    // center_lane_boundary_left and key points on the
    // left_lane_boundary.
    std::vector<double> left_lane_road_width;
    // The right-half road width between the pivot points on the
    // center_lane_boundary_right and key points on the
    // right_lane_boundary.
    std::vector<double> right_lane_road_width;

    GetRoadBoundary(
            nearby_path,
            center_line_s,
            Vec2d(0.0, 0.0),
            0.0,
            &left_lane_boundary,
            &right_lane_boundary,
            &center_lane_boundary_left,
            &center_lane_boundary_right,
            &center_lane_s_left,
            &center_lane_s_right,
            &left_lane_road_width,
            &right_lane_road_width);

    // If smaller than zero, the parking spot is on the right of the lane
    // Left, right, down or opposite of the boundary is decided when viewing the
    // parking spot upward
    const double average_l = (left_top_l + right_top_l) / 2.0;
    const double average_s = (left_top_s + right_top_s) / 2.0;

    std::vector<Vec2d> boundary_points;

    // TODO(jiaxuan): Write a half-boundary formation function and call it twice
    // to avoid duplicated manipulations on the left and right sides
    if (average_l < 0) {
        // if average_l is lower than zero, the parking spot is on the right
        // lane boundary and assume that the lane half width is average_l
        ADEBUG << "average_l is less than 0 in OpenSpaceROI";
        size_t point_size = right_lane_boundary.size();
        for (size_t i = 0; i < point_size; i++) {
            right_lane_boundary[i] -= center_lane_boundary_right[i];
            right_lane_boundary[i] /= right_lane_road_width[i];
            if (center_lane_s_right[i] < (average_s)) {
                right_lane_boundary[i] *= (std::fabs(left_top_l));
            } else {
                right_lane_boundary[i] *= (std::fabs(right_top_l));
            }
            right_lane_boundary[i] += center_lane_boundary_right[i];
        }

        auto point_left_to_left_top_connor_s
                = std::lower_bound(center_lane_s_right.begin(), center_lane_s_right.end(), left_top_s);
        size_t point_left_to_left_top_connor_index
                = std::distance(center_lane_s_right.begin(), point_left_to_left_top_connor_s);
        point_left_to_left_top_connor_index = point_left_to_left_top_connor_index == 0
                ? point_left_to_left_top_connor_index
                : point_left_to_left_top_connor_index - 1;
        auto point_left_to_left_top_connor_itr = right_lane_boundary.begin() + point_left_to_left_top_connor_index;
        auto point_right_to_right_top_connor_s
                = std::upper_bound(center_lane_s_right.begin(), center_lane_s_right.end(), right_top_s);
        size_t point_right_to_right_top_connor_index
                = std::distance(center_lane_s_right.begin(), point_right_to_right_top_connor_s);
        auto point_right_to_right_top_connor_itr = right_lane_boundary.begin() + point_right_to_right_top_connor_index;

        std::copy(right_lane_boundary.begin(), point_left_to_left_top_connor_itr, std::back_inserter(boundary_points));

        std::vector<Vec2d> parking_spot_boundary{left_top, left_down, right_down, right_top};

        std::copy(parking_spot_boundary.begin(), parking_spot_boundary.end(), std::back_inserter(boundary_points));

        std::copy(point_right_to_right_top_connor_itr, right_lane_boundary.end(), std::back_inserter(boundary_points));

        std::reverse_copy(left_lane_boundary.begin(), left_lane_boundary.end(), std::back_inserter(boundary_points));

        // reinsert the initial point to the back to from closed loop
        boundary_points.push_back(right_lane_boundary.front());

        // disassemble line into line2d segments
        for (size_t i = 0; i < point_left_to_left_top_connor_index; i++) {
            std::vector<Vec2d> segment{right_lane_boundary[i], right_lane_boundary[i + 1]};
            roi_parking_boundary->push_back(segment);
        }

        std::vector<Vec2d> left_stitching_segment{right_lane_boundary[point_left_to_left_top_connor_index], left_top};
        roi_parking_boundary->push_back(left_stitching_segment);

        std::vector<Vec2d> left_parking_spot_segment{left_top, left_down};
        std::vector<Vec2d> down_parking_spot_segment{left_down, right_down};
        std::vector<Vec2d> right_parking_spot_segment{right_down, right_top};
        roi_parking_boundary->push_back(left_parking_spot_segment);
        roi_parking_boundary->push_back(down_parking_spot_segment);
        roi_parking_boundary->push_back(right_parking_spot_segment);

        std::vector<Vec2d> right_stitching_segment{
                right_top, right_lane_boundary[point_right_to_right_top_connor_index]};
        roi_parking_boundary->push_back(right_stitching_segment);

        size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
        for (size_t i = point_right_to_right_top_connor_index; i < right_lane_boundary_last_index; i++) {
            std::vector<Vec2d> segment{right_lane_boundary[i], right_lane_boundary[i + 1]};
            roi_parking_boundary->push_back(segment);
        }

        size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
        for (size_t i = left_lane_boundary_last_index; i > 0; i--) {
            std::vector<Vec2d> segment{left_lane_boundary[i], left_lane_boundary[i - 1]};
            roi_parking_boundary->push_back(segment);
        }

    } else {
        // if average_l is higher than zero, the parking spot is on the left
        // lane boundary and assume that the lane half width is average_l
        ADEBUG << "average_l is greater than 0 in OpenSpaceROI";
        size_t point_size = left_lane_boundary.size();
        for (size_t i = 0; i < point_size; i++) {
            left_lane_boundary[i] -= center_lane_boundary_left[i];
            left_lane_boundary[i] /= left_lane_road_width[i];
            if (center_lane_s_right[i] < (average_s)) {
                left_lane_boundary[i] *= (std::fabs(right_top_l));
            } else {
                left_lane_boundary[i] *= (std::fabs(left_top_l));
            }
            left_lane_boundary[i] += center_lane_boundary_left[i];
            ADEBUG << "left_lane_boundary[" << i << "]: " << left_lane_boundary[i].x() << ", "
                   << left_lane_boundary[i].y();
        }

        auto point_right_to_right_top_connor_s
                = std::lower_bound(center_lane_s_left.begin(), center_lane_s_left.end(), right_top_s);
        size_t point_right_to_right_top_connor_index
                = std::distance(center_lane_s_left.begin(), point_right_to_right_top_connor_s);
        if (point_right_to_right_top_connor_index > 0) {
            --point_right_to_right_top_connor_index;
        }
        auto point_right_to_right_top_connor_itr = left_lane_boundary.begin() + point_right_to_right_top_connor_index;

        auto point_left_to_left_top_connor_s
                = std::upper_bound(center_lane_s_left.begin(), center_lane_s_left.end(), left_top_s);
        size_t point_left_to_left_top_connor_index
                = std::distance(center_lane_s_left.begin(), point_left_to_left_top_connor_s);
        auto point_left_to_left_top_connor_itr = left_lane_boundary.begin() + point_left_to_left_top_connor_index;

        std::copy(right_lane_boundary.begin(), right_lane_boundary.end(), std::back_inserter(boundary_points));

        std::reverse_copy(
                point_left_to_left_top_connor_itr, left_lane_boundary.end(), std::back_inserter(boundary_points));

        std::vector<Vec2d> parking_spot_boundary{left_top, left_down, right_down, right_top};
        std::copy(parking_spot_boundary.begin(), parking_spot_boundary.end(), std::back_inserter(boundary_points));

        std::reverse_copy(
                left_lane_boundary.begin(), point_right_to_right_top_connor_itr, std::back_inserter(boundary_points));

        // reinsert the initial point to the back to from closed loop
        boundary_points.push_back(right_lane_boundary.front());

        // disassemble line into line2d segments
        size_t right_lane_boundary_last_index = right_lane_boundary.size() - 1;
        for (size_t i = 0; i < right_lane_boundary_last_index; i++) {
            std::vector<Vec2d> segment{right_lane_boundary[i], right_lane_boundary[i + 1]};
            roi_parking_boundary->push_back(segment);
        }

        size_t left_lane_boundary_last_index = left_lane_boundary.size() - 1;
        for (size_t i = left_lane_boundary_last_index; i > point_left_to_left_top_connor_index; i--) {
            std::vector<Vec2d> segment{left_lane_boundary[i], left_lane_boundary[i - 1]};
            roi_parking_boundary->push_back(segment);
        }

        std::vector<Vec2d> left_stitching_segment{left_lane_boundary[point_left_to_left_top_connor_index], left_top};
        roi_parking_boundary->push_back(left_stitching_segment);

        std::vector<Vec2d> left_parking_spot_segment{left_top, left_down};
        std::vector<Vec2d> down_parking_spot_segment{left_down, right_down};
        std::vector<Vec2d> right_parking_spot_segment{right_down, right_top};
        roi_parking_boundary->push_back(left_parking_spot_segment);
        roi_parking_boundary->push_back(down_parking_spot_segment);
        roi_parking_boundary->push_back(right_parking_spot_segment);

        std::vector<Vec2d> right_stitching_segment{
                right_top, left_lane_boundary[point_right_to_right_top_connor_index]};
        roi_parking_boundary->push_back(right_stitching_segment);

        for (size_t i = point_right_to_right_top_connor_index; i > 0; --i) {
            std::vector<Vec2d> segment{left_lane_boundary[i], left_lane_boundary[i - 1]};
            roi_parking_boundary->push_back(segment);
        }
    }
    PrintCurves print_curves;
    const auto &origin_point = frame->mutable_open_space_info()->origin_point();
    double origin_heading = frame->mutable_open_space_info()->origin_heading();
    // AddParkingSpaceBoundary(frame, nearby_path, roi_parking_boundary);
    for (auto it : *roi_parking_boundary) {
        for (auto pt : it) {
            print_curves.AddPoint("roi_parking_boundary", pt);
        }
    }
    OpenSpaceRoiUtil::TransformByOriginPoint(origin_point, origin_heading, roi_parking_boundary);
    for (auto it : *roi_parking_boundary) {
        for (auto pt : it) {
            print_curves.AddPoint("transformed_roi_parking_boundary", pt);
        }
    }
    // Fuse line segments into convex contraints
    if (!FuseLineSegments(roi_parking_boundary)) {
        AERROR << "FuseLineSegments failed in parking ROI";
        return false;
    }
    print_curves.PrintToLog();
    auto *xy_boundary = frame->mutable_open_space_info()->mutable_ROI_xy_boundary();
    OpenSpaceRoiUtil::GetRoiXYBoundary(*roi_parking_boundary, xy_boundary);

    Vec2d vehicle_xy = Vec2d(vehicle_state_.x(), vehicle_state_.y());
    vehicle_xy -= origin_point;
    vehicle_xy.SelfRotate(-origin_heading);
    if (vehicle_xy.x() < xy_boundary->at(0) || vehicle_xy.x() > xy_boundary->at(1)
        || vehicle_xy.y() < xy_boundary->at(2) || vehicle_xy.y() > xy_boundary->at(3)) {
        AERROR << "vehicle outside of xy boundary of parking ROI";
        return false;
    }
    AINFO << "success get ROI";
    return true;
}

bool OpenSpaceRoiDeciderPark::AddParkingSpaceBoundary(
        Frame *const frame,
        const hdmap::Path &nearby_path,
        std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary) {
    const auto &park_and_go_status = injector_->planning_context()->planning_status().park_and_go();
    const double adc_init_x = park_and_go_status.adc_init_position().x();
    const double adc_init_y = park_and_go_status.adc_init_position().y();
    const double adc_init_heading = park_and_go_status.adc_init_heading();
    common::math::Vec2d adc_init_position = {adc_init_x, adc_init_y};
    const double adc_length = vehicle_params_.length();
    const double adc_width = vehicle_params_.width();
    // ADC box
    double shift_distance = vehicle_params_.front_edge_to_center() - 0.5 * adc_length;
    adc_init_position = adc_init_position + Vec2d::CreateUnitVec2d(adc_init_heading) * shift_distance;
    Box2d adc_box(adc_init_position, adc_init_heading, adc_length, adc_width);
    // get vertices from ADC box
    std::vector<common::math::Vec2d> adc_corners;
    adc_box.GetAllCorners(&adc_corners);
    PrintCurves print_curves;
    for (auto pt : adc_corners) {
        print_curves.AddPoint("adc_corners", pt);
    }
    for (const auto &parking_overlap : nearby_path.parking_space_overlaps()) {
        hdmap::Id id;
        id.set_id(parking_overlap.object_id);
        const auto target_parking_spot = hdmap_->GetParkingSpaceById(id);
        if (!target_parking_spot) {
            continue;
        }
        const auto parking_polygon = target_parking_spot->polygon();
        for (const auto &pt : parking_polygon.points()) {
            print_curves.AddPoint(id.id() + "parking_polygon", pt);
        }
        bool is_in_polygon = true;
        for (const auto &pt : adc_corners) {
            if (!parking_polygon.IsPointIn(pt)) {
                is_in_polygon = false;
                break;
            }
        }
        if (is_in_polygon) {
            auto points = parking_polygon.points();
            OpenSpaceRoiUtil::UpdateParkingPointsOrder(nearby_path, &points);
            std::vector<Vec2d> parking_boundary;
            for (size_t i = 0; i < points.size(); i++) {
                int t = static_cast<int>(i + 1) % static_cast<int>(points.size());
                parking_boundary.emplace_back(points.at(t).x(), points.at(t).y());
            }
            roi_parking_boundary->push_back(parking_boundary);
        }
    }
    print_curves.PrintToLog();
    return true;
}

}  // namespace planning
}  // namespace apollo
