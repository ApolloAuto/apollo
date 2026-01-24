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

#include "modules/planning/scenarios/free_space/free_space_scenario.h"

#include <vector>

#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_open_space/utils/open_space_roi_util.h"
#include "modules/planning/scenarios/free_space/stage_free_space.h"

namespace apollo {
namespace planning {
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::external_command::FreeSpaceCommand;

bool FreeSpaceScenario::Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) {
    // hook: Apollo License Verification: v_apollo_park
    if (init_) {
        return true;
    }

    if (!Scenario::Init(injector, name)) {
        AERROR << "failed to init scenario" << Name();
        return false;
    }

    if (!Scenario::LoadConfig<ScenarioFreeSpaceConfig>(&context_.scenario_config)) {
        AERROR << "fail to get config of scenario" << Name();
        return false;
    }
    hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
    CHECK_NOTNULL(hdmap_);
    init_ = true;
    return true;
}

bool FreeSpaceScenario::IsTransferable(const Scenario* const other_scenario, const Frame& frame) {
    const auto& planning_command = frame.local_view().planning_command;
    if (!planning_command->has_custom_command()) {
        return false;
    }
    if (!planning_command->custom_command().Is<FreeSpaceCommand>()) {
        return false;
    }
    context_.free_space_command.Clear();
    if (!planning_command->custom_command().UnpackTo(&context_.free_space_command)) {
        AERROR << "Free Space Command unpack error" << planning_command->DebugString();
        return false;
    }
    const auto& vehicle_state = frame.vehicle_state();
    const auto& vehicle_param = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
    double adc_speed = vehicle_state.linear_velocity();
    const double max_adc_stop_speed = vehicle_param.max_abs_speed_when_stopped();
    if (std::fabs(adc_speed) > max_adc_stop_speed) {
        AINFO << "adc not stopped" << adc_speed;
        return false;
    }
    apollo::common::math::Polygon2d ego_box(injector_->ego_info()->ego_box());
    Vec2d vec_to_center(
            (vehicle_param.front_edge_to_center() - vehicle_param.back_edge_to_center()) / 2.0,
            (vehicle_param.left_edge_to_center() - vehicle_param.right_edge_to_center()) / 2.0);
    const auto& end_pose = context_.free_space_command.parking_spot_pose();
    Vec2d position(end_pose.x(), end_pose.y());
    Vec2d center(position + vec_to_center.rotate(end_pose.heading()));
    Polygon2d end_box(Box2d(center, end_pose.heading(), vehicle_param.length(), vehicle_param.width()));
    PrintCurves print_curves;
    print_curves.AddPoint("ego_box", ego_box.points());
    print_curves.AddPoint("end_box", end_box.points());
    if (!context_.free_space_command.has_drivable_roi()) {
        AERROR << "drivable_roi is needed!";
        return false;
    }
    const auto& roi_polygon = context_.free_space_command.drivable_roi();
    std::vector<Vec2d> polygon_points;
    for (const auto& point : roi_polygon.point()) {
        polygon_points.emplace_back(point.x(), point.y());
    }
    print_curves.AddPoint("free_space_roi", polygon_points);
    Polygon2d polygon_roi(polygon_points);
    if (OpenSpaceRoiUtil::IsPolygonClockwise(polygon_points)) {
        AERROR << "drivable_roi should be counter-clockwise!";
        print_curves.PrintToLog();
        return false;
    }
    if (!polygon_roi.Contains(ego_box) || !polygon_roi.Contains(end_box)) {
        AERROR << "ego box or end box is out of roi";
        print_curves.PrintToLog();
        return false;
    }
    for (const auto& roi_polygon : context_.free_space_command.non_drivable_roi()) {
        std::vector<Vec2d> polygon_points;
        for (const auto& point : roi_polygon.point()) {
            polygon_points.emplace_back(point.x(), point.y());
        }
        print_curves.AddPoint("free_space_roi", polygon_points);
        Polygon2d polygon_roi(polygon_points);
        if (!OpenSpaceRoiUtil::IsPolygonClockwise(polygon_points)) {
            AERROR << "non_drivable_roi should be clockwise!";
            print_curves.PrintToLog();
            return false;
        }
        if (polygon_roi.HasOverlap(ego_box) || polygon_roi.HasOverlap(end_box)) {
            AERROR << "ego box or end box has collision with obs";
            print_curves.PrintToLog();
            return false;
        }
    }
    print_curves.PrintToLog();
    return true;
}  // namespace planning

}  // namespace planning
}  // namespace apollo
