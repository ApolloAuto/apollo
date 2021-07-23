/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/task_manager/parking_routing_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/task_manager/common/task_manager_gflags.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace task_manager {

using apollo::common::math::Vec2d;
using apollo::common::PointENU;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::routing::ParkingSpaceType;

ParkingRoutingManager::ParkingRoutingManager()
: monitor_logger_buffer_(apollo::common::monitor::
                         MonitorMessageItem::TASK_MANAGER) {}

common::Status ParkingRoutingManager::Init(
        const ParkingRoutingTask& parking_routing_task) {
    // get the message form routing
    has_space_ = parking_routing_task.routing_request().has_parking_info();
    has_space_id_ = parking_routing_task.routing_request().
                    parking_info().has_parking_space_id();
    id_ = parking_routing_task.routing_request().
                    parking_info().parking_space_id();
    return common::Status::OK();
}

bool ParkingRoutingManager::SizeVerification(
        const ParkingRoutingTask& parking_routing_task) {
  auto plot_type = parking_routing_task.routing_request()
                       .parking_info()
                       .parking_space_type();
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_length = vehicle_config.vehicle_param().length();
  double ego_width = vehicle_config.vehicle_param().width();
  if (!has_space_ || !has_space_id_) {
    AERROR << "No Valid park plot exits!";
    return false;
    }
    auto corner_point =
        parking_routing_task.routing_request().parking_info().corner_point();
    PointENU left_bottom_point = corner_point.point().at(0);
    PointENU right_bottom_point = corner_point.point().at(1);
    PointENU right_top_point = corner_point.point().at(2);;
    double length = sqrt((right_bottom_point.x() - right_top_point.x()) *
                         (right_bottom_point.x() - right_top_point.x()) +
                         (right_bottom_point.y() - right_top_point.y()) *
                         (right_bottom_point.y() - right_top_point.y()));
    double width = sqrt((right_bottom_point.x() - left_bottom_point.x()) *
                        (right_bottom_point.x() - left_bottom_point.x()) +
                        (right_bottom_point.y() - left_bottom_point.y()) *
                        (right_bottom_point.y() - left_bottom_point.y()));
    // judge by spot type
    if (plot_type == ParkingSpaceType::VERTICAL_PLOT) {
        if (length - FLAGS_plot_size_buffer < ego_length ||
            width - FLAGS_plot_size_buffer < ego_width) {
            monitor_logger_buffer_.WARN("veritical plot is not suit!");
            AERROR << "The veritical plot is small";
            return false;
        }
    } else if (plot_type == ParkingSpaceType::PARALLEL_PARKING) {
        if (width - FLAGS_plot_size_buffer < ego_length ||
            length - FLAGS_plot_size_buffer < ego_width) {
            monitor_logger_buffer_.WARN("parallel plot is not suit!");
            AERROR << "The parallel plot is small";
            return false;
        }
    }
    return true;
}

bool ParkingRoutingManager::RoadWidthVerification(
        const ParkingRoutingTask& parking_routing_task) {
    const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
    const double ego_width = vehicle_config.vehicle_param().width();
    const double road_width = parking_routing_task.lane_width();
    if (!has_space_ || !has_space_id_) {
        AERROR << "No Valid park plot exits!";
        return false;
    }
    if (ego_width > road_width + FLAGS_road_width_buffer) {
        AERROR << "the road width is small!";
        return false;
    }
    return true;
}

}  // namespace task_manager
}  // namespace apollo
