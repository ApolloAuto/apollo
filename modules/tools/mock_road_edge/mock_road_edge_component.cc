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
#include "modules/tools/mock_road_edge/mock_road_edge_component.h"

#include <algorithm>
#include <list>
#include <string>
#include <vector>

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/planning_base/common/history.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::JunctionInfo;
using apollo::perception::TrafficLightDetection;
using apollo::relative_map::MapMsg;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using apollo::storytelling::Stories;

bool MockRoadEdgeComponent::Init() {
  chassis_reader_ = node_->CreateReader<apollo::canbus::Chassis>(
      "/apollo/canbus/chassis",
      [this](const std::shared_ptr<apollo::canbus::Chassis>& chassis) {
        // AINFO << "Received chassis data: run chassis callback."
        //       << chassis->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        chassis_.CopyFrom(*chassis);
      });

  localization_reader_ =
      node_->CreateReader<apollo::localization::LocalizationEstimate>(
          "/apollo/localization/pose",
          [this](
              const std::shared_ptr<apollo::localization::LocalizationEstimate>&
                  localization) {
            // AINFO << "Received localization data: run localization callback."
            //       << localization->header().DebugString();
            std::lock_guard<std::mutex> lock(mutex_);
            localization_.CopyFrom(*localization);
          });

  planning_command_reader_ = node_->CreateReader<PlanningCommand>(
      "/apollo/planning/command",
      [this](const std::shared_ptr<PlanningCommand>& planning_command) {
        // AINFO << "Received planning data: run planning callback."
        //       << planning_command->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        planning_command_.CopyFrom(*planning_command);
      });

  mock_road_edge_writer_ =
      node_->CreateWriter<PerceptionEdgeInfo>("/apollo/perception/edge");

  current_pnc_map_ = apollo::cyber::plugin_manager::PluginManager::Instance()
                         ->CreateInstance<planning::PncMapBase>(
                             "apollo::planning::LaneFollowMap");
  return true;
}

bool MockRoadEdgeComponent::Proc() {
  if (!CheckInput()) {
    AERROR << "Check Input Error!";
    return false;
  }

  std::list<hdmap::RouteSegments> segments;
  if (!current_pnc_map_->GetRouteSegments(vehicle_state_, &segments)) {
    AERROR << "GetRouteSegments Error!";
    return false;
  }

  if (!current_pnc_map_->GetNearestPointFromRouting(vehicle_state_,
                                                    &adc_waypoint_)) {
    AERROR << "GetNearestPointFromRouting Error!";
    return false;
  }

  for (auto& seg : segments) {
    if (seg.PreviousAction() == routing::FORWARD &&
        adc_waypoint_.lane->lane().right_neighbor_forward_lane_id().empty() &&
        adc_waypoint_.lane->lane().right_neighbor_reverse_lane_id().empty()) {
      AINFO << "get Forward segments: " << seg.DebugString();
      MockEdgeInfo(&seg);
    } else {
      AWARN << "Can not get right edge";
      return false;
    }
  }

  common::util::FillHeader(node_->Name(), &edge_info_);
  mock_road_edge_writer_->Write(edge_info_);

  AINFO << "MockRoadEdgeComponent process";
  return true;
}

bool MockRoadEdgeComponent::CheckInput() {
  const double start_timestamp = Clock::NowInSeconds();
  Status status = vehicle_state_provider_.Update(localization_, chassis_);

  vehicle_state_ = vehicle_state_provider_.vehicle_state();
  const double vehicle_state_timestamp = vehicle_state_.timestamp();
  DCHECK_GE(start_timestamp, vehicle_state_timestamp)
      << "start_timestamp is behind vehicle_state_timestamp by "
      << start_timestamp - vehicle_state_timestamp << " secs";

  if (!status.ok() || !util::IsVehicleStateValid(vehicle_state_)) {
    const std::string msg =
        "Update VehicleStateProvider failed "
        "or the vehicle state is out dated.";
    AERROR << msg;
    return false;
  }

  if (planning_command_.is_motion_command() &&
      planning_command_.has_lane_follow_command() &&
      util::IsDifferentRouting(last_command_, planning_command_)) {
    last_command_ = planning_command_;
    // AINFO << "new_command:" << last_command_.DebugString();
    if (!current_pnc_map_->UpdatePlanningCommand(last_command_)) {
      AERROR << "Failed to update routing in lane_follow_map: "
             << last_command_.DebugString();
      return false;
    }
  }
  return true;
}

void MockRoadEdgeComponent::MockEdgeInfo(
    hdmap::RouteSegments* truncated_segments) {
  edge_info_.set_is_useable(false);
  edge_info_.clear_edge_point();

  bool find_adc_lane = false;
  double start_s = 0;
  double check_length = 0.0;
  bool get_road_edge = true;

  for (size_t i = 0; i < truncated_segments->size(); ++i) {
    const auto lane_ptr = truncated_segments->at(i).lane;
    if (!find_adc_lane &&
        adc_waypoint_.lane->id().id() == lane_ptr->id().id()) {
      find_adc_lane = true;
      PointENU ego_pt = adc_waypoint_.lane->GetSmoothPoint(adc_waypoint_.s);
      double ego_s = -10;
      double ego_l = -10;
      lane_ptr->GetProjection({ego_pt.x(), ego_pt.y()}, &ego_s, &ego_l);
      start_s = ego_s;
      start_s += 2.151;
    }

    if (find_adc_lane) {
      start_s = std::max(0.0, start_s);
      while (start_s < lane_ptr->total_length() && check_length < 9.0) {
        PointENU center_pt = lane_ptr->GetSmoothPoint(start_s);
        // const auto& turn = lane_ptr->lane().turn();
        // get_road_edge = (turn == hdmap::Lane::NO_TURN) &&
        // (!InJunction(center_pt, 0.5));

        double left_width = 0;
        double right_width = 0;
        if (get_road_edge) {
          lane_ptr->GetRoadWidth(start_s, &left_width, &right_width);
        } else {
          break;
          // lane_ptr->GetWidth(start_s, &left_width, &right_width);
        }

        double heading = lane_ptr->Heading(start_s);
        auto right_edge_point = edge_info_.add_edge_point();
        right_edge_point->set_x(center_pt.x() +
                                std::sin(heading) * right_width);
        right_edge_point->set_y(center_pt.y() -
                                std::cos(heading) * right_width);
        check_length += 0.3;
        start_s += 0.3;
        AINFO << "lae id: " << lane_ptr->id().id() << ", start_s: " << start_s
              << ", right road_width: " << right_width;
      }
      start_s -= lane_ptr->total_length();
    }
    if (!get_road_edge) {
      break;
    }
  }

  if (check_length > 5.0) {
    edge_info_.set_is_useable(true);
  }
}

bool MockRoadEdgeComponent::InJunction(const PointENU& check_point,
                                       const double radius) {
  std::vector<std::shared_ptr<const JunctionInfo>> junctions;
  HDMapUtil::BaseMap().GetJunctions(check_point, radius, &junctions);
  if (junctions.empty()) {
    return false;
  }
  for (const auto junction_info : junctions) {
    if (junction_info == nullptr || !junction_info->junction().has_polygon()) {
      continue;
    }
    std::vector<Vec2d> vertices;
    for (const auto& point : junction_info->junction().polygon().point()) {
      vertices.emplace_back(point.x(), point.y());
    }
    if (vertices.size() < 3) {
      continue;
    }
    Polygon2d junction_polygon{vertices};
    if (junction_polygon.IsPointIn({check_point.x(), check_point.y()})) {
      return true;
    }
  }

  return false;
}

}  // namespace planning
}  // namespace apollo
