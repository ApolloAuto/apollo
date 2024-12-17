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
 * @file lane_way_tool.cc
 */

#include "modules/external_command/command_processor/command_processor_base/util/lane_way_tool.h"

#include <string>
#include <vector>

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/external_command/command_processor/command_processor_base/util/message_reader.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace external_command {

LaneWayTool::LaneWayTool(const std::shared_ptr<cyber::Node> &node)
    : hdmap_(hdmap::HDMapUtil::BaseMapPtr()),
      message_reader_(MessageReader::Instance()) {
  message_reader_->RegisterMessage<apollo::localization::LocalizationEstimate>(
      FLAGS_localization_topic);
}

bool LaneWayTool::ConvertToLaneWayPoint(
    const Pose &pose, apollo::routing::LaneWaypoint *lane_way_point) const {
  hdmap::LaneInfoConstPtr nearest_lane;
  double nearest_s;
  double nearest_l;
  common::PointENU point;
  point.set_x(pose.x());
  point.set_y(pose.y());
  if (pose.has_heading()) {
    static constexpr double kSearchRadius = 3.0;
    static constexpr double kMaxHeadingDiff = 1.0;
    // Get the lane nearest to the pose with heading check and update the lane
    // info in LaneWayPoint.
    if (nullptr == hdmap_ ||
        hdmap_->GetNearestLaneWithHeading(point, kSearchRadius, pose.heading(),
                                          kMaxHeadingDiff, &nearest_lane,
                                          &nearest_s, &nearest_l) < 0) {
      AERROR << "Failed to get nearest lane with heading of pose "
             << pose.DebugString();
      return false;
    }
  } else {
    // Get the lane nearest to the pose without heading check and update the
    // lane info in LaneWayPoint.
    if (nullptr == hdmap_ ||
        hdmap_->GetNearestLane(point, &nearest_lane, &nearest_s, &nearest_l) <
            0) {
      AERROR << "Failed to get nearest lane of " << pose.DebugString();
      return false;
    }
  }
  // Check the lane type.
  if (nearest_lane->lane().type() != hdmap::Lane::CITY_DRIVING) {
    AERROR << pose.DebugString() << " Lane type is not correct "
           << apollo::hdmap::Lane::LaneType_Name(nearest_lane->lane().type());
    return false;
  }
  // Update the LaneWayPoint info.
  lane_way_point->set_id(nearest_lane->id().id());
  lane_way_point->set_s(nearest_s);
  auto *lane_way_pose = lane_way_point->mutable_pose();
  lane_way_pose->set_x(pose.x());
  lane_way_pose->set_y(pose.y());
  return true;
}

bool LaneWayTool::GetVehicleLaneWayPoint(
    apollo::routing::LaneWaypoint *lane_way_point) const {
  CHECK_NOTNULL(lane_way_point);
  // Get the current localization pose
  auto *localization =
      message_reader_->GetMessage<apollo::localization::LocalizationEstimate>(
          FLAGS_localization_topic);
  if (nullptr == localization) {
    AERROR << "Cannot get vehicle location!";
    return false;
  }
  external_command::Pose pose;
  pose.set_x(localization->pose().position().x());
  pose.set_y(localization->pose().position().y());
  pose.set_heading(localization->pose().heading());
  return ConvertToLaneWayPoint(pose, lane_way_point);
}

bool LaneWayTool::GetVehicleLaneWayPoints(
    std::vector<apollo::routing::LaneWaypoint> *lane_way_points) const {
  CHECK_NOTNULL(lane_way_points);
  // Get the current localization pose
  auto *localization =
      message_reader_->GetMessage<apollo::localization::LocalizationEstimate>(
          FLAGS_localization_topic);
  if (nullptr == localization) {
    AERROR << "Cannot get vehicle location!";
    return false;
  }
  apollo::common::PointENU center_enu;
  center_enu.set_x(localization->pose().position().x());
  center_enu.set_y(localization->pose().position().y());
  common::math::Vec2d center_point(center_enu.x(), center_enu.y());
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  static constexpr double kNearbyLaneDistance = 20.0;
  if (hdmap_->GetLanes(center_enu, kNearbyLaneDistance, &lanes) < 0) {
    AERROR << "get lanes failed" << center_enu.DebugString();
    return false;
  }
  for (const auto lane : lanes) {
    double s, l;
    if (!lane->GetProjection(center_point, &s, &l)) {
      AERROR << "get projection failed";
      return false;
    }
    if (s < 0.0 || s > lane->total_length()) {
      continue;
    }
    lane_way_points->emplace_back();
    auto &lane_way_point = lane_way_points->back();
    lane_way_point.mutable_pose()->set_x(center_point.x());
    lane_way_point.mutable_pose()->set_y(center_point.y());
    lane_way_point.set_id(lane->id().id());
    lane_way_point.set_s(s);
    AINFO << "GET" << lane_way_point.DebugString();
  }
  return true;
}

bool apollo::external_command::LaneWayTool::GetParkingLaneWayPoint(
    const std::string &parking_id,
    std::vector<apollo::routing::LaneWaypoint> *lane_way_points) const {
  hdmap::Id id;
  id.set_id(parking_id);
  auto parking_space_info = hdmap_->GetParkingSpaceById(id);
  auto points = parking_space_info->polygon().points();
  common::math::Vec2d center_point(0, 0);
  for (size_t i = 0; i < points.size(); i++) {
    center_point += points[i];
  }
  center_point /= 4.0;
  apollo::common::PointENU center_enu;
  center_enu.set_x(center_point.x());
  center_enu.set_y(center_point.y());
  for (const auto &parking_space_overlap_id :
       parking_space_info->parking_space().overlap_id()) {
    hdmap::OverlapInfoConstPtr overlap_info_ptr =
        hdmap_->GetOverlapById(parking_space_overlap_id);
    if (nullptr == overlap_info_ptr) {
      AERROR << "Cannot get overlap info!";
      continue;
    }
    for (const auto &object : overlap_info_ptr->overlap().object()) {
      hdmap::LaneInfoConstPtr lane_ptr = hdmap_->GetLaneById(object.id());
      if (nullptr == lane_ptr) {
        AERROR << "Cannot get lane info!";
        continue;
      }
      double s, l;
      if (!lane_ptr->GetProjection(center_point, &s, &l)) {
        AERROR << "get projection failed";
        continue;
      }
      lane_way_points->emplace_back();
      auto &lane_way_point = lane_way_points->back();
      lane_way_point.mutable_pose()->set_x(center_point.x());
      lane_way_point.mutable_pose()->set_y(center_point.y());
      lane_way_point.set_id(lane_ptr->id().id());
      lane_way_point.set_s(s);
      AINFO << "GET" << lane_way_point.DebugString();
    }
  }
  if (lane_way_points->empty()) {
    AERROR << "Cannot get lane way point!";
    return false;
  }
  return true;
}

bool apollo::external_command::LaneWayTool::GetPreciseParkingLaneWayPoint(
    hdmap::AreaInfoConstPtr &area_info,
    std::vector<apollo::routing::LaneWaypoint> *lane_way_points) const {
  if (nullptr == area_info) {
    AERROR << "Cannot get junction info!";
    return false;
  }

  for (const auto &junction_overlap_id : area_info->area().overlap_id()) {
    hdmap::OverlapInfoConstPtr overlap_info_ptr =
        hdmap_->GetOverlapById(junction_overlap_id);
    if (nullptr == overlap_info_ptr) {
      AERROR << "Cannot get overlap info!";
      continue;
    }

    for (const auto &object : overlap_info_ptr->overlap().object()) {
      hdmap::LaneInfoConstPtr lane_ptr = hdmap_->GetLaneById(object.id());
      if (nullptr == lane_ptr) {
        AERROR << "Cannot get lane info!";
        continue;
      }
      double traget_s = object.lane_overlap_info().end_s();
      apollo::common::PointENU traget_waypoint =
          lane_ptr->GetSmoothPoint(traget_s);
      lane_way_points->emplace_back();
      auto &lane_way_point = lane_way_points->back();
      lane_way_point.mutable_pose()->set_x(traget_waypoint.x());
      lane_way_point.mutable_pose()->set_y(traget_waypoint.y());
      lane_way_point.set_id(lane_ptr->id().id());
      lane_way_point.set_s(traget_s);
      AINFO << "GET" << lane_way_point.DebugString();
    }
  }
  if (lane_way_points->empty()) {
    AERROR << "Cannot get lane way point!";
    return false;
  }
  return true;
}

bool apollo::external_command::LaneWayTool::IsParkandgoScenario() const {
  auto *localization =
      message_reader_->GetMessage<apollo::localization::LocalizationEstimate>(
          FLAGS_localization_topic);
  if (nullptr == localization) {
    AERROR << "Cannot get vehicle location!";
    return false;
  }
  common::PointENU adc;
  adc.set_x(localization->pose().position().x());
  adc.set_y(localization->pose().position().y());
  external_command::Pose pose;
  double heading = localization->pose().heading();
  hdmap::LaneInfoConstPtr lane;
  double s = 0.0;
  double l = 0.0;
  if (hdmap_->GetNearestLaneWithHeading(adc, 2.0, heading, M_PI / 3.0, &lane,
                                        &s, &l) != 0 ||
      lane->lane().type() != hdmap::Lane::CITY_DRIVING) {
    return true;
  }
  return false;
}

}  // namespace external_command
}  // namespace apollo
