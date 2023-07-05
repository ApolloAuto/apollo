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

#include "modules/v2x/v2x_proxy/proto_adapter/proto_adapter.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "cyber/cyber.h"

namespace apollo {
namespace v2x {

OSLightype ProtoAdapter::LightTypeObu2Sys(int32_t type) {
  switch (type) {
    case 2:
      return OSLightype::SingleTrafficLight_Type_LEFT;
    case 3:
      return OSLightype::SingleTrafficLight_Type_RIGHT;
    case 4:
      return OSLightype::SingleTrafficLight_Type_U_TURN;
    case 1:
    default:
      return OSLightype::SingleTrafficLight_Type_STRAIGHT;
  }
}

bool ProtoAdapter::LightObu2Sys(const ObuLight &obu_light,
                                std::shared_ptr<OSLight> *os_light) {
  if (nullptr == os_light) {
    return false;
  }
  auto res = std::make_shared<OSLight>();
  if (nullptr == res) {
    return false;
  }
  if (!obu_light.has_header()) {
    return false;
  }
  if (obu_light.header().has_timestamp_sec()) {
    res->mutable_header()->set_timestamp_sec(
        obu_light.header().timestamp_sec());
  }
  if (obu_light.header().has_module_name()) {
    res->mutable_header()->set_module_name(obu_light.header().module_name());
  }
  if (0 == obu_light.road_traffic_light_size()) {
    return false;
  }
  bool flag_has_data = false;
  for (int idx_road = 0; idx_road < obu_light.road_traffic_light_size();
       idx_road++) {
    const auto &obu_road_light1 = obu_light.road_traffic_light(idx_road);
    // Set the road index for lane
    apollo::common::Direction tl_attr = apollo::common::Direction::EAST;
    switch (obu_road_light1.road_direction()) {
      case 1:
        tl_attr = apollo::common::Direction::EAST;
        break;
      case 2:
        tl_attr = apollo::common::Direction::WEST;
        break;
      case 3:
        tl_attr = apollo::common::Direction::SOUTH;
        break;
      case 4:
        tl_attr = apollo::common::Direction::NORTH;
        break;
      default:
        AINFO << "Road direction=" << obu_road_light1.road_direction()
              << " is invalid.";
    }
    // FOR-EACH LANE
    for (int idx_lane = 0; idx_lane < obu_road_light1.lane_traffic_light_size();
         idx_lane++) {
      const auto &obu_lane_light1 =
          obu_road_light1.lane_traffic_light(idx_lane);
      if (!obu_lane_light1.has_gps_x_m() || !obu_lane_light1.has_gps_y_m()) {
        AWARN << "Invalid lane_traffic_light: [" << idx_road << "][" << idx_lane
              << "]: no gps info here.";
        continue;
      }
      flag_has_data = true;
      auto *res_light1 = res->add_road_traffic_light();
      res_light1->set_gps_x_m(obu_lane_light1.gps_x_m());
      res_light1->set_gps_y_m(obu_lane_light1.gps_y_m());
      res_light1->set_road_attribute(tl_attr);
      // single traffic light
      for (int j = 0; j < obu_lane_light1.single_traffic_light_size(); j++) {
        auto *res_single1 = res_light1->add_single_traffic_light();
        const auto &obu_single1 = obu_lane_light1.single_traffic_light(j);
        if (obu_single1.has_id()) {
          res_single1->set_id(obu_single1.id());
        }
        if (obu_single1.has_color_remaining_time_s()) {
          res_single1->set_color_remaining_time_s(
              obu_single1.color_remaining_time_s());
        }
        if (obu_single1.has_next_remaining_time()) {
          res_single1->set_next_remaining_time_s(
              obu_single1.next_remaining_time());
        }
        if (obu_single1.has_right_turn_light()) {
          res_single1->set_right_turn_light(obu_single1.right_turn_light());
        }
        if (obu_single1.has_color()) {
          res_single1->set_color(obu_single1.color());
        }
        if (obu_single1.has_next_color()) {
          res_single1->set_next_color(obu_single1.next_color());
        }
        if (obu_single1.has_traffic_light_type()) {
          res_single1->add_traffic_light_type(
              LightTypeObu2Sys(obu_single1.traffic_light_type()));
        }
      }
    }
  }
  *os_light = res;
  return flag_has_data;
}

bool ProtoAdapter::RsiObu2Sys(const ObuRsi *obu_rsi,
                              std::shared_ptr<OSRsi> *os_rsi) {
  if (nullptr == obu_rsi) {
    return false;
  }
  if (nullptr == os_rsi) {
    return false;
  }
  auto res = std::make_shared<OSRsi>();
  if (nullptr == res) {
    return false;
  }
  if (!obu_rsi->has_alter_type()) {
    return false;
  }
  res->set_radius(obu_rsi->radius());
  res->set_rsi_type(obu_rsi->alter_type());
  if (obu_rsi->has_header()) {
    auto header = res->mutable_header();
    header->set_sequence_num(obu_rsi->header().sequence_num());
    header->set_timestamp_sec(obu_rsi->header().timestamp_sec());
    header->set_module_name("v2x");
  }
  switch (obu_rsi->alter_type()) {
    case RsiAlterType::SPEED_LIMIT:
    case RsiAlterType::SPEED_LIMIT_BRIDGE:
    case RsiAlterType::SPEED_LIMIT_TUNNEL:
      for (int index = 0; index < obu_rsi->points_size(); index++) {
        auto point = res->add_points();
        point->set_x(obu_rsi->points(index).x());
        point->set_y(obu_rsi->points(index).y());
        res->set_speed(std::atof(obu_rsi->description().c_str()));
      }
      break;
    case RsiAlterType::CONSTRUCTION_AHEAD:  // Construction Ahead
    case RsiAlterType::BUS_LANE:            // Bus Lane
    case RsiAlterType::TIDAL_LANE:          // tidal Lane
    case RsiAlterType::TRAFFIC_JAM:         // traffic jam
    case RsiAlterType::TRAFFIC_ACCIDENT:    // traffic accident
      for (int index = 0; index < obu_rsi->points_size(); index++) {
        auto point = res->add_points();
        point->set_x(obu_rsi->points(index).x());
        point->set_y(obu_rsi->points(index).y());
      }
      break;
    case RsiAlterType::NO_HONKING:          // No Honking
    case RsiAlterType::SLOW_DOWN_SECTION:   // slow down section
    case RsiAlterType::ACCIDENT_PRONE:      // Accident Prone
    case RsiAlterType::OVERSPEED_VEHICLE:   // overspeed vehicle
    case RsiAlterType::EMERGENCY_BRAKING:   // emergency braking
    case RsiAlterType::ANTIDROMIC_VEHICLE:  // antidromic vehicle
    case RsiAlterType::ZOMBIES_VEHICLE: {   // zombies vehicle
      auto point = res->add_points();
      point->set_x(obu_rsi->points(0).x());
      point->set_y(obu_rsi->points(0).y());
      break;
    }
    case RsiAlterType::CONTROLLOSS_VEHICLE:  // controlloss vehicle
    case RsiAlterType::SPECIAL_VEHICLE: {    // special vehicle
      auto point = res->add_points();
      point->set_x(obu_rsi->points(0).x());
      point->set_y(obu_rsi->points(0).y());
      res->set_speed(std::atof(obu_rsi->description().c_str()));
      break;
    }
    default:
      AINFO << "RSI type:" << obu_rsi->alter_type();
      break;
  }
  *os_rsi = res;
  return true;
}

bool ProtoAdapter::JunctionHd2obu(const HDJunction &hd_junction,
                                  std::shared_ptr<ObuJunction> *obu_junction) {
  if (nullptr == obu_junction) {
    return false;
  }
  auto res = std::make_shared<ObuJunction>();
  if (nullptr == res) {
    return false;
  }
  res->mutable_id()->set_id(hd_junction->id().id());
  for (const auto &point : hd_junction->polygon().points()) {
    auto res_point = res->mutable_polygon()->add_point();
    res_point->set_x(point.x());
    res_point->set_y(point.y());
    res_point->set_z(0);
  }
  *obu_junction = res;
  return true;
}

}  // namespace v2x
}  // namespace apollo
