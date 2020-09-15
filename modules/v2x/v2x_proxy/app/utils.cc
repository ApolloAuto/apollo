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
/**
 * @file utils.cc
 * @brief utils for v2x_proxy
 */

#include "modules/v2x/v2x_proxy/app/utils.h"

#include <sstream>

#include "modules/common/math/quaternion.h"

namespace apollo {
namespace v2x {

InternalData::InternalData() {
  remaining_time_ = new int32_t[kBufferSize];
  msg_timestamp_ = new double[kBufferSize];
  CHECK_NOTNULL(remaining_time_);
  CHECK_NOTNULL(msg_timestamp_);
  obu_light_ = std::make_shared<ObuLight>();
  this->reset();
}

InternalData::~InternalData() {
  delete[] remaining_time_;
  delete[] msg_timestamp_;
}

void InternalData::reset() {
  oslight_ = nullptr;
  intersection_id_ = -1;
  change_color_timestamp_ = 0.0;
}

bool InternalData::TrafficLightProc(
    const std::shared_ptr<::apollo::hdmap::HDMap> &hdmap, double distance,
    ::apollo::v2x::RoadTrafficLight *msg) {
  if (nullptr == msg) {
    return false;
  }
  if (!msg->has_gps_x_m() || !msg->has_gps_y_m()) {
    AERROR << "Error::v2x traffic_light ignore, gps point is null";
    return false;
  }
  ::apollo::common::PointENU point;
  point.set_x(msg->gps_x_m());
  point.set_y(msg->gps_y_m());
  if (0 == msg->single_traffic_light_size()) {
    AERROR << "Error::v2x traffic_light ignore, size of single light is 0.";
    return false;
  }
  if (0 == msg->single_traffic_light(0).traffic_light_type_size()) {
    return false;
  }
  ::apollo::hdmap::LaneInfoConstPtr laneinfo;
  double dummy_s = 0, dummy_l = 0;
  if (0 != hdmap->GetNearestLane(point, &laneinfo, &dummy_s, &dummy_l)) {
    return false;
  }
  std::vector<::apollo::hdmap::SignalInfoConstPtr> signals;
  if (0 != hdmap->GetForwardNearestSignalsOnLane(point, distance, &signals)) {
    AERROR << "Error::v2x traffic_light ignore, hdmap get no signals."
           << "traffic light size : " << signals.size() << " "
           << std::setiosflags(std::ios::fixed) << std::setprecision(11)
           << "Point:x=" << point.x() << ",y=" << point.y();
    return false;
  }
  if (signals.empty()) {
    AERROR << "Get traffic light size : " << signals.size() << " "
           << std::setiosflags(std::ios::fixed) << std::setprecision(11)
           << "Point:x=" << point.x() << ",y=" << point.y();
    return false;
  }
  AINFO << "the size of traffic light from HDMap is: " << signals.size();
  auto tl_type = msg->single_traffic_light(0).traffic_light_type(0);
  auto color = msg->single_traffic_light(0).color();
  auto remaining_time = msg->single_traffic_light(0).color_remaining_time_s();
  auto next_color = msg->single_traffic_light(0).next_color();
  auto next_remaining_time =
      msg->single_traffic_light(0).next_remaining_time_s();
  msg->clear_single_traffic_light();
  for (size_t i = 0; i < signals.size(); i++) {
    auto signal_info = signals[i];
    auto single = msg->add_single_traffic_light();
    single->set_id(signal_info->id().id());
    single->add_traffic_light_type(tl_type);
    single->set_color(color);
    single->set_color_remaining_time_s(remaining_time);
    single->set_next_color(next_color);
    single->set_next_remaining_time_s(next_remaining_time);
  }
  return true;
}

bool IsRushHour() {
  std::time_t local = std::time(nullptr);
  std::tm now = {};
  localtime_r(&local, &now);
  return now.tm_hour > 16;
}

bool InternalData::ProcTrafficlight(
    const std::shared_ptr<::apollo::hdmap::HDMap> &hdmap,
    const ObuLight *x2v_traffic_light, const std::string &junction_id,
    bool flag_u_turn, double distance, double check_time,
    std::shared_ptr<OSLight> *os_light) {
  if (nullptr == os_light) {
    return false;
  }
  if (nullptr == *os_light) {
    return false;
  }
  if (!x2v_traffic_light) {
    if (junction_id == kUnknownJunctionId) {
      return false;
    }
    return true;
  }
  if (junction_id != x2v_traffic_light->hdmap_junction_id()) {
    AWARN << "current junction id " << junction_id
          << ", received the junction id "
          << x2v_traffic_light->hdmap_junction_id();
    return false;
  }
  std::shared_ptr<OSLight> os_traffic_light = nullptr;
  if (!ProtoAdapter::LightObu2Sys(*x2v_traffic_light, &os_traffic_light)) {
    return false;
  }
  int num_os_traffic_light = os_traffic_light->road_traffic_light_size();
  if (0 == num_os_traffic_light) {
    AERROR << "Ignored no traffic light contained after conventor.";
    return false;
  }
  std::shared_ptr<OSLight> sim_traffic_light_data = nullptr;
  // enter the new intersection if the sim message is not null, clear
  auto cur_junction_id = x2v_traffic_light->intersection_id();
  auto tmp_os_traffic_light = std::make_shared<OSLight>();
  tmp_os_traffic_light->CopyFrom(*os_traffic_light);
  // clear road traffic light
  tmp_os_traffic_light->clear_road_traffic_light();
  for (int i = 0; i < num_os_traffic_light; i++) {
    auto os_current_light = os_traffic_light->mutable_road_traffic_light(i);
    if (!TrafficLightProc(hdmap, distance, os_current_light)) {
      AERROR << "Traffic light proc failed";
      continue;
    }
    if (os_current_light->single_traffic_light_size() > 0) {
      auto tmp_os_current_light =
          tmp_os_traffic_light->add_road_traffic_light();
      tmp_os_current_light->CopyFrom(*(os_current_light));
    }
  }
  tmp_os_traffic_light->set_confidence(IsRushHour() ? 0.5 : 1.0);
  AINFO << "all traffic light send to os BEFORE is: "
        << os_traffic_light->DebugString();
  if (0 == tmp_os_traffic_light->road_traffic_light_size()) {
    return false;
  }
  cur_junction_id = x2v_traffic_light->intersection_id();
  tmp_os_traffic_light->set_intersection_id(cur_junction_id);
  // enter a new junction, need to clear the list
  if (cur_junction_id != intersection_id_) {
    AINFO << "Enter New Juncion: " << cur_junction_id;
    oslight_ = nullptr;
    intersection_id_ = cur_junction_id;
    int num_traffic_light = tmp_os_traffic_light->road_traffic_light_size();
    for (int i = 0; i < num_traffic_light; i++) {
      auto remaining_time = tmp_os_traffic_light->road_traffic_light(i)
                                .single_traffic_light(0)
                                .color_remaining_time_s();
      remaining_time_[i] = remaining_time;
      msg_timestamp_[i] = x2v_traffic_light->header().timestamp_sec();
    }
  } else {
    ADEBUG << "Same Juncion: " << cur_junction_id;
    if (flag_u_turn) {
      for (unsigned int i = 0; i < kBufferSize; i++) {
        msg_timestamp_[i] = 0.0;
        remaining_time_[i] = -1;
      }
      oslight_ = nullptr;
    }
    int num_traffic_light = tmp_os_traffic_light->road_traffic_light_size();
    for (int i = 0; i < num_traffic_light; i++) {
      auto remaining_time = tmp_os_traffic_light->road_traffic_light(i)
                                .single_traffic_light(0)
                                .color_remaining_time_s();
      if ((remaining_time_[i] != remaining_time)) {
        remaining_time_[i] = remaining_time;
        msg_timestamp_[i] = x2v_traffic_light->header().timestamp_sec();
      }
    }
    if (!!oslight_) {
      int road_valid_size =
          std::min(oslight_->road_traffic_light_size(),
                   tmp_os_traffic_light->road_traffic_light_size());
      for (int i = 0; i < road_valid_size; i++) {
        const auto &last_msg_road = oslight_->road_traffic_light(i);
        auto current_msg_road =
            tmp_os_traffic_light->mutable_road_traffic_light(i);
        int single_valid_size =
            std::min(last_msg_road.single_traffic_light_size(),
                     current_msg_road->single_traffic_light_size());
        for (int j = 0; j < single_valid_size; j++) {
          const auto &last_msg_single_traffic_light =
              last_msg_road.single_traffic_light(j);
          auto current_msg_single_traffic_light =
              current_msg_road->mutable_single_traffic_light(j);
          if (last_msg_single_traffic_light.color() ==
              current_msg_single_traffic_light->color()) {
            if (current_msg_single_traffic_light->color_remaining_time_s() >
                last_msg_single_traffic_light.color_remaining_time_s()) {
              AINFO << "correct the remaining time";
              current_msg_single_traffic_light->set_color_remaining_time_s(
                  last_msg_single_traffic_light.color_remaining_time_s());
            }
          }
        }
      }
    }
  }
  oslight_ = std::make_shared<::apollo::v2x::IntersectionTrafficLightData>();
  oslight_->CopyFrom(*tmp_os_traffic_light);
  (*os_light)->CopyFrom(*tmp_os_traffic_light);
  return true;
}

bool InternalData::ProcPlanningMessage(
    const ::apollo::planning::ADCTrajectory *planning_msg,
    const OSLight *last_os_light,
    std::shared_ptr<::apollo::perception::TrafficLightDetection> *res_light) {
  if (!planning_msg || !res_light || !(*res_light)) {
    return false;
  }
  // Keep this blank header for protect other module against coredump.
  (*res_light)->mutable_header();
  if (!planning_msg->has_debug() ||
      !planning_msg->debug().has_planning_data()) {
    return false;
  }
  const auto &planning_debug = planning_msg->debug().planning_data();
  if (!planning_debug.has_signal_light() ||
      0 == planning_debug.signal_light().signal_size()) {
    return false;
  }
  const std::string light_id =
      planning_debug.signal_light().signal(0).light_id();
  if (!last_os_light || light_id.empty()) {
    return true;  // output traffic light without v2x;
  }
  ::apollo::common::Direction attr = ::apollo::common::Direction::EAST;
  bool found = false;
  for (int idx = 0; idx < last_os_light->road_traffic_light_size(); idx++) {
    const auto &road_tl = last_os_light->road_traffic_light(idx);
    if (0 == road_tl.single_traffic_light_size()) {
      continue;
    }
    if (road_tl.single_traffic_light(0).id() == light_id) {
      attr = road_tl.road_attribute();
      found = true;
      break;
    }
  }
  if (!found) {
    AWARN << "Failed to find light_id from os_light: " << light_id
          << " , Ignored";
    return true;  // output traffic light without v2x;
  }
  (*res_light)->clear_traffic_light();
  auto res_frame_id = std::to_string(last_os_light->has_intersection_id()
                                         ? last_os_light->intersection_id()
                                         : -1);
  (*res_light)->mutable_header()->set_frame_id(res_frame_id);
  AINFO << "Selected road attr: " << ::apollo::common::Direction_Name(attr);
  std::set<std::string> idset;
  for (int idx = 0; idx < last_os_light->road_traffic_light_size(); idx++) {
    const auto &road_tl = last_os_light->road_traffic_light(idx);
    if (0 == road_tl.single_traffic_light_size()) {
      continue;
    }
    if (road_tl.road_attribute() != attr) {
      continue;
    }
    const auto &single_tl = road_tl.single_traffic_light(0);
    if (single_tl.traffic_light_type_size() < 1) {
      continue;
    }
    auto *light1 = (*res_light)->add_traffic_light();
    // SET ID
    // light1->set_id(single_tl.id());
    light1->set_id(SingleTrafficLight_Type_Name(  //
        single_tl.traffic_light_type(0)));
    idset.emplace(single_tl.id());

    if (single_tl.has_color()) {
      switch (single_tl.color()) {
        case OSLightColor::SingleTrafficLight_Color_RED:
          light1->set_color(
              ::apollo::perception::TrafficLight_Color::TrafficLight_Color_RED);
          break;
        case OSLightColor::SingleTrafficLight_Color_YELLOW:
          light1->set_color(::apollo::perception::TrafficLight_Color::
                                TrafficLight_Color_YELLOW);
          break;
        case OSLightColor::SingleTrafficLight_Color_GREEN:
        case OSLightColor::SingleTrafficLight_Color_FLASH_GREEN:
          light1->set_color(::apollo::perception::TrafficLight_Color::
                                TrafficLight_Color_GREEN);
          break;
        case OSLightColor::SingleTrafficLight_Color_BLACK:
          light1->set_color(::apollo::perception::TrafficLight_Color::
                                TrafficLight_Color_BLACK);
          break;
        default:
          light1->set_color(::apollo::perception::TrafficLight_Color::
                                TrafficLight_Color_UNKNOWN);
          break;
      }
    }
    // REMAINING_TIME
    if (single_tl.has_color_remaining_time_s()) {
      light1->set_remaining_time(single_tl.color_remaining_time_s());
    }
  }
  return true;
}
namespace utils {
bool FindAllRoadId(const std::shared_ptr<::apollo::hdmap::HDMap> &hdmap,
                   const ::apollo::hdmap::LaneInfoConstPtr &start_laneinfo,
                   const ::apollo::hdmap::LaneInfoConstPtr &end_laneinfo,
                   size_t max_road_count,
                   std::unordered_set<std::string> *result_id_set) {
  if (!result_id_set) {
    return false;
  }
  size_t road_counter = 0;
  ::apollo::hdmap::Id id;
  result_id_set->clear();
  result_id_set->insert(start_laneinfo->road_id().id());
  ::apollo::hdmap::LaneInfoConstPtr start_laneinfo_tmp = start_laneinfo;
  while (true) {
    if (0 == start_laneinfo_tmp->lane().successor_id_size()) {
      AINFO << "The lane has no successor";
      return false;
    }
    id = start_laneinfo_tmp->lane().successor_id(0);
    AINFO << "Lane id " << id.id();
    start_laneinfo_tmp = hdmap->GetLaneById(id);
    if (start_laneinfo_tmp == nullptr) {
      AINFO << "Get lane by id is null.";
      return false;
    }
    result_id_set->insert(start_laneinfo_tmp->road_id().id());
    if (start_laneinfo_tmp->road_id().id() == end_laneinfo->road_id().id()) {
      std::stringstream ss;
      ss << "find all the road id: ";
      for (const auto &item : *result_id_set) {
        ss << item << " ";
      }
      AINFO << ss.str();
      return true;
    }
    road_counter++;
    if (road_counter > max_road_count) {
      AINFO << "not find the end road id after try " << road_counter;
      return false;
    }
  }
}

bool CheckCarInSet(const std::shared_ptr<::apollo::hdmap::HDMap> &hdmap,
                   const std::unordered_set<std::string> &id_set,
                   const ::apollo::hdmap::LaneInfoConstPtr &car_laneinfo,
                   size_t max_lane_count) {
  size_t lane_counter = 0;
  ::apollo::hdmap::Id id;
  auto car_laneinfo_tmp = car_laneinfo;
  while (true) {
    if (id_set.count(car_laneinfo_tmp->road_id().id()) == 1) {
      AINFO << "find the car is in the speed limit region";
      return true;
    }
    if (car_laneinfo_tmp->lane().successor_id_size() == 0) {
      AWARN << "The lane of the card no successor";
      return false;
    }
    id = car_laneinfo_tmp->lane().successor_id(0);
    AINFO << "Lane id " << id.id();
    car_laneinfo_tmp = hdmap->GetLaneById(id);
    if (car_laneinfo_tmp == nullptr) {
      AWARN << "Get lane by id is null.";
      return false;
    } else {
      if (++lane_counter > max_lane_count) {
        AWARN << "not find the road in after try to " << lane_counter;
        return false;
      }
    }
  }
}

bool GetRsuInfo(const std::shared_ptr<::apollo::hdmap::HDMap> &hdmap,
                const OSLocation &os_location,
                const std::set<std::string> &rsu_whitelist, double distance,
                double max_heading_difference,
                std::shared_ptr<::apollo::v2x::CarStatus> *v2x_car_status,
                std::string *out_junction_id, double *out_heading) {
  if (!v2x_car_status) {
    return false;
  }
  if (!out_junction_id) {
    return false;
  }
  if (!out_heading) {
    return false;
  }
  *out_junction_id = kUnknownJunctionId;
  auto res = std::make_shared<::apollo::v2x::CarStatus>();
  if (nullptr == res) {
    return false;
  }
  if (!os_location.has_header()) {
    return false;
  }
  if (!os_location.has_pose()) {
    return false;
  }
  res->mutable_localization()->CopyFrom(os_location);
  ::apollo::common::PointENU point;
  point.set_x(os_location.pose().position().x());
  point.set_y(os_location.pose().position().y());
  double heading = ::apollo::common::math::QuaternionToHeading(
      os_location.pose().orientation().qw(),
      os_location.pose().orientation().qx(),
      os_location.pose().orientation().qy(),
      os_location.pose().orientation().qz());
  *out_heading = heading;
  std::vector<::apollo::hdmap::RSUInfoConstPtr> rsus;
  if (0 != hdmap->GetForwardNearestRSUs(point, distance, heading,
                                        max_heading_difference, &rsus) ||
      rsus.empty()) {
    AINFO << "no rsu is found";
    return false;
  }
  AINFO << "Get " << rsus.size() << " rsu(s)";
  if (rsu_whitelist.find(rsus[0]->id().id() + "_tl") == rsu_whitelist.cend()) {
    AINFO << "This rsu id '" << rsus[0]->id().id()
          << "' is not in the white list";
    return false;
  }
  AINFO << "This RSU is in the white list";
  AINFO << "Junction id " << rsus[0]->rsu().junction_id().id();
  auto junction_info = hdmap->GetJunctionById(rsus[0]->rsu().junction_id());
  if (nullptr == junction_info) {
    return false;
  }
  std::shared_ptr<::apollo::v2x::ObuJunction> obu_junction = nullptr;
  if (!ProtoAdapter::JunctionHd2obu(junction_info, &obu_junction)) {
    return false;
  }
  res->mutable_junction()->CopyFrom(*obu_junction);
  *v2x_car_status = res;
  *out_junction_id = junction_info->id().id();
  return true;
}

OSLightColor GetNextColor(OSLightColor color) {
  switch (color) {
    case OSLightColor::SingleTrafficLight_Color_GREEN:
      return OSLightColor::SingleTrafficLight_Color_YELLOW;
    case OSLightColor::SingleTrafficLight_Color_YELLOW:
      return OSLightColor::SingleTrafficLight_Color_RED;
    case OSLightColor::SingleTrafficLight_Color_RED:
      return OSLightColor::SingleTrafficLight_Color_GREEN;
    default:
      return color;
  }
}

void UniqueOslight(OSLight *os_light) {
  if (nullptr == os_light) {
    return;
  }
  auto tmp_os_light = std::make_shared<OSLight>();
  tmp_os_light->CopyFrom(*os_light);
  os_light->clear_road_traffic_light();
  std::set<std::string> idset;
  for (int idx_tl = 0; idx_tl < tmp_os_light->road_traffic_light_size();
       idx_tl++) {
    const auto &tmp_road_tl = tmp_os_light->road_traffic_light(idx_tl);
    for (int idx_single = 0;
         idx_single < tmp_road_tl.single_traffic_light_size(); idx_single++) {
      const auto &single = tmp_road_tl.single_traffic_light(idx_single);
      if (0 == single.traffic_light_type_size()) {
        continue;
      }
      std::string tmpid =
          single.id() +
          std::to_string(static_cast<int>(single.traffic_light_type(0)));
      // Has ID
      if (idset.find(tmpid) != idset.cend()) {
        continue;
      }
      idset.insert(tmpid);
      // UNIQUE ID
      auto res_tl = os_light->add_road_traffic_light();
      res_tl->set_gps_x_m(tmp_road_tl.gps_x_m());
      res_tl->set_gps_y_m(tmp_road_tl.gps_y_m());
      res_tl->set_road_attribute(tmp_road_tl.road_attribute());
      auto res_single = res_tl->add_single_traffic_light();
      res_single->CopyFrom(single);
    }
  }
}
}  // namespace utils
}  // namespace v2x
}  // namespace apollo
