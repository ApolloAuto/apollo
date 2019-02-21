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
 * @brief This file provides the implementation of the class
 * "NavigationProvider".
 */

#include "modules/tools/navi_generator/backend/util/navigation_provider.h"

#include <string>
#include <vector>

#include "google/protobuf/util/json_util.h"
#include "modules/localization/msf/common/util/frame_transform.h"
#include "modules/tools/navi_generator/backend/database/db_operator.h"
#include "modules/tools/navi_generator/backend/util/navigation_matcher.h"
#include "modules/tools/navi_generator/proto/navigation_coordinate.pb.h"
#include "modules/tools/navi_generator/proto/navigation_map_data.pb.h"
#include "modules/tools/navi_generator/proto/navigation_response.pb.h"
#include "third_party/json/json.hpp"

namespace apollo {
namespace navi_generator {
namespace util {

using Json = nlohmann::json;
using apollo::localization::msf::WGS84Corr;
using google::protobuf::util::JsonStringToMessage;
using google::protobuf::util::MessageToJsonString;

namespace {
constexpr double kSinsRadToDeg = 57.295779513;
constexpr int kUtmZoneId = 49;
}  // namespace

bool NavigationProvider::GetRoutePathAsJson(const Json& expected_route,
                                            const bool get_all_lane,
                                            Json* const matched_route) {
  CHECK_NOTNULL(matched_route);
  MapData map_data;
  if (!JsonStringToMessage(expected_route.dump(), &map_data).ok()) {
    AERROR << "Failed to convert Json to MapData";
    GenerateFailureResponse(get_all_lane, matched_route);
    return false;
  }
  std::uint64_t start_way_id;
  std::uint64_t end_way_id;
  std::uint64_t dump;
  NavigationMatcher matcher;
  NaviResponse navi_response;
  navi_response.set_type("requestRoute");
  NaviRoutePlans* res_data = navi_response.mutable_res_data();
  WGS84Corr corr;
  WGS84Corr point;
  point.log = map_data.start().lng();
  point.lat = map_data.start().lat();
  if (!matcher.MatchWayWithPos(point, &start_way_id, &dump, &corr)) {
    AERROR << "Failed to find the starting point";
    GenerateFailureResponse(get_all_lane, matched_route);
    return false;
  }
  NaviWGS84Corr* start_corr = res_data->mutable_start();
  start_corr->set_lng(corr.log);
  start_corr->set_lat(corr.lat);
  point.log = map_data.end().lng();
  point.lat = map_data.end().lat();
  if (!matcher.MatchWayWithPos(point, &end_way_id, &dump, &corr)) {
    AERROR << "Failed to find the end point";
    GenerateFailureResponse(get_all_lane, matched_route);
    return false;
  }
  NaviWGS84Corr* end_corr = res_data->mutable_end();
  end_corr->set_lng(corr.log);
  end_corr->set_lat(corr.lat);

  std::vector<Way> db_route;
  DBOperator dboperator;
  if (!dboperator.QueryRouteWithStartEndWay(start_way_id, end_way_id,
                                            &db_route)) {
    AERROR << "Failed to find route from the starting point to the end point";
    GenerateFailureResponse(get_all_lane, matched_route);
    return false;
  }
  if (!matcher.MatchRoute(map_data, db_route)) {
    AERROR << "Failed to find the expected route";
    GenerateFailureResponse(get_all_lane, matched_route);
    return false;
  }
  res_data->set_num_plans(1);
  NaviRoutePlan* route_plans = res_data->add_route_plans();
  route_plans->set_route_plan_index(0);
  route_plans->set_num_routes(db_route.size());
  Way way;
  for (std::size_t i = 0; i < db_route.size(); ++i) {
    if (!dboperator.QueryWayWithWayId(db_route[i].way_id, &way)) {
      AERROR << "Failed to query way with way id";
      GenerateFailureResponse(get_all_lane, matched_route);
      return false;
    }
    std::vector<NaviData> navi_data_vec;
    if (!dboperator.QueryNaviDataWithWayId(db_route[i].way_id,
                                           &navi_data_vec)) {
      AERROR << "Failed to query navi data with way id";
      GenerateFailureResponse(get_all_lane, matched_route);
      return false;
    }
    if (!get_all_lane && navi_data_vec.size() > 1) {
      navi_data_vec.erase(navi_data_vec.begin() + 1, navi_data_vec.end());
    }
    NaviRoute* routes = route_plans->add_routes();
    routes->set_num_navis(navi_data_vec.size());
    routes->set_route_index(i);
    routes->set_speed_min(way.speed_min);
    routes->set_speed_max(way.speed_max);
    for (std::size_t i = 0; i < navi_data_vec.size(); ++i) {
      NaviPath* navis = routes->add_navis();
      navis->set_navi_index(i);
      const std::vector<unsigned char>& data = navi_data_vec[i].data;
      FileOperator fileoperator;
      const std::string filename = "temp.txt";
      fileoperator.Export(filename, data);
      std::vector<apollo::planning::ReferencePoint> lanepoints;
      fileoperator.Import(filename, &lanepoints);

      for (const auto& point : lanepoints) {
        NaviWGS84Corr* path = navis->add_path();
        WGS84Corr wgs84;
        UtmXYToLatlon(point.x(), point.y(), kUtmZoneId, false, &wgs84);
        path->set_lng(wgs84.log * kSinsRadToDeg);
        path->set_lat(wgs84.lat * kSinsRadToDeg);
      }
    }
  }

  NaviSummary* result = navi_response.mutable_result();
  result->set_success(0);
  result->set_msg("There are " + std::to_string(res_data->num_plans()) +
                  " route plans.");
  std::string json_str;
  if (!MessageToJsonString(navi_response, &json_str).ok()) {
    AERROR << "Failed to convert from NaviGenResponse to Json";
    GenerateFailureResponse(get_all_lane, matched_route);
    return false;
  }
  *matched_route = Json::parse(json_str);
  return true;
}

void NavigationProvider::GenerateFailureResponse(
    const bool get_all_lane, nlohmann::json* const failure_response) {
  NaviResponse navi_response;
  navi_response.set_type("requestRoute");
  NaviSummary* result = navi_response.mutable_result();
  result->set_success(1);
  if (get_all_lane) {
    result->set_msg("No navigation found.");
  } else {
    result->set_msg("No route found.");
  }
  std::string json_str;
  if (!MessageToJsonString(navi_response, &json_str).ok()) {
    AERROR << "Failed to convert from NaviGenResponse to Json";
  } else {
    *failure_response = Json::parse(json_str);
  }
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
