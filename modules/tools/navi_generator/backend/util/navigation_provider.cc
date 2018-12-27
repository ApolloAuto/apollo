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

#include "modules/tools/navi_generator/backend/database/db_operator.h"
#include "modules/tools/navi_generator/backend/util/navi_gen_json_converter.h"
#include "modules/tools/navi_generator/backend/util/navigation_matcher.h"
#include "third_party/json/json.hpp"

namespace apollo {
namespace navi_generator {
namespace util {

using Json = nlohmann::json;

bool NavigationProvider::GetRoutePathAsJson(const Json& expected_route,
                                            Json* const matched_route) {
  MapData map_data;
  NaviGenJsonConverter converter;
  if (!converter.JsonToMapData(expected_route, &map_data)) {
    AERROR << "Failed to convert Json to MapData";
    return false;
  }

  std::uint64_t start_way_id;
  std::uint64_t start_line_num;
  std::uint64_t end_way_id;
  std::uint64_t end_line_num;

  apollo::localization::msf::WGS84Corr found_pos_start;
  apollo::localization::msf::WGS84Corr found_pos_end;
  NavigationMatcher matcher;

  if (!matcher.MatchWayWithPos(map_data.start, &start_way_id, &start_line_num,
                               &found_pos_start)) {
    AERROR << "Failed to find the starting point";
    return false;
  }

  if (!matcher.MatchWayWithPos(map_data.end, &end_way_id, &end_line_num,
                               &found_pos_end)) {
    AERROR << "Failed to find the end point";
    return false;
  }

  std::vector<Way> db_route;
  DBOperator dboperator;
  if (!dboperator.QueryRouteWithStartEndWay(start_way_id, end_way_id,
                                            &db_route)) {
    AERROR << "Failed to find route from the starting point to the end point";
    return false;
  }

  if (!matcher.MatchRoute(expected_route, db_route)) {
    AERROR << "Failed to find the expected route";
    return false;
  }

  NaviGenResponse navi_gen_response;
  navi_gen_response.type = "requestRoute";
  navi_gen_response.result.success = 0;

  NaviGenResData res_data;
  res_data.start.log = found_pos_start.log;
  res_data.start.lat = found_pos_start.lat;
  res_data.end.log = found_pos_end.log;
  res_data.end.lat = found_pos_end.lat;

  // TODO(LK): calc num_planes
  res_data.num_plans = 1;
  navi_gen_response.result.msg =
      "There are " + std::to_string(res_data.num_plans) + " plans.";

  NaviGenRoutePlans route_plans;
  route_plans.route_plan_index = 0;
  NaviGenRoutePlan route_plan;
  route_plan.num_routes = db_route.size();

  for (std::size_t i = 0; i < db_route.size(); ++i) {
    std::vector<NaviData> navi_data_vector;
    NaviGenRoutes routes;
    routes.num_navis = 1;
    routes.route_index = i;
    NaviGenNavis navis;
    Way way;
    if (!dboperator.QueryWayWithWayId(db_route[i].way_id, &way)) {
      AERROR << "Failed to query way with way id";
      return false;
    }
    routes.speed_min = way.speed_min;
    routes.speed_max = way.speed_max;
    if (!dboperator.QueryNaviDataWithWayId(db_route[i].way_id,
                                           &navi_data_vector)) {
      AERROR << "Failed to query navi data with way id";
      return false;
    }

    for (std::size_t j = 0; j < navi_data_vector.size(); ++j) {
      std::vector<unsigned char> data = navi_data_vector[j].data;
      FileOperator fileoperator;
      const std::string filename = "temp.txt";
      fileoperator.Export(filename, data);
      std::vector<apollo::planning::ReferencePoint> lanepoints;
      fileoperator.Import(filename, &lanepoints);

      apollo::localization::msf::WGS84Corr path;
      for (std::size_t k = 0; k < lanepoints.size(); ++k) {
        path.log = lanepoints[k].x();
        path.lat = lanepoints[k].y();
        navis.path.emplace_back(path);
      }
    }

    navis.navi_index = i;
    routes.navis.emplace_back(navis);
    route_plan.routes.emplace_back(routes);
  }
  route_plans.route_plan = route_plan;
  res_data.route_plans.emplace_back(route_plans);
  navi_gen_response.res_data = res_data;

  if (!converter.NaviGenResponseToJson(navi_gen_response, matched_route)) {
    AERROR << "Failed to convert from NaviGenResponse to Json";
    return false;
  }

  return true;
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
