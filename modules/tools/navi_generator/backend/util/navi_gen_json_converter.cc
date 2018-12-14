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
 * "NaviGenJsonConverter".
 */

#include "modules/tools/navi_generator/backend/util/navi_gen_json_converter.h"

#include "modules/common/log.h"

namespace apollo {
namespace navi_generator {
namespace util {

using Json = nlohmann::json;

bool NaviGenJsonConverter::NaviGenResponseToJson(
    const NaviGenResponse& navi_gen_response, Json* const js) {
  CHECK_NOTNULL(js);
  (*js)["type"] = navi_gen_response.type;
  (*js)["result"]["success"] = navi_gen_response.result.success;
  (*js)["result"]["msg"] = navi_gen_response.result.msg;
  (*js)["res_data"]["start"]["Lat"] = navi_gen_response.res_data.start.lat;
  (*js)["res_data"]["start"]["Lng"] = navi_gen_response.res_data.start.log;
  (*js)["res_data"]["end"]["Lat"] = navi_gen_response.res_data.end.lat;
  (*js)["res_data"]["end"]["Lng"] = navi_gen_response.res_data.end.log;
  (*js)["res_data"]["numPlans"] = navi_gen_response.res_data.num_plans;

  for (std::size_t i = 0; i < navi_gen_response.res_data.route_plans.size();
       ++i) {
    if (!NaviGenRoutePlansToJson(navi_gen_response.res_data.route_plans[i],
                                 &((*js)["res_data"]["routePlans"][i]))) {
      AERROR << "NaviGenResponseToJson failed";
    }
  }
  return true;
}

bool NaviGenJsonConverter::NaviGenRoutePlansToJson(
    const NaviGenRoutePlans& route_plans, Json* const js) {
  CHECK_NOTNULL(js);
  (*js)["routePlanIndex"] = route_plans.route_plan_index;
  (*js)["routePlan"]["numRoutes"] = route_plans.route_plan.num_routes;

  for (std::size_t i = 0; i < route_plans.route_plan.routes.size(); ++i) {
    if (!NaviGenRoutesToJson(route_plans.route_plan.routes[i],
                             &((*js)["routePlan"]["routes"][i]))) {
      AERROR << "NaviGenRoutePlansToJson failed";
    }
  }
  return true;
}

bool NaviGenJsonConverter::NaviGenRoutesToJson(const NaviGenRoutes& routes,
                                               Json* const js) {
  CHECK_NOTNULL(js);
  (*js)["routeIndex"] = routes.route_index;
  (*js)["speed_min"] = routes.speed_min;
  (*js)["speed_max"] = routes.speed_max;
  (*js)["numNavis"] = 1;
  for (std::size_t i = 0; i < routes.navis.size(); ++i) {
    if (!NaviGenNavisToJson(routes.navis[i], &((*js)["navis"][i]))) {
      AERROR << "NaviGenRoutesToJson failed";
    }
  }
  return true;
}

bool NaviGenJsonConverter::NaviGenNavisToJson(const NaviGenNavis& navis,
                                              Json* const js) {
  CHECK_NOTNULL(js);
  (*js)["naviIndex"] = navis.navi_index;
  for (std::size_t i = 0; i < navis.path.size(); ++i) {
    (*js)["navis"]["path"][i]["Lng"] = navis.path[i].log;
    (*js)["navis"]["path"][i]["Lat"] = navis.path[i].lat;
  }
  return true;
}

bool NaviGenJsonConverter::JsonToNaviGenRoutePlans(
    const Json& js, NaviGenRoutePlans* const route_plans) {
  CHECK_NOTNULL(route_plans);
  route_plans->route_plan_index = js["routePlanIndex"];
  route_plans->route_plan.num_routes = js["routePlan"]["numRoutes"];
  for (std::size_t i = 0; i < js["routePlan"]["routes"].size(); ++i) {
    NaviGenRoutes routes;
    if (!JsonToNaviGenRoutes(js["routePlan"]["routes"][i], &routes)) {
      AERROR << "JsonToNaviGenRoutePlans failed";
      return false;
    }
    route_plans->route_plan.routes.emplace_back(routes);
  }
  return true;
}

bool NaviGenJsonConverter::JsonToNaviGenRoutes(const Json& js,
                                               NaviGenRoutes* const routes) {
  CHECK_NOTNULL(routes);
  routes->route_index = js["routeIndex"];
  routes->speed_min = js["speed_min"];
  routes->speed_max = js["speed_max"];
  routes->num_navis = 1;
  for (std::size_t i = 0; i < js["navis"].size(); ++i) {
    NaviGenNavis navis;
    if (!JsonToNaviGenNavis(js["navis"][i], &navis)) {
      AERROR << "JsonToNaviGenRoutes failed";
      return false;
    }
    routes->navis.emplace_back(navis);
  }
  return true;
}

bool NaviGenJsonConverter::JsonToNaviGenNavis(const Json& js,
                                              NaviGenNavis* const navis) {
  CHECK_NOTNULL(navis);
  navis->navi_index = js["naviIndex"];
  for (std::size_t i = 0; i < js["navi"]["path"].size(); ++i) {
    apollo::localization::msf::WGS84Corr path;
    path.log = js["navi"]["path"][i]["Lng"];
    path.lat = js["navi"]["path"][i]["Lat"];
    navis->path.emplace_back(path);
  }
  return true;
}

bool NaviGenJsonConverter::JsonToMapData(const Json& js,
                                         MapData* const map_data) {
  CHECK_NOTNULL(map_data);
  map_data->type = js["type"];
  map_data->start.log = js["start"]["Lng"];
  map_data->start.lat = js["start"]["Lat"];
  map_data->end.log = js["end"]["Lng"];
  map_data->end.lat = js["end"]["Lat"];

  for (std::size_t i = 0; i < js["waypoint"].size(); ++i) {
    apollo::localization::msf::WGS84Corr waypoint;
    if (!JsonToMapWayPoint(js["waypoint"][i], &waypoint)) {
      AERROR << "JsonToMapWayPoint failed.";
      return false;
    }
    map_data->waypoint.emplace_back(waypoint);
  }
  map_data->num_plans = js["numPlans"];
  for (std::size_t i = 0; i < js["routePlans"].size(); ++i) {
    MapRoutePlans route_plans;
    if (!JsonToMapRoutePlans(js["routePlans"][i], &route_plans)) {
      AERROR << "JsonToMapRoutePlans failed";
      return false;
    }
    map_data->route_plans.emplace_back(route_plans);
  }
  return true;
}

bool NaviGenJsonConverter::JsonToMapWayPoint(
    const Json& WayPoint,
    apollo::localization::msf::WGS84Corr* const waypoint) {
  AINFO << "JsonToMapWayPoint start";
  CHECK_NOTNULL(waypoint);
  waypoint->log = WayPoint["Lng"];
  waypoint->lat = WayPoint["Lat"];
  return true;
}

bool NaviGenJsonConverter::JsonToMapRoutePlans(
    const Json& RoutePlans, MapRoutePlans* const route_plans) {
  CHECK_NOTNULL(route_plans);
  route_plans->route_plan_index = RoutePlans["routePlanIndex"];
  for (std::size_t i = 0; i < RoutePlans["routePlan"].size(); ++i) {
    MapRoutePlan route_plan;
    if (!JsonToMapRoutePlan(RoutePlans["routePlan"], &route_plan)) {
      AERROR << "JsonToMapRoutePlan failed";
      return false;
    }
    route_plans->route_plan = route_plan;
  }
  return true;
}

bool NaviGenJsonConverter::JsonToMapRoutePlan(const Json& RoutePlan,
                                              MapRoutePlan* const route_plan) {
  CHECK_NOTNULL(route_plan);
  route_plan->num_routes = RoutePlan["numRoutes"];
  for (std::size_t i = 0; i < RoutePlan["routes"].size(); ++i) {
    MapRoutes routes;
    if (!JsonToMapRoutes(RoutePlan["routes"][i], &routes)) {
      AERROR << "JsonToMapRoutes failed";
      return false;
    }
    route_plan->routes.emplace_back(routes);
  }
  return true;
}

bool NaviGenJsonConverter::JsonToMapRoutes(const Json& Routes,
                                           MapRoutes* const routes) {
  CHECK_NOTNULL(routes);
  routes->route_index = Routes["routeIndex"];
  AINFO << "routes->route_index = " << routes->route_index;
  for (std::size_t i = 0; i < Routes["route"].size(); ++i) {
    MapRoute route;
    if (!JsonToMapRoute(Routes["route"], &route)) {
      AERROR << "JsonToMapRoute failed";
      return false;
    }
    routes->route = route;
  }
  return true;
}

bool NaviGenJsonConverter::JsonToMapRoute(const Json& Route,
                                          MapRoute* const route) {
  CHECK_NOTNULL(route);
  route->num_steps = Route["numSteps"];
  for (std::size_t i = 0; i < Route["step"].size(); ++i) {
    MapStep step;
    if (!JsonToMapStep(Route["step"][i], &step)) {
      AERROR << "JsonToMapStep failed";
      return false;
    }
    route->step.emplace_back(step);
  }
  for (std::size_t i = 0; i < Route["path"].size(); ++i) {
    apollo::localization::msf::WGS84Corr path;
    if (!JsonToMapPath(Route["path"][i], &path)) {
      AERROR << "JsonToMapPath failed";
      return false;
    }
    route->path.emplace_back(path);
  }
  return true;
}

bool NaviGenJsonConverter::JsonToMapPath(
    const Json& Path, apollo::localization::msf::WGS84Corr* const path) {
  CHECK_NOTNULL(path);
  path->log = Path["Lng"];
  path->lat = Path["Lat"];
  return true;
}

bool NaviGenJsonConverter::JsonToMapStep(const Json& Step,
                                         MapStep* const step) {
  CHECK_NOTNULL(step);
  step->step_index = Step["stepIndex"];
  step->step.log = Step["Lng"];
  step->step.lat = Step["Lat"];
  return true;
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
