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
 * @brief This file provides the declaration of the class
 * "NaviGenJsonConverter".
 */

#ifndef MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_NAVI_GEN_JSON_CONVERTER_H_
#define MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_NAVI_GEN_JSON_CONVERTER_H_

#include <cstdio>
#include <string>
#include <vector>

#include "modules/localization/msf/common/util/frame_transform.h"
#include "third_party/json/json.hpp"

/**
 * @namespace apollo::navi_generator::util
 * @brief apollo::navi_generator::util
 */
namespace apollo {
namespace navi_generator {
namespace util {

struct NaviGenNavis {
  std::size_t navi_index;
  std::vector<apollo::localization::msf::WGS84Corr> path;
};

struct NaviGenRoutes {
  std::size_t route_index;
  double speed_min;
  double speed_max;
  std::size_t num_navis;
  std::vector<NaviGenNavis> navis;
};

struct NaviGenRoutePlan {
  std::size_t num_routes;
  std::vector<NaviGenRoutes> routes;
};

struct NaviGenRoutePlans {
  std::size_t route_plan_index;
  NaviGenRoutePlan route_plan;
};

struct NaviGenResData {
  apollo::localization::msf::WGS84Corr start;
  apollo::localization::msf::WGS84Corr end;
  std::size_t num_plans;
  std::vector<NaviGenRoutePlans> route_plans;
};

struct NaviGenResult {
  std::size_t success;
  std::string msg;
};

struct NaviGenResponse {
  std::string type;
  NaviGenResult result;
  NaviGenResData res_data;
};

struct MapStep {
  std::size_t step_index;
  apollo::localization::msf::WGS84Corr step;
};

struct MapRoute {
  std::size_t num_steps;
  std::vector<MapStep> step;
  std::vector<apollo::localization::msf::WGS84Corr> path;
};

struct MapRoutes {
  std::size_t route_index;
  MapRoute route;
};

struct MapRoutePlan {
  std::size_t num_routes;
  std::vector<MapRoutes> routes;
};

struct MapRoutePlans {
  std::size_t route_plan_index;
  MapRoutePlan route_plan;
};

struct MapData {
  std::string type;
  apollo::localization::msf::WGS84Corr start;
  apollo::localization::msf::WGS84Corr end;
  std::vector<apollo::localization::msf::WGS84Corr> waypoint;
  std::size_t num_plans;
  std::vector<MapRoutePlans> route_plans;
};

class NaviGenJsonConverter {
 public:
  NaviGenJsonConverter() = default;
  ~NaviGenJsonConverter() = default;

 public:
  bool JsonToMapData(const nlohmann::json& js, MapData* const map_data);

  bool NaviGenResponseToJson(const NaviGenResponse& navi_gen_response,
                             nlohmann::json* const js);

 private:
  bool NaviGenNavisToJson(const NaviGenNavis& navis, nlohmann::json* const js);

  bool NaviGenRoutesToJson(const NaviGenRoutes& routes,
                           nlohmann::json* const js);

  bool NaviGenRoutePlansToJson(const NaviGenRoutePlans& route_plans,
                               nlohmann::json* const js);

  bool JsonToNaviGenNavis(const nlohmann::json& js, NaviGenNavis* const navis);

  bool JsonToNaviGenRoutes(const nlohmann::json& js,
                           NaviGenRoutes* const routes);

  bool JsonToNaviGenRoutePlans(const nlohmann::json& js,
                               NaviGenRoutePlans* const route_plans);

  bool JsonToMapWayPoint(const nlohmann::json& WayPoint,
                         apollo::localization::msf::WGS84Corr* const waypoint);

  bool JsonToMapRoutePlans(const nlohmann::json& RoutePlans,
                           MapRoutePlans* const route_plans);

  bool JsonToMapRoutePlan(const nlohmann::json& routePlan,
                          MapRoutePlan* const route_plan);

  bool JsonToMapRoutes(const nlohmann::json& Routes, MapRoutes* const routes);

  bool JsonToMapRoute(const nlohmann::json& Route, MapRoute* const route);

  bool JsonToMapStep(const nlohmann::json& Step, MapStep* const step);

  bool JsonToMapPath(const nlohmann::json& Path,
                     apollo::localization::msf::WGS84Corr* const path);
};

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo

#endif  // MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_NAVI_GEN_JSON_CONVERTER_H_
