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
 * "NavigationMatcher".
 */

#include "modules/tools/navi_generator/backend/util/navigation_matcher.h"

#include <algorithm>
#include <limits>
#include <vector>

#include "modules/common/log.h"
#include "modules/tools/navi_generator/backend/common/navi_generator_gflags.h"
#include "modules/tools/navi_generator/backend/database/db_operator.h"
#include "modules/tools/navi_generator/backend/util/navi_gen_json_converter.h"
#include "modules/tools/navi_generator/backend/util/quad_tiles_maker.h"

namespace apollo {
namespace navi_generator {
namespace util {

using Json = nlohmann::json;

namespace {
constexpr double kDefaultAltitude = -1.0;
constexpr double kEquatorialCircumference = 40000000.0;
constexpr std::uint64_t kExpanLineNum = 40;
constexpr double kMaxDist = 15.0;
constexpr double kMinDist = 0.5;
constexpr double kEpsilon = 0.001;
// kSinsRadToDeg = 180 / pi
constexpr double kSinsRadToDeg = 57.295779513;
}  // namespace

bool NavigationMatcher::MatchWayWithPos(
    const apollo::localization::msf::WGS84Corr position,
    std::uint64_t* const way_id, std::uint64_t* const line_num,
    apollo::localization::msf::WGS84Corr* const found_pos) {
  CHECK_NOTNULL(way_id);
  CHECK_NOTNULL(line_num);
  CHECK_NOTNULL(found_pos);

  QuadTilesMaker quad_tiles_maker;

  QuadTile quad_tile;
  if (!quad_tiles_maker.MakeQuadTile(position.lat, position.log,
                                     kDefaultAltitude,
                                     FLAGS_quadtile_zoom_level, &quad_tile)) {
    return false;
  }

  std::string node_value;
  if (!quad_tiles_maker.IdAsString(FLAGS_quadtile_zoom_level, &quad_tile,
                                   &node_value)) {
    return false;
  }

  DBOperator db_operator;
  std::vector<NaviInfoWithPos> navi_info;
  if (!db_operator.QueryNaviWithPos(node_value, &navi_info)) {
    AERROR << "Failed to query Navi with position";
    return false;
  }

  double distance = 0.0;
  if (!FindMinDist(navi_info, navi_info[0].node.data_line_number, position,
                   way_id, line_num, &distance, found_pos)) {
    AERROR << "Failed to find min distance";
    return false;
  }

  if (distance - kMaxDist > kEpsilon) {
    AERROR << "The min distance exceeded the threshold";
    return false;
  }

  return true;
}

bool NavigationMatcher::FindMinDist(
    const std::vector<NaviInfoWithPos>& navi_info,
    const std::uint64_t data_line_number,
    const apollo::localization::msf::WGS84Corr position,
    std::uint64_t* const way_id, std::uint64_t* const line_num,
    double* const min_dist,
    apollo::localization::msf::WGS84Corr* const found_pos) {
  CHECK_NOTNULL(way_id);
  CHECK_NOTNULL(line_num);
  CHECK_NOTNULL(min_dist);
  CHECK_NOTNULL(found_pos);

  std::vector<double> min_dist_vector;
  std::vector<std::uint64_t> min_linenum_vector;
  std::vector<double> min_x_vector;
  std::vector<double> min_y_vector;
  std::vector<int> min_index_vector;
  std::vector<std::uint64_t> min_way_id_vector;

  for (std::size_t navi_info_num = 0; navi_info_num < navi_info.size();
       ++navi_info_num) {
    std::uint64_t inside_line_number =
        navi_info[navi_info_num].node.data_line_number;
    std::vector<double> dist_vector;
    std::vector<std::uint64_t> linenum_vector;
    std::vector<double> x_vector;
    std::vector<double> y_vector;
    std::vector<std::uint64_t> way_id_vector;

    for (std::size_t navi_data_num = 0;
         navi_data_num < navi_info[navi_info_num].navi_data.size();
         ++navi_data_num) {
      NaviData navi_data = navi_info[navi_info_num].navi_data[navi_data_num];
      std::uint64_t linenum = 0;
      double dist_min = std::numeric_limits<double>::max();
      auto data = navi_data.data;

      FileOperator fileoperator;
      const std::string filename = "temp.txt";
      fileoperator.Export(filename, data);
      std::vector<apollo::planning::ReferencePoint> lanepoints;
      fileoperator.Import(filename, &lanepoints);

      int low_iter = (inside_line_number > kExpanLineNum)
                         ? inside_line_number - kExpanLineNum
                         : 0;
      int high_iter = (lanepoints.size() > inside_line_number + kExpanLineNum)
                          ? inside_line_number + kExpanLineNum
                          : lanepoints.size();
      double min_x, min_y;

      for (int i = low_iter; i < high_iter; ++i) {
        // lanepoints:UTM,change WGS84 to UTM
        apollo::localization::msf::UTMCoor xy;
        double Log = position.log / kSinsRadToDeg;
        double Lat = position.lat / kSinsRadToDeg;
        LatlonToUtmXY(Log, Lat, &xy);
        double dist = std::sqrt(std::pow(xy.x - lanepoints[i].x(), 2) +
                                std::pow(xy.y - lanepoints[i].y(), 2));
        if (dist < dist_min) {
          dist_min = dist;
          linenum = i;
          min_x = lanepoints[i].x();
          min_y = lanepoints[i].y();
        }
      }
      dist_vector.emplace_back(dist_min);
      linenum_vector.emplace_back(linenum);
      x_vector.emplace_back(min_x);
      y_vector.emplace_back(min_y);
      way_id_vector.emplace_back(navi_info[navi_info_num].way_id);
    }

    if (dist_vector.size() > 0) {
      auto distance =
          std::min_element(std::begin(dist_vector), std::end(dist_vector));
      int index = std::distance(std::begin(dist_vector), distance);
      min_dist_vector.emplace_back(*distance);
      min_linenum_vector.emplace_back(linenum_vector[index]);
      min_x_vector.emplace_back(x_vector[index]);
      min_y_vector.emplace_back(y_vector[index]);
      min_index_vector.emplace_back(index);
      min_way_id_vector.emplace_back(way_id_vector[index]);
    } else {
      AERROR << "There is no dist_vector";
      continue;
    }
  }
  if (min_dist_vector.size() > 0) {
    auto min_distance = std::min_element(std::begin(min_dist_vector),
                                         std::end(min_dist_vector));
    int index = std::distance(std::begin(min_dist_vector), min_distance);
    *line_num = min_linenum_vector[index];
    *way_id = min_way_id_vector[index];
    found_pos->log = min_x_vector[index];
    found_pos->lat = min_y_vector[index];
  } else {
    AERROR << "There is no min_dist_vector";
    return false;
  }

  return true;
}

bool NavigationMatcher::MatchPoint(
    const double x, const double y,
    const std::vector<apollo::planning::ReferencePoint>& lanepoints) {
  double dist_min = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < lanepoints.size(); ++i) {
    apollo::localization::msf::UTMCoor xy;
    apollo::localization::msf::UTMCoor yx;
    double lng = x / kSinsRadToDeg;
    double lat = y / kSinsRadToDeg;
    LatlonToUtmXY(lng, lat, &xy);
    double dist = std::sqrt(std::pow(xy.x - lanepoints[i].x(), 2) +
                            std::pow(xy.y - lanepoints[i].y(), 2));
    if (dist < dist_min) {
      dist_min = dist;
      if (dist_min < kMinDist) {
        return true;
      }
    }
  }

  if (dist_min > kMaxDist) {
    AERROR << "Failed to match point. min distance is " << dist_min;
    return false;
  }

  return true;
}

bool NavigationMatcher::MatchStep(
    const double x, const double y,
    const std::vector<std::uint64_t> way_id_vector) {
  DBOperator dboperator;
  for (std::size_t i = 0; i < way_id_vector.size(); ++i) {
    std::vector<NaviData> navi_data_vector;
    dboperator.QueryNaviDataWithWayId(way_id_vector[i], &navi_data_vector);
    for (std::size_t j = 0; j < navi_data_vector.size(); ++j) {
      std::vector<unsigned char> data = navi_data_vector[j].data;
      FileOperator fileoperator;
      const std::string filename = "temp.txt";
      fileoperator.Export(filename, data);
      std::vector<apollo::planning::ReferencePoint> lanepoints;
      fileoperator.Import(filename, &lanepoints);
      if (!MatchPoint(x, y, lanepoints)) {
        return false;
      }
    }
  }
  return true;
}

bool NavigationMatcher::MatchRoute(const Json& expected_route,
                                   const std::vector<Way>& route) {
  MapData map_data;
  NaviGenJsonConverter converter;
  if (!converter.JsonToMapData(expected_route, &map_data)) {
    return false;
  }
  int baidu_num_plans = map_data.route_plans.size();

  std::vector<std::uint64_t> way_id_vector;
  for (std::size_t i = 0; i < route.size(); ++i) {
    way_id_vector.emplace_back(route[i].way_id);
  }

  for (int i = 0; i < baidu_num_plans; ++i) {
    std::vector<MapRoutes> baidu_routes =
        map_data.route_plans[i].route_plan.routes;
    for (std::size_t j = 0; j < baidu_routes.size(); ++j) {
      MapStep first_step = baidu_routes[j].route.step[0];
      double step_log = first_step.step.log;
      double step_lat = first_step.step.lat;
      if (!MatchStep(step_log, step_lat, way_id_vector)) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
