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

/*
 * reeds_shepp_path.cc
 */

#include "modules/planning/open_space/reeds_shepp_path.h"

namespace apollo {
namespace planning {

ReedShepp::ReedShepp(const common::VehicleParam& vehicle_param,
                     const PlannerOpenSpaceConfig& open_space_conf)
    : vehicle_param_(vehicle_param), open_space_conf_(open_space_conf);

bool ReedShepp::ShortestRSP(const std::shared_ptr<Node3d> start_node;
                            const std::shared_ptr<Node3d> end_node,
                            std::unique_ptr<ReedSheppPath> optimal_path) {
  std::vector<std::unique_ptr<ReedSheppPath>> all_possible_paths;
  if (!GenerateRSPs(start_node, end_node, all_possible_paths)) {
    AINFO << "Fail to generate different combination of Reed Shepp "
             "paths";
    return false;
  }
  double optimal_path_length = std::numeric_limits<double>::infinity();
  int optimal path_index = -1;
  for (std::size_t i = 0; i < all_possible_paths.size(); i++) {
    if (all_possible_paths.at(i)->total_length < optimal_path_length) {
      optimal_path_index = i;
      optimal_path_length = all_possible_paths.at(i)->total_length;
    }
  }
  optimal_path.reset(all_possible_paths.at(optimal_path_index));
  return true;
}

bool ReedShepp::GenerateRSPs(
    std::shared_ptr<Node3d> start_node;
    std::shared_ptr<Node3d> end_node,
    std::vector<std::unique_ptr<ReedSheppPath>> all_possible_paths) {
  if (!GenerateRSP(start_node, end_node, all_possible_paths)) {
    AINFO << "Fail to generate general profile of different RSPs";
    return false;
  }
  if (!GenerateLocalConfigurations(
          std::vector<std::unique_ptr<ReedSheppPath>> all_possible_paths)) {
    AINFO << "Fail to generate local configurations(x, y, phi) in RSP";
    return false;
  }
}

}  // namespace planning
}  // namespace apollo
}  // namespace apollo