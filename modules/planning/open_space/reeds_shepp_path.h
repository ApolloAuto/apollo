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
 * reeds_shepp_path.h
 */

#ifndef MODULES_PLANNING_OPEN_SPACE_REEDS_SHEPP_PATH_H_
#define MODULES_PLANNING_OPEN_SPACE_REEDS_SHEPP_PATH_H_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace planning {

struct ReedSheppPath {
  std::vector<double> segs_lengths;
  double total_length;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> final_phi;
  // true for driving forward and false for driving backward
  std::vector<bool> gear;
};

struct RSPParam {
  bool flag;
  bool backward;
  double t;
  double u;
  double v;
}

class ReedShepp {
 public:
  ReedShepp(const common::VehicleParam& vehicle_param,
            const PlannerOpenSpaceConfig& open_space_conf);
  virtual ~ReedShepp() = default;
  // Pick the shortest path from all possible combination of movement primitives
  // by Reed Shepp
  bool ShortestRSP(const std::shared_ptr<Node3d> start_node;
                   const std::shared_ptr<Node3d> end_node,
                   ReedSheppPath* optimal_path);

 private:
  // Generate all possible combination of movement primitives by Reed Shepp and
  // interpolate them
  bool GenerateRSPs(const std::shared_ptr<Node3d> start_node,
                    const std::shared_ptr<Node3d> end_node,
                    std::vector<ReedSheppPath>* all_possible_paths);
  // Set the general profile of the movement primitives
  bool GenerateRSP(const std::shared_ptr<Node3d> start_node,
                   const std::shared_ptr<Node3d> end_node,
                   std::vector<ReedSheppPath>* all_possible_paths);
  // Set local exact configurations profile of each movement primitive
  bool GenerateLocalConfigurations(
      std::vector<ReedSheppPath>* all_possible_paths);
  // Interpolation usde in GenetateLocalConfiguration
  std::vector<ReedSheppPath> Interpolation();
  // motion primitives combination setup function
  std::vector<ReedSheppPath> SetRSP(RSPParam& param, std::vector<ReedSheppPath>* all_possible_paths);
  // Six different combination of motion primitive in Reed Shepp path used in
  // GenerateRSP()
  bool SCS(double x, double y, double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CSC(double x, double y, double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CCC(double x, double y, double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CCCC(double x, double y, double phi,
            std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSC(double x, double y, double phi,
            std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSCC(double x, double y, double phi,
             std::vector<ReedSheppPath>* all_possible_paths);
  // different options for different combination of motion primitives
  void LSL(double x, double y, double phi, RSPParam* param);
  void LSR(double x, double y, double phi, RSPParam* param);
  void LRL(double x, double y, double phi, RSPParam* param);
  void SLS(double x, double y, double phi, RSPParam* param);
  void LRLRn(double x, double y, double phi, RSPParam* param);
  void LRLRp(double x, double y, double phi, RSPParam* param);
  void LRSR(double x, double y, double phi, RSPParam* param);
  void LRSL(double x, double y, double phi, RSPParam* param);
  void LRSLR(double x, double y, double phi, RSPParam* param);

 private:
  common::VehicleParam vehicle_param_;
  PlannerOpenSpaceConfig open_space_conf_;
  double max_kappa_;
};
}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPEN_SPACE_REEDS_SHEPP_PATH_H_