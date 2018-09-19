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
    : vehicle_param_(vehicle_param), open_space_conf_(open_space_conf) {
  max_kappa_ = std::tan(open_space_conf.max_steering()) /
               vehicle_param_.front_edge_to_center();
};

bool ReedShepp::ShortestRSP(const std::shared_ptr<Node3d> start_node;
                            const std::shared_ptr<Node3d> end_node,
                            ReedSheppPath* const optimal_path) {
  std::vector<ReedSheppPath> all_possible_paths;
  if (!GenerateRSPs(start_node, end_node, &all_possible_paths)) {
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
  *optimal_path = all_possible_paths.at(optimal_path_index);
  return true;
}

bool ReedShepp::GenerateRSPs(const std::shared_ptr<Node3d> start_node;
                             const std::shared_ptr<Node3d> end_node,
                             std::vector<ReedSheppPath>* all_possible_paths) {
  if (!GenerateRSP(start_node, end_node, all_possible_paths)) {
    AINFO << "Fail to generate general profile of different RSPs";
    return false;
  }
  if (!GenerateLocalConfigurations(all_possible_paths)) {
    AINFO << "Fail to generate local configurations(x, y, phi) in RSP";
    return false;
  }
}

bool ReedShepp::GenerateRSP(const std::shared_ptr<Node3d> start_node,
                            const std::shared_ptr<Node3d> end_node,
                            std::vector<ReedSheppPath>* all_possible_paths) {
  double dx = end_node->GetX() - start_node->GetX();
  double dy = end_node->GetY() - start_node->GetY();
  double dphi = end_node->GetPhi() - start_node->GetPhi();
  double c = std::cos(start_node->GetPhi());
  double s = std::sin(start_node->GetPhi());
  // normalize the initial point to (0,0,0)
  double x = (c * dx + s * dy) * max_kappa_;
  double y = (-s * dx + c * dy) * max_kappa_;
  if (!SCS(x, y, dphi, all_possible_paths)) {
    AINFO << "Fail at SCS";
    return false;
  }
  if (!CSC(x, y, dphi, all_possible_paths)) {
    AINFO << "Fail at CSC";
    return false;
  }
  if (!CCC(x, y, dphi, all_possible_paths)) {
    AINFO << "Fail at CCC";
    return false;
  }
  if (!CCCC(x, y, dphi, all_possible_paths)) {
    AINFO << "Fail at CCCC";
    return false;
  }
  if (!CCSC(x, y, dphi, all_possible_paths)) {
    AINFO << "Fail at CCSC";
    return false;
  }
  if (!CCSCC(x, y, dphi, all_possible_paths)) {
    AINFO << "Fail at CCSCC";
    return false;
  }
  if (all_possible_paths->size() == 0) {
    return false;
  }
  return true;
}

bool ReedShepp::SCS(double x, double y, double phi,
                    std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam SLS_param;
  SLS(x, y, phi, &SLS_param);
  if (SLS_param.flag && !SetRSP(SLS_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with SLS_param";
    return false;
  }
  RSPParam SRS_param;
  SLS(x, -y, -phi, &SRS_param);
  if (SRS_param.flag && !SetRSP(SRS_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with SRS_param";
    return false;
  }
  return true;
}

bool ReedShepp::CSC(double x, double y, double phi,
                    std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LSL1_param;
  LSL(x, y, phi, &LSL1_param);
  if (LSL1_param.flag && !SetRSP(LSL1_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with LSL_param";
    return false;
  }

  RSPParam LSL2_param;
  LSL(-x, y, -phi, &LSL2_param);
  LSL2_param.t = -LSL2_param.t;
  LSL2_param.u = -LSL2_param.u;
  LSL2_param.v = -LSL2_param.v;
  if (LSL2_param.flag && !SetRSP(LSL2_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with LSL2_param";
    return false;
  }

  RSPParam LSL3_param;
  LSL(x, -y, -phi, &LSL3_param);
  if (LSL3_param.flag && !SetRSP(LSL3_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with LSL3_param";
    return false;
  }

  RSPParam LSL4_param;
  LSL(-x, -y, phi, &LSL4_param);
  LSL4_param.t = -LSL4_param.t;
  LSL4_param.u = -LSL4_param.u;
  LSL4_param.v = -LSL4_param.v;
  if (LSL4_param.flag && !SetRSP(LSL4_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with LSL4_param";
    return false;
  }

  RSPParam LSR1_param;
  LSR(x, y, phi, &LSR1_param);
  if (LSR1_param.flag && !SetRSP(LSR1_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with LSR1_param";
    return false;
  }

  RSPParam LSR2_param;
  LSR(-x, y, -phi, &LSR2_param);
  LSR2_param.t = -LSR2_param.t;
  LSR2_param.u = -LSR2_param.u;
  LSR2_param.v = -LSR2_param.v;
  if (LSR2_param.flag && !SetRSP(LSR2_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with LSR2_param";
    return false;
  }

  RSPParam LSR3_param;
  LSR(x, -y, -phi, &LSR3_param);
  if (LSR3_param.flag && !SetRSP(LSR3_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with LSR3_param";
    return false;
  }

  RSPParam LSR4_param;
  LSR(-x, -y, phi, &LSR4_param);
  LSR4_param.t = -LSR4_param.t;
  LSR4_param.u = -LSR4_param.u;
  LSR4_param.v = -LSR4_param.v;
  if (LSR4_param.flag && !SetRSP(LSR4_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with LSR4_param";
    return false;
  }
}

bool ReedShepp::CCC(double x, double y, double phi,
           std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LRL1_param;
  LRL(x, y, phi, &LRL1_param);
  if (LRL1_param.flag && !SetRSP(LRL1_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with LRL_param";
    return false;
  }

  RSPParam LRL2_param;
  LRL(-x, y, -phi, &LRL2_param);
  LRL2_param.t = -LRL2_param.t;
  LRL2_param.u = -LRL2_param.u;
  LRL2_param.v = -LRL2_param.v;
  if (LRL2_param.flag && !SetRSP(LRL2_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with LRL2_param";
    return false;
  }

  RSPParam LRL3_param;
  LRL(x, -y, -phi, &LRL3_param);
  if (LRL3_param.flag && !SetRSP(LRL3_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with LRL3_param";
    return false;
  }

  RSPParam LRL4_param;
  LRL(-x, -y, phi, &LRL4_param);
  LRL4_param.t = -LRL4_param.t;
  LRL4_param.u = -LRL4_param.u;
  LRL4_param.v = -LRL4_param.v;
  if (LRL4_param.flag && !SetRSP(LRL4_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with LRL4_param";
    return false;
  }

  // backward
  xb = x*std::cos(phi) + y*std::sin(phi);
  yb = x*std::sin(phi) - y*std::cos(phi);

  RSPParam LRL5_param;
  LRL5_param.backward = true;
  LRL(xb, yb, phi, &LRL5_param);
  if (LRL5_param.flag && !SetRSP(LRL5_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with LRL5_param";
    return false;
  }

  RSPParam LRL6_param;
  LRL6_param.backward = true;
  LRL(-xb, yb, -phi, &LRL6_param);
  LRL6_param.t = -LRL6_param.t;
  LRL6_param.u = -LRL6_param.u;
  LRL6_param.v = -LRL6_param.v;
  if (LRL6_param.flag && !SetRSP(LRL6_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with LRL6_param";
    return false;
  }

  RSPParam LRL7_param;
  LRL7_param.backward = true;
  LRL(xb, -yb, -phi, &LRL7_param);
  if (LRL7_param.flag && !SetRSP(LRL7_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with LRL7_param";
    return false;
  }

  RSPParam LRL8_param;
  LRL8_param.backward = true;
  LRL(-xb, -yb, phi, &LRL8_param);
  LRL8_param.t = -LRL8_param.t;
  LRL8_param.u = -LRL8_param.u;
  LRL8_param.v = -LRL8_param.v;
  if (LRL8_param.flag && !SetRSP(LRL8_param, all_possible_paths)) {
    AINFO << "Fail at SetRSP with LRL8_param";
    return false;
  }
}


}  // namespace planning
}  // namespace apollo