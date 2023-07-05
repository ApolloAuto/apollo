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
 * @file
 */

#include "modules/planning/open_space/coarse_trajectory_generator/reeds_shepp_path.h"

namespace apollo {
namespace planning {

ReedShepp::ReedShepp(const common::VehicleParam& vehicle_param,
                     const PlannerOpenSpaceConfig& open_space_conf)
    : vehicle_param_(vehicle_param),
      planner_open_space_config_(open_space_conf) {
  max_kappa_ = std::tan(vehicle_param_.max_steer_angle() *
                        planner_open_space_config_.warm_start_config()
                            .traj_kappa_contraint_ratio() /
                        vehicle_param_.steer_ratio()) /
               vehicle_param_.wheel_base();
  AINFO << "max kappa: " << max_kappa_;
  AINFO_IF(FLAGS_enable_parallel_hybrid_a) << "parallel REEDShepp";
}

std::pair<double, double> ReedShepp::calc_tau_omega(const double u,
                                                    const double v,
                                                    const double xi,
                                                    const double eta,
                                                    const double phi) {
  double delta = common::math::NormalizeAngle(u - v);
  double A = std::sin(u) - std::sin(delta);
  double B = std::cos(u) - std::cos(delta) - 1.0;

  double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
  double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
  double tau = 0.0;
  if (t2 < 0) {
    tau = common::math::NormalizeAngle(t1 + M_PI);
  } else {
    tau = common::math::NormalizeAngle(t1);
  }
  double omega = common::math::NormalizeAngle(tau - u + v - phi);
  return std::make_pair(tau, omega);
}

bool ReedShepp::ShortestRSP(const std::shared_ptr<Node3d> start_node,
                            const std::shared_ptr<Node3d> end_node,
                            std::shared_ptr<ReedSheppPath> optimal_path) {
  std::vector<ReedSheppPath> all_possible_paths;
  if (!GenerateRSPs(start_node, end_node, &all_possible_paths)) {
    ADEBUG << "Fail to generate different combination of Reed Shepp "
              "paths";
    return false;
  }

  double optimal_path_length = std::numeric_limits<double>::infinity();
  size_t optimal_path_index = 0;
  size_t paths_size = all_possible_paths.size();
  for (size_t i = 0; i < paths_size; ++i) {
    if (all_possible_paths.at(i).total_length > 0 &&
        all_possible_paths.at(i).total_length < optimal_path_length) {
      optimal_path_index = i;
      optimal_path_length = all_possible_paths.at(i).total_length;
    }
  }

  if (!GenerateLocalConfigurations(start_node, end_node,
                                   &(all_possible_paths[optimal_path_index]))) {
    ADEBUG << "Fail to generate local configurations(x, y, phi) in SetRSP";
    return false;
  }

  if (std::abs(all_possible_paths[optimal_path_index].x.back() -
               end_node->GetX()) > 1e-3 ||
      std::abs(all_possible_paths[optimal_path_index].y.back() -
               end_node->GetY()) > 1e-3 ||
      std::abs(all_possible_paths[optimal_path_index].phi.back() -
               end_node->GetPhi()) > 1e-3) {
    ADEBUG << "RSP end position not right";
    for (size_t i = 0;
         i < all_possible_paths[optimal_path_index].segs_types.size(); ++i) {
      ADEBUG << "types are "
             << all_possible_paths[optimal_path_index].segs_types[i];
    }
    ADEBUG << "x, y, phi are: "
           << all_possible_paths[optimal_path_index].x.back() << ", "
           << all_possible_paths[optimal_path_index].y.back() << ", "
           << all_possible_paths[optimal_path_index].phi.back();
    ADEBUG << "end x, y, phi are: " << end_node->GetX() << ", "
           << end_node->GetY() << ", " << end_node->GetPhi();
    return false;
  }
  (*optimal_path).x = all_possible_paths[optimal_path_index].x;
  (*optimal_path).y = all_possible_paths[optimal_path_index].y;
  (*optimal_path).phi = all_possible_paths[optimal_path_index].phi;
  (*optimal_path).gear = all_possible_paths[optimal_path_index].gear;
  (*optimal_path).total_length =
      all_possible_paths[optimal_path_index].total_length;
  (*optimal_path).segs_types =
      all_possible_paths[optimal_path_index].segs_types;
  (*optimal_path).segs_lengths =
      all_possible_paths[optimal_path_index].segs_lengths;
  return true;
}

bool ReedShepp::GenerateRSPs(const std::shared_ptr<Node3d> start_node,
                             const std::shared_ptr<Node3d> end_node,
                             std::vector<ReedSheppPath>* all_possible_paths) {
  if (FLAGS_enable_parallel_hybrid_a) {
    // AINFO << "parallel hybrid a*";
    if (!GenerateRSPPar(start_node, end_node, all_possible_paths)) {
      ADEBUG << "Fail to generate general profile of different RSPs";
      return false;
    }
  } else {
    if (!GenerateRSP(start_node, end_node, all_possible_paths)) {
      AERROR << "Fail to generate general profile of different RSPs";
      return false;
    }
  }
  return true;
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
    ADEBUG << "Fail at SCS";
  }
  if (!CSC(x, y, dphi, all_possible_paths)) {
    ADEBUG << "Fail at CSC";
  }
  if (!CCC(x, y, dphi, all_possible_paths)) {
    ADEBUG << "Fail at CCC";
  }
  if (!CCCC(x, y, dphi, all_possible_paths)) {
    ADEBUG << "Fail at CCCC";
  }
  if (!CCSC(x, y, dphi, all_possible_paths)) {
    ADEBUG << "Fail at CCSC";
  }
  if (!CCSCC(x, y, dphi, all_possible_paths)) {
    ADEBUG << "Fail at CCSCC";
  }
  if (all_possible_paths->empty()) {
    ADEBUG << "No path generated by certain two configurations";
    return false;
  }
  return true;
}

bool ReedShepp::SCS(const double x, const double y, const double phi,
                    std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam SLS_param;
  SLS(x, y, phi, &SLS_param);
  double SLS_lengths[3] = {SLS_param.t, SLS_param.u, SLS_param.v};
  char SLS_types[] = "SLS";
  if (SLS_param.flag &&
      !SetRSP(3, SLS_lengths, SLS_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with SLS_param";
    return false;
  }

  RSPParam SRS_param;
  SLS(x, -y, -phi, &SRS_param);
  double SRS_lengths[3] = {SRS_param.t, SRS_param.u, SRS_param.v};
  char SRS_types[] = "SRS";
  if (SRS_param.flag &&
      !SetRSP(3, SRS_lengths, SRS_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with SRS_param";
    return false;
  }
  return true;
}

bool ReedShepp::CSC(const double x, const double y, const double phi,
                    std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LSL1_param;
  LSL(x, y, phi, &LSL1_param);
  double LSL1_lengths[3] = {LSL1_param.t, LSL1_param.u, LSL1_param.v};
  char LSL1_types[] = "LSL";
  if (LSL1_param.flag &&
      !SetRSP(3, LSL1_lengths, LSL1_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LSL_param";
    return false;
  }

  RSPParam LSL2_param;
  LSL(-x, y, -phi, &LSL2_param);
  double LSL2_lengths[3] = {-LSL2_param.t, -LSL2_param.u, -LSL2_param.v};
  char LSL2_types[] = "LSL";
  if (LSL2_param.flag &&
      !SetRSP(3, LSL2_lengths, LSL2_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LSL2_param";
    return false;
  }

  RSPParam LSL3_param;
  LSL(x, -y, -phi, &LSL3_param);
  double LSL3_lengths[3] = {LSL3_param.t, LSL3_param.u, LSL3_param.v};
  char LSL3_types[] = "RSR";
  if (LSL3_param.flag &&
      !SetRSP(3, LSL3_lengths, LSL3_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LSL3_param";
    return false;
  }

  RSPParam LSL4_param;
  LSL(-x, -y, phi, &LSL4_param);
  double LSL4_lengths[3] = {-LSL4_param.t, -LSL4_param.u, -LSL4_param.v};
  char LSL4_types[] = "RSR";
  if (LSL4_param.flag &&
      !SetRSP(3, LSL4_lengths, LSL4_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LSL4_param";
    return false;
  }

  RSPParam LSR1_param;
  LSR(x, y, phi, &LSR1_param);
  double LSR1_lengths[3] = {LSR1_param.t, LSR1_param.u, LSR1_param.v};
  char LSR1_types[] = "LSR";
  if (LSR1_param.flag &&
      !SetRSP(3, LSR1_lengths, LSR1_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LSR1_param";
    return false;
  }

  RSPParam LSR2_param;
  LSR(-x, y, -phi, &LSR2_param);
  double LSR2_lengths[3] = {-LSR2_param.t, -LSR2_param.u, -LSR2_param.v};
  char LSR2_types[] = "LSR";
  if (LSR2_param.flag &&
      !SetRSP(3, LSR2_lengths, LSR2_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LSR2_param";
    return false;
  }

  RSPParam LSR3_param;
  LSR(x, -y, -phi, &LSR3_param);
  double LSR3_lengths[3] = {LSR3_param.t, LSR3_param.u, LSR3_param.v};
  char LSR3_types[] = "RSL";
  if (LSR3_param.flag &&
      !SetRSP(3, LSR3_lengths, LSR3_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LSR3_param";
    return false;
  }

  RSPParam LSR4_param;
  LSR(-x, -y, phi, &LSR4_param);
  double LSR4_lengths[3] = {-LSR4_param.t, -LSR4_param.u, -LSR4_param.v};
  char LSR4_types[] = "RSL";
  if (LSR4_param.flag &&
      !SetRSP(3, LSR4_lengths, LSR4_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LSR4_param";
    return false;
  }
  return true;
}

bool ReedShepp::CCC(const double x, const double y, const double phi,
                    std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LRL1_param;
  LRL(x, y, phi, &LRL1_param);
  double LRL1_lengths[3] = {LRL1_param.t, LRL1_param.u, LRL1_param.v};
  char LRL1_types[] = "LRL";
  if (LRL1_param.flag &&
      !SetRSP(3, LRL1_lengths, LRL1_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRL_param";
    return false;
  }

  RSPParam LRL2_param;
  LRL(-x, y, -phi, &LRL2_param);
  double LRL2_lengths[3] = {-LRL2_param.t, -LRL2_param.u, -LRL2_param.v};
  char LRL2_types[] = "LRL";
  if (LRL2_param.flag &&
      !SetRSP(3, LRL2_lengths, LRL2_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRL2_param";
    return false;
  }

  RSPParam LRL3_param;
  LRL(x, -y, -phi, &LRL3_param);
  double LRL3_lengths[3] = {LRL3_param.t, LRL3_param.u, LRL3_param.v};
  char LRL3_types[] = "RLR";
  if (LRL3_param.flag &&
      !SetRSP(3, LRL3_lengths, LRL3_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRL3_param";
    return false;
  }

  RSPParam LRL4_param;
  LRL(-x, -y, phi, &LRL4_param);
  double LRL4_lengths[3] = {-LRL4_param.t, -LRL4_param.u, -LRL4_param.v};
  char LRL4_types[] = "RLR";
  if (LRL4_param.flag &&
      !SetRSP(3, LRL4_lengths, LRL4_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRL4_param";
    return false;
  }

  // backward
  double xb = x * std::cos(phi) + y * std::sin(phi);
  double yb = x * std::sin(phi) - y * std::cos(phi);

  RSPParam LRL5_param;
  LRL(xb, yb, phi, &LRL5_param);
  double LRL5_lengths[3] = {LRL5_param.v, LRL5_param.u, LRL5_param.t};
  char LRL5_types[] = "LRL";
  if (LRL5_param.flag &&
      !SetRSP(3, LRL5_lengths, LRL5_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRL5_param";
    return false;
  }

  RSPParam LRL6_param;
  LRL(-xb, yb, -phi, &LRL6_param);
  double LRL6_lengths[3] = {-LRL6_param.v, -LRL6_param.u, -LRL6_param.t};
  char LRL6_types[] = "LRL";
  if (LRL6_param.flag &&
      !SetRSP(3, LRL6_lengths, LRL6_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRL6_param";
    return false;
  }

  RSPParam LRL7_param;
  LRL(xb, -yb, -phi, &LRL7_param);
  double LRL7_lengths[3] = {LRL7_param.v, LRL7_param.u, LRL7_param.t};
  char LRL7_types[] = "RLR";
  if (LRL7_param.flag &&
      !SetRSP(3, LRL7_lengths, LRL7_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRL7_param";
    return false;
  }

  RSPParam LRL8_param;
  LRL(-xb, -yb, phi, &LRL8_param);
  double LRL8_lengths[3] = {-LRL8_param.v, -LRL8_param.u, -LRL8_param.t};
  char LRL8_types[] = "RLR";
  if (LRL8_param.flag &&
      !SetRSP(3, LRL8_lengths, LRL8_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRL8_param";
    return false;
  }
  return true;
}

bool ReedShepp::CCCC(const double x, const double y, const double phi,
                     std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LRLRn1_param;
  LRLRn(x, y, phi, &LRLRn1_param);
  double LRLRn1_lengths[4] = {LRLRn1_param.t, LRLRn1_param.u, -LRLRn1_param.u,
                              LRLRn1_param.v};
  char LRLRn1_types[] = "LRLR";
  if (LRLRn1_param.flag &&
      !SetRSP(4, LRLRn1_lengths, LRLRn1_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRLRn_param";
    return false;
  }

  RSPParam LRLRn2_param;
  LRLRn(-x, y, -phi, &LRLRn2_param);
  double LRLRn2_lengths[4] = {-LRLRn2_param.t, -LRLRn2_param.u, LRLRn2_param.u,
                              -LRLRn2_param.v};
  char LRLRn2_types[] = "LRLR";
  if (LRLRn2_param.flag &&
      !SetRSP(4, LRLRn2_lengths, LRLRn2_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRLRn2_param";
    return false;
  }

  RSPParam LRLRn3_param;
  LRLRn(x, -y, -phi, &LRLRn3_param);
  double LRLRn3_lengths[4] = {LRLRn3_param.t, LRLRn3_param.u, -LRLRn3_param.u,
                              LRLRn3_param.v};
  char LRLRn3_types[] = "RLRL";
  if (LRLRn3_param.flag &&
      !SetRSP(4, LRLRn3_lengths, LRLRn3_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRLRn3_param";
    return false;
  }

  RSPParam LRLRn4_param;
  LRLRn(-x, -y, phi, &LRLRn4_param);
  double LRLRn4_lengths[4] = {-LRLRn4_param.t, -LRLRn4_param.u, LRLRn4_param.u,
                              -LRLRn4_param.v};
  char LRLRn4_types[] = "RLRL";
  if (LRLRn4_param.flag &&
      !SetRSP(4, LRLRn4_lengths, LRLRn4_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRLRn4_param";
    return false;
  }

  RSPParam LRLRp1_param;
  LRLRp(x, y, phi, &LRLRp1_param);
  double LRLRp1_lengths[4] = {LRLRp1_param.t, LRLRp1_param.u, LRLRp1_param.u,
                              LRLRp1_param.v};
  char LRLRp1_types[] = "LRLR";
  if (LRLRp1_param.flag &&
      !SetRSP(4, LRLRp1_lengths, LRLRp1_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRLRp1_param";
    return false;
  }

  RSPParam LRLRp2_param;
  LRLRp(-x, y, -phi, &LRLRp2_param);
  double LRLRp2_lengths[4] = {-LRLRp2_param.t, -LRLRp2_param.u, -LRLRp2_param.u,
                              -LRLRp2_param.v};
  char LRLRp2_types[] = "LRLR";
  if (LRLRp2_param.flag &&
      !SetRSP(4, LRLRp2_lengths, LRLRp2_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRLRp2_param";
    return false;
  }

  RSPParam LRLRp3_param;
  LRLRp(x, -y, -phi, &LRLRp3_param);
  double LRLRp3_lengths[4] = {LRLRp3_param.t, LRLRp3_param.u, LRLRp3_param.u,
                              LRLRp3_param.v};
  char LRLRp3_types[] = "RLRL";
  if (LRLRp3_param.flag &&
      !SetRSP(4, LRLRp3_lengths, LRLRp3_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRLRp3_param";
    return false;
  }

  RSPParam LRLRp4_param;
  LRLRp(-x, -y, phi, &LRLRp4_param);
  double LRLRp4_lengths[4] = {-LRLRp4_param.t, -LRLRp4_param.u, -LRLRp4_param.u,
                              -LRLRp4_param.v};
  char LRLRp4_types[] = "RLRL";
  if (LRLRp4_param.flag &&
      !SetRSP(4, LRLRp4_lengths, LRLRp4_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRLRp4_param";
    return false;
  }
  return true;
}

bool ReedShepp::CCSC(const double x, const double y, const double phi,
                     std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LRSL1_param;
  LRSL(x, y, phi, &LRSL1_param);
  double LRSL1_lengths[4] = {LRSL1_param.t, -0.5 * M_PI, LRSL1_param.u,
                             LRSL1_param.v};
  char LRSL1_types[] = "LRSL";
  if (LRSL1_param.flag &&
      !SetRSP(4, LRSL1_lengths, LRSL1_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSL1_param";
    return false;
  }

  RSPParam LRSL2_param;
  LRSL(-x, y, -phi, &LRSL2_param);
  double LRSL2_lengths[4] = {-LRSL2_param.t, 0.5 * M_PI, -LRSL2_param.u,
                             -LRSL2_param.v};
  char LRSL2_types[] = "LRSL";
  if (LRSL2_param.flag &&
      !SetRSP(4, LRSL2_lengths, LRSL2_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSL2_param";
    return false;
  }

  RSPParam LRSL3_param;
  LRSL(x, -y, -phi, &LRSL3_param);
  double LRSL3_lengths[4] = {LRSL3_param.t, -0.5 * M_PI, LRSL3_param.u,
                             LRSL3_param.v};
  char LRSL3_types[] = "RLSR";
  if (LRSL3_param.flag &&
      !SetRSP(4, LRSL3_lengths, LRSL3_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSL3_param";
    return false;
  }

  RSPParam LRSL4_param;
  LRSL(-x, -y, phi, &LRSL4_param);
  double LRSL4_lengths[4] = {-LRSL4_param.t, 0.5 * M_PI, -LRSL4_param.u,
                             -LRSL4_param.v};
  char LRSL4_types[] = "RLSR";
  if (LRSL4_param.flag &&
      !SetRSP(4, LRSL4_lengths, LRSL4_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSL4_param";
    return false;
  }

  RSPParam LRSR1_param;
  LRSR(x, y, phi, &LRSR1_param);
  double LRSR1_lengths[4] = {LRSR1_param.t, -0.5 * M_PI, LRSR1_param.u,
                             LRSR1_param.v};
  char LRSR1_types[] = "LRSR";
  if (LRSR1_param.flag &&
      !SetRSP(4, LRSR1_lengths, LRSR1_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSR1_param";
    return false;
  }

  RSPParam LRSR2_param;
  LRSR(-x, y, -phi, &LRSR2_param);
  double LRSR2_lengths[4] = {-LRSR2_param.t, 0.5 * M_PI, -LRSR2_param.u,
                             -LRSR2_param.v};
  char LRSR2_types[] = "LRSR";
  if (LRSR2_param.flag &&
      !SetRSP(4, LRSR2_lengths, LRSR2_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSR2_param";
    return false;
  }

  RSPParam LRSR3_param;
  LRSR(x, -y, -phi, &LRSR3_param);
  double LRSR3_lengths[4] = {LRSR3_param.t, -0.5 * M_PI, LRSR3_param.u,
                             LRSR3_param.v};
  char LRSR3_types[] = "RLSL";
  if (LRSR3_param.flag &&
      !SetRSP(4, LRSR3_lengths, LRSR3_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSR3_param";
    return false;
  }

  RSPParam LRSR4_param;
  LRSR(-x, -y, phi, &LRSR4_param);
  double LRSR4_lengths[4] = {-LRSR4_param.t, 0.5 * M_PI, -LRSR4_param.u,
                             -LRSR4_param.v};
  char LRSR4_types[] = "RLSL";
  if (LRSR4_param.flag &&
      !SetRSP(4, LRSR4_lengths, LRSR4_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSR4_param";
    return false;
  }

  // backward
  double xb = x * std::cos(phi) + y * std::sin(phi);
  double yb = x * std::sin(phi) - y * std::cos(phi);

  RSPParam LRSL5_param;
  LRSL(xb, yb, phi, &LRSL5_param);
  double LRSL5_lengths[4] = {LRSL5_param.v, LRSL5_param.u, -0.5 * M_PI,
                             LRSL5_param.t};
  char LRSL5_types[] = "LSRL";
  if (LRSL5_param.flag &&
      !SetRSP(4, LRSL5_lengths, LRSL5_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRLRn_param";
    return false;
  }

  RSPParam LRSL6_param;
  LRSL(-xb, yb, -phi, &LRSL6_param);
  double LRSL6_lengths[4] = {-LRSL6_param.v, -LRSL6_param.u, 0.5 * M_PI,
                             -LRSL6_param.t};
  char LRSL6_types[] = "LSRL";
  if (LRSL6_param.flag &&
      !SetRSP(4, LRSL6_lengths, LRSL6_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSL6_param";
    return false;
  }

  RSPParam LRSL7_param;
  LRSL(xb, -yb, -phi, &LRSL7_param);
  double LRSL7_lengths[4] = {LRSL7_param.v, LRSL7_param.u, -0.5 * M_PI,
                             LRSL7_param.t};
  char LRSL7_types[] = "RSLR";
  if (LRSL7_param.flag &&
      !SetRSP(4, LRSL7_lengths, LRSL7_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSL7_param";
    return false;
  }

  RSPParam LRSL8_param;
  LRSL(-xb, -yb, phi, &LRSL8_param);
  double LRSL8_lengths[4] = {-LRSL8_param.v, -LRSL8_param.u, 0.5 * M_PI,
                             -LRSL8_param.t};
  char LRSL8_types[] = "RSLR";
  if (LRSL8_param.flag &&
      !SetRSP(4, LRSL8_lengths, LRSL8_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSL8_param";
    return false;
  }

  RSPParam LRSR5_param;
  LRSR(xb, yb, phi, &LRSR5_param);
  double LRSR5_lengths[4] = {LRSR5_param.v, LRSR5_param.u, -0.5 * M_PI,
                             LRSR5_param.t};
  char LRSR5_types[] = "RSRL";
  if (LRSR5_param.flag &&
      !SetRSP(4, LRSR5_lengths, LRSR5_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSR5_param";
    return false;
  }

  RSPParam LRSR6_param;
  LRSR(-xb, yb, -phi, &LRSR6_param);
  double LRSR6_lengths[4] = {-LRSR6_param.v, -LRSR6_param.u, 0.5 * M_PI,
                             -LRSR6_param.t};
  char LRSR6_types[] = "RSRL";
  if (LRSR6_param.flag &&
      !SetRSP(4, LRSR6_lengths, LRSR6_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSR6_param";
    return false;
  }

  RSPParam LRSR7_param;
  LRSR(xb, -yb, -phi, &LRSR7_param);
  double LRSR7_lengths[4] = {LRSR7_param.v, LRSR7_param.u, -0.5 * M_PI,
                             LRSR7_param.t};
  char LRSR7_types[] = "LSLR";
  if (LRSR7_param.flag &&
      !SetRSP(4, LRSR7_lengths, LRSR7_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSR7_param";
    return false;
  }

  RSPParam LRSR8_param;
  LRSR(-xb, -yb, phi, &LRSR8_param);
  double LRSR8_lengths[4] = {-LRSR8_param.v, -LRSR8_param.u, 0.5 * M_PI,
                             -LRSR8_param.t};
  char LRSR8_types[] = "LSLR";
  if (LRSR8_param.flag &&
      !SetRSP(4, LRSR8_lengths, LRSR8_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSR8_param";
    return false;
  }
  return true;
}

bool ReedShepp::CCSCC(const double x, const double y, const double phi,
                      std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LRSLR1_param;
  LRSLR(x, y, phi, &LRSLR1_param);
  double LRSLR1_lengths[5] = {LRSLR1_param.t, -0.5 * M_PI, LRSLR1_param.u,
                              -0.5 * M_PI, LRSLR1_param.v};
  char LRSLR1_types[] = "LRSLR";
  if (LRSLR1_param.flag &&
      !SetRSP(5, LRSLR1_lengths, LRSLR1_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSLR1_param";
    return false;
  }

  RSPParam LRSLR2_param;
  LRSLR(-x, y, -phi, &LRSLR2_param);
  double LRSLR2_lengths[5] = {-LRSLR2_param.t, 0.5 * M_PI, -LRSLR2_param.u,
                              0.5 * M_PI, -LRSLR2_param.v};
  char LRSLR2_types[] = "LRSLR";
  if (LRSLR2_param.flag &&
      !SetRSP(5, LRSLR2_lengths, LRSLR2_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSLR2_param";
    return false;
  }

  RSPParam LRSLR3_param;
  LRSLR(x, -y, -phi, &LRSLR3_param);
  double LRSLR3_lengths[5] = {LRSLR3_param.t, -0.5 * M_PI, LRSLR3_param.u,
                              -0.5 * M_PI, LRSLR3_param.v};
  char LRSLR3_types[] = "RLSRL";
  if (LRSLR3_param.flag &&
      !SetRSP(5, LRSLR3_lengths, LRSLR3_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSLR3_param";
    return false;
  }

  RSPParam LRSLR4_param;
  LRSLR(-x, -y, phi, &LRSLR4_param);
  double LRSLR4_lengths[5] = {-LRSLR4_param.t, 0.5 * M_PI, -LRSLR4_param.u,
                              0.5 * M_PI, -LRSLR4_param.v};
  char LRSLR4_types[] = "RLSRL";
  if (LRSLR4_param.flag &&
      !SetRSP(5, LRSLR4_lengths, LRSLR4_types, all_possible_paths)) {
    ADEBUG << "Fail at SetRSP with LRSLR4_param";
    return false;
  }
  return true;
}

void ReedShepp::LSL(const double x, const double y, const double phi,
                    RSPParam* param) {
  std::pair<double, double> polar =
      common::math::Cartesian2Polar(x - std::sin(phi), y - 1.0 + std::cos(phi));
  double u = polar.first;
  double t = polar.second;
  double v = 0.0;
  if (t >= 0.0) {
    v = common::math::NormalizeAngle(phi - t);
    if (v >= 0.0) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

void ReedShepp::LSR(const double x, const double y, const double phi,
                    RSPParam* param) {
  std::pair<double, double> polar =
      common::math::Cartesian2Polar(x + std::sin(phi), y - 1.0 - std::cos(phi));
  double u1 = polar.first * polar.first;
  double t1 = polar.second;
  double u = 0.0;
  double theta = 0.0;
  double t = 0.0;
  double v = 0.0;
  if (u1 >= 4.0) {
    u = std::sqrt(u1 - 4.0);
    theta = std::atan2(2.0, u);
    t = common::math::NormalizeAngle(t1 + theta);
    v = common::math::NormalizeAngle(t - phi);
    if (t >= 0.0 && v >= 0.0) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

void ReedShepp::LRL(const double x, const double y, const double phi,
                    RSPParam* param) {
  std::pair<double, double> polar =
      common::math::Cartesian2Polar(x - std::sin(phi), y - 1.0 + std::cos(phi));
  double u1 = polar.first;
  double t1 = polar.second;
  double u = 0.0;
  double t = 0.0;
  double v = 0.0;
  if (u1 <= 4.0) {
    u = -2.0 * std::asin(0.25 * u1);
    t = common::math::NormalizeAngle(t1 + 0.5 * u + M_PI);
    v = common::math::NormalizeAngle(phi - t + u);
    if (t >= 0.0 && u <= 0.0) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

void ReedShepp::SLS(const double x, const double y, const double phi,
                    RSPParam* param) {
  double phi_mod = common::math::NormalizeAngle(phi);
  double xd = 0.0;
  double u = 0.0;
  double t = 0.0;
  double v = 0.0;
  double epsilon = 1e-1;
  if (y > 0.0 && phi_mod > epsilon && phi_mod < M_PI) {
    xd = -y / std::tan(phi_mod) + x;
    t = xd - std::tan(phi_mod / 2.0);
    u = phi_mod;
    v = std::sqrt((x - xd) * (x - xd) + y * y) - tan(phi_mod / 2.0);
    param->flag = true;
    param->u = u;
    param->t = t;
    param->v = v;
  } else if (y < 0.0 && phi_mod > epsilon && phi_mod < M_PI) {
    xd = -y / std::tan(phi_mod) + x;
    t = xd - std::tan(phi_mod / 2.0);
    u = phi_mod;
    v = -std::sqrt((x - xd) * (x - xd) + y * y) - std::tan(phi_mod / 2.0);
    param->flag = true;
    param->u = u;
    param->t = t;
    param->v = v;
  }
}

void ReedShepp::LRLRn(const double x, const double y, const double phi,
                      RSPParam* param) {
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));
  double u = 0.0;
  if (rho <= 1.0 && rho >= 0.0) {
    u = std::acos(rho);
    if (u >= 0 && u <= 0.5 * M_PI) {
      std::pair<double, double> tau_omega = calc_tau_omega(u, -u, xi, eta, phi);
      if (tau_omega.first >= 0.0 && tau_omega.second <= 0.0) {
        param->flag = true;
        param->u = u;
        param->t = tau_omega.first;
        param->v = tau_omega.second;
      }
    }
  }
}

void ReedShepp::LRLRp(const double x, const double y, const double phi,
                      RSPParam* param) {
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho = (20.0 - xi * xi - eta * eta) / 16.0;
  double u = 0.0;
  if (rho <= 1.0 && rho >= 0.0) {
    u = -std::acos(rho);
    if (u >= 0 && u <= 0.5 * M_PI) {
      std::pair<double, double> tau_omega = calc_tau_omega(u, u, xi, eta, phi);
      if (tau_omega.first >= 0.0 && tau_omega.second >= 0.0) {
        param->flag = true;
        param->u = u;
        param->t = tau_omega.first;
        param->v = tau_omega.second;
      }
    }
  }
}

void ReedShepp::LRSR(const double x, const double y, const double phi,
                     RSPParam* param) {
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  std::pair<double, double> polar = common::math::Cartesian2Polar(-eta, xi);
  double rho = polar.first;
  double theta = polar.second;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  if (rho >= 2.0) {
    t = theta;
    u = 2.0 - rho;
    v = common::math::NormalizeAngle(t + 0.5 * M_PI - phi);
    if (t >= 0.0 && u <= 0.0 && v <= 0.0) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

void ReedShepp::LRSL(const double x, const double y, const double phi,
                     RSPParam* param) {
  double xi = x - std::sin(phi);
  double eta = y - 1.0 + std::cos(phi);
  std::pair<double, double> polar = common::math::Cartesian2Polar(xi, eta);
  double rho = polar.first;
  double theta = polar.second;
  double r = 0.0;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;

  if (rho >= 2.0) {
    r = std::sqrt(rho * rho - 4.0);
    u = 2.0 - r;
    t = common::math::NormalizeAngle(theta + std::atan2(r, -2.0));
    v = common::math::NormalizeAngle(phi - 0.5 * M_PI - t);
    if (t >= 0.0 && u <= 0.0 && v <= 0.0) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

void ReedShepp::LRSLR(const double x, const double y, const double phi,
                      RSPParam* param) {
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  std::pair<double, double> polar = common::math::Cartesian2Polar(xi, eta);
  double rho = polar.first;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  if (rho >= 2.0) {
    u = 4.0 - std::sqrt(rho * rho - 4.0);
    if (u <= 0.0) {
      t = common::math::NormalizeAngle(
          atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
      v = common::math::NormalizeAngle(t - phi);

      if (t >= 0.0 && v >= 0.0) {
        param->flag = true;
        param->u = u;
        param->t = t;
        param->v = v;
      }
    }
  }
}

bool ReedShepp::SetRSP(const int size, const double* lengths, const char* types,
                       std::vector<ReedSheppPath>* all_possible_paths) {
  ReedSheppPath path;
  std::vector<double> length_vec(lengths, lengths + size);
  std::vector<char> type_vec(types, types + size);
  path.segs_lengths = length_vec;
  path.segs_types = type_vec;
  double sum = 0.0;
  for (int i = 0; i < size; ++i) {
    sum += std::abs(lengths[i]);
  }
  path.total_length = sum;
  if (path.total_length <= 0.0) {
    AERROR << "total length smaller than 0";
    return false;
  }
  all_possible_paths->emplace_back(path);
  return true;
}

// TODO(Jinyun) : reformulate GenerateLocalConfigurations.
bool ReedShepp::GenerateLocalConfigurations(
    const std::shared_ptr<Node3d> start_node,
    const std::shared_ptr<Node3d> end_node, ReedSheppPath* shortest_path) {
  double step_scaled =
      planner_open_space_config_.warm_start_config().step_size() * max_kappa_;

  size_t point_num = static_cast<size_t>(
      std::floor(shortest_path->total_length / step_scaled +
                 static_cast<double>(shortest_path->segs_lengths.size()) + 4));
  std::vector<double> px(point_num, 0.0);
  std::vector<double> py(point_num, 0.0);
  std::vector<double> pphi(point_num, 0.0);
  std::vector<bool> pgear(point_num, true);
  int index = 1;
  double d = 0.0;
  double pd = 0.0;
  double ll = 0.0;

  if (shortest_path->segs_lengths.at(0) > 0.0) {
    pgear.at(0) = true;
    d = step_scaled;
  } else {
    pgear.at(0) = false;
    d = -step_scaled;
  }
  pd = d;
  for (size_t i = 0; i < shortest_path->segs_types.size(); ++i) {
    char m = shortest_path->segs_types.at(i);
    double l = shortest_path->segs_lengths.at(i);
    if (l > 0.0) {
      d = step_scaled;
    } else {
      d = -step_scaled;
    }
    double ox = px.at(index);
    double oy = py.at(index);
    double ophi = pphi.at(index);
    index--;
    if (i >= 1 && shortest_path->segs_lengths.at(i - 1) *
                          shortest_path->segs_lengths.at(i) >
                      0) {
      pd = -d - ll;
    } else {
      pd = d - ll;
    }
    while (std::abs(pd) <= std::abs(l)) {
      index++;
      Interpolation(index, pd, m, ox, oy, ophi, &px, &py, &pphi, &pgear);
      pd += d;
    }
    ll = l - pd - d;
    index++;
    Interpolation(index, l, m, ox, oy, ophi, &px, &py, &pphi, &pgear);
  }
  double epsilon = 1e-15;
  while (std::fabs(px.back()) < epsilon && std::fabs(py.back()) < epsilon &&
         std::fabs(pphi.back()) < epsilon && pgear.back()) {
    px.pop_back();
    py.pop_back();
    pphi.pop_back();
    pgear.pop_back();
  }
  for (size_t i = 0; i < px.size(); ++i) {
    shortest_path->x.push_back(std::cos(-start_node->GetPhi()) * px.at(i) +
                               std::sin(-start_node->GetPhi()) * py.at(i) +
                               start_node->GetX());
    shortest_path->y.push_back(-std::sin(-start_node->GetPhi()) * px.at(i) +
                               std::cos(-start_node->GetPhi()) * py.at(i) +
                               start_node->GetY());
    shortest_path->phi.push_back(
        common::math::NormalizeAngle(pphi.at(i) + start_node->GetPhi()));
  }
  shortest_path->gear = pgear;
  for (size_t i = 0; i < shortest_path->segs_lengths.size(); ++i) {
    shortest_path->segs_lengths.at(i) =
        shortest_path->segs_lengths.at(i) / max_kappa_;
  }
  shortest_path->total_length = shortest_path->total_length / max_kappa_;
  return true;
}

void ReedShepp::Interpolation(const int index, const double pd, const char m,
                              const double ox, const double oy,
                              const double ophi, std::vector<double>* px,
                              std ::vector<double>* py,
                              std::vector<double>* pphi,
                              std::vector<bool>* pgear) {
  double ldx = 0.0;
  double ldy = 0.0;
  double gdx = 0.0;
  double gdy = 0.0;
  if (m == 'S') {
    px->at(index) = ox + pd / max_kappa_ * std::cos(ophi);
    py->at(index) = oy + pd / max_kappa_ * std::sin(ophi);
    pphi->at(index) = ophi;
  } else {
    ldx = std::sin(pd) / max_kappa_;
    if (m == 'L') {
      ldy = (1.0 - std::cos(pd)) / max_kappa_;
    } else if (m == 'R') {
      ldy = (1.0 - std::cos(pd)) / -max_kappa_;
    }
    gdx = std::cos(-ophi) * ldx + std::sin(-ophi) * ldy;
    gdy = -std::sin(-ophi) * ldx + std::cos(-ophi) * ldy;
    px->at(index) = ox + gdx;
    py->at(index) = oy + gdy;
  }

  if (pd > 0.0) {
    pgear->at(index) = true;
  } else {
    pgear->at(index) = false;
  }

  if (m == 'L') {
    pphi->at(index) = ophi + pd;
  } else if (m == 'R') {
    pphi->at(index) = ophi - pd;
  }
}

bool ReedShepp::SetRSPPar(const int size, const double* lengths,
                          const std::string& types,
                          std::vector<ReedSheppPath>* all_possible_paths,
                          const int idx) {
  ReedSheppPath path;
  std::vector<double> length_vec(lengths, lengths + size);
  std::vector<char> type_vec(types.begin(), types.begin() + size);
  path.segs_lengths = length_vec;
  path.segs_types = type_vec;
  double sum = 0.0;
  for (int i = 0; i < size; ++i) {
    sum += std::abs(lengths[i]);
  }
  path.total_length = sum;
  if (path.total_length <= 0.0) {
    AERROR << "total length smaller than 0";
    return false;
  }

  all_possible_paths->at(idx) = path;
  return true;
}

bool ReedShepp::GenerateRSPPar(const std::shared_ptr<Node3d> start_node,
                               const std::shared_ptr<Node3d> end_node,
                               std::vector<ReedSheppPath>* all_possible_paths) {
  double dx = end_node->GetX() - start_node->GetX();
  double dy = end_node->GetY() - start_node->GetY();
  double dphi = end_node->GetPhi() - start_node->GetPhi();
  double c = std::cos(start_node->GetPhi());
  double s = std::sin(start_node->GetPhi());
  // normalize the initial point to (0,0,0)
  double x = (c * dx + s * dy) * this->max_kappa_;
  double y = (-s * dx + c * dy) * this->max_kappa_;
  // backward
  double xb = x * std::cos(dphi) + y * std::sin(dphi);
  double yb = x * std::sin(dphi) - y * std::cos(dphi);

  int RSP_nums = 46;
  all_possible_paths->resize(RSP_nums);
  bool succ = true;
#pragma omp parallel for schedule(dynamic, 2) num_threads(8)
  for (int i = 0; i < RSP_nums; ++i) {
    RSPParam RSP_param;
    int tmp_length = 0;
    double RSP_lengths[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double x_param = 1.0;
    double y_param = 1.0;
    std::string rd_type;

    if (i > 2 && i % 2) {
      x_param = -1.0;
    }
    if (i > 2 && i % 4 < 2) {
      y_param = -1.0;
    }

    if (i < 2) {  // SCS case
      if (i == 1) {
        y_param = -1.0;
        rd_type = "SRS";
      } else {
        rd_type = "SLS";
      }
      SLS(x, y_param * y, y_param * dphi, &RSP_param);
      tmp_length = 3;
      RSP_lengths[0] = RSP_param.t;
      RSP_lengths[1] = RSP_param.u;
      RSP_lengths[2] = RSP_param.v;
    } else if (i < 6) {  // CSC, LSL case
      LSL(x_param * x, y_param * y, x_param * y_param * dphi, &RSP_param);
      if (y_param > 0) {
        rd_type = "LSL";
      } else {
        rd_type = "RSR";
      }
      tmp_length = 3;
      RSP_lengths[0] = x_param * RSP_param.t;
      RSP_lengths[1] = x_param * RSP_param.u;
      RSP_lengths[2] = x_param * RSP_param.v;
    } else if (i < 10) {  // CSC, LSR case
      LSR(x_param * x, y_param * y, x_param * y_param * dphi, &RSP_param);
      if (y_param > 0) {
        rd_type = "LSR";
      } else {
        rd_type = "RSL";
      }
      tmp_length = 3;
      RSP_lengths[0] = x_param * RSP_param.t;
      RSP_lengths[1] = x_param * RSP_param.u;
      RSP_lengths[2] = x_param * RSP_param.v;
    } else if (i < 14) {  // CCC, LRL case
      LRL(x_param * x, y_param * y, x_param * y_param * dphi, &RSP_param);
      if (y_param > 0) {
        rd_type = "LRL";
      } else {
        rd_type = "RLR";
      }
      tmp_length = 3;
      RSP_lengths[0] = x_param * RSP_param.t;
      RSP_lengths[1] = x_param * RSP_param.u;
      RSP_lengths[2] = x_param * RSP_param.v;
    } else if (i < 18) {  // CCC, LRL case, backward
      LRL(x_param * xb, y_param * yb, x_param * y_param * dphi, &RSP_param);
      if (y_param > 0) {
        rd_type = "LRL";
      } else {
        rd_type = "RLR";
      }
      tmp_length = 3;
      RSP_lengths[0] = x_param * RSP_param.v;
      RSP_lengths[1] = x_param * RSP_param.u;
      RSP_lengths[2] = x_param * RSP_param.t;
    } else if (i < 22) {  // CCCC, LRLRn
      LRLRn(x_param * x, y_param * y, x_param * y_param * dphi, &RSP_param);
      if (y_param > 0.0) {
        rd_type = "LRLR";
      } else {
        rd_type = "RLRL";
      }
      tmp_length = 4;
      RSP_lengths[0] = x_param * RSP_param.t;
      RSP_lengths[1] = x_param * RSP_param.u;
      RSP_lengths[2] = -x_param * RSP_param.u;
      RSP_lengths[3] = x_param * RSP_param.v;
    } else if (i < 26) {  // CCCC, LRLRp
      LRLRp(x_param * x, y_param * y, x_param * y_param * dphi, &RSP_param);
      if (y_param > 0.0) {
        rd_type = "LRLR";
      } else {
        rd_type = "RLRL";
      }
      tmp_length = 4;
      RSP_lengths[0] = x_param * RSP_param.t;
      RSP_lengths[1] = x_param * RSP_param.u;
      RSP_lengths[2] = -x_param * RSP_param.u;
      RSP_lengths[3] = x_param * RSP_param.v;
    } else if (i < 30) {  // CCSC, LRLRn
      tmp_length = 4;
      LRLRn(x_param * x, y_param * y, x_param * y_param * dphi, &RSP_param);
      if (y_param > 0.0) {
        rd_type = "LRSL";
      } else {
        rd_type = "RLSR";
      }
      RSP_lengths[0] = x_param * RSP_param.t;
      if (x_param < 0 && y_param > 0) {
        RSP_lengths[1] = 0.5 * M_PI;
      } else {
        RSP_lengths[1] = -0.5 * M_PI;
      }
      if (x_param > 0 && y_param < 0) {
        RSP_lengths[2] = RSP_param.u;
      } else {
        RSP_lengths[2] = -RSP_param.u;
      }
      RSP_lengths[3] = x_param * RSP_param.v;
    } else if (i < 34) {  // CCSC, LRLRp
      tmp_length = 4;
      LRLRp(x_param * x, y_param * y, x_param * y_param * dphi, &RSP_param);
      if (y_param) {
        rd_type = "LRSR";
      } else {
        rd_type = "RLSL";
      }
      RSP_lengths[0] = x_param * RSP_param.t;
      if (x_param < 0 && y_param > 0) {
        RSP_lengths[1] = 0.5 * M_PI;
      } else {
        RSP_lengths[1] = -0.5 * M_PI;
      }
      RSP_lengths[2] = x_param * RSP_param.u;
      RSP_lengths[3] = x_param * RSP_param.v;
    } else if (i < 38) {  // CCSC, LRLRn, backward
      tmp_length = 4;
      LRLRn(x_param * xb, y_param * yb, x_param * y_param * dphi, &RSP_param);
      if (y_param > 0) {
        rd_type = "LSRL";
      } else {
        rd_type = "RSLR";
      }
      RSP_lengths[0] = x_param * RSP_param.v;
      RSP_lengths[1] = x_param * RSP_param.u;
      RSP_lengths[2] = -x_param * 0.5 * M_PI;
      RSP_lengths[3] = x_param * RSP_param.t;
    } else if (i < 42) {  // CCSC, LRLRp, backward
      tmp_length = 4;
      LRLRp(x_param * xb, y_param * yb, x_param * y_param * dphi, &RSP_param);
      if (y_param > 0) {
        rd_type = "RSRL";
      } else {
        rd_type = "LSLR";
      }
      RSP_lengths[0] = x_param * RSP_param.v;
      RSP_lengths[1] = x_param * RSP_param.u;
      RSP_lengths[2] = -x_param * M_PI * 0.5;
      RSP_lengths[3] = x_param * RSP_param.t;
    } else {  // CCSCC, LRSLR
      tmp_length = 5;
      LRSLR(x_param * x, y_param * y, x_param * y_param * dphi, &RSP_param);
      if (y_param > 0.0) {
        rd_type = "LRSLR";
      } else {
        rd_type = "RLSRL";
      }
      RSP_lengths[0] = x_param * RSP_param.t;
      RSP_lengths[1] = -x_param * 0.5 * M_PI;
      RSP_lengths[2] = x_param * RSP_param.u;
      RSP_lengths[3] = -x_param * 0.5 * M_PI;
      RSP_lengths[4] = x_param * RSP_param.v;
    }

    if (tmp_length > 0) {
      if (RSP_param.flag &&
          !SetRSPPar(tmp_length, RSP_lengths, rd_type, all_possible_paths, i)) {
        AERROR << "Fail at SetRSP, idx#: " << i;
        succ = false;
      }
    }
  }

  if (!succ) {
    AERROR << "RSP parallel fails";
    return false;
  }
  if (all_possible_paths->size() == 0) {
    AERROR << "No path generated by certain two configurations";
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
