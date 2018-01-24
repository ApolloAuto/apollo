/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planner/open_space/open_space_planner.h"

#include <fstream>
#include <utility>

#include "modules/common/log.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;

OpenSpacePlanner::OpenSpacePlanner() {}

Status OpenSpacePlanner::Init(const PlanningConfig&) {
  return Status::OK();
}

apollo::common::Status OpenSpacePlanner::Plan(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  // Problem setup

  // TODO(QiL) : cleaning up : load control configs from VehicleParam at
  // initialization
  // horizon
  std::size_t horizon = 80;
  // nominal sampling time
  float ts = 0.3;
  // wheelbase
  float wheelbase_length = 2.7;

  // TODO(QiL): cleaning up : load ego car matrix from VehicleParam at
  // initialization
  Eigen::MatrixXd ego(4, 1);
  ego << 3.7, 1, 1, 1;

  // initial state

  // TODO(QiL): Step 1 : Get initial state from VehicleState when enabled.

  Eigen::MatrixXd x0(4, 1);
  x0 << -12, 11, 0, 0;

  // std::vector<double> x0({-12, 11, 0, 0});

  // final state

  // TODO(QiL): Step 2 ： Take final state from decision / or em planner when
  // enabled.

  Eigen::MatrixXd xF(4, 1);
  xF << 0, 1.2, M_PI / 2, 0;

  // std::vector<double> xf({0, 1.2, M_PI / 2, 0});

  // TODO(QiL): Step 3 : Get obstacles from map/perception convex sets from
  // vetices using H-represetntation
  std::size_t nOb = 3;  // number of obstacles
  Eigen::MatrixXd vOb(3, 1);
  vOb << 4, 4, 4;

  // TODO(QiL) : Clean up, represent lOb with better format
  // vetices cw presentation

  // std::vector<std::vector<Eigen::MatrixXd>> lOb = {{[-20; 5]}};
  std::vector<std::vector<std::vector<double>>> lOb = {
      {{-20, 5}, {-1.3, 5}, {-1.3, -5}, {-20, -5}, {-20, 5}},
      {{1.3, 5}, {20, 5}, {20, -5}, {1.3, -5}, {1.3, 5}},
      {{-20, 15}, {20, 15}, {20, 11}, {-20, 11}, {-20, 15}}};
  // TODO(QiL) : more efficient way of initialize lOb in open space planner
  //  std::vector<std::vector<Eigen::MatrixXd>> lOb;
  Eigen::MatrixXd ob1(4, 1), ob2(4, 1), ob3(4, 1);
  ob1 << -1.3, 5, 20, 5;
  ob2 << 20, 5, -1.3, 5;
  ob3 << 20, 15, 20, -11;

  // TODO(QiL): Step 5 : Add absolute constraints (States and Controls) from
  // perception/map
  // [x_lower, x_upper, - y_lower, y_upper]
  Eigen::MatrixXd XYbounds(4, 1);
  XYbounds << -15, 15, 1, 10;

  // TODO(QiL): Step 6 ： Fromulate warmstart matrix

  // warm start variables
  Eigen::MatrixXd xWS = Eigen::MatrixXd::Zero(4, horizon + 1);
  Eigen::MatrixXd uWS = Eigen::MatrixXd::Zero(2, horizon);
  Eigen::MatrixXd timeWS = Eigen::MatrixXd::Zero(1, horizon + 1);

  warm_start_.reset(
      new WarmStartProblem(horizon, ts, wheelbase_length, x0, xF, XYbounds));

  std::vector<double> x1_result, x2_result, x3_result, x4_result, u1_result,
      u2_result, t_result;

  bool ret_status =
      warm_start_->Solve(&x1_result, &x2_result, &x3_result, &x4_result,
                         &u1_result, &u2_result, &t_result);
  if (ret_status) {
    ADEBUG << "Warm start problem solved successfully!";
  } else {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Warm start problem failed to solve");
  }
  // TODO(QiL) : Step 7 : Formualte H representation of obstacles

  Eigen::MatrixXd AOb = Eigen::MatrixXd::Zero(vOb.sum(), 2);
  Eigen::MatrixXd bOb = Eigen::MatrixXd::Zero(vOb.sum(), 1);

  Status ret = ObsHRep(nOb, vOb, lOb, &AOb, &bOb);

  // TODO(QiL): Step 8 : Formulate distance approach problem
  // solution from distance approach
  Eigen::MatrixXd xp1 = Eigen::MatrixXd::Zero(4, horizon + 1);
  Eigen::MatrixXd up1 = Eigen::MatrixXd::Zero(2, horizon);
  Eigen::MatrixXd scaleTime1 = Eigen::MatrixXd::Zero(1, horizon + 1);

  // TODO(QiL) : update the I/O to make the warm start problem and distance
  // approach problem connect
  distance_approach_.reset(new DistanceApproachProblem(
      x0, xF, horizon, ts, wheelbase_length, ego, xWS, uWS, timeWS, XYbounds,
      nOb, vOb, AOb, bOb));

  // result for distance approach problem
  std::vector<double> x1_result_2, x2_result_2, x3_result_2, x4_result_2,
      u1_result_2, u2_result_2, t_result_2;
  bool status = distance_approach_->Solve(
      &x1_result_2, &x2_result_2, &x3_result_2, &x4_result_2, &u1_result_2,
      &u2_result_2, &t_result_2);
  if (status) {
    ADEBUG << "Distance approach problem solved successfully!";
  } else {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Distance approach problem failed to solve");
  }

  // TODO(QiL): Step 9 : Publish trajectoryPoint in planning trajectory
  return Status::OK();
}

Status ObsHRep(const std::size_t& nOb, const Eigen::MatrixXd& vOb,
               const std::vector<std::vector<std::vector<double>>>& lOb,
               Eigen::MatrixXd* A_all, Eigen::MatrixXd* b_all) {
  // TODO(QiL) : Code replacement : find alternative ways for H presentation
  // caculation
  /*
  CHECK(nOb == lOb.rows()) << "No. of obstacles size mismatch, nOb : " << nOb
                           << ", lOb.rows() : " << lOb.rows();
*/
  A_all->resize(vOb.sum(), 2);
  b_all->resize(vOb.sum(), 1);

  int counter = 1;

  // start building H representation
  // TODO(QiL) : Add basic sanity check for H representation.
  for (std::size_t i = 1; i != nOb; ++i) {
    Eigen::MatrixXd A_i(int(vOb(i - 1, 0)), 2);
    Eigen::MatrixXd b_i(int(vOb(i - 1, 0)), 1);

    // take two subsequent vertices, and computer hyperplane
    for (int j = 1; j != vOb(i, 1); ++j) {
      std::vector<double> v1 = lOb[i - 1][j - 1];
      std::vector<double> v2 = lOb[i - 1][j];

      Eigen::MatrixXd A_tmp(2, 1), b_tmp(2, 1), ab(2, 2);
      // find hyperplane passing through v1 and v2
      if (v1[0] == v2[0]) {
        if (v2[1] < v1[1]) {
          A_tmp << 1, 0;
          b_tmp << v1[0];
        } else {
          A_tmp << -1, 0;
          b_tmp << v1[1];
        }
      } else if (v1[1] == v2[1]) {
        if (v1[1] < v2[1]) {
          A_tmp << 0, 1;
          b_tmp << v1[1];
        } else {
          A_tmp << 0, -1;
          b_tmp << -v1[1];
        }
      } else {
        Eigen::MatrixXd tmp1(2, 2);
        tmp1 << v1[0], 1, v2[0], 1;
        Eigen::MatrixXd tmp2(2, 1);
        tmp2 << v1[1], v2[1];
        ab = tmp2 * tmp1.inverse();
        double a = ab(0, 0);
        double b = ab(1, 0);

        if (v1[0] < v2[0]) {
          A_tmp << -a, 1;
          b_tmp << b;
        } else {
          A_tmp << a, -1;
          b_tmp << -b;
        }
      }

      // store vertices
      A_i.block(j, 0, 2, 1) = A_tmp;
      b_i.block(j, 0, 1, 1) = b_tmp;
    }

    AINFO << "size of A_j is : " << A_i.size();

    A_all->block(counter, 0, vOb(i, 0) - 1, 1) = A_i;
    b_all->block(counter, 0, vOb(i, 0) - 1, 1) = b_i;
  }

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
