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

Status OpenSpacePlanner::Plan(const TrajectoryPoint& planning_init_point,
                              Frame*, ReferenceLineInfo* reference_line_info) {
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
  int nOb = 3;  // number of obstacles
  Eigen::MatrixXd vOb(3, 1);
  vOb << 4, 4, 4;

  // TODO(QiL) : Clean up, represent lOb with better format
  // vetices cw presentation
  /*
  lOb =
      [
        [ [-20; 5], [-1.3; 5], [-1.3; - 5], [-20; - 5], [-20; 5] ],
        [ [1.3; 5], [20; 5], [20; - 5], [1.3; - 5], [1.3; 5] ],
        [ [-20; 15], [20; 15], [20; 11], [ -20, 11 ], [-20; 15] ]
      ]
*/
  Eigen::MatrixXd lOb(4, 1);
  Eigen::MatrixXd ob1(4, 1), ob2(4, 1), ob3(4, 1);
  ob1 << -1.3, 5, 20, 5;
  ob2 << 20, 5, -1.3, 5;
  ob3 << 20, 15, 20, -11;

  // TODO(QiL): Step 5 : Add absolute constraints (States and Controls) from
  // perception/map
  //[x_lower, x_upper, - y_lower, y_upper]
  Eigen::MatrixXd XYbounds(4, 1);
  XYbounds << -15, 15, 1, 10;

  // TODO(QiL): Step 6 ： Fromulate warmstart matrix

  // warm start variables
  Eigen::MatrixXd xWS = Eigen::MatrixXd::Zero(4, horizon + 1);
  Eigen::MatrixXd uWS = Eigen::MatrixXd::Zero(2, horizon);
  Eigen::MatrixXd timeWS = Eigen::MatrixXd::Zero(1, horizon + 1);

  warm_start_.reset(
      new WarmStartProblem(horizon, ts, wheelbase_length, x0, xF, XYbounds));

  // TODO(QiL) : Step 7 : Formualte H representation of obstacles

  Eigen::MatrixXd A_all = Eigen::MatrixXd::Zero(vOb.sum(), 2);
  Eigen::MatrixXd b_all = Eigen::MatrixXd::Zero(vOb.sum(), 1);

  Status ret = ObsHRep(nOb, vOb, lOb, &A_all, &b_all);

  // TODO(QiL): Step 8 : Formulate distance approach problem
  // solution from distance approach
  Eigen::MatrixXd xp1 = Eigen::MatrixXd::Zero(4, horizon + 1);
  Eigen::MatrixXd up1 = Eigen::MatrixXd::Zero(2, horizon);
  Eigen::MatrixXd scaleTime1 = Eigen::MatrixXd::Zero(1, horizon + 1);
  bool exitflag1 = 0;
  float time1 = 0;

  // TODO(QiL): Step 9 : Publish trajectoryPoint in planning trajectory
  return Status::OK();
}

Status ObsHRep(int& nOb, Eigen::MatrixXd& vOb, Eigen::MatrixXd& lOb,
               Eigen::MatrixXd* A_all, Eigen::MatrixXd* b_all) {
  // TODO(QiL) : Code replacement : find alternative ways for H presentation
  // caculation
  CHECK(nOb == lOb.rows()) << "No. of obstacles size mismatch, nOb : " << nOb
                           << ", lOb.rows() : " << lOb.rows();

  int counter = 1;

  // start building H representation
  for (int i = 1; i <= nOb; ++i) {
  }
  return Status::OK();
}
}  // namespace planning
}  // namespace apollo
