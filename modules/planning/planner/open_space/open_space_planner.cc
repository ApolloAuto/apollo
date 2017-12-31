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
  int N = 80;
  // nominal sampling time
  float Ts = 0.3;
  // wheelbase
  float L = 2.7;

  // TODO(QiL): cleaning up : load ego car matrix from VehicleParam at
  // initialization
  Eigen::MatrixXd ego(4, 1);
  ego << 3.7, 1, 1, 1;

  // initial state

  // TODO(QiL): Step 1 : Get initial state from VehicleState when enabled.
  Eigen::MatrixXd x0(4, 1);
  x0 << -12, 11, 0, 0;

  // final state

  // TODO(QiL): Step 2 ： Take final state from decision / or em planner when
  // enabled.
  Eigen::MatrixXd xF(4, 1);
  xF << 0, 1.2, M_PI / 2, 0;

  // TODO(QiL): Step 3 : Get obstacles from map/perception convex sets from
  // vetices using H-represetntation
  int nOb = 3;  // number of obstacles
  Eigen::MatrixXd vOb(3, 1);
  vOb << 4, 4, 4;

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
  Eigen::MatrixXd xWS = Eigen::MatrixXd::Zero(4, N + 1);
  Eigen::MatrixXd uWS = Eigen::MatrixXd::Zero(2, N);
  Eigen::MatrixXd timeWS = Eigen::MatrixXd::Zero(1, N + 1);

  // TODO(QiL): Step 7 : Formulate distance approach problem
  // solution from distance approach
  Eigen::MatrixXd xp1 = Eigen::MatrixXd::Zero(4, N + 1);
  Eigen::MatrixXd up1 = Eigen::MatrixXd::Zero(2, N);
  Eigen::MatrixXd scaleTime1 = Eigen::MatrixXd::Zero(1, N + 1);
  bool exitflag1 = 0;
  float time1 = 0;

  // TODO(QiL): Step 8 : Publish trajectoryPoint in planning trajectory
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
