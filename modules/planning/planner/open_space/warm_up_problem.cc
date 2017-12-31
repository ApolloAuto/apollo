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

/*
 * warm_up_problem.cc
 */

#include "modules/planning/planner/open_space/warm_up_problem.h"

#include <algorithm>
#include <iomanip>
#include <utility>

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::time::Clock;

WarmUpProblem::WarmUpProblem() {}

bool WarmUpProblem::Solve() const {
  bool status;
  return status == Ipopt::Solve_Succeeded ||
         status == Ipopt::Solved_To_Acceptable_Level;
}

}  // namespace planning
}  // namespace apollo
