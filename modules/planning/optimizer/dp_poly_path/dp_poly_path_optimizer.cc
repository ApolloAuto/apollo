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

/**
 * @file
 **/

#include "modules/planning/optimizer/dp_poly_path/dp_poly_path_optimizer.h"

#include <string>

#include "modules/common/util/file.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/optimizer/dp_poly_path/dp_road_graph.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

DpPolyPathOptimizer::DpPolyPathOptimizer(const std::string &name)
    : PathOptimizer(name) {}

bool DpPolyPathOptimizer::Init() {
  if (!common::util::GetProtoFromFile(FLAGS_dp_poly_path_config_file,
                                      &config_)) {
    AERROR << "failed to load config file " << FLAGS_dp_poly_path_config_file;
    return false;
  }
  is_init_ = true;
  return true;
}

Status DpPolyPathOptimizer::Process(const SpeedData &speed_data,
                                    const ReferenceLine &reference_line,
                                    const common::TrajectoryPoint &init_point,
                                    PathDecision *const path_decision,
                                    PathData *const path_data) {
  if (!is_init_) {
    AERROR << "Please call Init() before Process().";
    return Status(ErrorCode::PLANNING_ERROR, "Not inited.");
  }
  CHECK_NOTNULL(path_decision);
  CHECK_NOTNULL(path_data);
  DPRoadGraph dp_road_graph(config_, init_point, speed_data);
  if (!dp_road_graph.FindPathTunnel(reference_line, path_data)) {
    AERROR << "Failed to find tunnel in road graph";
    return Status(ErrorCode::PLANNING_ERROR, "dp_road_graph path generation");
  }
  const auto &obstacles = frame_->GetObstacles().Items();
  IdDecisionList decision_list;
  if (!dp_road_graph.ComputeObjectDecision(
          *path_data, speed_data, reference_line, obstacles, &decision_list)) {
    AERROR << "Failed to make decision based on tunnel";
    return Status(ErrorCode::PLANNING_ERROR, "dp_road_graph decision ");
  }
  for (const auto &decision : decision_list) {
    if (!path_decision->AddDecision("dp_poly_path", decision.first,
                                    decision.second)) {
      AERROR << "Failed to add decision for object: " << decision.first;
      return Status(ErrorCode::PLANNING_ERROR,
                    "obstacles and PathDecision does not match");
    }
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
