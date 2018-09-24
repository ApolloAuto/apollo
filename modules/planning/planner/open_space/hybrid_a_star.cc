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
 * hybrid_a_star.cc
 */

#include "modules/planning/planner/open_space/hybrid_a_star.h"

namespace apollo {
namespace planning {

HybridAStar::HybridAStar(double sx, double sy, double sphi, double ex,
                         double ey, double ephi,
                         std::vector<const Obstacle *> obstacles) {
  CHECK(common::util::GetProtoFromFile(FLAGS_open_space_config_filename,
                                       &open_space_conf_))
      << "Failed to load open space planner config file "
      << FLAGS_open_space_config_filename;
  start_node_.reset(new Node3d(sx, sy, sphi, open_space_conf_));
  end_node_.reset(new Node3d(ex, ey, ephi, open_space_conf_));
  obstacles_ = obstacles;
  next_node_num_ = open_space_conf_.next_node_num();
}

bool HybridAStar::Plan() {
  open_set_.insert(std::make_pair(start_node_->GetIndex(), start_node_));
  open_pq_.push(
      std::make_pair(start_node_->GetIndex(), start_node_->GetCost()));
  while (!open_pq_.empty()) {
    std::size_t current_id = open_pq_.top().first;
    open_pq_.pop();
    std::shared_ptr<Node3d> current_node = open_set_[current_id];
    // check if a analystic curve could be connected from current configuration
    // to the end configuration
    if (AnalyticExpansion(current_node)) {
      AINFO << "Reach the end configuration with Reed Sharp";
      close_set_.insert(std::make_pair(end_node_->GetIndex(), end_node_));
      break;
    }
    close_set_.insert(std::make_pair(current_id, current_node));
    //
    for (std::size_t i = 0; i < next_node_num_; i++) {
      std::shared_ptr<Node3d> next_node = Next_node_generator(i);
      // boundary and validity check
      if (!Validitycheck(next_node)) {
        continue;
      }
      // check if the node is already in the close set
      if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
        continue;
      }

      if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
        open_set_.insert(std::make_pair(next_node->GetIndex(), next_node));
        // TODO: only calculate cost here
        open_pq_.push(
            std::make_pair(next_node->GetIndex(), next_node->GetCost()));
      } else {
        // reintial the cost for rewiring
      }
    }
  }
  result_ = GetResult();
  return true;
}
}  // namespace planning
}  // namespace apollo