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
 * node3d.cc
 */

#include "modules/planning/open_space/node3d.h"

namespace apollo {
namespace planning {

Node3d::Node3d(double x, double y, double phi,
               const PlannerOpenSpaceConfig& open_space_conf) {
  x_ = x;
  y_ = y;
  phi_ = phi;
  x_grid_ = static_cast<std::size_t>((x_ - open_space_conf.min_x()) /
                                     open_space_conf.xy_grid_resolution());
  y_grid_ = static_cast<std::size_t>((y_ - open_space_conf.min_y()) /
                                     open_space_conf.xy_grid_resolution());
  phi_grid_ = static_cast<std::size_t>((phi_ - (-M_PI)) /
                                       open_space_conf.phi_grid_resolution());
  index_ = phi_grid_ * (open_space_conf.max_x() - open_space_conf.min_x()) *
               (open_space_conf.max_y() - open_space_conf.min_y()) +
           y_grid_ * (open_space_conf.max_x() - open_space_conf.min_x()) +
           x_grid_;
}

Node3d::Node3d(double x, double y, double phi) {
  x_ = x;
  y_ = y;
  phi_ = phi;
}

Node3d::Node3d(double x, double y, double phi, std::vector<double> traversed_x,
               std::vector<double> traversed_y,
               std::vector<double> traversed_phi,
               const PlannerOpenSpaceConfig& open_space_conf) {
  x_ = x;
  y_ = y;
  phi_ = phi;
  x_grid_ = static_cast<std::size_t>((x_ - open_space_conf.min_x()) /
                                     open_space_conf.xy_grid_resolution());
  y_grid_ = static_cast<std::size_t>((y_ - open_space_conf.min_y()) /
                                     open_space_conf.xy_grid_resolution());
  phi_grid_ = static_cast<std::size_t>((phi_ - (-M_PI)) /
                                       open_space_conf.phi_grid_resolution());
  index_ = phi_grid_ * (open_space_conf.max_x() - open_space_conf.min_x()) *
               (open_space_conf.max_y() - open_space_conf.min_y()) +
           y_grid_ * (open_space_conf.max_x() - open_space_conf.min_x()) +
           x_grid_;
  traversed_x_ = traversed_x;
  traversed_y_ = traversed_y;
  traversed_phi_ = traversed_phi;
}

Box2d Node3d::GetBoundingBox(const common::VehicleParam& vehicle_param_) {
  double ego_length = vehicle_param_.length();
  double ego_width = vehicle_param_.width();
  Box2d ego_box({x_, y_}, phi_, ego_length, ego_width);
  double shift_distance =
      ego_length / 2.0 - vehicle_param_.back_edge_to_center();
  Vec2d shift_vec{shift_distance * std::cos(phi_),
                  shift_distance * std::sin(phi_)};
  ego_box.Shift(shift_vec);
  return ego_box;
}

bool Node3d::operator==(const std::shared_ptr<Node3d> right) const {
  return x_grid_ == right->GetGridX() && y_grid_ == right->GetGridY() &&
         phi_grid_ == right->GetGridPhi();
}

}  // namespace planning
}  // namespace apollo
