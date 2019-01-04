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

#include "modules/planning/open_space/coarse_trajectory_generator/node3d.h"

namespace apollo {
namespace planning {

Node3d::Node3d(double x, double y, double phi) {
  x_ = x;
  y_ = y;
  phi_ = phi;
}

Node3d::Node3d(double x, double y, double phi,
               const std::vector<double>& XYbounds,
               const PlannerOpenSpaceConfig& open_space_conf) {
  CHECK(XYbounds.size() == 4)
      << "XYbounds size is not 4, but" << XYbounds.size();
  x_ = x;
  y_ = y;
  phi_ = phi;
  CHECK_GE(x_, XYbounds[0])
      << "x_ is smaller than xmin when constructing node3d";
  CHECK_GE(y_, XYbounds[2])
      << "y_ is smaller than ymin when constructing node3d";
  x_grid_ = static_cast<size_t>(
      (x_ - XYbounds[0]) /
      open_space_conf.warm_start_config().xy_grid_resolution());
  y_grid_ = static_cast<size_t>(
      (y_ - XYbounds[2]) /
      open_space_conf.warm_start_config().xy_grid_resolution());
  phi_grid_ = static_cast<size_t>(
      (phi_ - (-M_PI)) /
      open_space_conf.warm_start_config().phi_grid_resolution());
  index_ = static_cast<size_t>(
      static_cast<double>(phi_grid_) * (XYbounds[1] - XYbounds[0]) *
          (XYbounds[3] - XYbounds[2]) +
      static_cast<double>(y_grid_) * (XYbounds[1] - XYbounds[0]) +
      static_cast<double>(x_grid_));
}

Node3d::Node3d(std::vector<double> traversed_x, std::vector<double> traversed_y,
               std::vector<double> traversed_phi,
               const std::vector<double>& XYbounds,
               const PlannerOpenSpaceConfig& open_space_conf) {
  CHECK(XYbounds.size() == 4)
      << "XYbounds size is not 4, but" << XYbounds.size();
  x_ = traversed_x.back();
  y_ = traversed_y.back();
  phi_ = traversed_phi.back();
  CHECK_GE(x_, XYbounds[0])
      << "x_ is smaller than xmin when constructing node3d";
  CHECK_GE(y_, XYbounds[2])
      << "y_ is smaller than ymin when constructing node3d";
  // XYbounds in xmin, xmax, ymin, ymax
  x_grid_ = static_cast<size_t>(
      (x_ - XYbounds[0]) /
      open_space_conf.warm_start_config().xy_grid_resolution());
  y_grid_ = static_cast<size_t>(
      (y_ - XYbounds[2]) /
      open_space_conf.warm_start_config().xy_grid_resolution());
  phi_grid_ = static_cast<size_t>(
      (phi_ - (-M_PI)) /
      open_space_conf.warm_start_config().phi_grid_resolution());
  index_ = static_cast<size_t>(
      static_cast<double>(phi_grid_) * (XYbounds[1] - XYbounds[0]) *
          (XYbounds[3] - XYbounds[2]) +
      static_cast<double>(y_grid_) * (XYbounds[1] - XYbounds[0]) +
      static_cast<double>(x_grid_));
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

size_t Node3d::GetSize() {
  DCHECK(traversed_x_.size() == traversed_y_.size() &&
         traversed_x_.size() == traversed_phi_.size());
  return traversed_x_.size();
}

}  // namespace planning
}  // namespace apollo
