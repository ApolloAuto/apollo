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
 * node3d.h
 */

#ifndef MODULES_PLANNING_OPEN_SPACE_NODE3D_H_
#define MODULES_PLANNING_OPEN_SPACE_NODE3D_H_

#include <memory>
#include <vector>

#include "modules/common/math/box2d.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/constraint_checker/collision_checker.h"

namespace apollo {
namespace planning {

using apollo::common::math::Box2d;

class Node3d {
 public:
  explicit Node3d(double x, double y, double phi, double x_grid, double y_grid,
                double phi_grid);
  virtual ~Node3d() = default;
  Box2d GetBoundingBox(const common::VehicleParam& vehicle_param_);
  double GetCost() { return current_cost_ + heuristic_cost_; };
  std::size_t GetGridX() { return x_grid_; };
  std::size_t GetGridY() { return y_grid_; };
  std::size_t GetGridPhi() { return phi_grid_; };
  double GetX() { return x_; };
  double GetY() { return y_; };
  double GetPhi() { return phi_; };
  bool operator==(const SmartPtr<Node3d> right) const;
  int GetIndex();

 private:
  double x_ = 0.0;
  double y_ = 0.0;
  double phi_ = 0.0;
  std::size_t x_grid_ = 0;
  std::size_t y_grid_ = 0;
  std::size_t phi_grid_ = 0;
  double current_cost_ = 0.0;
  double heuristic_cost_ = 0.0;
  SmartPtr<Node3d> pre_node = nullptr;
  double steering_ = 0.0;
  // true for moving forward and false for moving backward
  bool direction_ = true;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPEN_SPACE_NODE3D_H_