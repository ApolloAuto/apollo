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
#include "modules/planning/open_space/hybrid_a_star.h"

#include <iostream>

namespace apollo {
namespace planning {

class HybridAObstacleContainer {
 public:
  HybridAObstacleContainer() = default;
  void AddVirtualObstacle(const double x, const double y, const double heading,
                          const double length, const double width,
                          const int id) {
    Vec2d obstacle_center(x, y);
    Box2d obstacle_box(obstacle_center, heading, length, width);
    std::unique_ptr<PathObstacle> obstacle =
        PathObstacle::CreateStaticVirtualObstacles(std::to_string(id),
                                                   obstacle_box);
    obstacles_list.Add(obstacle->Id(), *obstacle);
  }
  ThreadSafeIndexedObstacles* GetObstacleList() { return &obstacles_list; }

 private:
  ThreadSafeIndexedObstacles obstacles_list;
};

class HybridAResultContainer {
 public:
  HybridAResultContainer() = default;
  void LoadResult() {
    x_ = std::move(result_.x);
    y_ = std::move(result_.y);
    phi_ = std::move(result_.phi);
    v_ = std::move(result_.v);
    a_ = std::move(result_.a);
    steer_ = std::move(result_.steer);
  }
  std::vector<double>* GetX() { return &x_; }
  std::vector<double>* GetY() { return &y_; }
  std::vector<double>* GetPhi() { return &phi_; }
  std::vector<double>* GetV() { return &v_; }
  std::vector<double>* GetA() { return &a_; }
  std::vector<double>* GetSteer() { return &steer_; }
  Result* PrepareResult() { return &result_; }

 private:
  Result result_;
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> phi_;
  std::vector<double> v_;
  std::vector<double> a_;
  std::vector<double> steer_;
};

extern "C" {
HybridAStar* CreatePlannerPtr() {
  apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;

  CHECK(apollo::common::util::GetProtoFromFile(
      FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
      << "Failed to load open space config file "
      << FLAGS_planner_open_space_config_filename;
  return new HybridAStar(planner_open_space_config_);
}
HybridAObstacleContainer* CreateObstaclesPtr() {
  return new HybridAObstacleContainer();
}
HybridAResultContainer* CreateResultPtr() {
  return new HybridAResultContainer();
}
void AddVirtualObstacle(HybridAObstacleContainer* obstacles_ptr, const double x,
                        const double y, const double heading,
                        const double length, const double width, const int id) {
  obstacles_ptr->AddVirtualObstacle(x, y, heading, length, width, id);
}
bool Plan(HybridAStar* planner_ptr, HybridAObstacleContainer* obstacles_ptr,
          HybridAResultContainer* result_ptr, double sx, double sy, double sphi,
          double ex, double ey, double ephi) {
  return planner_ptr->Plan(sx, sy, sphi, ex, ey, ephi,
                           obstacles_ptr->GetObstacleList(),
                           result_ptr->PrepareResult());
}
void GetResult(HybridAResultContainer* result_ptr, double* x, double* y,
               double* phi, double* v, double* a, double* steer,
               std::size_t* output_size) {
  result_ptr->LoadResult();
  std::size_t size = result_ptr->GetX()->size();
  std::cout << "return size is " << size << std::endl;
  for (std::size_t i = 0; i < size; i++) {
    x[i] = result_ptr->GetX()->at(i);
    y[i] = result_ptr->GetY()->at(i);
    phi[i] = result_ptr->GetPhi()->at(i);
    v[i] = result_ptr->GetV()->at(i);
  }
  for (std::size_t i = 0; i < size - 1; i++) {
    a[i] = result_ptr->GetA()->at(i);
    steer[i] = result_ptr->GetSteer()->at(i);
  }
  *output_size = size;
}
};

}  // namespace planning
}  // namespace apollo

int main(int32_t argc, char** argv) { return 0; }
