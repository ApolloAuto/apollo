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

/**
 * @file
 **/

#include "modules/planning/open_space/trajectory_smoother/distance_approach_problem.h"
#include "modules/planning/open_space/trajectory_smoother/dual_variable_warm_start_problem.h"
#include "modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"

namespace apollo {
namespace planning {

class ObstacleContainer {
 public:
  ObstacleContainer() = default;

  bool VPresentationObstacle(
      const double* ROI_distance_approach_parking_boundary) {
    obstacles_num_ = 4;
    obstacles_edges_num_.resize(4, 1);
    obstacles_edges_num_ << 2, 1, 2, 1;
    size_t index = 0;
    for (size_t i = 0; i < obstacles_num_; i++) {
      std::vector<Vec2d> vertices_cw;
      for (int j = 0; j < obstacles_edges_num_(i, 0) + 1; j++) {
        Vec2d vertice =
            Vec2d(ROI_distance_approach_parking_boundary[index],
                  ROI_distance_approach_parking_boundary[index + 1]);
        index += 2;
        vertices_cw.emplace_back(vertice);
      }
      obstacles_vertices_vec_.emplace_back(vertices_cw);
    }
    return true;
  }

  bool HPresentationObstacle() {
    obstacles_A_ = Eigen::MatrixXd::Zero(obstacles_edges_num_.sum(), 2);
    obstacles_b_ = Eigen::MatrixXd::Zero(obstacles_edges_num_.sum(), 1);
    // vertices using H-represetntation
    if (!ObsHRep(obstacles_num_, obstacles_edges_num_, obstacles_vertices_vec_,
                 &obstacles_A_, &obstacles_b_)) {
      AINFO << "Fail to present obstacle in hyperplane";
      return false;
    }
    return true;
  }

  bool ObsHRep(const size_t& obstacles_num,
               const Eigen::MatrixXi& obstacles_edges_num,
               const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
               Eigen::MatrixXd* A_all, Eigen::MatrixXd* b_all) {
    if (obstacles_num != obstacles_vertices_vec.size()) {
      AINFO << "obstacles_num != obstacles_vertices_vec.size()";
      return false;
    }

    A_all->resize(obstacles_edges_num.sum(), 2);
    b_all->resize(obstacles_edges_num.sum(), 1);

    int counter = 0;
    double kEpsilon = 1.0e-5;
    // start building H representation
    for (size_t i = 0; i < obstacles_num; ++i) {
      size_t current_vertice_num = obstacles_edges_num(i, 0);
      Eigen::MatrixXd A_i(current_vertice_num, 2);
      Eigen::MatrixXd b_i(current_vertice_num, 1);

      // take two subsequent vertices, and computer hyperplane
      for (size_t j = 0; j < current_vertice_num; ++j) {
        Vec2d v1 = obstacles_vertices_vec[i][j];
        Vec2d v2 = obstacles_vertices_vec[i][j + 1];

        Eigen::MatrixXd A_tmp(2, 1), b_tmp(1, 1), ab(2, 1);
        // find hyperplane passing through v1 and v2
        if (std::abs(v1.x() - v2.x()) < kEpsilon) {
          if (v2.y() < v1.y()) {
            A_tmp << 1, 0;
            b_tmp << v1.x();
          } else {
            A_tmp << -1, 0;
            b_tmp << -v1.x();
          }
        } else if (std::abs(v1.y() - v2.y()) < kEpsilon) {
          if (v1.x() < v2.x()) {
            A_tmp << 0, 1;
            b_tmp << v1.y();
          } else {
            A_tmp << 0, -1;
            b_tmp << -v1.y();
          }
        } else {
          Eigen::MatrixXd tmp1(2, 2);
          tmp1 << v1.x(), 1, v2.x(), 1;
          Eigen::MatrixXd tmp2(2, 1);
          tmp2 << v1.y(), v2.y();
          ab = tmp1.inverse() * tmp2;
          double a = ab(0, 0);
          double b = ab(1, 0);

          if (v1.x() < v2.x()) {
            A_tmp << -a, 1;
            b_tmp << b;
          } else {
            A_tmp << a, -1;
            b_tmp << -b;
          }
        }

        // store vertices
        A_i.block(j, 0, 1, 2) = A_tmp.transpose();
        b_i.block(j, 0, 1, 1) = b_tmp;
      }

      A_all->block(counter, 0, A_i.rows(), 2) = A_i;
      b_all->block(counter, 0, b_i.rows(), 1) = b_i;
      counter += static_cast<int>(current_vertice_num);
    }
    return true;
  }

  void AddObstacle(const double* ROI_distance_approach_parking_boundary) {
    // the obstacles are hard coded into vertice sets of 3, 2, 3, 2
    if (!(VPresentationObstacle(ROI_distance_approach_parking_boundary) &&
          HPresentationObstacle())) {
      AINFO << "obstacle presentation fails";
    }
  }

  const std::vector<std::vector<Vec2d>>& GetObstacleVec() {
    return obstacles_vertices_vec_;
  }
  Eigen::MatrixXd GetAMatrix() { return obstacles_A_; }
  Eigen::MatrixXd GetbMatrix() { return obstacles_b_; }
  size_t GetObstaclesNum() { return obstacles_num_; }
  Eigen::MatrixXi GetObstaclesEdgesNum() { return obstacles_edges_num_; }

 private:
  size_t obstacles_num_ = 0;
  Eigen::MatrixXi obstacles_edges_num_;
  std::vector<std::vector<Vec2d>> obstacles_vertices_vec_;
  Eigen::MatrixXd obstacles_A_;
  Eigen::MatrixXd obstacles_b_;
};

class ResultContainer {
 public:
  ResultContainer() = default;
  void LoadHybridAResult() {
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
  HybridAStartResult* PrepareHybridAResult() { return &result_; }
  Eigen::MatrixXd* PrepareStateResult() { return &state_result_ds_; }
  Eigen::MatrixXd* PrepareControlResult() { return &control_result_ds_; }
  Eigen::MatrixXd* PrepareTimeResult() { return &time_result_ds_; }
  Eigen::MatrixXd* PrepareLResult() { return &dual_l_result_ds_; }
  Eigen::MatrixXd* PrepareNResult() { return &dual_n_result_ds_; }

 private:
  HybridAStartResult result_;
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> phi_;
  std::vector<double> v_;
  std::vector<double> a_;
  std::vector<double> steer_;
  Eigen::MatrixXd state_result_ds_;
  Eigen::MatrixXd control_result_ds_;
  Eigen::MatrixXd time_result_ds_;
  Eigen::MatrixXd dual_l_result_ds_;
  Eigen::MatrixXd dual_n_result_ds_;
};

extern "C" {
HybridAStar* CreateHybridAPtr() {
  apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;
  CHECK(apollo::common::util::GetProtoFromFile(
      FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
      << "Failed to load open space config file "
      << FLAGS_planner_open_space_config_filename;
  return new HybridAStar(planner_open_space_config_);
}
ObstacleContainer* DistanceCreateObstaclesPtr() {
  return new ObstacleContainer();
}
ResultContainer* DistanceCreateResultPtr() { return new ResultContainer(); }

void AddObstacle(ObstacleContainer* obstacles_ptr,
                 const double* ROI_distance_approach_parking_boundary) {
  obstacles_ptr->AddObstacle(ROI_distance_approach_parking_boundary);
}
bool DistancePlan(HybridAStar* hybridA_ptr, ObstacleContainer* obstacles_ptr,
                  ResultContainer* result_ptr, double sx, double sy,
                  double sphi, double ex, double ey, double ephi,
                  double* XYbounds) {
  apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;
  CHECK(apollo::common::util::GetProtoFromFile(
      FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
      << "Failed to load open space config file "
      << FLAGS_planner_open_space_config_filename;
  std::vector<double> XYbounds_(XYbounds, XYbounds + 4);
  if (!hybridA_ptr->Plan(sx, sy, sphi, ex, ey, ephi, XYbounds_,
                         obstacles_ptr->GetObstacleVec(),
                         result_ptr->PrepareHybridAResult())) {
    AINFO << "Hybrid A fail";
    return false;
  }
  // load Warm Start result(horizon is the "N", not the size of step points)
  size_t horizon_ = result_ptr->PrepareHybridAResult()->x.size() - 1;
  Eigen::MatrixXd xWS = Eigen::MatrixXd::Zero(4, horizon_ + 1);
  Eigen::MatrixXd uWS = Eigen::MatrixXd::Zero(2, horizon_);
  Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result_ptr->PrepareHybridAResult()->x.data(), horizon_ + 1);
  Eigen::VectorXd y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result_ptr->PrepareHybridAResult()->y.data(), horizon_ + 1);
  Eigen::VectorXd phi = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result_ptr->PrepareHybridAResult()->phi.data(), horizon_ + 1);
  Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result_ptr->PrepareHybridAResult()->v.data(), horizon_ + 1);
  Eigen::VectorXd steer = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result_ptr->PrepareHybridAResult()->steer.data(), horizon_);
  Eigen::VectorXd a = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result_ptr->PrepareHybridAResult()->a.data(), horizon_);
  xWS.row(0) = x;
  xWS.row(1) = y;
  xWS.row(2) = phi;
  xWS.row(3) = v;
  uWS.row(0) = steer;
  uWS.row(1) = a;

  Eigen::MatrixXd x0(4, 1);
  x0 << sx, sy, sphi, 0.0;

  Eigen::MatrixXd xF(4, 1);
  xF << ex, ey, ephi, 0.0;

  Eigen::MatrixXd last_time_u(2, 1);
  last_time_u << 0.0, 0.0;

  common::VehicleParam vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  // nominal sampling time
  float ts_ = planner_open_space_config_.delta_t();

  // load vehicle configuration
  Eigen::MatrixXd ego_(4, 1);
  double front_to_center = vehicle_param_.front_edge_to_center();
  double back_to_center = vehicle_param_.back_edge_to_center();
  double left_to_center = vehicle_param_.left_edge_to_center();
  double right_to_center = vehicle_param_.right_edge_to_center();
  ego_ << front_to_center, right_to_center, back_to_center, left_to_center;

  // result for distance approach problem
  Eigen::MatrixXd l_warm_up;
  Eigen::MatrixXd n_warm_up;

  DualVariableWarmStartProblem* dual_variable_warm_start_ptr =
      new DualVariableWarmStartProblem(planner_open_space_config_);

  if (FLAGS_use_dual_variable_warm_start) {
    bool dual_variable_warm_start_status = dual_variable_warm_start_ptr->Solve(
        horizon_, ts_, ego_, obstacles_ptr->GetObstaclesNum(),
        obstacles_ptr->GetObstaclesEdgesNum(), obstacles_ptr->GetAMatrix(),
        obstacles_ptr->GetbMatrix(), xWS, &l_warm_up, &n_warm_up);

    if (dual_variable_warm_start_status) {
      AINFO << "Dual variable problem solved successfully!";
    } else {
      AERROR << "Dual variable problem solving failed";
      return false;
    }
  } else {
    l_warm_up =
        0.5 * Eigen::MatrixXd::Ones(obstacles_ptr->GetObstaclesEdgesNum().sum(),
                                    horizon_ + 1);
    n_warm_up = 0.5 * Eigen::MatrixXd::Ones(
                          4 * obstacles_ptr->GetObstaclesNum(), horizon_ + 1);
  }

  DistanceApproachProblem* distance_approach_ptr =
      new DistanceApproachProblem(planner_open_space_config_);

  bool status = distance_approach_ptr->Solve(
      x0, xF, last_time_u, horizon_, ts_, ego_, xWS, uWS, l_warm_up, n_warm_up,
      XYbounds_, obstacles_ptr->GetObstaclesNum(),
      obstacles_ptr->GetObstaclesEdgesNum(), obstacles_ptr->GetAMatrix(),
      obstacles_ptr->GetbMatrix(), result_ptr->PrepareStateResult(),
      result_ptr->PrepareControlResult(), result_ptr->PrepareTimeResult(),
      result_ptr->PrepareLResult(), result_ptr->PrepareNResult());
  if (!status) {
    AINFO << "Distance fail";
    return false;
  }
  return true;
}
void DistanceGetResult(ResultContainer* result_ptr,
                       ObstacleContainer* obstacles_ptr, double* x, double* y,
                       double* phi, double* v, double* a, double* steer,
                       double* opt_x, double* opt_y, double* opt_phi,
                       double* opt_v, double* opt_a, double* opt_steer,
                       double* opt_time, double* opt_dual_l, double* opt_dual_n,
                       size_t* output_size) {
  result_ptr->LoadHybridAResult();
  size_t size = result_ptr->GetX()->size();
  size_t size_by_distance = result_ptr->PrepareStateResult()->cols();
  if (size != size_by_distance) {
    AINFO << "sizes by hybrid A and distance approach not consistent";
  }
  std::cout << "return size is " << size << std::endl;
  for (size_t i = 0; i < size; i++) {
    x[i] = result_ptr->GetX()->at(i);
    y[i] = result_ptr->GetY()->at(i);
    phi[i] = result_ptr->GetPhi()->at(i);
    v[i] = result_ptr->GetV()->at(i);
  }
  for (size_t i = 0; i < size - 1; i++) {
    a[i] = result_ptr->GetA()->at(i);
    steer[i] = result_ptr->GetSteer()->at(i);
  }
  output_size[0] = size;

  size_t obstacles_edges_sum = obstacles_ptr->GetObstaclesEdgesNum().sum();
  size_t obstacles_num_to_car = 4 * obstacles_ptr->GetObstaclesNum();
  for (size_t i = 0; i < size_by_distance; i++) {
    opt_x[i] = (*(result_ptr->PrepareStateResult()))(0, i);
    opt_y[i] = (*(result_ptr->PrepareStateResult()))(1, i);
    opt_phi[i] = (*(result_ptr->PrepareStateResult()))(2, i);
    opt_v[i] = (*(result_ptr->PrepareStateResult()))(3, i);
    opt_time[i] = (*(result_ptr->PrepareTimeResult()))(0, i);
    for (size_t j = 0; j < obstacles_edges_sum; j++) {
      opt_dual_l[i * obstacles_edges_sum + j] =
          (*(result_ptr->PrepareLResult()))(j, i);
    }
    for (size_t k = 0; k < obstacles_num_to_car; k++) {
      opt_dual_n[i * obstacles_num_to_car + k] =
          (*(result_ptr->PrepareNResult()))(k, i);
    }
  }

  for (size_t i = 0; i < size_by_distance - 1; i++) {
    opt_a[i] = (*(result_ptr->PrepareControlResult()))(0, i);
    opt_steer[i] = (*(result_ptr->PrepareControlResult()))(1, i);
  }
}
};

}  // namespace planning
}  // namespace apollo
