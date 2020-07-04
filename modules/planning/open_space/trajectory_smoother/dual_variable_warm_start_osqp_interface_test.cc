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
#include "modules/planning/open_space/trajectory_smoother/dual_variable_warm_start_osqp_interface.h"

#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/open_space/trajectory_smoother/dual_variable_warm_start_ipopt_qp_interface.h"

namespace apollo {
namespace planning {

class DualVariableWarmStartOSQPInterfaceTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    FLAGS_planner_open_space_config_filename =
        "/apollo/modules/planning/testdata/conf/"
        "open_space_standard_parking_lot.pb.txt";

    ACHECK(apollo::cyber::common::GetProtoFromFile(
        FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
        << "Failed to load open space config file "
        << FLAGS_planner_open_space_config_filename;

    ProblemSetup();
  }

 protected:
  void ProblemSetup();

 protected:
  size_t horizon_ = 5;
  size_t obstacles_num_ = 4;
  double ts_ = 0.1;
  Eigen::MatrixXd ego_ = Eigen::MatrixXd::Ones(4, 1);
  Eigen::MatrixXd last_time_u_ = Eigen::MatrixXd::Zero(2, 1);
  Eigen::MatrixXi obstacles_edges_num_;
  Eigen::MatrixXd obstacles_A_ = Eigen::MatrixXd::Ones(10, 2);
  Eigen::MatrixXd obstacles_b_ = Eigen::MatrixXd::Ones(10, 1);
  int num_of_variables_ = 0;
  double rx_ = 0.0;
  double ry_ = 0.0;
  double r_yaw_ = 0.0;

  int num_of_constraints_ = 0;

  std::unique_ptr<DualVariableWarmStartOSQPInterface> ptop_ = nullptr;
  DualVariableWarmStartIPOPTQPInterface* pt_gt_ = nullptr;
  PlannerOpenSpaceConfig planner_open_space_config_;
};

void DualVariableWarmStartOSQPInterfaceTest::ProblemSetup() {
  obstacles_edges_num_ = Eigen::MatrixXi(obstacles_num_, 1);
  obstacles_edges_num_ << 2, 1, 2, 1;
  Eigen::MatrixXd xWS = Eigen::MatrixXd::Ones(4, horizon_ + 1);
  ptop_.reset(new DualVariableWarmStartOSQPInterface(
      horizon_, ts_, ego_, obstacles_edges_num_, obstacles_num_, obstacles_A_,
      obstacles_b_, xWS, planner_open_space_config_));
  pt_gt_ = new DualVariableWarmStartIPOPTQPInterface(
      horizon_, ts_, ego_, obstacles_edges_num_, obstacles_num_, obstacles_A_,
      obstacles_b_, xWS, planner_open_space_config_);
}

TEST_F(DualVariableWarmStartOSQPInterfaceTest, initilization) {
  EXPECT_NE(ptop_, nullptr);
}

TEST_F(DualVariableWarmStartOSQPInterfaceTest, optimize) {
  int obstacles_edges_sum = obstacles_edges_num_.sum();
  Eigen::MatrixXd l_warm_up(obstacles_edges_sum, horizon_ + 1);
  Eigen::MatrixXd n_warm_up(4 * obstacles_num_, horizon_ + 1);

  bool res = ptop_->optimize();
  EXPECT_TRUE(res);
  ptop_->get_optimization_results(&l_warm_up, &n_warm_up);

  // regard ipopt_qp as ground truth result
  Eigen::MatrixXd l_warm_up_gt(obstacles_edges_sum, horizon_ + 1);
  Eigen::MatrixXd n_warm_up_gt(4 * obstacles_num_, horizon_ + 1);

  Ipopt::SmartPtr<Ipopt::TNLP> problem = pt_gt_;
  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  auto ipopt_config_tmp =
      planner_open_space_config_.dual_variable_warm_start_config()
          .ipopt_config();

  app->Options()->SetIntegerValue("print_level",
                                  ipopt_config_tmp.ipopt_print_level());
  app->Options()->SetIntegerValue("mumps_mem_percent",
                                  ipopt_config_tmp.mumps_mem_percent());
  app->Options()->SetNumericValue("mumps_pivtol",
                                  ipopt_config_tmp.mumps_pivtol());
  app->Options()->SetIntegerValue("max_iter",
                                  ipopt_config_tmp.ipopt_max_iter());
  app->Options()->SetNumericValue("tol", ipopt_config_tmp.ipopt_tol());
  app->Options()->SetNumericValue(
      "acceptable_constr_viol_tol",
      ipopt_config_tmp.ipopt_acceptable_constr_viol_tol());
  app->Options()->SetNumericValue(
      "min_hessian_perturbation",
      ipopt_config_tmp.ipopt_min_hessian_perturbation());
  app->Options()->SetNumericValue(
      "jacobian_regularization_value",
      ipopt_config_tmp.ipopt_jacobian_regularization_value());
  app->Options()->SetStringValue(
      "print_timing_statistics",
      ipopt_config_tmp.ipopt_print_timing_statistics());
  app->Options()->SetStringValue("alpha_for_y",
                                 ipopt_config_tmp.ipopt_alpha_for_y());
  app->Options()->SetStringValue("recalc_y", ipopt_config_tmp.ipopt_recalc_y());
  app->Options()->SetStringValue("mehrotra_algorithm", "yes");

  app->Initialize();
  app->OptimizeTNLP(problem);

  // Retrieve some statistics about the solve
  pt_gt_->get_optimization_results(&l_warm_up_gt, &n_warm_up_gt);

  // compare ipopt_qp and osqp results
  for (int r = 0; r < obstacles_edges_sum; ++r) {
    for (int c = 0; c < static_cast<int>(horizon_) + 1; ++c) {
      EXPECT_NEAR(l_warm_up(r, c), l_warm_up_gt(r, c), 1e-6)
          << "r: " << r << ", c: " << c;
    }
  }
  for (int r = 0; r < 4 * static_cast<int>(obstacles_num_); ++r) {
    for (int c = 0; c < static_cast<int>(horizon_) + 1; ++c) {
      EXPECT_NEAR(n_warm_up(r, c), n_warm_up_gt(r, c), 1e-6)
          << "r: " << r << ",c: " << c;
    }
  }
}
}  // namespace planning
}  // namespace apollo
