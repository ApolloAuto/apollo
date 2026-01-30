/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include "modules/planning/planning_open_space/trajectory_smoother/distance_approach_ipopt_cuda_interface.h"

#include "gtest/gtest.h"

#include "cyber/common/file.h"

namespace apollo {
namespace planning {

class DistanceApproachIPOPTCUDAInterfaceTest : public ::testing::Test {
public:
    virtual void SetUp() {
        FLAGS_planner_open_space_config_filename
                = "/apollo/modules/planning/planning_base/testdata/conf/"
                  "open_space_standard_parking_lot.pb.txt";
        ACHECK(apollo::cyber::common::GetProtoFromFile(
                FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
                << "Failed to load open space config file " << FLAGS_planner_open_space_config_filename;

        distance_approach_config_ = planner_open_space_config_.distance_approach_config();

        ProblemSetup();
    }

protected:
    void ProblemSetup();

protected:
    size_t horizon_ = 43;
    size_t obstacles_num_ = 4;
    double ts_ = 0.5;
    Eigen::MatrixXd ego_ = Eigen::MatrixXd::Ones(4, 1);
    Eigen::MatrixXd x0_ = Eigen::MatrixXd::Ones(4, 1);
    Eigen::MatrixXd xf_ = 10 * Eigen::MatrixXd::Ones(4, 1);
    Eigen::MatrixXd last_time_u_ = Eigen::MatrixXd::Zero(2, 1);
    std::vector<double> XYbounds_ = {1.0, 1.0, 1.0, 1.0};
    Eigen::MatrixXd xWS_ = Eigen::MatrixXd::Ones(4, 44);
    Eigen::MatrixXd uWS_ = Eigen::MatrixXd::Ones(2, 43);
    Eigen::MatrixXi obstacles_edges_num_;  // {2, 1, 2, 1}
    size_t obstacles_edges_sum_;
    Eigen::MatrixXd obstacles_A_ = Eigen::MatrixXd::Ones(6, 2);
    Eigen::MatrixXd obstacles_b_ = Eigen::MatrixXd::Ones(6, 1);
    bool use_fix_time_ = false;
    std::unique_ptr<DistanceApproachIPOPTCUDAInterface> ptop_ = nullptr;
    PlannerOpenSpaceConfig planner_open_space_config_;
    DistanceApproachConfig distance_approach_config_;
};

void DistanceApproachIPOPTCUDAInterfaceTest::ProblemSetup() {
    // obstacles_edges_num_ = 4 * Eigen::MatrixXi::Ones(obstacles_num_, 1);
    obstacles_edges_num_ = Eigen::MatrixXi(obstacles_num_, 1);
    obstacles_edges_num_ << 2, 1, 2, 1;
    obstacles_edges_sum_ = obstacles_edges_num_.sum();
    Eigen::MatrixXd l_warm_up_ = Eigen::MatrixXd::Ones(obstacles_edges_sum_, horizon_ + 1);
    Eigen::MatrixXd n_warm_up_ = Eigen::MatrixXd::Ones(4 * obstacles_num_, horizon_ + 1);
    ptop_.reset(new DistanceApproachIPOPTCUDAInterface(
            horizon_,
            ts_,
            ego_,
            xWS_,
            uWS_,
            l_warm_up_,
            n_warm_up_,
            x0_,
            xf_,
            last_time_u_,
            XYbounds_,
            obstacles_edges_num_,
            obstacles_num_,
            obstacles_A_,
            obstacles_b_,
            planner_open_space_config_));
}

TEST_F(DistanceApproachIPOPTCUDAInterfaceTest, initilization) {
    EXPECT_NE(ptop_, nullptr);
}

TEST_F(DistanceApproachIPOPTCUDAInterfaceTest, get_bounds_info) {
    int n = 1274;
    int m = 2194;
    double x_l[1274];
    double x_u[1274];
    double g_l[2194];
    double g_u[2194];
    bool res = ptop_->get_bounds_info(n, x_l, x_u, m, g_l, g_u);
    EXPECT_TRUE(res);
}

TEST_F(DistanceApproachIPOPTCUDAInterfaceTest, get_starting_point) {
    int n = 1274;
    int m = 2194;
    bool init_x = true;
    bool init_z = false;
    bool init_lambda = false;
    double x[1274];
    double z_L[2194];
    double z_U[2194];
    double lambda[2194];
    bool res = ptop_->get_starting_point(n, init_x, x, init_z, z_L, z_U, m, init_lambda, lambda);
    EXPECT_TRUE(res);
}

TEST_F(DistanceApproachIPOPTCUDAInterfaceTest, eval_f) {
    int n = 1274;
    double obj_value = 0.0;
    double x[1274];
    std::fill_n(x, n, 1.2);
    bool res = ptop_->eval_f(n, x, true, obj_value);
    EXPECT_DOUBLE_EQ(obj_value, 1443.3600000000008) << "eval_f: " << obj_value;
    EXPECT_TRUE(res);
}

TEST_F(DistanceApproachIPOPTCUDAInterfaceTest, eval_jac_g_par) {
    int n = 1274;
    int m = 2194;
    int kNnzJac = 0;
    int kNnzhss = 0;
    Ipopt::TNLP::IndexStyleEnum index_style;
    // to-do: check get_nlp_info
    bool res = ptop_->get_nlp_info(n, m, kNnzJac, kNnzhss, index_style);
    EXPECT_TRUE(res);

    double x[1274];
    std::fill_n(x, n, 1.2);
    int new_x = 0;
    // to-do: check eval_jac_g_ser
    int i_row_ser[kNnzJac];
    int j_col_ser[kNnzJac];
    double values_ser[kNnzJac];
    std::fill_n(values_ser, n, 0.0);
    bool res2 = ptop_->eval_jac_g_ser(n, x, new_x, m, kNnzJac, i_row_ser, j_col_ser, nullptr);
    EXPECT_TRUE(res2);
    bool res3 = ptop_->eval_jac_g_ser(n, x, new_x, m, kNnzJac, i_row_ser, j_col_ser, values_ser);
    EXPECT_TRUE(res3);

    int i_row_par[kNnzJac];
    int j_col_par[kNnzJac];
    double values_par[kNnzJac];
    std::fill_n(values_par, n, 0.0);
    bool res4 = ptop_->eval_jac_g_par(n, x, new_x, m, kNnzJac, i_row_par, j_col_par, nullptr);
    EXPECT_TRUE(res4);
    for (int i = 0; i < kNnzJac; ++i) {
        EXPECT_EQ(i_row_ser[i], i_row_par[i]) << "iRow differ at index " << i;
        EXPECT_EQ(j_col_ser[i], j_col_par[i]) << "iCol differ at index " << i;
    }

    bool res5 = ptop_->eval_jac_g_par(n, x, new_x, m, kNnzJac, i_row_par, j_col_par, values_par);
    EXPECT_TRUE(res5);
    for (int i = 0; i < kNnzJac; ++i) {
        EXPECT_EQ(values_ser[i], values_par[i]) << "values differ at index " << i;
    }
}

TEST_F(DistanceApproachIPOPTCUDAInterfaceTest, eval_grad_f_hand) {
    int n = 1274;
    double x[1274];
    std::fill_n(x, n, 1.2);
    bool new_x = false;

    double grad_f_adolc[1274];
    double grad_f_hand[1274];
    std::fill_n(grad_f_adolc, n, 0.0);
    std::fill_n(grad_f_hand, n, 0.0);

    bool res = ptop_->eval_grad_f(n, x, new_x, grad_f_adolc);
    EXPECT_TRUE(res);
    bool res2 = ptop_->eval_grad_f_hand(n, x, new_x, grad_f_hand);
    EXPECT_TRUE(res2);

    for (int i = 0; i < n; ++i) {
        EXPECT_EQ(grad_f_adolc[i], grad_f_hand[i]) << "grad_f differ at index " << i;
    }
}
}  // namespace planning
}  // namespace apollo
