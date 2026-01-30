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

#include <future>

#include "cyber/common/file.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_open_space/coarse_trajectory_generator/hybrid_a_star.h"
#include "modules/planning/planning_open_space/trajectory_smoother/distance_approach_problem.h"
#include "modules/planning/planning_open_space/trajectory_smoother/dual_variable_warm_start_problem.h"

namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;

class ObstacleContainer {
public:
    ObstacleContainer() = default;

    bool VPresentationObstacle(const double* ROI_distance_approach_parking_boundary) {
        obstacles_num_ = 4;
        obstacles_edges_num_.resize(4, 1);
        obstacles_edges_num_ << 2, 1, 2, 1;
        size_t index = 0;
        for (size_t i = 0; i < obstacles_num_; i++) {
            std::vector<Vec2d> vertices_cw;
            for (int j = 0; j < obstacles_edges_num_(i, 0) + 1; j++) {
                Vec2d vertice
                        = Vec2d(ROI_distance_approach_parking_boundary[index],
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
        // vertices using H-representation
        if (!ObsHRep(obstacles_num_, obstacles_edges_num_, obstacles_vertices_vec_, &obstacles_A_, &obstacles_b_)) {
            AINFO << "Fail to present obstacle in hyperplane";
            return false;
        }
        return true;
    }

    bool ObsHRep(
            const size_t obstacles_num,
            const Eigen::MatrixXi& obstacles_edges_num,
            const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
            Eigen::MatrixXd* A_all,
            Eigen::MatrixXd* b_all) {
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
        if (!(VPresentationObstacle(ROI_distance_approach_parking_boundary) && HPresentationObstacle())) {
            AINFO << "obstacle presentation fails";
        }
    }

    const std::vector<std::vector<Vec2d>>& GetObstacleVec() const {
        return obstacles_vertices_vec_;
    }
    const Eigen::MatrixXd& GetAMatrix() const {
        return obstacles_A_;
    }
    const Eigen::MatrixXd& GetbMatrix() const {
        return obstacles_b_;
    }
    size_t GetObstaclesNum() const {
        return obstacles_num_;
    }
    const Eigen::MatrixXi& GetObstaclesEdgesNum() const {
        return obstacles_edges_num_;
    }

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
    std::vector<double>* GetX() {
        return &x_;
    }
    std::vector<double>* GetY() {
        return &y_;
    }
    std::vector<double>* GetPhi() {
        return &phi_;
    }
    std::vector<double>* GetV() {
        return &v_;
    }
    std::vector<double>* GetA() {
        return &a_;
    }
    std::vector<double>* GetSteer() {
        return &steer_;
    }
    HybridAStartResult* PrepareHybridAResult() {
        return &result_;
    }
    Eigen::MatrixXd* PrepareStateResult() {
        return &state_result_ds_;
    }
    Eigen::MatrixXd* PrepareControlResult() {
        return &control_result_ds_;
    }
    Eigen::MatrixXd* PrepareTimeResult() {
        return &time_result_ds_;
    }
    Eigen::MatrixXd* PrepareLResult() {
        return &dual_l_result_ds_;
    }
    Eigen::MatrixXd* PrepareNResult() {
        return &dual_n_result_ds_;
    }
    double* GetHybridTime() {
        return &hybrid_time_;
    }
    double* GetDualTime() {
        return &dual_time_;
    }
    double* GetIpoptTime() {
        return &ipopt_time_;
    }

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
    double hybrid_time_;
    double dual_time_;
    double ipopt_time_;
};

extern "C" {
HybridAStar* CreateHybridAPtr() {
    apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;
    ACHECK(apollo::cyber::common::GetProtoFromFile(
            FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
            << "Failed to load open space config file " << FLAGS_planner_open_space_config_filename;
    return new HybridAStar(planner_open_space_config_);
}
ObstacleContainer* DistanceCreateObstaclesPtr() {
    return new ObstacleContainer();
}
ResultContainer* DistanceCreateResultPtr() {
    return new ResultContainer();
}

void AddObstacle(ObstacleContainer* obstacles_ptr, const double* ROI_distance_approach_parking_boundary) {
    obstacles_ptr->AddObstacle(ROI_distance_approach_parking_boundary);
}

double InterpolateUsingLinearApproximation(const double p0, const double p1, const double w) {
    return p0 * (1.0 - w) + p1 * w;
}

std::vector<double> VectorLinearInterpolation(const std::vector<double>& x, int extend_size) {
    // interplation example:
    // x: [x0, x1, x2], extend_size: 3
    // output: [y0(x0), y1, y2, y3(x1), y4, y5, y6(x2)]
    size_t origin_last = x.size() - 1;
    std::vector<double> res(origin_last * extend_size + 1, 0.0);

    for (size_t i = 0; i < origin_last * extend_size; ++i) {
        size_t idx0 = i / extend_size;
        size_t idx1 = idx0 + 1;
        double w = static_cast<double>(i % extend_size) / static_cast<double>(extend_size);
        res[i] = InterpolateUsingLinearApproximation(x[idx0], x[idx1], w);
    }

    res.back() = x.back();
    return res;
}

bool DistanceSmoothing(
        const apollo::planning::PlannerOpenSpaceConfig& planner_open_space_config,
        const ObstacleContainer& obstacles,
        double sx,
        double sy,
        double sphi,
        double ex,
        double ey,
        double ephi,
        const std::vector<double>& XYbounds,
        HybridAStartResult* hybrid_a_star_result,
        Eigen::MatrixXd* state_result_ds_,
        Eigen::MatrixXd* control_result_ds_,
        Eigen::MatrixXd* time_result_ds_,
        Eigen::MatrixXd* dual_l_result_ds_,
        Eigen::MatrixXd* dual_n_result_ds_,
        double& dual_time,
        double& ipopt_time) {
    // load Warm Start result(horizon is the "N", not the size of step points)
    size_t horizon_ = hybrid_a_star_result->x.size() - 1;
    // nominal sampling time
    float ts_ = planner_open_space_config.delta_t();

    Eigen::VectorXd x;
    Eigen::VectorXd y;
    Eigen::VectorXd phi;
    Eigen::VectorXd v;
    Eigen::VectorXd steer;
    Eigen::VectorXd a;

    // TODO(Runxin): extend logics in future
    if (horizon_ <= 10 && horizon_ > 2 && planner_open_space_config.enable_linear_interpolation()) {
        // TODO(Runxin): extend this number
        int extend_size = 5;
        // modify state and control vectors sizes
        horizon_ = extend_size * horizon_;
        // modify delta t
        ts_ = ts_ / static_cast<float>(extend_size);

        std::vector<double> x_extend = VectorLinearInterpolation(hybrid_a_star_result->x, extend_size);
        x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(x_extend.data(), horizon_ + 1);

        std::vector<double> y_extend = VectorLinearInterpolation(hybrid_a_star_result->y, extend_size);
        y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(y_extend.data(), horizon_ + 1);

        std::vector<double> phi_extend = VectorLinearInterpolation(hybrid_a_star_result->phi, extend_size);
        phi = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(phi_extend.data(), horizon_ + 1);

        std::vector<double> v_extend = VectorLinearInterpolation(hybrid_a_star_result->v, extend_size);
        v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(v_extend.data(), horizon_ + 1);

        steer = Eigen::VectorXd(horizon_);
        a = Eigen::VectorXd(horizon_);
        for (size_t i = 0; i < static_cast<size_t>(horizon_); ++i) {
            steer[i] = hybrid_a_star_result->steer[i / extend_size];
            a[i] = hybrid_a_star_result->a[i / extend_size];
        }
        ADEBUG << "hybrid A x: ";
        for (size_t i = 0; i < hybrid_a_star_result->x.size(); ++i) {
            ADEBUG << "i: " << i << ", val: " << hybrid_a_star_result->x[i];
        }
        ADEBUG << "interpolated x: \n" << x;

        ADEBUG << "hybrid A steer: ";
        for (size_t i = 0; i < hybrid_a_star_result->steer.size(); ++i) {
            ADEBUG << "i: " << i << ", val: " << hybrid_a_star_result->steer[i];
        }
        ADEBUG << "interpolated steer: \n" << steer;
    } else {
        x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(hybrid_a_star_result->x.data(), horizon_ + 1);
        y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(hybrid_a_star_result->y.data(), horizon_ + 1);
        phi = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(hybrid_a_star_result->phi.data(), horizon_ + 1);
        v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(hybrid_a_star_result->v.data(), horizon_ + 1);
        steer = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(hybrid_a_star_result->steer.data(), horizon_);
        a = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(hybrid_a_star_result->a.data(), horizon_);
    }

    Eigen::MatrixXd xWS = Eigen::MatrixXd::Zero(4, horizon_ + 1);
    Eigen::MatrixXd uWS = Eigen::MatrixXd::Zero(2, horizon_);
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

    common::VehicleParam vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();

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
    Eigen::MatrixXd s_warm_up = Eigen::MatrixXd::Zero(obstacles.GetObstaclesNum(), horizon_ + 1);

    DualVariableWarmStartProblem* dual_variable_warm_start_ptr
            = new DualVariableWarmStartProblem(planner_open_space_config);

    const auto t1 = std::chrono::system_clock::now();
    if (FLAGS_use_dual_variable_warm_start) {
        bool dual_variable_warm_start_status = dual_variable_warm_start_ptr->Solve(
                horizon_,
                ts_,
                ego_,
                obstacles.GetObstaclesNum(),
                obstacles.GetObstaclesEdgesNum(),
                obstacles.GetAMatrix(),
                obstacles.GetbMatrix(),
                xWS,
                &l_warm_up,
                &n_warm_up,
                &s_warm_up);

        if (dual_variable_warm_start_status) {
            AINFO << "Dual variable problem solved successfully!";
        } else {
            AERROR << "Dual variable problem solving failed";
            return false;
        }
    } else {
        l_warm_up = Eigen::MatrixXd::Zero(obstacles.GetObstaclesEdgesNum().sum(), horizon_ + 1);
        n_warm_up = Eigen::MatrixXd::Zero(4 * obstacles.GetObstaclesNum(), horizon_ + 1);
    }
    const auto t2 = std::chrono::system_clock::now();
    dual_time = std::chrono::duration<double>(t2 - t1).count() * 1000;

    DistanceApproachProblem* distance_approach_ptr = new DistanceApproachProblem(planner_open_space_config);

    bool status = distance_approach_ptr->Solve(
            x0,
            xF,
            last_time_u,
            horizon_,
            ts_,
            ego_,
            xWS,
            uWS,
            l_warm_up,
            n_warm_up,
            s_warm_up,
            XYbounds,
            obstacles.GetObstaclesNum(),
            obstacles.GetObstaclesEdgesNum(),
            obstacles.GetAMatrix(),
            obstacles.GetbMatrix(),
            state_result_ds_,
            control_result_ds_,
            time_result_ds_,
            dual_l_result_ds_,
            dual_n_result_ds_);
    const auto t3 = std::chrono::system_clock::now();
    ipopt_time = std::chrono::duration<double>(t3 - t2).count() * 1000;

    if (!status) {
        AERROR << "Distance fail";
        return false;
    }
    return true;
}

bool DistancePlan(
        HybridAStar* hybridA_ptr,
        ObstacleContainer* obstacles_ptr,
        ResultContainer* result_ptr,
        double sx,
        double sy,
        double sphi,
        double ex,
        double ey,
        double ephi,
        double* XYbounds) {
    apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;
    ACHECK(apollo::cyber::common::GetProtoFromFile(
            FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
            << "Failed to load open space config file " << FLAGS_planner_open_space_config_filename;
    AINFO << "FLAGS_planner_open_space_config_filename: " << FLAGS_planner_open_space_config_filename;

    double hybrid_total = 0.0;
    double dual_total = 0.0;
    double ipopt_total = 0.0;

    std::string flag_file_path = "/apollo/modules/planning/planning_component/conf/planning.conf";
    google::SetCommandLineOption("flagfile", flag_file_path.c_str());

    HybridAStartResult hybrid_astar_result;
    std::vector<double> XYbounds_(XYbounds, XYbounds + 4);
    std::vector<std::vector<common::math::Vec2d>> soft_boundary_vertices_vec;

    const auto start_timestamp = std::chrono::system_clock::now();
    if (!hybridA_ptr->Plan(
                sx,
                sy,
                sphi,
                ex,
                ey,
                ephi,
                XYbounds_,
                obstacles_ptr->GetObstacleVec(),
                &hybrid_astar_result,
                soft_boundary_vertices_vec,
                false)) {
        AINFO << "Hybrid A Star fails";
        return false;
    }
    const auto end_timestamp = std::chrono::system_clock::now();
    std::chrono::duration<double> time_diff = end_timestamp - start_timestamp;
    hybrid_total = time_diff.count() * 1000;

    if (FLAGS_enable_parallel_trajectory_smoothing) {
        std::vector<HybridAStartResult> partition_trajectories;
        if (!hybridA_ptr->TrajectoryPartition(hybrid_astar_result, &partition_trajectories)) {
            AERROR << "TrajectoryPartition failed";
            *(result_ptr->PrepareHybridAResult()) = hybrid_astar_result;
            return false;
        }
        size_t size = partition_trajectories.size();
        std::vector<Eigen::MatrixXd> state_result_ds_vec;
        std::vector<Eigen::MatrixXd> control_result_ds_vec;
        std::vector<Eigen::MatrixXd> time_result_ds_vec;
        std::vector<Eigen::MatrixXd> dual_l_result_ds_vec;
        std::vector<Eigen::MatrixXd> dual_n_result_ds_vec;
        state_result_ds_vec.resize(size);
        control_result_ds_vec.resize(size);
        time_result_ds_vec.resize(size);
        dual_l_result_ds_vec.resize(size);
        dual_n_result_ds_vec.resize(size);
        std::vector<std::future<bool>> results;

        // In parallel
        // TODO(Jinyun): fix memory issue
        // for (size_t i = 0; i < size; ++i) {
        //   double piece_wise_sx = partition_trajectories[i].x.front();
        //   double piece_wise_sy = partition_trajectories[i].y.front();
        //   double piece_wise_sphi = partition_trajectories[i].phi.front();
        //   double piece_wise_ex = partition_trajectories[i].x.back();
        //   double piece_wise_ey = partition_trajectories[i].y.back();
        //   double piece_wise_ephi = partition_trajectories[i].phi.back();
        //   auto* ith_trajectories = &partition_trajectories[i];
        //   auto* ith_state_result = &state_result_ds_vec[i];
        //   auto* ith_control_result = &control_result_ds_vec[i];
        //   auto* ith_time_result = &time_result_ds_vec[i];
        //   auto* ith_dual_l_result = &dual_l_result_ds_vec[i];
        //   auto* ith_dual_n_result = &dual_n_result_ds_vec[i];
        //   results.push_back(
        //       cyber::Async(&DistanceSmoothing,
        //       std::ref(planner_open_space_config_),
        //                    std::ref(*obstacles_ptr), piece_wise_sx,
        //                    piece_wise_sy, piece_wise_sphi, piece_wise_ex,
        //                    piece_wise_ey, piece_wise_ephi, std::ref(XYbounds_),
        //                    ith_trajectories, ith_state_result,
        //                    ith_control_result, ith_time_result,
        //                    ith_dual_l_result, ith_dual_n_result));
        // }
        // for (auto& result : results) {
        //   if (!result.get()) {
        //     AERROR << "Failure in a piece of trajectory.";
        //     return false;
        //   }
        // }

        // In for loop
        double dual_tmp = 0.0;
        double ipopt_tmp = 0.0;
        for (size_t i = 0; i < size; ++i) {
            double piece_wise_sx = partition_trajectories[i].x.front();
            double piece_wise_sy = partition_trajectories[i].y.front();
            double piece_wise_sphi = partition_trajectories[i].phi.front();
            double piece_wise_ex = partition_trajectories[i].x.back();
            double piece_wise_ey = partition_trajectories[i].y.back();
            double piece_wise_ephi = partition_trajectories[i].phi.back();

            if (!DistanceSmoothing(
                        planner_open_space_config_,
                        *obstacles_ptr,
                        piece_wise_sx,
                        piece_wise_sy,
                        piece_wise_sphi,
                        piece_wise_ex,
                        piece_wise_ey,
                        piece_wise_ephi,
                        XYbounds_,
                        &partition_trajectories[i],
                        &state_result_ds_vec[i],
                        &control_result_ds_vec[i],
                        &time_result_ds_vec[i],
                        &dual_l_result_ds_vec[i],
                        &dual_n_result_ds_vec[i],
                        dual_tmp,
                        ipopt_tmp)) {
                *(result_ptr->PrepareHybridAResult()) = hybrid_astar_result;
                return false;
            }
            dual_total += dual_tmp;
            ipopt_total += ipopt_tmp;
        }

        // Retrieve result in one single trajectory
        size_t trajectory_point_size = 0;
        for (size_t i = 0; i < size; ++i) {
            if (state_result_ds_vec[i].cols() < 2) {
                AERROR << "state horizon smaller than 2";
                return false;
            }
            AINFO << "trajectory idx: "
                  << "idx range: " << trajectory_point_size << ", "
                  << trajectory_point_size + static_cast<size_t>(state_result_ds_vec[i].cols()) - 1;
            trajectory_point_size += static_cast<size_t>(state_result_ds_vec[i].cols()) - 1;
        }
        ++trajectory_point_size;

        const uint64_t state_dimension = state_result_ds_vec.front().rows();
        Eigen::MatrixXd state_result_ds;
        state_result_ds.resize(state_dimension, trajectory_point_size);
        uint64_t k = 0;
        for (size_t i = 0; i < size; ++i) {
            // leave out the last repeated point so set column minus one
            uint64_t state_col_num = state_result_ds_vec[i].cols() - 1;
            for (uint64_t j = 0; j < state_col_num; ++j) {
                state_result_ds.col(k) = state_result_ds_vec[i].col(j);
                ++k;
            }
        }
        state_result_ds.col(k) = state_result_ds_vec.back().col(state_result_ds_vec.back().cols() - 1);

        const uint64_t control_dimension = control_result_ds_vec.front().rows();
        Eigen::MatrixXd control_result_ds;
        control_result_ds.resize(control_dimension, trajectory_point_size - 1);
        k = 0;

        for (size_t i = 0; i < size; ++i) {
            uint64_t control_col_num = control_result_ds_vec[i].cols() - 1;
            for (uint64_t j = 0; j < control_col_num; ++j) {
                control_result_ds.col(k) = control_result_ds_vec[i].col(j);
                ++k;
            }
        }

        const uint64_t time_dimension = time_result_ds_vec.front().rows();
        Eigen::MatrixXd time_result_ds;
        time_result_ds.resize(time_dimension, trajectory_point_size - 1);
        k = 0;
        for (size_t i = 0; i < size; ++i) {
            uint64_t time_col_num = time_result_ds_vec[i].cols() - 1;
            for (uint64_t j = 0; j < time_col_num; ++j) {
                time_result_ds.col(k) = time_result_ds_vec[i].col(j);
                ++k;
            }
        }

        *(result_ptr->PrepareHybridAResult()) = hybrid_astar_result;
        *(result_ptr->PrepareStateResult()) = state_result_ds;
        *(result_ptr->PrepareControlResult()) = control_result_ds;
        *(result_ptr->PrepareTimeResult()) = time_result_ds;
        *(result_ptr->GetHybridTime()) = hybrid_total;
        *(result_ptr->GetDualTime()) = dual_total;
        *(result_ptr->GetIpoptTime()) = ipopt_total;
    } else {
        Eigen::MatrixXd state_result_ds;
        Eigen::MatrixXd control_result_ds;
        Eigen::MatrixXd time_result_ds;
        Eigen::MatrixXd dual_l_result_ds;
        Eigen::MatrixXd dual_n_result_ds;
        if (!DistanceSmoothing(
                    planner_open_space_config_,
                    *obstacles_ptr,
                    sx,
                    sy,
                    sphi,
                    ex,
                    ey,
                    ephi,
                    XYbounds_,
                    &hybrid_astar_result,
                    &state_result_ds,
                    &control_result_ds,
                    &time_result_ds,
                    &dual_l_result_ds,
                    &dual_n_result_ds,
                    dual_total,
                    ipopt_total)) {
            AERROR << "DistanceSmoothing failed";
            *(result_ptr->PrepareHybridAResult()) = hybrid_astar_result;
            return false;
        }
        *(result_ptr->PrepareHybridAResult()) = hybrid_astar_result;
        *(result_ptr->PrepareStateResult()) = state_result_ds;
        *(result_ptr->PrepareControlResult()) = control_result_ds;
        *(result_ptr->PrepareTimeResult()) = time_result_ds;
        *(result_ptr->PrepareLResult()) = dual_l_result_ds;
        *(result_ptr->PrepareNResult()) = dual_n_result_ds;
        *(result_ptr->GetHybridTime()) = hybrid_total;
        *(result_ptr->GetDualTime()) = dual_total;
        *(result_ptr->GetIpoptTime()) = ipopt_total;
    }
    return true;
}

void DistanceGetResult(
        ResultContainer* result_ptr,
        ObstacleContainer* obstacles_ptr,
        double* x,
        double* y,
        double* phi,
        double* v,
        double* a,
        double* steer,
        double* opt_x,
        double* opt_y,
        double* opt_phi,
        double* opt_v,
        double* opt_a,
        double* opt_steer,
        double* opt_time,
        double* opt_dual_l,
        double* opt_dual_n,
        size_t* output_size,
        double* hybrid_time,
        double* dual_time,
        double* ipopt_time) {
    result_ptr->LoadHybridAResult();
    size_t size = result_ptr->GetX()->size();
    size_t size_by_distance = result_ptr->PrepareStateResult()->cols();
    AERROR_IF(size != size_by_distance) << "sizes by hybrid A and distance approach not consistent";
    for (size_t i = 0; i < size; ++i) {
        x[i] = result_ptr->GetX()->at(i);
        y[i] = result_ptr->GetY()->at(i);
        phi[i] = result_ptr->GetPhi()->at(i);
        v[i] = result_ptr->GetV()->at(i);
    }
    for (size_t i = 0; i + 1 < size; ++i) {
        a[i] = result_ptr->GetA()->at(i);
        steer[i] = result_ptr->GetSteer()->at(i);
    }
    output_size[0] = size;

    size_t obstacles_edges_sum = obstacles_ptr->GetObstaclesEdgesNum().sum();
    size_t obstacles_num_to_car = 4 * obstacles_ptr->GetObstaclesNum();
    for (size_t i = 0; i < size_by_distance; ++i) {
        opt_x[i] = (*(result_ptr->PrepareStateResult()))(0, i);
        opt_y[i] = (*(result_ptr->PrepareStateResult()))(1, i);
        opt_phi[i] = (*(result_ptr->PrepareStateResult()))(2, i);
        opt_v[i] = (*(result_ptr->PrepareStateResult()))(3, i);
    }

    if (result_ptr->PrepareTimeResult() != 0) {
        for (size_t i = 0; i + 1 < size_by_distance; ++i) {
            opt_time[i] = (*(result_ptr->PrepareTimeResult()))(0, i);
        }
    }

    if (result_ptr->PrepareLResult()->cols() != 0 && result_ptr->PrepareNResult() != 0) {
        for (size_t i = 0; i + 1 < size_by_distance; ++i) {
            for (size_t j = 0; j < obstacles_edges_sum; ++j) {
                opt_dual_l[i * obstacles_edges_sum + j] = (*(result_ptr->PrepareLResult()))(j, i);
            }
            for (size_t k = 0; k < obstacles_num_to_car; ++k) {
                opt_dual_n[i * obstacles_num_to_car + k] = (*(result_ptr->PrepareNResult()))(k, i);
            }
        }
    }
    for (size_t i = 0; i + 1 < size_by_distance; ++i) {
        opt_a[i] = (*(result_ptr->PrepareControlResult()))(1, i);
        opt_steer[i] = (*(result_ptr->PrepareControlResult()))(0, i);
    }

    hybrid_time[0] = *(result_ptr->GetHybridTime());
    dual_time[0] = *(result_ptr->GetDualTime());
    ipopt_time[0] = *(result_ptr->GetIpoptTime());
}
};

}  // namespace planning
}  // namespace apollo
