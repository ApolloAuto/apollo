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

#include "modules/planning/planning_open_space/coarse_trajectory_generator/hybrid_a_star.h"

#include <limits>
#include <unordered_set>

#include "modules/planning/planning_base/common/path/discretized_path.h"
#include "modules/planning/planning_base/common/speed/speed_data.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_speed_problem.h"

namespace apollo {
namespace planning {

using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;

HybridAStar::HybridAStar(const PlannerOpenSpaceConfig& open_space_conf) {
    planner_open_space_config_.CopyFrom(open_space_conf);
    reed_shepp_generator_ = std::make_unique<ReedShepp>(vehicle_param_, planner_open_space_config_);
    grid_a_star_heuristic_generator_ = std::make_unique<GridSearch>(planner_open_space_config_);
    next_node_num_ = planner_open_space_config_.warm_start_config().next_node_num();
    max_steer_angle_ = vehicle_param_.max_steer_angle() / vehicle_param_.steer_ratio()
            * planner_open_space_config_.warm_start_config().traj_kappa_contraint_ratio();
    step_size_ = planner_open_space_config_.warm_start_config().step_size();
    xy_grid_resolution_ = planner_open_space_config_.warm_start_config().xy_grid_resolution();
    arc_length_ = planner_open_space_config_.warm_start_config().phi_grid_resolution() * vehicle_param_.wheel_base()
            / std::tan(max_steer_angle_ * 2 / (next_node_num_ / 2 - 1));
    if (arc_length_ < std::sqrt(2) * xy_grid_resolution_) {
        arc_length_ = std::sqrt(2) * xy_grid_resolution_;
    }
    AINFO << "arc_length" << arc_length_;
    delta_t_ = planner_open_space_config_.delta_t();
    traj_forward_penalty_ = planner_open_space_config_.warm_start_config().traj_forward_penalty();
    traj_back_penalty_ = planner_open_space_config_.warm_start_config().traj_back_penalty();
    traj_gear_switch_penalty_ = planner_open_space_config_.warm_start_config().traj_gear_switch_penalty();
    traj_steer_penalty_ = planner_open_space_config_.warm_start_config().traj_steer_penalty();
    traj_steer_change_penalty_ = planner_open_space_config_.warm_start_config().traj_steer_change_penalty();
    acc_weight_ = planner_open_space_config_.iterative_anchoring_smoother_config().s_curve_config().acc_weight();
    jerk_weight_ = planner_open_space_config_.iterative_anchoring_smoother_config().s_curve_config().jerk_weight();
    kappa_penalty_weight_
            = planner_open_space_config_.iterative_anchoring_smoother_config().s_curve_config().kappa_penalty_weight();
    ref_s_weight_ = planner_open_space_config_.iterative_anchoring_smoother_config().s_curve_config().ref_s_weight();
    ref_v_weight_ = planner_open_space_config_.iterative_anchoring_smoother_config().s_curve_config().ref_v_weight();
    max_forward_v_ = planner_open_space_config_.iterative_anchoring_smoother_config().max_forward_v();
    max_reverse_v_ = planner_open_space_config_.iterative_anchoring_smoother_config().max_reverse_v();
    max_forward_acc_ = planner_open_space_config_.iterative_anchoring_smoother_config().max_forward_acc();
    max_reverse_acc_ = planner_open_space_config_.iterative_anchoring_smoother_config().max_reverse_acc();
    max_acc_jerk_ = planner_open_space_config_.iterative_anchoring_smoother_config().max_acc_jerk();
}

HybridAStar::HybridAStar(const WarmStartConfig& warm_start_conf) {
    planner_warm_start_config_.CopyFrom(warm_start_conf);
    reed_shepp_generator_ = std::make_unique<ReedShepp>(vehicle_param_, planner_warm_start_config_);
    grid_a_star_heuristic_generator_ = std::make_unique<GridSearch>(planner_warm_start_config_);
    next_node_num_ = planner_warm_start_config_.next_node_num();
    max_steer_angle_ = vehicle_param_.max_steer_angle() / vehicle_param_.steer_ratio()
            * planner_warm_start_config_.traj_kappa_contraint_ratio();
    max_kappa_ = std::tan(max_steer_angle_) / vehicle_param_.wheel_base();
    step_size_ = planner_warm_start_config_.step_size();
    xy_grid_resolution_ = planner_warm_start_config_.xy_grid_resolution();
    arc_length_ = std::sqrt(2) * xy_grid_resolution_;
    AINFO << "arc_length" << arc_length_;
    delta_t_ = FLAGS_open_space_delta_t;
    traj_forward_penalty_ = planner_warm_start_config_.traj_forward_penalty();
    traj_back_penalty_ = planner_warm_start_config_.traj_back_penalty();
    traj_gear_switch_penalty_ = planner_warm_start_config_.traj_gear_switch_penalty();
    traj_steer_penalty_ = planner_warm_start_config_.traj_steer_penalty();
    traj_steer_change_penalty_ = planner_warm_start_config_.traj_steer_change_penalty();
    soft_boundary_penalty_ = planner_warm_start_config_.soft_boundary_penalty();
    acc_weight_ = FLAGS_open_space_acc_weight;
    jerk_weight_ = FLAGS_open_space_jerk_weight;
    kappa_penalty_weight_ = FLAGS_open_space_kappa_weight;
    ref_s_weight_ = FLAGS_open_space_reference_s_weight;
    ref_v_weight_ = FLAGS_open_space_reference_v_weight;
    max_forward_v_ = FLAGS_open_space_max_forward_v;
    max_reverse_v_ = FLAGS_open_space_max_reverse_v;
    max_forward_acc_ = FLAGS_open_space_max_forward_acc;
    max_reverse_acc_ = FLAGS_open_space_max_reverse_acc;
    max_acc_jerk_ = FLAGS_open_space_max_jerk;
    traj_expected_shortest_length_ = planner_warm_start_config_.traj_expected_shortest_length();
    traj_short_length_penalty_ = planner_warm_start_config_.traj_short_length_penalty();
}

bool HybridAStar::AnalyticExpansion(
        std::shared_ptr<Node3d> current_node,
        std::shared_ptr<Node3d>* candidate_final_node) {
    std::shared_ptr<ReedSheppPath> reeds_shepp_to_check = std::make_shared<ReedSheppPath>();
    if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_, reeds_shepp_to_check)) {
        return false;
    }
    if (!RSPCheck(reeds_shepp_to_check)) {
        return false;
    }
    // load the whole RSP as nodes and add to the close set
    *candidate_final_node = LoadRSPinCS(reeds_shepp_to_check, current_node);
    return true;
}

bool HybridAStar::RSPCheck(const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end) {
    std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
            reeds_shepp_to_end->x,
            reeds_shepp_to_end->y,
            reeds_shepp_to_end->phi,
            XYbounds_,
            planner_warm_start_config_));
    return ValidityCheck(node) && RSPLengthCheck(reeds_shepp_to_end);
}

bool HybridAStar::RSPLengthCheck(const std::shared_ptr<ReedSheppPath> reeds_shepp_path) {
    for (int i = 0; i < reeds_shepp_path->segs_lengths.size(); i++) {
        if (reeds_shepp_path->segs_types[i] != 'S') {
            double dist = std::fabs(reeds_shepp_path->segs_lengths[i]) / max_kappa_ * reeds_shepp_path->radius;
            if (dist < 0.1) {
                return false;
            }
        } else {
            double dist = std::fabs(reeds_shepp_path->segs_lengths[i]) / max_kappa_;
            if (dist < 0.1) {
                return false;
            }
        }
    }
    return true;
}

bool HybridAStar::ValidityCheck(std::shared_ptr<Node3d> node) {
    CHECK_NOTNULL(node);
    CHECK_GT(node->GetStepSize(), 0U);

    if (obstacles_linesegments_vec_.empty()) {
        return true;
    }

    size_t node_step_size = node->GetStepSize();
    const auto& traversed_x = node->GetXs();
    const auto& traversed_y = node->GetYs();
    const auto& traversed_phi = node->GetPhis();

    // The first {x, y, phi} is collision free unless they are start and end
    // configuration of search problem
    size_t check_start_index = 0;
    if (node_step_size == 1) {
        check_start_index = 0;
    } else {
        check_start_index = 1;
    }

    for (size_t i = check_start_index; i < node_step_size; ++i) {
        if (traversed_x[i] > XYbounds_[1] || traversed_x[i] < XYbounds_[0] || traversed_y[i] > XYbounds_[3]
            || traversed_y[i] < XYbounds_[2]) {
            return false;
        }
        Box2d bounding_box = Node3d::GetBoundingBox(vehicle_param_, traversed_x[i], traversed_y[i], traversed_phi[i]);
        for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
            for (const common::math::LineSegment2d& linesegment : obstacle_linesegments) {
                if (bounding_box.HasOverlap(linesegment)) {
                    ADEBUG << "collision start at x: " << linesegment.start().x();
                    ADEBUG << "collision start at y: " << linesegment.start().y();
                    ADEBUG << "collision end at x: " << linesegment.end().x();
                    ADEBUG << "collision end at y: " << linesegment.end().y();
                    return false;
                }
            }
        }
    }
    return true;
}

std::shared_ptr<Node3d> HybridAStar::LoadRSPinCS(
        const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
        std::shared_ptr<Node3d> current_node) {
    std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(new Node3d(
            reeds_shepp_to_end->x,
            reeds_shepp_to_end->y,
            reeds_shepp_to_end->phi,
            XYbounds_,
            planner_warm_start_config_));
    end_node->SetPre(current_node);
    end_node->SetTrajCost(current_node->GetTrajCost() + reeds_shepp_to_end->cost);
    return end_node;
}

double HybridAStar::GetSoftBoundaryCost(const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end) {
    double x = 0;
    double y = 0;
    double cost = 0;
    for (int i = 0; i < reeds_shepp_to_end->x.size(); i++) {
        double point_cost;
        x = reeds_shepp_to_end->x[i];
        y = reeds_shepp_to_end->y[i];
        point_cost = soft_boundary_penalty_ / (grid_a_star_heuristic_generator_->GetObstacleDistance(x, y) + 1e-6);
        AINFO << "reedsheep point cost: " << point_cost;
        cost += point_cost;
    }
    AINFO << "reedsheep soft cost: " << cost;
    return cost;
}

std::shared_ptr<Node3d> HybridAStar::Next_node_generator(std::shared_ptr<Node3d> current_node, size_t next_node_index) {
    double steering = 0.0;
    double traveled_distance = 0.0;
    if (next_node_index < static_cast<double>(next_node_num_) / 2) {
        steering = -max_steer_angle_
                + (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1))
                        * static_cast<double>(next_node_index);
        traveled_distance = step_size_;
    } else {
        size_t index = next_node_index - next_node_num_ / 2;
        steering = -max_steer_angle_
                + (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) * static_cast<double>(index);
        traveled_distance = -step_size_;
    }
    // take above motion primitive to generate a curve driving the car to a
    // different grid
    std::vector<double> intermediate_x;
    std::vector<double> intermediate_y;
    std::vector<double> intermediate_phi;
    double last_x = current_node->GetX();
    double last_y = current_node->GetY();
    double last_phi = current_node->GetPhi();
    intermediate_x.push_back(last_x);
    intermediate_y.push_back(last_y);
    intermediate_phi.push_back(last_phi);
    for (size_t i = 0; i < arc_length_ / step_size_; ++i) {
        const double next_phi = last_phi + traveled_distance / vehicle_param_.wheel_base() * std::tan(steering);
        const double next_x = last_x + traveled_distance * std::cos((last_phi + next_phi) / 2.0);
        const double next_y = last_y + traveled_distance * std::sin((last_phi + next_phi) / 2.0);
        intermediate_x.push_back(next_x);
        intermediate_y.push_back(next_y);
        intermediate_phi.push_back(common::math::NormalizeAngle(next_phi));
        last_x = next_x;
        last_y = next_y;
        last_phi = next_phi;
    }
    // check if the vehicle runs outside of XY boundary
    if (intermediate_x.back() > XYbounds_[1] || intermediate_x.back() < XYbounds_[0]
        || intermediate_y.back() > XYbounds_[3] || intermediate_y.back() < XYbounds_[2]) {
        return nullptr;
    }
    std::shared_ptr<Node3d> next_node = std::shared_ptr<Node3d>(
            new Node3d(intermediate_x, intermediate_y, intermediate_phi, XYbounds_, planner_warm_start_config_));
    next_node->SetPre(current_node);
    next_node->SetDirec(traveled_distance > 0.0);
    next_node->SetSteer(steering);
    int dir = traveled_distance > 0.0 ? 1 : -1;
    next_node->SetTravelDist(
            current_node->GetDirec() == next_node->GetDirec() ? current_node->GetTravelDist() + arc_length_ * dir
                                                              : arc_length_ * dir);
    return next_node;
}

void HybridAStar::CalculateNodeCost(std::shared_ptr<Node3d> current_node, std::shared_ptr<Node3d> next_node) {
    next_node->SetTrajCost(current_node->GetTrajCost() + TrajCost(current_node, next_node));
    // evaluate heuristic cost
    double optimal_path_cost = 0.0;
    optimal_path_cost += HoloObstacleHeuristic(next_node);
    next_node->SetHeuCost(optimal_path_cost);
}

double HybridAStar::TrajCost(std::shared_ptr<Node3d> current_node, std::shared_ptr<Node3d> next_node) {
    // evaluate cost on the trajectory and add current cost
    double piecewise_cost = 0.0;
    if (next_node->GetDirec()) {
        piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) * step_size_ * traj_forward_penalty_;
    } else {
        piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) * step_size_ * traj_back_penalty_;
    }
    AINFO << "traj cost: " << piecewise_cost;
    if (current_node->GetDirec() != next_node->GetDirec()) {
        piecewise_cost += traj_gear_switch_penalty_;
        if (std::fabs(current_node->GetTravelDist()) < traj_expected_shortest_length_) {
            piecewise_cost += traj_short_length_penalty_;
        }
    }
    piecewise_cost += traj_steer_penalty_ * std::abs(next_node->GetSteer());
    AINFO << "steer pos: " << traj_steer_penalty_ * std::abs(next_node->GetSteer());
    piecewise_cost += traj_steer_change_penalty_ * std::abs(next_node->GetSteer() - current_node->GetSteer());
    AINFO << "steer change: "
          << traj_steer_change_penalty_ * std::abs(next_node->GetSteer() - current_node->GetSteer());

    piecewise_cost += soft_boundary_penalty_
            / (grid_a_star_heuristic_generator_->GetObstacleDistance(current_node->GetX(), current_node->GetY())
               + 1e-6);
    // double temp = soft_boundary_penalty_ / (grid_a_star_heuristic_generator_->
    // GetObstacleDistance(current_node->GetX(), current_node->GetY()) + 1e-6);

    // if (temp > 0.01) {
    //   AINFO << "---soft boundary: " << temp;
    // } else {
    //   AINFO << "soft boundary: 0";
    // }

    // AINFO << "soft boundary: " << temp;
    return piecewise_cost;
}

double HybridAStar::HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node) {
    return grid_a_star_heuristic_generator_->CheckDpMap(next_node->GetX(), next_node->GetY());
}

bool HybridAStar::GetResult(HybridAStartResult* result) {
    std::shared_ptr<Node3d> current_node = final_node_;
    std::vector<double> hybrid_a_x;
    std::vector<double> hybrid_a_y;
    std::vector<double> hybrid_a_phi;
    AINFO << "switch node:" << current_node->GetXs().back() << ", " << current_node->GetYs().back();
    AINFO << "switch node:" << current_node->GetXs().front() << ", " << current_node->GetYs().front();
    AINFO << "cost: " << final_node_->GetCost() << "," << final_node_->GetTrajCost();
    while (current_node->GetPreNode() != nullptr) {
        std::vector<double> x = current_node->GetXs();
        std::vector<double> y = current_node->GetYs();
        std::vector<double> phi = current_node->GetPhis();
        if (x.empty() || y.empty() || phi.empty()) {
            AERROR << "result size check failed";
            return false;
        }
        if (x.size() != y.size() || x.size() != phi.size()) {
            AERROR << "states sizes are not equal";
            return false;
        }
        std::reverse(x.begin(), x.end());
        std::reverse(y.begin(), y.end());
        std::reverse(phi.begin(), phi.end());
        x.pop_back();
        y.pop_back();
        phi.pop_back();
        hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
        hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
        hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
        current_node = current_node->GetPreNode();
    }
    hybrid_a_x.push_back(current_node->GetX());
    hybrid_a_y.push_back(current_node->GetY());
    hybrid_a_phi.push_back(current_node->GetPhi());
    std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
    std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
    std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
    (*result).x = hybrid_a_x;
    (*result).y = hybrid_a_y;
    (*result).phi = hybrid_a_phi;

    if (!GetTemporalProfile(result)) {
        AERROR << "GetSpeedProfile from Hybrid Astar path fails";
        return false;
    }

    if (result->x.size() != result->y.size() || result->x.size() != result->v.size()
        || result->x.size() != result->phi.size()) {
        AERROR << "state sizes not equal, " << "result->x.size(): " << result->x.size() << "result->y.size()"
               << result->y.size() << "result->phi.size()" << result->phi.size() << "result->v.size()"
               << result->v.size();
        return false;
    }
    if (result->a.size() != result->steer.size() || result->x.size() - result->a.size() != 1) {
        AERROR << "control sizes not equal or not right";
        AERROR << " acceleration size: " << result->a.size();
        AERROR << " steer size: " << result->steer.size();
        AERROR << " x size: " << result->x.size();
        return false;
    }
    return true;
}

bool HybridAStar::GenerateSpeedAcceleration(HybridAStartResult* result) {
    AINFO << "GenerateSpeedAcceleration";
    // Sanity Check
    if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
        AERROR << "result size check when generating speed and acceleration fail";
        return false;
    }
    const size_t x_size = result->x.size();

    // load velocity from position
    // initial and end speed are set to be zeros
    result->v.push_back(0.0);
    for (size_t i = 1; i + 1 < x_size; ++i) {
        double discrete_v = (((result->x[i + 1] - result->x[i]) / delta_t_) * std::cos(result->phi[i])
                             + ((result->x[i] - result->x[i - 1]) / delta_t_) * std::cos(result->phi[i]))
                        / 2.0
                + (((result->y[i + 1] - result->y[i]) / delta_t_) * std::sin(result->phi[i])
                   + ((result->y[i] - result->y[i - 1]) / delta_t_) * std::sin(result->phi[i]))
                        / 2.0;
        result->v.push_back(discrete_v);
    }
    result->v.push_back(0.0);

    // load acceleration from velocity
    for (size_t i = 0; i + 1 < x_size; ++i) {
        const double discrete_a = (result->v[i + 1] - result->v[i]) / delta_t_;
        result->a.push_back(discrete_a);
    }

    // load steering from phi
    for (size_t i = 0; i + 1 < x_size; ++i) {
        double discrete_steer = (result->phi[i + 1] - result->phi[i]) * vehicle_param_.wheel_base() / step_size_;
        if (result->v[i] > 0.0) {
            discrete_steer = std::atan(discrete_steer);
        } else {
            discrete_steer = std::atan(-discrete_steer);
        }
        result->steer.push_back(discrete_steer);
    }
    return true;
}

bool HybridAStar::GenerateSCurveSpeedAcceleration(HybridAStartResult* result) {
    AINFO << "GenerateSCurveSpeedAcceleration";
    // sanity check
    CHECK_NOTNULL(result);
    if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
        AERROR << "result size check when generating speed and acceleration fail";
        return false;
    }
    if (result->x.size() != result->y.size() || result->x.size() != result->phi.size()) {
        AERROR << "result sizes not equal";
        return false;
    }

    // get gear info
    double init_heading = result->phi.front();
    const Vec2d init_tracking_vector(result->x[1] - result->x[0], result->y[1] - result->y[0]);
    const double gear = std::abs(common::math::NormalizeAngle(init_heading - init_tracking_vector.Angle())) < M_PI_2;

    // get path lengh
    size_t path_points_size = result->x.size();

    double accumulated_s = 0.0;
    result->accumulated_s.clear();
    auto last_x = result->x.front();
    auto last_y = result->y.front();
    for (size_t i = 0; i < path_points_size; ++i) {
        double x_diff = result->x[i] - last_x;
        double y_diff = result->y[i] - last_y;
        accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
        result->accumulated_s.push_back(accumulated_s);
        last_x = result->x[i];
        last_y = result->y[i];
    }
    // assume static initial state
    const double init_v = 0.0;
    const double init_a = 0.0;

    // minimum time speed optimization
    // TODO(Jinyun): move to confs

    SpeedData speed_data;

    // TODO(Jinyun): explore better time horizon heuristic
    const double path_length = result->accumulated_s.back();
    const double total_t = std::max(
            gear ? 1.5 * (max_forward_v_ * max_forward_v_ + path_length * max_forward_acc_)
                            / (max_forward_acc_ * max_forward_v_)
                 : 1.5 * (max_reverse_v_ * max_reverse_v_ + path_length * max_reverse_acc_)
                            / (max_reverse_acc_ * max_reverse_v_),
            10.0);
    if (total_t + delta_t_ >= delta_t_ * std::numeric_limits<size_t>::max()) {
        AERROR << "Number of knots overflow. total_t: " << total_t << ", delta_t: " << delta_t_;
        return false;
    }
    const size_t num_of_knots = static_cast<size_t>(total_t / delta_t_) + 1;

    PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t_, {0.0, std::abs(init_v), std::abs(init_a)});

    // set end constraints
    std::vector<std::pair<double, double>> x_bounds(num_of_knots, {0.0, path_length});

    const double max_v = gear ? max_forward_v_ : max_reverse_v_;
    const double max_acc = gear ? max_forward_acc_ : max_reverse_acc_;

    const auto upper_dx = std::fmax(max_v, std::abs(init_v));
    std::vector<std::pair<double, double>> dx_bounds(num_of_knots, {0.0, upper_dx});
    std::vector<std::pair<double, double>> ddx_bounds(num_of_knots, {-max_acc, max_acc});

    x_bounds[num_of_knots - 1] = std::make_pair(path_length, path_length);
    dx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);
    ddx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);

    // TODO(Jinyun): move to confs
    std::vector<double> x_ref(num_of_knots, path_length);
    piecewise_jerk_problem.set_x_ref(ref_s_weight_, std::move(x_ref));
    piecewise_jerk_problem.set_dx_ref(ref_v_weight_, max_v * 0.8);
    piecewise_jerk_problem.set_weight_ddx(acc_weight_);
    piecewise_jerk_problem.set_weight_dddx(jerk_weight_);
    piecewise_jerk_problem.set_x_bounds(std::move(x_bounds));
    piecewise_jerk_problem.set_dx_bounds(std::move(dx_bounds));
    piecewise_jerk_problem.set_ddx_bounds(std::move(ddx_bounds));
    piecewise_jerk_problem.set_dddx_bound(max_acc_jerk_);

    // solve the problem
    if (!piecewise_jerk_problem.Optimize()) {
        AERROR << "Piecewise jerk speed optimizer failed!";
        return false;
    }

    // extract output
    const std::vector<double>& s = piecewise_jerk_problem.opt_x();
    const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
    const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

    // assign speed point by gear
    speed_data.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
    const double kEpislon = 1.0e-6;
    const double sEpislon = 1.0e-6;
    for (size_t i = 1; i < num_of_knots; ++i) {
        if (s[i - 1] - s[i] > kEpislon) {
            ADEBUG << "unexpected decreasing s in speed smoothing at time " << static_cast<double>(i) * delta_t_
                   << "with total time " << total_t;
            break;
        }
        speed_data.AppendSpeedPoint(
                s[i], delta_t_ * static_cast<double>(i), ds[i], dds[i], (dds[i] - dds[i - 1]) / delta_t_);
        // cut the speed data when it is about to meet end condition
        if (path_length - s[i] < sEpislon) {
            break;
        }
    }

    // combine speed and path profile
    DiscretizedPath path_data;
    for (size_t i = 0; i < path_points_size; ++i) {
        common::PathPoint path_point;
        path_point.set_x(result->x[i]);
        path_point.set_y(result->y[i]);
        path_point.set_theta(result->phi[i]);
        path_point.set_s(result->accumulated_s[i]);
        path_data.push_back(std::move(path_point));
    }

    HybridAStartResult combined_result;

    // TODO(Jinyun): move to confs
    const double kDenseTimeResoltuion = 0.5;
    const double time_horizon = speed_data.TotalTime() + kDenseTimeResoltuion * 1.0e-6;
    if (path_data.empty()) {
        AERROR << "path data is empty";
        return false;
    }
    for (double cur_rel_time = 0.0; cur_rel_time < time_horizon; cur_rel_time += kDenseTimeResoltuion) {
        common::SpeedPoint speed_point;
        if (!speed_data.EvaluateByTime(cur_rel_time, &speed_point)) {
            AERROR << "Fail to get speed point with relative time " << cur_rel_time;
            return false;
        }

        if (speed_point.s() > path_data.Length()
            || (speed_point.v() < 0.05 && speed_point.s() > path_data.Length() / 2)) {
            break;
        }

        common::PathPoint path_point = path_data.Evaluate(speed_point.s());

        combined_result.x.push_back(path_point.x());
        combined_result.y.push_back(path_point.y());
        combined_result.phi.push_back(path_point.theta());
        combined_result.accumulated_s.push_back(path_point.s());
        if (!gear) {
            combined_result.v.push_back(-speed_point.v());
            combined_result.a.push_back(-speed_point.a());
        } else {
            combined_result.v.push_back(speed_point.v());
            combined_result.a.push_back(speed_point.a());
        }
    }

    combined_result.a.pop_back();

    // recalc step size
    path_points_size = combined_result.x.size();

    // load steering from phi
    for (size_t i = 0; i + 1 < path_points_size; ++i) {
        double discrete_steer = (combined_result.phi[i + 1] - combined_result.phi[i]) * vehicle_param_.wheel_base()
                / (combined_result.accumulated_s[i + 1] - combined_result.accumulated_s[i]);
        discrete_steer = gear ? std::atan(discrete_steer) : std::atan(-discrete_steer);
        combined_result.steer.push_back(discrete_steer);
    }

    *result = combined_result;
    return true;
}

bool HybridAStar::TrajectoryPartition(
        const HybridAStartResult& result,
        std::vector<HybridAStartResult>* partitioned_result) {
    const auto& x = result.x;
    const auto& y = result.y;
    const auto& phi = result.phi;
    if (x.size() != y.size() || x.size() != phi.size()) {
        AERROR << "states sizes are not equal when do trajectory partitioning of "
                  "Hybrid A Star result";
        return false;
    }

    size_t horizon = x.size();
    partitioned_result->clear();
    partitioned_result->emplace_back();
    auto* current_traj = &(partitioned_result->back());
    double heading_angle = phi.front();
    const Vec2d init_tracking_vector(x[1] - x[0], y[1] - y[0]);
    double tracking_angle = init_tracking_vector.Angle();
    bool current_gear = std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) < (M_PI_2);
    AINFO << x[0] << " " << y[0] << " " << phi[0];
    for (size_t i = 0; i < horizon - 1; ++i) {
        heading_angle = phi[i];
        const Vec2d tracking_vector(x[i + 1] - x[i], y[i + 1] - y[i]);
        tracking_angle = tracking_vector.Angle();
        bool gear = std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) < (M_PI_2);
        if (gear != current_gear) {
            current_traj->x.push_back(x[i]);
            current_traj->y.push_back(y[i]);
            current_traj->phi.push_back(phi[i]);
            partitioned_result->emplace_back();
            current_traj = &(partitioned_result->back());
            current_gear = gear;
        }
        current_traj->x.push_back(x[i]);
        current_traj->y.push_back(y[i]);
        current_traj->phi.push_back(phi[i]);
    }
    AINFO << x.back() << " " << y.back() << " " << phi.back();
    current_traj->x.push_back(x.back());
    current_traj->y.push_back(y.back());
    current_traj->phi.push_back(phi.back());

    const auto start_timestamp = std::chrono::system_clock::now();

    // Retrieve v, a and steer from path
    for (auto& result : *partitioned_result) {
        if (FLAGS_use_s_curve_speed_smooth) {
            if (!GenerateSCurveSpeedAcceleration(&result)) {
                AERROR << "GenerateSCurveSpeedAcceleration fail";
                return false;
            }
        } else {
            if (!GenerateSpeedAcceleration(&result)) {
                AERROR << "GenerateSpeedAcceleration fail";
                return false;
            }
        }
    }

    const auto end_timestamp = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end_timestamp - start_timestamp;
    ADEBUG << "speed profile total time: " << diff.count() * 1000.0 << " ms.";
    return true;
}

bool HybridAStar::GetTemporalProfile(HybridAStartResult* result) {
    std::vector<HybridAStartResult> partitioned_results;
    if (!TrajectoryPartition(*result, &partitioned_results)) {
        AERROR << "TrajectoryPartition fail";
        return false;
    }
    ADEBUG << "PARTION SIZE " << partitioned_results.size();
    HybridAStartResult stitched_result;
    for (const auto& result : partitioned_results) {
        std::copy(result.x.begin(), result.x.end() - 1, std::back_inserter(stitched_result.x));
        std::copy(result.y.begin(), result.y.end() - 1, std::back_inserter(stitched_result.y));
        std::copy(result.phi.begin(), result.phi.end() - 1, std::back_inserter(stitched_result.phi));
        std::copy(result.v.begin(), result.v.end() - 1, std::back_inserter(stitched_result.v));
        std::copy(result.a.begin(), result.a.end(), std::back_inserter(stitched_result.a));
        std::copy(result.steer.begin(), result.steer.end(), std::back_inserter(stitched_result.steer));
    }
    stitched_result.x.push_back(partitioned_results.back().x.back());
    stitched_result.y.push_back(partitioned_results.back().y.back());
    stitched_result.phi.push_back(partitioned_results.back().phi.back());
    stitched_result.v.push_back(partitioned_results.back().v.back());
    *result = stitched_result;
    return true;
}

bool HybridAStar::Plan(
        double sx,
        double sy,
        double sphi,
        double ex,
        double ey,
        double ephi,
        const std::vector<double>& XYbounds,
        const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
        HybridAStartResult* result,
        const std::vector<std::vector<common::math::Vec2d>>& soft_boundary_vertices_vec,
        bool reeds_sheep_last_straight) {
    reed_shepp_generator_->reeds_sheep_last_straight_ = reeds_sheep_last_straight;
    // clear containers
    open_set_.clear();
    close_set_.clear();
    open_pq_ = decltype(open_pq_)();
    final_node_ = nullptr;
    PrintCurves print_curves;
    std::vector<std::vector<common::math::LineSegment2d>> obstacles_linesegments_vec;
    for (const auto& obstacle_vertices : obstacles_vertices_vec) {
        size_t vertices_num = obstacle_vertices.size();
        std::vector<common::math::LineSegment2d> obstacle_linesegments;
        for (size_t i = 0; i < vertices_num - 1; ++i) {
            common::math::LineSegment2d line_segment
                    = common::math::LineSegment2d(obstacle_vertices[i], obstacle_vertices[i + 1]);
            obstacle_linesegments.emplace_back(line_segment);
        }
        obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
    }
    obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);
    std::vector<std::vector<common::math::LineSegment2d>> soft_boundary_linesegments_vec;
    for (const auto& soft_boundary_vertices : soft_boundary_vertices_vec) {
        size_t vertices_num = soft_boundary_vertices.size();
        std::vector<common::math::LineSegment2d> soft_boundary_linesegments;
        for (size_t i = 0; i < vertices_num - 1; ++i) {
            common::math::LineSegment2d line_segment
                    = common::math::LineSegment2d(soft_boundary_vertices[i], soft_boundary_vertices[i + 1]);
            soft_boundary_linesegments.emplace_back(line_segment);
        }
        soft_boundary_linesegments_vec.emplace_back(soft_boundary_linesegments);
    }
    soft_boundary_linesegments_vec_ = std::move(soft_boundary_linesegments_vec);
    for (size_t i = 0; i < obstacles_linesegments_vec_.size(); i++) {
        for (auto linesg : obstacles_linesegments_vec_[i]) {
            std::string name = std::to_string(i) + "roi_boundary";
            print_curves.AddPoint(name, linesg.start());
            print_curves.AddPoint(name, linesg.end());
        }
    }
    Vec2d sposition(sx, sy);
    Vec2d svec_to_center(
            (vehicle_param_.front_edge_to_center() - vehicle_param_.back_edge_to_center()) / 2.0,
            (vehicle_param_.left_edge_to_center() - vehicle_param_.right_edge_to_center()) / 2.0);
    Vec2d scenter(sposition + svec_to_center.rotate(sphi));
    Box2d sbox(scenter, sphi, vehicle_param_.length(), vehicle_param_.width());
    print_curves.AddPoint("vehicle_start_box", sbox.GetAllCorners());
    Vec2d eposition(ex, ey);
    print_curves.AddPoint("end_position", eposition);
    Vec2d evec_to_center(
            (vehicle_param_.front_edge_to_center() - vehicle_param_.back_edge_to_center()) / 2.0,
            (vehicle_param_.left_edge_to_center() - vehicle_param_.right_edge_to_center()) / 2.0);
    Vec2d ecenter(eposition + evec_to_center.rotate(ephi));
    Box2d ebox(ecenter, ephi, vehicle_param_.length(), vehicle_param_.width());
    print_curves.AddPoint("vehicle_end_box", ebox.GetAllCorners());
    XYbounds_ = XYbounds;
    // load nodes and obstacles
    start_node_.reset(new Node3d({sx}, {sy}, {sphi}, XYbounds_, planner_warm_start_config_));
    end_node_.reset(new Node3d({ex}, {ey}, {ephi}, XYbounds_, planner_warm_start_config_));
    AINFO << "start node" << sx << "," << sy << "," << sphi;
    AINFO << "end node " << ex << "," << ey << "," << ephi;
    if (!ValidityCheck(start_node_)) {
        AERROR << "start_node in collision with obstacles";
        AERROR << start_node_->GetX() << "," << start_node_->GetY() << "," << start_node_->GetPhi();
        print_curves.PrintToLog();
        return false;
    }
    if (!ValidityCheck(end_node_)) {
        AERROR << "end_node in collision with obstacles";
        print_curves.PrintToLog();
        return false;
    }
    double map_time = Clock::NowInSeconds();
    grid_a_star_heuristic_generator_->GenerateDpMap(
            ex, ey, XYbounds_, obstacles_linesegments_vec_, soft_boundary_linesegments_vec_);
    ADEBUG << "map time " << Clock::NowInSeconds() - map_time;
    // load open set, pq
    open_set_.insert(start_node_->GetIndex());
    open_pq_.emplace(start_node_, start_node_->GetCost());
    // Hybrid A* begins
    size_t explored_node_num = 0;
    size_t available_result_num = 0;
    auto best_explored_num = explored_node_num;
    auto best_available_result_num = available_result_num;
    double astar_start_time = Clock::NowInSeconds();
    double heuristic_time = 0.0;
    double rs_time = 0.0;
    double node_generator_time = 0.0;
    double validity_check_time = 0.0;
    size_t max_explored_num = planner_warm_start_config_.max_explored_num();
    size_t desired_explored_num = std::min(
            planner_warm_start_config_.desired_explored_num(), planner_warm_start_config_.max_explored_num());
    static constexpr int kMaxNodeNum = 200000;
    std::vector<std::shared_ptr<Node3d>> candidate_final_nodes;
    while (!open_pq_.empty() && open_pq_.size() < kMaxNodeNum && available_result_num < desired_explored_num
           && explored_node_num < max_explored_num) {
        std::shared_ptr<Node3d> current_node = open_pq_.top().first;
        if (current_node->GetPreNode() != nullptr) {
            AINFO << "print_open_pq:" << "(" << current_node->GetPreNode()->GetX() << ", "
                  << current_node->GetPreNode()->GetY() << ")" << ";" << "(" << current_node->GetX() << ", "
                  << current_node->GetY() << ");";
        }
        open_pq_.pop();
        const double rs_start_time = Clock::NowInSeconds();
        std::shared_ptr<Node3d> final_node = nullptr;
        if (AnalyticExpansion(current_node, &final_node)) {
            if (final_node_ == nullptr || final_node_->GetTrajCost() > final_node->GetTrajCost()) {
                final_node_ = final_node;
                best_explored_num = explored_node_num + 1;
                best_available_result_num = available_result_num + 1;
            }
            available_result_num++;
        }
        explored_node_num++;
        const double rs_end_time = Clock::NowInSeconds();
        rs_time += rs_end_time - rs_start_time;
        close_set_.insert(current_node->GetIndex());

        if (Clock::NowInSeconds() - astar_start_time > planner_warm_start_config_.astar_max_search_time()
            && available_result_num > 0) {
            break;
        }

        size_t begin_index = 0;
        size_t end_index = next_node_num_;
        std::unordered_set<std::string> temp_set;
        for (size_t i = begin_index; i < end_index; ++i) {
            const double gen_node_time = Clock::NowInSeconds();
            std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
            node_generator_time += Clock::NowInSeconds() - gen_node_time;

            // boundary check failure handle
            if (next_node == nullptr) {
                continue;
            }
            // check if the node is already in the close set
            if (close_set_.count(next_node->GetIndex()) > 0) {
                continue;
            }
            // collision check
            const double validity_check_start_time = Clock::NowInSeconds();
            if (!ValidityCheck(next_node)) {
                continue;
            }
            validity_check_time += Clock::NowInSeconds() - validity_check_start_time;
            if (open_set_.count(next_node->GetIndex()) == 0) {
                const double start_time = Clock::NowInSeconds();
                CalculateNodeCost(current_node, next_node);
                const double end_time = Clock::NowInSeconds();
                heuristic_time += end_time - start_time;
                temp_set.insert(next_node->GetIndex());
                open_pq_.emplace(next_node, next_node->GetCost());
            }
        }
        open_set_.insert(temp_set.begin(), temp_set.end());
    }

    if (final_node_ == nullptr) {
        AERROR << "Hybird A* cannot find a valid path";
        print_curves.PrintToLog();
        return false;
    }

    AINFO << "open_pq_.empty()" << (open_pq_.empty() ? "true" : "false");
    AINFO << "open_pq_.size()" << open_pq_.size();
    AINFO << "desired_explored_num" << desired_explored_num;
    AINFO << "min cost is : " << final_node_->GetTrajCost();
    AINFO << "max_explored_num is " << max_explored_num;
    AINFO << "explored node num is " << explored_node_num << "available_result_num " << available_result_num;
    AINFO << "best_explored_num is " << best_explored_num << "best_available_result_num is "
          << best_available_result_num;
    AINFO << "cal node time is " << heuristic_time << "validity_check_time " << validity_check_time
          << "node_generator_time " << node_generator_time;
    AINFO << "reed shepp time is " << rs_time;
    AINFO << "hybrid astar total time is " << Clock::NowInSeconds() - astar_start_time;

    print_curves.AddPoint("rs_point", final_node_->GetXs().front(), final_node_->GetYs().front());
    if (!GetResult(result)) {
        AERROR << "GetResult failed";
        print_curves.PrintToLog();
        return false;
    }
    for (size_t i = 0; i < result->x.size(); i++) {
        print_curves.AddPoint("warm_path", result->x[i], result->y[i]);
    }
    // PrintBox print_box("warm_path_box");
    // for (size_t i = 0; i < result->x.size(); i = i + 5) {
    //   print_box.AddAdcBox(result->x[i], result->y[i], result->phi[i]);
    // }
    // print_box.PrintToLog();
    print_curves.PrintToLog();
    return true;
}
}  // namespace planning
}  // namespace apollo
