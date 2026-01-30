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

/*
 * @file
 */

#include "modules/planning/planning_open_space/trajectory_smoother/iterative_anchoring_smoother.h"

#include <algorithm>
#include <limits>
#include <random>

#include "modules/planning/planning_open_space/proto/planner_open_space_config.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_base/math/discrete_points_math.h"
#include "modules/planning/planning_base/math/discretized_points_smoothing/fem_pos_deviation_smoother.h"
#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_speed_problem.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::LineSegment2d;
using apollo::common::math::NormalizeAngle;
using apollo::common::math::Vec2d;

IterativeAnchoringSmoother::IterativeAnchoringSmoother(const PlannerOpenSpaceConfig& planner_open_space_config) {
    // TODO(Jinyun, Yu): refactor after stabilized.
    const auto& vehicle_param = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
    ego_length_ = vehicle_param.length();
    ego_width_ = vehicle_param.width();
    center_shift_distance_ = ego_length_ / 2.0 - vehicle_param.back_edge_to_center();
    planner_open_space_config_ = planner_open_space_config;
    iterative_anchoring_config_ = planner_open_space_config_.iterative_anchoring_smoother_config();
    AINFO << "config:" << planner_open_space_config_.DebugString();
}

IterativeAnchoringSmoother::IterativeAnchoringSmoother(const IterativeAnchoringConfig& iterative_anchoring_config) {
    // TODO(Jinyun, Yu): refactor after stabilized.
    const auto& vehicle_param = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
    ego_length_ = vehicle_param.length();
    ego_width_ = vehicle_param.width();
    center_shift_distance_ = ego_length_ / 2.0 - vehicle_param.back_edge_to_center();
    iterative_anchoring_config_ = iterative_anchoring_config;
    AINFO << "config:" << iterative_anchoring_config_.DebugString();
}

bool IterativeAnchoringSmoother::Smooth(
        const Eigen::MatrixXd& xWS,
        const double init_a,
        const double init_v,
        const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
        DiscretizedTrajectory* discretized_trajectory) {
    if (xWS.cols() < 2) {
        AERROR << "reference points size smaller than two, smoother early "
                  "returned";
        return false;
    }
    const auto start_timestamp = std::chrono::system_clock::now();

    // Set gear of the trajectory
    gear_ = CheckGear(xWS);

    // Set obstacle in form of linesegments
    std::vector<std::vector<LineSegment2d>> obstacles_linesegments_vec;
    for (const auto& obstacle_vertices : obstacles_vertices_vec) {
        size_t vertices_num = obstacle_vertices.size();
        std::vector<LineSegment2d> obstacle_linesegments;
        for (size_t i = 0; i + 1 < vertices_num; ++i) {
            LineSegment2d line_segment = LineSegment2d(obstacle_vertices[i], obstacle_vertices[i + 1]);
            obstacle_linesegments.emplace_back(line_segment);
        }
        obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
    }
    obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);

    // Interpolate the traj
    DiscretizedPath warm_start_path;
    size_t xWS_size = xWS.cols();
    double accumulated_s = 0.0;
    Vec2d last_path_point(xWS(0, 0), xWS(1, 0));
    for (size_t i = 0; i < xWS_size; ++i) {
        Vec2d cur_path_point(xWS(0, i), xWS(1, i));
        accumulated_s += cur_path_point.DistanceTo(last_path_point);
        PathPoint path_point;
        path_point.set_x(xWS(0, i));
        path_point.set_y(xWS(1, i));
        path_point.set_theta(xWS(2, i));
        path_point.set_s(accumulated_s);
        warm_start_path.push_back(std::move(path_point));
        last_path_point = cur_path_point;
    }
    if (std::fabs(accumulated_s) < 0.1) {
      AERROR << "warm_start_path smaller than 0.1 m, can't enforce "
                  "heading continuity";
      return false;
    }

    const double interpolated_delta_s = iterative_anchoring_config_.interpolated_delta_s();
    std::vector<std::pair<double, double>> interpolated_warm_start_point2ds;
    double path_length = warm_start_path.Length();
    int count = std::ceil(path_length / interpolated_delta_s);
    double delta_s = path_length / (count < 4 ? 4 : count);
    path_length += delta_s * 1.0e-6;
    for (double s = 0; s < path_length; s += delta_s) {
        const auto point2d = warm_start_path.Evaluate(s);
        interpolated_warm_start_point2ds.emplace_back(point2d.x(), point2d.y());
    }

    const size_t interpolated_size = interpolated_warm_start_point2ds.size();
    if (interpolated_size < 6) {
        ADEBUG << "interpolated_warm_start_path smaller than 4, can't enforce "
                  "initial zero kappa";
        enforce_initial_kappa_ = false;
    } else {
        enforce_initial_kappa_ = true;
    }

    // Adjust heading to ensure heading continuity
    AdjustStartEndHeading(xWS, &interpolated_warm_start_point2ds);

    // Reset path profile by discrete point heading and curvature estimation
    DiscretizedPath interpolated_warm_start_path;
    if (!SetPathProfile(interpolated_warm_start_point2ds, &interpolated_warm_start_path)) {
        AERROR << "Set path profile fails";
        return false;
    }

    // Generate feasible bounds for each path point
    std::vector<double> bounds;
    if (!GenerateInitialBounds(interpolated_warm_start_path, &bounds)) {
        AERROR << "Generate initial bounds failed, path point to close to obstacle";
        return false;
    }

    // Check initial path collision avoidance, if it fails, smoother assumption
    // fails. Try reanchoring
    input_colliding_point_index_.clear();
    if (!CheckCollisionAvoidance(interpolated_warm_start_path, &input_colliding_point_index_)) {
        AERROR << "Interpolated input path points colliding with obstacle";
        // if (!ReAnchoring(colliding_point_index, &interpolated_warm_start_path)) {
        //   AERROR << "Fail to reanchor colliding input path points";
        //   return false;
        // }
    }

    const auto path_smooth_start_timestamp = std::chrono::system_clock::now();

    // Smooth path to have smoothed x, y, phi, kappa and s
    DiscretizedPath smoothed_path_points;
    if (!SmoothPath(interpolated_warm_start_path, bounds, &smoothed_path_points)) {
        return false;
    }
    // PrintCurves print_curve;
    // for (size_t i = 0; i < smoothed_path_points.size(); i++) {
    //   print_curve.AddPoint("iter_smoothed_xy", smoothed_path_points[i].x(),
    //                        smoothed_path_points[i].y());
    //   print_curve.AddPoint("iter_smoothed_s-kappa",
    //   smoothed_path_points[i].s(),
    //                        smoothed_path_points[i].kappa());
    //   print_curve.AddPoint("iter_smoothed_s-theta",
    //   smoothed_path_points[i].s(),
    //                        smoothed_path_points[i].theta());
    // }
    // for (size_t i = 0; i < interpolated_warm_start_path.size(); i++) {
    //   print_curve.AddPoint("iter_warm_xy", interpolated_warm_start_path[i].x(),
    //                        interpolated_warm_start_path[i].y());
    //   print_curve.AddPoint("iter_warm_s-kappa",
    //                        interpolated_warm_start_path[i].s(),
    //                        interpolated_warm_start_path[i].kappa());
    //   print_curve.AddPoint("iter_warm_s-theta",
    //                        interpolated_warm_start_path[i].s(),
    //                        interpolated_warm_start_path[i].theta());
    // }
    // print_curve.PrintToLog();
    const auto path_smooth_end_timestamp = std::chrono::system_clock::now();
    std::chrono::duration<double> path_smooth_diff = path_smooth_end_timestamp - path_smooth_start_timestamp;
    ADEBUG << "iterative anchoring path smoother time: " << path_smooth_diff.count() * 1000.0 << " ms.";

    const auto speed_smooth_start_timestamp = std::chrono::system_clock::now();

    // Smooth speed to have smoothed v and a
    SpeedData smoothed_speeds;
    if (!SmoothSpeed(init_a, init_v, smoothed_path_points.Length(), &smoothed_speeds)) {
        return false;
    }

    // TODO(Jinyun): Evaluate performance
    // SpeedData smoothed_speeds;
    // if (!GenerateStopProfileFromPolynomial(
    //         init_a, init_v, smoothed_path_points.Length(), &smoothed_speeds)) {
    //   return false;
    // }

    const auto speed_smooth_end_timestamp = std::chrono::system_clock::now();
    std::chrono::duration<double> speed_smooth_diff = speed_smooth_end_timestamp - speed_smooth_start_timestamp;
    ADEBUG << "iterative anchoring speed smoother time: " << speed_smooth_diff.count() * 1000.0 << " ms.";

    // Combine path and speed
    if (!CombinePathAndSpeed(smoothed_path_points, smoothed_speeds, discretized_trajectory)) {
        return false;
    }

    AdjustPathAndSpeedByGear(discretized_trajectory);

    const auto end_timestamp = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end_timestamp - start_timestamp;
    ADEBUG << "iterative anchoring smoother total time: " << diff.count() * 1000.0 << " ms.";

    ADEBUG << "discretized_trajectory size " << discretized_trajectory->size();
    return true;
}

void IterativeAnchoringSmoother::AdjustStartEndHeading(
        const Eigen::MatrixXd& xWS,
        std::vector<std::pair<double, double>>* const point2d) {
    // Sanity check
    CHECK_NOTNULL(point2d);
    CHECK_GT(xWS.cols(), 1);
    CHECK_GT(point2d->size(), 3U);

    // Set initial heading and bounds
    const double initial_heading = xWS(2, 0);
    const double end_heading = xWS(2, xWS.cols() - 1);

    // Adjust the point position to have heading by finite element difference of
    // the point and the other point equal to the given warm start initial or end
    // heading
    const double first_to_second_dx = point2d->at(1).first - point2d->at(0).first;
    const double first_to_second_dy = point2d->at(1).second - point2d->at(0).second;
    const double first_to_second_s
            = std::sqrt(first_to_second_dx * first_to_second_dx + first_to_second_dy * first_to_second_dy);
    Vec2d first_point(point2d->at(0).first, point2d->at(0).second);
    Vec2d initial_vec(first_to_second_s, 0);
    initial_vec.SelfRotate(gear_ ? initial_heading : NormalizeAngle(initial_heading + M_PI));
    initial_vec += first_point;
    point2d->at(1) = std::make_pair(initial_vec.x(), initial_vec.y());

    const size_t path_size = point2d->size();
    const double second_last_to_last_dx = point2d->at(path_size - 1).first - point2d->at(path_size - 2).first;
    const double second_last_to_last_dy = point2d->at(path_size - 1).second - point2d->at(path_size - 2).second;
    const double second_last_to_last_s = std::sqrt(
            second_last_to_last_dx * second_last_to_last_dx + second_last_to_last_dy * second_last_to_last_dy);
    Vec2d last_point(point2d->at(path_size - 1).first, point2d->at(path_size - 1).second);
    Vec2d end_vec(second_last_to_last_s, 0);
    end_vec.SelfRotate(gear_ ? NormalizeAngle(end_heading + M_PI) : end_heading);
    end_vec += last_point;
    point2d->at(path_size - 2) = std::make_pair(end_vec.x(), end_vec.y());
}

bool IterativeAnchoringSmoother::ReAnchoring(
        const std::vector<size_t>& colliding_point_index,
        DiscretizedPath* path_points) {
    CHECK_NOTNULL(path_points);
    if (colliding_point_index.empty()) {
        ADEBUG << "no point needs to be re-anchored";
        return true;
    }

    CHECK_GT(path_points->size(), *(std::max_element(colliding_point_index.begin(), colliding_point_index.end())));

    for (const auto index : colliding_point_index) {
        if (index == 0 || index == path_points->size() - 1) {
            AERROR << "Initial and end points collision avoid condition failed.";
            return false;
        }
        if (index == 1 || index == path_points->size() - 2) {
            AERROR << "second to last point or second point pos reanchored. Heading "
                      "discontinuity might "
                      "happen";
        }
    }

    // TODO(Jinyun): move to confs
    const size_t reanchoring_trails_num = static_cast<size_t>(iterative_anchoring_config_.reanchoring_trails_num());
    const double reanchoring_pos_stddev = iterative_anchoring_config_.reanchoring_pos_stddev();
    const double reanchoring_length_stddev = iterative_anchoring_config_.reanchoring_length_stddev();
    std::random_device rd;
    std::default_random_engine gen = std::default_random_engine(rd());
    std::normal_distribution<> pos_dis{0, reanchoring_pos_stddev};
    std::normal_distribution<> length_dis{0, reanchoring_length_stddev};

    for (const auto index : colliding_point_index) {
        bool reanchoring_success = false;
        for (size_t i = 0; i < reanchoring_trails_num; ++i) {
            // Get ego box for collision check on collision point index
            bool is_colliding = false;
            for (size_t j = index - 1; j < index + 2; ++j) {
                const double heading
                        = gear_ ? path_points->at(j).theta() : NormalizeAngle(path_points->at(j).theta() + M_PI);
                Box2d ego_box(
                        {path_points->at(j).x() + center_shift_distance_ * std::cos(heading),
                         path_points->at(j).y() + center_shift_distance_ * std::sin(heading)},
                        heading,
                        ego_length_,
                        ego_width_);
                for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
                    for (const LineSegment2d& linesegment : obstacle_linesegments) {
                        if (ego_box.HasOverlap(linesegment)) {
                            is_colliding = true;
                            break;
                        }
                    }
                    if (is_colliding) {
                        break;
                    }
                }
                if (is_colliding) {
                    break;
                }
            }

            if (is_colliding) {
                // Adjust the point by randomly move around the original points
                if (index == 1) {
                    const double adjust_theta = path_points->at(index - 1).theta();
                    const double delta_s = std::abs(path_points->at(index).s() - path_points->at(index - 1).s());
                    double rand_dev = common::math::Clamp(length_dis(gen), 0.8, -0.8);
                    double adjusted_delta_s = delta_s * (1.0 + rand_dev);
                    path_points->at(index).set_x(
                            path_points->at(index - 1).x() + adjusted_delta_s * std::cos(adjust_theta));
                    path_points->at(index).set_y(
                            path_points->at(index - 1).y() + adjusted_delta_s * std::sin(adjust_theta));
                } else if (index == path_points->size() - 2) {
                    const double adjust_theta = NormalizeAngle(path_points->at(index + 1).theta() + M_PI);
                    const double delta_s = std::abs(path_points->at(index + 1).s() - path_points->at(index).s());
                    double rand_dev = common::math::Clamp(length_dis(gen), 0.8, -0.8);
                    double adjusted_delta_s = delta_s * (1.0 + rand_dev);
                    path_points->at(index).set_x(
                            path_points->at(index + 1).x() + adjusted_delta_s * std::cos(adjust_theta));
                    path_points->at(index).set_y(
                            path_points->at(index + 1).y() + adjusted_delta_s * std::sin(adjust_theta));
                } else {
                    double rand_dev_x = common::math::Clamp(
                            pos_dis(gen), 2.0 * reanchoring_pos_stddev, -2.0 * reanchoring_pos_stddev);
                    double rand_dev_y = common::math::Clamp(
                            pos_dis(gen), 2.0 * reanchoring_pos_stddev, -2.0 * reanchoring_pos_stddev);
                    path_points->at(index).set_x(path_points->at(index).x() + rand_dev_x);
                    path_points->at(index).set_y(path_points->at(index).y() + rand_dev_y);
                }

                // Adjust heading accordingly
                // TODO(Jinyun): refactor into math module
                // current point heading adjustment
                for (size_t i = index - 1; i < index + 2; ++i) {
                    path_points->at(i).set_theta(CalcHeadings(*path_points, i));
                }
            } else {
                reanchoring_success = true;
                break;
            }
        }

        if (!reanchoring_success) {
            AERROR << "interpolated points at index " << index << "can't be successfully reanchored";
            return false;
        }
    }
    return true;
}

bool IterativeAnchoringSmoother::GenerateInitialBounds(
        const DiscretizedPath& path_points,
        std::vector<double>* initial_bounds) {
    CHECK_NOTNULL(initial_bounds);
    initial_bounds->clear();

    const bool estimate_bound = iterative_anchoring_config_.estimate_bound();
    const double default_bound = iterative_anchoring_config_.default_bound();
    const double vehicle_shortest_dimension = iterative_anchoring_config_.vehicle_shortest_dimension();
    const double kEpislon = 1e-8;

    if (!estimate_bound) {
        std::vector<double> default_bounds(path_points.size(), default_bound);
        *initial_bounds = std::move(default_bounds);
        return true;
    }

    // TODO(Jinyun): refine obstacle formulation and speed it up
    for (const auto& path_point : path_points) {
        double min_bound = std::numeric_limits<double>::infinity();
        for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
            for (const LineSegment2d& linesegment : obstacle_linesegments) {
                min_bound = std::min(min_bound, linesegment.DistanceTo({path_point.x(), path_point.y()}));
            }
        }
        min_bound -= vehicle_shortest_dimension;
        min_bound = min_bound < kEpislon ? 0.0 : min_bound;
        initial_bounds->push_back(min_bound);
    }
    return true;
}

bool IterativeAnchoringSmoother::SmoothPath(
        const DiscretizedPath& raw_path_points,
        const std::vector<double>& bounds,
        DiscretizedPath* smoothed_path_points,
        std::vector<std::vector<common::math::Vec2d>> point_box) {
    std::vector<std::pair<double, double>> raw_point2d;
    std::vector<double> flexible_bounds;
    for (const auto& path_point : raw_path_points) {
        raw_point2d.emplace_back(path_point.x(), path_point.y());
    }
    flexible_bounds = bounds;

    FemPosDeviationSmoother fem_pos_smoother(iterative_anchoring_config_.fem_pos_deviation_smoother_config());

    // TODO(Jinyun): move to confs
    const size_t max_iteration_num = 50;

    bool is_collision_free = false;
    std::vector<size_t> colliding_point_index;
    std::vector<std::pair<double, double>> smoothed_point2d;
    size_t counter = 0;

    while (!is_collision_free) {
        if (counter > max_iteration_num) {
            AERROR << "Maximum iteration reached, path smoother early stops";
            return true;
        }

        AdjustPathBounds(colliding_point_index, &flexible_bounds);

        std::vector<double> opt_x;
        std::vector<double> opt_y;
        if (!fem_pos_smoother.Solve(raw_point2d, flexible_bounds, &opt_x, &opt_y)) {
            AERROR << "Smoothing path fails";
            return false;
        }

        if (opt_x.size() < 2 || opt_y.size() < 2) {
            AERROR << "Return by fem_pos_smoother is wrong. Size smaller than 2 ";
            return false;
        }

        CHECK_EQ(opt_x.size(), opt_y.size());

        size_t point_size = opt_x.size();
        smoothed_point2d.clear();
        for (size_t i = 0; i < point_size; ++i) {
            smoothed_point2d.emplace_back(opt_x[i], opt_y[i]);
        }

        if (!SetPathProfile(smoothed_point2d, smoothed_path_points)) {
            AERROR << "Set path profile fails";
            return false;
        }

        is_collision_free = CheckCollisionAvoidance(*smoothed_path_points, &colliding_point_index);

        ADEBUG << "loop iteration number is " << counter;
        ++counter;
    }
    return true;
}

bool IterativeAnchoringSmoother::CheckCollisionAvoidance(
        const DiscretizedPath& path_points,
        std::vector<size_t>* colliding_point_index) {
    CHECK_NOTNULL(colliding_point_index);

    colliding_point_index->clear();
    size_t path_points_size = path_points.size();
    for (size_t i = 0; i < path_points_size; ++i) {
        // Skip checking collision for thoese points colliding originally
        bool skip_checking = false;
        for (const auto index : input_colliding_point_index_) {
            if (i == index) {
                skip_checking = true;
                break;
            }
        }
        if (skip_checking) {
            continue;
        }

        const double heading = gear_ ? path_points[i].theta() : NormalizeAngle(path_points[i].theta() + M_PI);
        Box2d ego_box(
                {path_points[i].x() + center_shift_distance_ * std::cos(heading),
                 path_points[i].y() + center_shift_distance_ * std::sin(heading)},
                heading,
                ego_length_,
                ego_width_);

        bool is_colliding = false;
        for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
            for (const LineSegment2d& linesegment : obstacle_linesegments) {
                if (ego_box.HasOverlap(linesegment)) {
                    colliding_point_index->push_back(i);
                    ADEBUG << "point at " << i << "collied with LineSegment " << linesegment.DebugString();
                    is_colliding = true;
                    break;
                }
            }
            if (is_colliding) {
                break;
            }
        }
    }

    if (!colliding_point_index->empty()) {
        return false;
    }
    return true;
}

void IterativeAnchoringSmoother::AdjustPathBounds(
        const std::vector<size_t>& colliding_point_index,
        std::vector<double>* bounds) {
    CHECK_NOTNULL(bounds);

    const double collision_decrease_ratio = iterative_anchoring_config_.collision_decrease_ratio();

    for (const auto index : colliding_point_index) {
        bounds->at(index) *= collision_decrease_ratio;
    }

    // Anchor the end points to enforce the initial end end heading continuity and
    // zero kappa
    bounds->at(0) = 0.0;
    bounds->at(1) = 0.0;
    bounds->at(bounds->size() - 1) = 0.0;
    bounds->at(bounds->size() - 2) = 0.0;
    if (enforce_initial_kappa_) {
        bounds->at(2) = 0.0;
    }
}

bool IterativeAnchoringSmoother::SetPathProfile(
        const std::vector<std::pair<double, double>>& point2d,
        DiscretizedPath* raw_path_points) {
    CHECK_NOTNULL(raw_path_points);
    raw_path_points->clear();
    // Compute path profile
    std::vector<double> headings;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    std::vector<double> accumulated_s;
    if (!DiscretePointsMath::ComputePathProfile(point2d, &headings, &accumulated_s, &kappas, &dkappas)) {
        return false;
    }
    CHECK_EQ(point2d.size(), headings.size());
    CHECK_EQ(point2d.size(), kappas.size());
    CHECK_EQ(point2d.size(), dkappas.size());
    CHECK_EQ(point2d.size(), accumulated_s.size());

    // Load into path point
    size_t points_size = point2d.size();
    for (size_t i = 0; i < points_size; ++i) {
        PathPoint path_point;
        path_point.set_x(point2d[i].first);
        path_point.set_y(point2d[i].second);
        path_point.set_theta(headings[i]);
        path_point.set_s(accumulated_s[i]);
        path_point.set_kappa(kappas[i]);
        path_point.set_dkappa(dkappas[i]);
        raw_path_points->push_back(std::move(path_point));
    }
    return true;
}

bool IterativeAnchoringSmoother::CheckGear(const Eigen::MatrixXd& xWS) {
    CHECK_GT(xWS.size(), 1);
    double init_heading_angle = xWS(2, 0);
    const Vec2d init_tracking_vector(xWS(0, 1) - xWS(0, 0), xWS(1, 1) - xWS(1, 0));
    double init_tracking_angle = init_tracking_vector.Angle();
    return std::abs(NormalizeAngle(init_tracking_angle - init_heading_angle)) < M_PI_2;
}

bool IterativeAnchoringSmoother::SmoothSpeed(
        const double init_a,
        const double init_v,
        const double path_length,
        SpeedData* smoothed_speeds) {
    const double max_forward_v = FLAGS_open_space_max_forward_v;
    const double max_reverse_v = FLAGS_open_space_max_reverse_v;
    const double max_forward_acc = FLAGS_open_space_max_forward_acc;
    const double max_reverse_acc = FLAGS_open_space_max_reverse_acc;
    const double max_acc_jerk = FLAGS_open_space_max_jerk;
    const double delta_t = FLAGS_open_space_delta_t;

    const double total_t = 2 * path_length / max_reverse_acc * 10;
    ADEBUG << "total_t is : " << total_t << "path_length" << path_length;
    const size_t num_of_knots = static_cast<size_t>(total_t / delta_t) + 1;

    PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t, {0.0, std::abs(init_v), 0.0});

    // set end constraints
    std::vector<std::pair<double, double>> x_bounds(num_of_knots, {0.0, path_length});

    const double max_v = gear_ ? max_forward_v : max_reverse_v;
    const double max_acc = gear_ ? max_forward_acc : max_reverse_acc;

    const auto upper_dx = std::fmax(max_v, std::abs(init_v));
    std::vector<std::pair<double, double>> dx_bounds(num_of_knots, {0.0, upper_dx});
    std::vector<std::pair<double, double>> ddx_bounds(num_of_knots, {-max_acc, max_acc});

    x_bounds[num_of_knots - 1] = std::make_pair(path_length, path_length);
    dx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);
    ddx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);

    std::vector<double> x_ref(num_of_knots, path_length);
    piecewise_jerk_problem.set_x_ref(FLAGS_open_space_reference_s_weight, std::move(x_ref));
    piecewise_jerk_problem.set_weight_ddx(FLAGS_open_space_acc_weight);
    piecewise_jerk_problem.set_weight_dddx(FLAGS_open_space_jerk_weight);
    piecewise_jerk_problem.set_x_bounds(x_bounds);
    piecewise_jerk_problem.set_dx_bounds(dx_bounds);
    piecewise_jerk_problem.set_ddx_bounds(ddx_bounds);
    piecewise_jerk_problem.set_dddx_bound(max_acc_jerk);

    // Solve the problem
    if (!piecewise_jerk_problem.Optimize()) {
        AERROR << "Piecewise jerk speed optimizer failed!";
        AERROR << "init v" << init_v << ",a" << init_a << "jerk bound" << max_acc_jerk;
        AERROR << "xbounds; dxbounds;ddxbounds";
        for (size_t i = 0; i < num_of_knots; i++) {
            AERROR << x_bounds[i].first << ", " << x_bounds[i].second << ";" << dx_bounds[i].first << ","
                   << dx_bounds[i].second << ";" << ddx_bounds[i].first << "," << ddx_bounds[i].second;
        }
        return false;
    }

    // Extract output
    const std::vector<double>& s = piecewise_jerk_problem.opt_x();
    const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
    const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

    // Assign speed point by gear
    smoothed_speeds->AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
    const double kEpislon = 1.0e-1;
    const double sEpislon = 1.0e-1;
    for (size_t i = 1; i < num_of_knots; ++i) {
        if (s[i - 1] - s[i] > kEpislon) {
            AERROR << "unexpected decreasing s in speed smoothing at time " << static_cast<double>(i) * delta_t
                   << "with total time " << total_t;
            return false;
        }
        smoothed_speeds->AppendSpeedPoint(
                s[i], delta_t * static_cast<double>(i), ds[i], dds[i], (dds[i] - dds[i - 1]) / delta_t);
        // Cut the speed data when it is about to meet end condition
        if (path_length - s[i] < sEpislon) {
            break;
        }
    }
    return true;
}

bool IterativeAnchoringSmoother::CombinePathAndSpeed(
        const DiscretizedPath& path_points,
        const SpeedData& speed_points,
        DiscretizedTrajectory* discretized_trajectory) {
    CHECK_NOTNULL(discretized_trajectory);
    discretized_trajectory->clear();
    // TODO(Jinyun): move to confs
    const double kDenseTimeResolution = 0.1;
    const double time_horizon = speed_points.TotalTime() + kDenseTimeResolution * 1.0e-6;
    if (path_points.empty()) {
        AERROR << "path data is empty";
        return false;
    }
    ADEBUG << "speed_points.TotalTime() " << speed_points.TotalTime();
    for (double cur_rel_time = 0.0; cur_rel_time < time_horizon; cur_rel_time += kDenseTimeResolution) {
        common::SpeedPoint speed_point;
        if (!speed_points.EvaluateByTime(cur_rel_time, &speed_point)) {
            AERROR << "Fail to get speed point with relative time " << cur_rel_time;
            return false;
        }

        if (speed_point.s() > path_points.Length()) {
            break;
        }

        common::PathPoint path_point = path_points.Evaluate(speed_point.s());

        common::TrajectoryPoint trajectory_point;
        trajectory_point.mutable_path_point()->CopyFrom(path_point);
        trajectory_point.set_v(speed_point.v());
        trajectory_point.set_a(speed_point.a());
        trajectory_point.set_relative_time(speed_point.t());
        discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
    }

    common::PathPoint path_point = path_points.Evaluate(path_points.Length());
    common::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(0.0);
    trajectory_point.set_a(0.0);
    trajectory_point.set_relative_time(speed_points.TotalTime() + kDenseTimeResolution);
    discretized_trajectory->AppendTrajectoryPoint(trajectory_point);

    ADEBUG << "path length before combine " << path_points.Length();
    ADEBUG << "trajectory length after combine " << discretized_trajectory->GetSpatialLength();
    return true;
}

void IterativeAnchoringSmoother::AdjustPathAndSpeedByGear(DiscretizedTrajectory* discretized_trajectory) {
    if (gear_) {
        return;
    }
    std::for_each(
            discretized_trajectory->begin(), discretized_trajectory->end(), [](TrajectoryPoint& trajectory_point) {
                trajectory_point.mutable_path_point()->set_theta(
                        NormalizeAngle(trajectory_point.path_point().theta() + M_PI));
                trajectory_point.mutable_path_point()->set_s(-1.0 * trajectory_point.path_point().s());
                trajectory_point.mutable_path_point()->set_kappa(-1.0 * trajectory_point.path_point().kappa());
                // dkappa stays the same as direction of both kappa and s are reversed
                trajectory_point.set_v(-1.0 * trajectory_point.v());
                trajectory_point.set_a(-1.0 * trajectory_point.a());
            });
}

bool IterativeAnchoringSmoother::GenerateStopProfileFromPolynomial(
        const double init_acc,
        const double init_speed,
        const double stop_distance,
        SpeedData* smoothed_speeds) {
    static constexpr double kMaxT = 8.0;
    static constexpr double kUnitT = 0.2;
    for (double t = 2.0; t <= kMaxT; t += kUnitT) {
        QuinticPolynomialCurve1d curve(0.0, init_speed, init_acc, stop_distance, 0.0, 0.0, t);
        if (!IsValidPolynomialProfile(curve)) {
            continue;
        }
        for (double curve_t = 0.0; curve_t <= t; curve_t += kUnitT) {
            const double curve_s = curve.Evaluate(0, curve_t);
            const double curve_v = curve.Evaluate(1, curve_t);
            const double curve_a = curve.Evaluate(2, curve_t);
            const double curve_da = curve.Evaluate(3, curve_t);
            smoothed_speeds->AppendSpeedPoint(curve_s, curve_t, curve_v, curve_a, curve_da);
        }
        return true;
    }
    AERROR << "GenerateStopProfileFromPolynomial fails.";
    return false;
}

bool IterativeAnchoringSmoother::IsValidPolynomialProfile(const QuinticPolynomialCurve1d& curve) {
    for (double evaluate_t = 0.1; evaluate_t <= curve.ParamLength(); evaluate_t += 0.2) {
        const double v = curve.Evaluate(1, evaluate_t);
        const double a = curve.Evaluate(2, evaluate_t);
        static constexpr double kEpsilon = 1e-3;
        if (v < -kEpsilon || a > 1.0) {
            return false;
        }
    }
    return true;
}

double IterativeAnchoringSmoother::CalcHeadings(const DiscretizedPath& path_points, const size_t index) {
    CHECK_GT(path_points.size(), 2U);
    double dx = 0.0;
    double dy = 0.0;
    if (index == 0) {
        dx = path_points[index + 1].x() - path_points[index].x();
        dy = path_points[index + 1].y() - path_points[index].y();
    } else if (index == path_points.size() - 1) {
        dx = path_points[index].x() - path_points[index - 1].x();
        dy = path_points[index].y() - path_points[index - 1].y();
    } else {
        dx = 0.5 * (path_points[index + 1].x() - path_points[index - 1].x());
        dy = 0.5 * (path_points[index + 1].y() - path_points[index - 1].y());
    }
    return std::atan2(dy, dx);
}
}  // namespace planning
}  // namespace apollo
