Optimize trajectory based on the coarse trajectory.

# Introduction
The goal of this part is to optimizes the initial trajectory in the open space.
open_space_trajectory_optimizer is able to call a variety of different optimization algorithms.

# Where is the code
Please refer [code](https://github.com/ApolloAuto/apollo/tree/master/modules/planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_optimizer.cc)

# Code Reading
1. Input: The meaning of the input information is as follows: stitching trajectory form the open_space_trajectory_provider, planned target point, boundary of x and y, rotation angle relative to the corner of parking space, the reference origin point,line segment of boundary.
   ``` cpp
   Status OpenSpaceTrajectoryOptimizer::Plan(const std::vector<common::TrajectoryPoint>& stitching_trajectory,
                                             const std::vector<double>& end_pose, 
                                             const std::vector<double>& XYbounds,
                                             double rotate_angle, 
                                             const Vec2d& translate_origin,
                                             const Eigen::MatrixXi& obstacles_edges_num,
                                             const Eigen::MatrixXd& obstacles_A, 
                                             const Eigen::MatrixXd& obstacles_b,
                                             const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
                                             double* time_latency)
   ```

2. Process1: Before optimization, some unreasonable cases are exited from the optimization process and implement some preprocessing. The following case: the input data is empty, the starting point of planning is near the end point. The end of the stitching trajectory is rotated and translated, and the trajectory information is converted according to the corner of the parking space.
   ``` cpp
   if (XYbounds.empty() || end_pose.empty() || obstacles_edges_num.cols() == 0 ||
       obstacles_A.cols() == 0 || obstacles_b.cols() == 0) {
       ADEBUG << "OpenSpaceTrajectoryOptimizer input data not ready";
       return Status(ErrorCode::PLANNING_ERROR, "OpenSpaceTrajectoryOptimizer input data not ready");
  }
   ```
  ``` cpp
  if (IsInitPointNearDestination(stitching_trajectory.back(), end_pose,
                                 rotate_angle, translate_origin)) {
     ADEBUG << "Planning init point is close to destination, skip new "
               "trajectory generation";
     return Status(ErrorCode::OK,
                  "Planning init point is close to destination, skip new "
                  "trajectory generation");
  }
  ```
  ``` cpp
  PathPointNormalizing(rotate_angle, translate_origin, &init_x, &init_y,
                       &init_phi)
  ```
3. Process2: Generate the coarse trajectory based on the warm start technology that is Hybrid A* algorithm.
   ``` cpp
   if (warm_start_->Plan(init_x, init_y, init_phi, end_pose[0], end_pose[1],
                        end_pose[2], XYbounds, obstacles_vertices_vec,
                        &result)) {
    ADEBUG << "State warm start problem solved successfully!";
    } else {
        ADEBUG << "State warm start problem failed to solve";
        return Status(ErrorCode::PLANNING_ERROR,
                    "State warm start problem failed to solve");
    }
   ```
4. Process3: According to FLAGS_enable_parallel_trajectory_smoothing to achieve different optimization process. When FLAGS_enable_parallel_trajectory_smoothing is false, the optimization process is as follows: first, the (x, y, phi, V) and (ster, a) of the initial trajectory points in hybrid_a_star are stored into xws and UWS respectively through LoadHybridAstarResultInEigen(), and xws and UWS are used to generate the subsequent smooth trajectory; second, generate the smooth trajectory by the function GenerateDistanceApproachTraj()
``` cpp
    LoadHybridAstarResultInEigen(&result, &xWS, &uWS);

    const double init_steer = trajectory_stitching_point.steer();
    const double init_a = trajectory_stitching_point.a();
    Eigen::MatrixXd last_time_u(2, 1);
    last_time_u << init_steer, init_a;

    const double init_v = trajectory_stitching_point.v();

    if (!GenerateDistanceApproachTraj(
            xWS, uWS, XYbounds, obstacles_edges_num, obstacles_A, obstacles_b,
            obstacles_vertices_vec, last_time_u, init_v, &state_result_ds,
            &control_result_ds, &time_result_ds, &l_warm_up, &n_warm_up,
            &dual_l_result_ds, &dual_n_result_ds)) {
      return Status(ErrorCode::PLANNING_ERROR,
                    "distance approach smoothing problem failed to solve");
    }
```

5. Process4: When FLAGS_enable_parallel_trajectory_smoothing is true,the optimization process is as follows: first,the trajectorypartition() is used to segment the initial trajectory; second, Use loadhybridastarresultineigen() to store the partitioned trajetory into xws and UWS respectively; third, set the initial information(steer,a,V) of each trajectory; the initial information of the first trajectory is the end point of the stitching trajectory; in the next trajectory, the initial information is set to zero, because the next trajectory always starts from the static state of the vehicle.
    ``` cpp
    if (!warm_start_->TrajectoryPartition(result, &partition_trajectories)) {
        return Status(ErrorCode::PLANNING_ERROR, "Hybrid Astar partition failed");
    }
    ```
6. Process5: Use combinetrajectories() to integrate the parameter information after segmented optimization.
   ``` cpp
   CombineTrajectories(xWS_vec, uWS_vec, state_result_ds_vec,
                       control_result_ds_vec, time_result_ds_vec,
                       l_warm_up_vec, n_warm_up_vec, dual_l_result_ds_vec,
                       dual_n_result_ds_vec, &xWS, &uWS, &state_result_ds,
                       &control_result_ds, &time_result_ds, &l_warm_up,
                       &n_warm_up, &dual_l_result_ds, &dual_n_result_ds)
   ```
7. Process6: Converting trajectory information to world coordinate system.
   ``` cpp
   for (size_t i = 0; i < state_size; ++i) {
        PathPointDeNormalizing(rotate_angle, translate_origin,
                               &(state_result_ds(0, i)), 
                               &(state_result_ds(1, i)),
                               &(state_result_ds(2, i)));
  }
   ```
8. Process7: The trajectory information is loaded by loadtrajectory(). Because the current optimization does not consider the end point control state, the end-point control state of the trajectory is processed (Steer = 0, a = 0).
    ``` cpp
    LoadTrajectory(state_result_ds, control_result_ds, time_result_ds)
    ```

9. Output: the optput is optimized trajectory information,


# Algorithm Detail
    ```cpp
    LoadHybridAstarResultInEigen(&partition_trajectories[i], &xWS_vec[i],&uWS_vec[i])
    ```
    the function is to transform the initial trajectory information into the form needed for optimization.
1. parameter:the initial trajectory and parameter matrix.
2. introduction:the trajectory information is transformed into matrix form based on horizon.
3. process: 1. transform the x,y,phi,v,steer to the matrix combined with horizon; 2. store the transformed information to the matrix.

