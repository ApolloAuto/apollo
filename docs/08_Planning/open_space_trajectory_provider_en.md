# GENERATE FINAL TRAJECTORY

# Introduction
The goal of this part is to generate the final trajectory in the open space. Open_space_trajectory_provider is very important to control the flow and call the hybrid a star and trajectory smoothing algorithm. 

# Where is the code
Please refer to [open_space_trajectory_provider.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/tasks/open_space_trajectory_provider/open_space_trajectory_provider.cc)

# Code Reading
1. Input: open_space_trajectory_provider::Process() is called by the OPEN_SPACE_TRAJECTORY_PROVIDER task of VALET_PARKING_PARKING stage.

2. There is a stop trajectory which generated in the park and go check stage. In order to ensure safety, it is necessary in this case. 
    ``` cpp
    if (injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_park_and_go()
          ->in_check_stage()) {
        ADEBUG << "ParkAndGo Stage Check.";
        GenerateStopTrajectory(trajectory_data);
        return Status::OK();
    }
    ```
3. Start thread when getting in Process() for the first time. This will call the GenerateTrajectoryThread() function to plan the first trajectory and will update three kinds of trajectory state: trajectory_updated_, trajectory_skipped_, trajectory_error_.
    ``` cpp
    if (FLAGS_enable_open_space_planner_thread && !thread_init_flag_) {
        task_future_ = cyber::Async(&OpenSpaceTrajectoryProvider::GenerateTrajectoryThread, this);
        thread_init_flag_ = true;
    }
    ```
4. Whether vehicle is stoped due to fallback is determined by the IsVehicleStopDueToFallBack() function. This determines the final trajectory planning.
5. If vehicle is stopped due to fallback, replan stitching trajectory by ComputeReinitStitchingTrajectory() function. If not, replan stitching trajectory by ComputeStitchingTrajectory(), please refer to [trajectory_stitcher.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/planning_base/common/trajectory_stitcher.cc).
6. Generate trajectory depends on the FLAGS_enable_open_space_planner_thread. A stop trajectory is generated in the following cases:
   1. Planning thread is stopped. 
   2. The vehicle arrives near the destination. 
   3. trajectory_error_ is triggered for more than 10 seconds.
   4. Previous frame planning failed. 
   5. If the trajectory can be updated normally, the optimized trajectory is output normally. 

7. Output: the optput is final trajectory information.

# Algorithm Detail
    ``` cpp
    bool OpenSpaceTrajectoryProvider::IsVehicleStopDueToFallBack(const bool is_on_fallback, 
                                                                 const common::VehicleState& vehicle_state)
    ```
    The function is used to judge whether the vehicle is stopped due to fallback.
1. Parameter: The input parameter is fallback flag of previous frame and vehicle states.
2. Introduction: The flag and vehicle state can be used to design the logic. 
3. Process detail: 
    1. Fallback flag judgment: if the flag is false, then return false. 
    2. When the vehicle speed and acceleration are less than the threshold, the result is true, indicating that it is caused by fall        back.

    ``` cpp
    std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(const VehicleState& vehicle_state, 
                                                                                const double current_timestamp,
                                                                                const double planning_cycle_time, 
                                                                                const size_t preserved_points_num,
                                                                                const bool replan_by_offset, 
                                                                                const PublishableTrajectory* prev_trajectory,
                                                                                std::string* replan_reason)
    ```
    The function is used to stitch trajectory and is used to replan based on some unreasonable case.
1. Parameter: Vehicle state, current_timestamp, planning cycle time, replan_by_offset, previous trajectory and the reason of replanning.
2. Introduction: Handle some unreasonable case by replanning, stitch trajectory and post process. 
3. Process detail: 
   1. It will re-plan the trajectory in following cases:
      1. Stitching trajectory is disabled by gflag.
      2. There is no previous trajectory.
      3. Not in autopilot mode. 
      4. The number of points in the previous frame is zero. 
      5. The current time is less than the trajectory start time of the previous frame.
      6. The current time is more than the trajetory end time of the previous frame.
      7. The matching path point is empty. 
      8. The horizontal and vertical deviation of the projection point is greater than the threshold. 
    2. Stitch trajectory according to the planning period and the position of the projection point.
    3. Determine whether each trajectory point of the stitching trajectory is empty, and if it is empty, it will replan.

    ``` cpp
    std::vector<TrajectoryPoint>TrajectoryStitcher::ComputeReinitStitchingTrajectory(const double planning_cycle_time, 
                                                                                     const VehicleState& vehicle_state)
    ```
    The function is used to initialize stitching trajectory and get the initial point. 
1. Parameter: The planned cycle time and vehicle state
2. Introduction: The function can get the diffrent initial point based on the different logic.
3. Process detail:
   1. When the vehicle speed and acceleration are less than the threshold, the message of initial point is from vehicle state.           2. When the vehicle speed and acceleration satisfy the threshold, the vehicle state is calculated based on the kinematics model.


