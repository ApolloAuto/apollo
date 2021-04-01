Generate final trajectory based on the open space.

# Introduction
The goal of this part is to generate the final trajectory in the open space.open_space_trajectory_provider is very important to control the flow and call of hybridastar and trajectory smoothing algorithm. 

# Where is the code
Please refer [code](https://github.com/ApolloAuto/apollo/tree/master/modules/planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_provider.cc)

# Code Reading
1. Input:  open_space_trajectory_provider::Process() is called by the OPEN_SPACE_TRAJECTORY_PROVIDER task of VALET_PARKING_PARKING stage, please refer (https://github.com/ApolloAuto/apollo/blob/master/modules/planning/conf/scenario/valet_parking_config.pb.txt).

2. Process1: there is a stop trajectory which generated in the park and go check stage. In order to ensure safety, it is necessary in this case. 
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
3. Process2: Start thread when getting in Process() for the first time. This will call the GenerateTrajectoryThread() function to plan the first trajectory and will update three kinds of trajectory state: trajectory_updated_, trajectory_skipped_, trajectory_error_.
    ``` cpp
    if (FLAGS_enable_open_space_planner_thread && !thread_init_flag_) {
        task_future_ = cyber::Async(&OpenSpaceTrajectoryProvider::GenerateTrajectoryThread, this);
        thread_init_flag_ = true;
  }
    ```
4. Process3: whether the vehicle stop is due to fallback is determined by the IsVehicleStopDueToFallBack().This determines the final trajectory planning.
5. Process4: if vehicle stop caused by fallback,replanning stitching trajectory by ComputeReinitStitchingTrajectory(). If not,replanning stitching trajectory by ComputeStitchingTrajectory(), please refer(https://github.com/ApolloAuto/apollo/blob/master/modules/planning/common/trajectory_stitcher.cc).
6. Process5: The generation of trajectory depends on the FLAGS_enable_open_space_planner_thread. A stop trajectory is generated in the following cases: 1.planning thread is stop; 2.the vehicle arrives near the destination; 3. trajectory_error_ is triggered for more than 10 seconds; 4. previous frame planning failed. If the trajectory can be updated normally, the optimized trajectory is output normally. 

7. Output: the optput is final trajectory information.


# Algorithm Detail
    ``` cpp
    bool OpenSpaceTrajectoryProvider::IsVehicleStopDueToFallBack(const bool is_on_fallback, 
                                                                 const common::VehicleState& vehicle_state)
    ```
    the function is used to judge whether the vehicle stop is due to fallback.
1. parameter: the input parameter is fallback flag of previous frame and vehicle states.
2. introduction: the flag and vehicle state can be used to design the logic. 
3. Process: 1. fallback flag judgment: if the flag is false, then return false; 
            2. When the vehicle speed and acceleration are less than the threshold, the result is true, indicating that it is caused by fall back.

    ``` cpp
    std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(const VehicleState& vehicle_state, 
                                                                                const double current_timestamp,
                                                                                const double planning_cycle_time, 
                                                                                const size_t preserved_points_num,
                                                                                const bool replan_by_offset, 
                                                                                const PublishableTrajectory* prev_trajectory,
                                                                                std::string* replan_reason)
    ```
    the function is used to stitch trajectory and is used to replan based on some unreasonable case.
1. parameter: vehicle state, current_timestamp, planning cycle time, replan_by_offset, previous trajectory and the reason of replanning.
2. introduction: handle some unreasonable case by replanning, stitch trajectory and post process. 
3. process: 1.the following case will replan:1.stitching trajectory are not allowed; 2.no previous trajectory; 3.not in autopilot mode; 4.the number of points in the previous frame is zero; 5.the current time is less than the trajectory start time of the previous frame; 6.the current time is more than the trajetory end time of the previous frame; 7. the matching path point is empty; 8.the horizontal and vertical deviation of the projection point is greater than the threshold. 2.stitch trajectory according to the planning period and the position of the projection point; 3.Determine whether each trajectory point of the stitching trajectory is empty, and if it is empty, will replan.

    ``` cpp
    std::vector<TrajectoryPoint>TrajectoryStitcher::ComputeReinitStitchingTrajectory(const double planning_cycle_time, 
                                                                                     const VehicleState& vehicle_state)
    ```
    the function is used to initialize stitching trajectory and get the initial point. 
1. parameter: the planned cycle time and vehicle state
2. introduction: the function can get the diffrent initial point based on the different logic.
3. process:1. When the vehicle speed and acceleration are less than the threshold, the message of initial point is from vehicle state;              2. When the vehicle speed and acceleration satisfy the threshold, the vehicle state if from the predicted state based on                    bicycle model.

