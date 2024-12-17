# OPEN SPACE TRAJECTORY PARTITION

# Introduction

Apollo planning is scenario based, where each driving use case is treated as a different driving scenario.

Open space trajectory partition task is used to partition and optimize stiched trajectroy obtained from open space trajectory provider task.

# Where is the code

Please refer [open space trajectory parition](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/tasks/open_space_trajectory_partition/open_space_trajectory_partition.cc).

# Code Reading

1. Input : stitched trajectory(without optimization) / vehicle position info.

2. The interpolated trajectory is obtained by calling ```InterpolateTrajectory()``` to increase stitched trajectory points.
    ```cpp
    void InterpolateTrajectory(
        const DiscretizedTrajectory& stitched_trajectory_result,
        DiscretizedTrajectory* interpolated_trajectory);
    ```
3. According to the heading angle and traking angle, the gear shift point can be determined.
   Use ```std::vector<TrajGearPair>``` to store gear infomation and trajectory points, then partition trajectory into a group of trajectories from the gear shift point.
    ```cpp
    void PartitionTrajectory(const DiscretizedTrajectory& trajectory,
                             std::vector<TrajGearPair>* partitioned_trajectories);       
    ```
4. If replan due to fallback stop, the position init staus will be set to false.
   
   When replan success, we use ```AdjustRelativeTimeAndS()``` to adjust partitioned trajectories obtained from step 3.
    ```cpp
    void AdjustRelativeTimeAndS(
        const std::vector<TrajGearPair>& partitioned_trajectories,
        const size_t current_trajectory_index,
        const size_t closest_trajectory_point_index,
        DiscretizedTrajectory* stitched_trajectory_result,
        TrajGearPair* current_partitioned_trajectory);
    ```
5. When fallback is not required, choose the closest partitioned trajectory to follow.
   
   1. Base on ADC position, heading, body size and velocity info, we get vehicle center point info and moving direction.
    ```cpp
        void UpdateVehicleInfo();
    ```
   2. Encode partitioned trajectories;
    ```cpp
        bool EncodeTrajectory(const DiscretizedTrajectory& trajectory,
                            std::string* const encoding);     
    ```
   3. To find the closest point on trajectory, a search range needed to be determined. It requires the distance between path end point and ADC enter point, the heading search difference and the head track difference all within the threshold.

      Base on ADC box and path point box, the IOU(intersection over union) is computed for each path point in the search range. 

      If the IOU of the trajectory end point is bigger than threshold and partitioned trajectories group has other trajectory can be used, it means ADC reach the end of a trajectory, another trajectory to be used.

      Then update trajectory history to store which trajectory has been used.
    ```cpp
        bool CheckReachTrajectoryEnd(const DiscretizedTrajectory& trajectory,
                                    const canbus::Chassis::GearPosition& gear,
                                    const size_t trajectories_size,
                                    const size_t trajectories_index,
                                    size_t* current_trajectory_index,
                                    size_t* current_trajectory_point_index);
    ```
    ```cpp
        bool CheckTrajTraversed(const std::string& trajectory_encoding_to_check);
    ```
    ```cpp
        void UpdateTrajHistory(const std::string& chosen_trajectory_encoding);
    ```
   4. When there is no need for ADC to switch to next trajectory, use IOU info mentioned above to find the closest trajectory point(the biggest IOU point) to follow.

   5. If the closest trajectory point doesn't belong to current trajectory or couldn't find closest trajectory point due to some unnormal cases, we use ```UseFailSafeSearch()``` to get a safe trajectory to follow.

      When using this function, we only care about distance between path end point and ADC enter point to find the search range, no more limitation on angle difference.
    ```cpp
        bool UseFailSafeSearch(
            const std::vector<TrajGearPair>& partitioned_trajectories,
            const std::vector<std::string>& trajectories_encodings,
            size_t* current_trajectory_index, size_t* current_trajectory_point_index);
    ```
   6. If ```FLAGS_use_gear_shift_trajectory()``` set to be true, a small part trajectory obtained by calling ```GenerateGearShiftTrajectory()``` will be added to make vehicle moving smoothly during gear shifting.
   Otherwise we use ```AdjustRelativeTimeAndS()``` to adjust partitioned trajectory.
    ```cpp
        bool InsertGearShiftTrajectory(
            const bool flag_change_to_next, const size_t current_trajectory_index,
            const std::vector<TrajGearPair>& partitioned_trajectories,
            TrajGearPair* gear_switch_idle_time_trajectory);
    ```
    ```cpp
        void GenerateGearShiftTrajectory(
            const canbus::Chassis::GearPosition& gear_position,
            TrajGearPair* gear_switch_idle_time_trajectory);
    ```
    ```cpp
        void AdjustRelativeTimeAndS(
            const std::vector<TrajGearPair>& partitioned_trajectories,
            const size_t current_trajectory_index,
            const size_t closest_trajectory_point_index,
            DiscretizedTrajectory* stitched_trajectory_result,
            TrajGearPair* current_partitioned_trajectory);
    ```
6. Return process status.   

7. Output: partitioned trajectories.

               

