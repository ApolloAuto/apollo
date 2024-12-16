# PARKING SCENARIO

# Introduction

Apollo planning is scenario based, where each driving use case is treated as a different driving scenario.

There are three scenairos, park and go, pull over and valet parking, which related to park planning.

1. park and go: the park and go scenario was designed to handle curb side parking, planning a new trajectory to the next destination and then driving along that trajectory. This scenario is extremely useful in situations like curb-side delivery or passenger pickup or drop-off. 

2. pull over: the pull over scenario was designed especially for maneuvering to the side of the road upon reaching your destination like for curb-side parallel parking. 

3. valet parking: the valet parking scenario was designed to safely park your ego car in a targeted parking spot.

# Where is the code

Please refer [park](https://github.com/ApolloAuto/apollo/tree/master/modules/planning/scenarios/valet_parking) & [park and go](https://github.com/ApolloAuto/apollo/tree/master/modules/planning/scenarios/park_and_go).

# Code Reading

All three scenarios contain specific stages, the function of scenarios are realized through the conversion of stages. Thus, figure out the process of stages conversion is the key to understand this code.  

## PARK AND GO SCENARIO
1. This scenario consists of four stages, check stage, adjust stage, pre cruise stage and cruise stage. 

2. check stage:
 
   1. In check stage, by calling ```checkadcreadytocruise()```to check whether ADC's gear info, ADC's velocity, obstacle position, ADC's heading and ADC's lateral station meet the requirements.
   ```cpp
        bool CheckADCReadyToCruise(
            const common::VehicleStateProvider* vehicle_state_provider, Frame* frame,
            const ScenarioParkAndGoConfig& scenario_config);
   ```
   2. If ADC is ready to cruise, check stage is finished and we switch to cruise stage. Otherwise we switch to adjust stage.

3. adjust stage:
  
   1. In adjust stage, we run open space planning algorithms to adjust ADC position.
   ```cpp
        bool ExecuteTaskOnOpenSpace(Frame* frame);
   ```
   2. Once position adjustment is done, we check whether ADC reaches the end of trajectory.

   3. Then we check whether ADC is ready to cruise by calling ```CheckADCReadyToCruise()```.

   4. If ADC is ready to cruise and reaches the end of trajectory, adjust stage is finished.
 
      1. If steering percentage within the threshold, we switch to cruise stage.

      2. Otherwise we reset init position of ADC and switch to pre cruise stage.
     ```cpp
          void ResetInitPostion();
     ```
   5. Otherwise ADC stay in adjust stage to adjust ADC position.

4. pre cruise stage:
  
   1. In pre cruise stage, we run open space planning algorithms to adjust ADC with the init position.

   2. Then we check whether the steering percentage within the threshold.

   3. If so, pre cruise stage is finished and we switch to cruise stage.

   4. Otherwise ADC stay in pre cruise stage.

5. cruise stage: 
   1. We run an on lane planning algorithms to adjust ADC position.
   ```cpp
        bool ExecuteTaskOnReferenceLine(
            const common::TrajectoryPoint& planning_start_point, Frame* frame);         
   ```
   2. Then we check whether the lateral distacne between ADC and target line within the threshold.
   ```cpp
        ParkAndGoStatus CheckADCParkAndGoCruiseCompleted(
            const ReferenceLineInfo& reference_line_info);
   ```
   3. If so, cruise stage is finished and quit park and go scenario is done.

   4. Otherwise ADC stay in cruise stage until ADC cruises to a desired position.

   5. The conversion of stages can be seen in 
    ![Diagram](images/parking_scenairo_fig_1.png).          

## PULL OVER SCENARIO
1. This scenario consists of three stages, approach stage, retry approach parking stage and retry parking stage.

2. approach stage:
   1. We run an on lane planning algorithms to approach pull over target position. 

   2. At first, we check path points data to see whether the s, l and theta error between ADC and the target path point within the threshold.
      1. If so, the pull over status is set to PARK_COMPLETE.

      2. Otherwise we add a stop fence for adc to pause at a better position.

      3. However, if we can't find a suitable new stop fence, approach stage is finished and we switch to retry appoach parking stage. 
     ```cpp
          PullOverStatus CheckADCPullOverPathPoint(
               const ReferenceLineInfo& reference_line_info,
               const ScenarioPullOverConfig& scenario_config,
               const common::PathPoint& path_point,
               const PlanningContext* planning_context);
     ```
   3. Then we check whether adc parked properly.
      1. If ADC pass the destination or park properly, approach stage is finished and pull over scenario is done.

      2. If adc park failed, approach stage is finished and we switch to retry appoach parking stage.
     ```cpp
        PullOverStatus CheckADCPullOver(
            const common::VehicleStateProvider* vehicle_state_provider,
            const ReferenceLineInfo& reference_line_info,
            const ScenarioPullOverConfig& scenario_config,
            const PlanningContext* planning_context);
     ```

3. retry approach parking stage:
   1. We run an on lane planning algorithms to reach the stop line of open space planner.

   2. Check whether ADC stop properly.
   ```cpp
        bool CheckADCStop(const Frame& frame);
   ```
   3. If so, retry approach parking stage is finished and switch to retry parking stage.
 
4. retry parking stage:
   1. We run an open space planning algorithms to park.

   2. Check whether ADC park properly(check distance and theta diff).  
   ```cpp
        bool CheckADCPullOverOpenSpace();
   ```
   3. If so, retry parking stage is finished and pull over scenario is done.
  
   4. Otherwise ADC stay in the stage until ADC park properly.

   5. The conversion of stages can be seen in 
    ![Diagram](images/parking_scenairo_fig_2.png).    

## VALET PARKING SCENARIO
1. This scenario consists of two stages, approach parking spot stage and parking stage.

2. approach parking spot stage:
   1. We run an on lane planning algorithms to approach the designated parking spot.

   2. ADC cruises to a halt once it has found the right stopping point required in order to reverse into the parking spot.
   ```cpp
        bool CheckADCStop(const Frame& frame);
   ```
   3. If stop properly, approach parking spot stage is finished and switch to parking stage.

   4. Otherwise ADC stay in the stage until it approaches to desired parking spot.

3. parking stage:
   1. Open Space Planner algorithm is used to generate a zig-zag trajectory which involves both forward and reverse driving (gear changes) in order to safely park the ego car.

   2. The conversion of stages can be seen in 
   ![Diagram](images/parking_scenairo_fig_3.png).