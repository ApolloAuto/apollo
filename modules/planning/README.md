# Planning

## Introduction
  Given localization, vehicle status (position, velocity, acceleration, chassis), map, routing,
  perception and prediction, planning will compute a trajectory that is safe and comfortable for controller to execute.

  It supports two major planners:
  * RTK replay planner (since Apollo 1.0)
  * EM planner (since Apollo 1.5)

  The RTK replay planner first loads a recorded trajectory in the initialization and sends the proper segment of the trajectory according to current system time and vehicle position.

  In the EM planner, driving decisions and driving trajectory will be calculated based on map, routing and obstacles. DP (dynamic programming) based methods are used first to determine a raw path and speed profile, and then QP (quadratic programming) based methods are used to futher optimize the path and speed profile to get a smooth trajectory.

## Input
  * RTK replay planner:
    * Localization
    * Recorded RTK trajectory (put into the folder modules/planning/data, and change the gflag file name in planning/common/planning_gflags)
  * EM planner:
    * Localization
    * Perception
    * Prediction
    * HD Map (in modules/map/data)
    * routing

## Output
  * A collision-free and comfortable trajectory for control module to execute.
