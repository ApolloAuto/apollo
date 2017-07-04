# Planning

## Introduction
  Given localization (vehicle status (position, velocity, acceleration)),
  perception (future work), prediction (future work) and decision (future work),
  compute a trajectory that is safe and comfortable for controller to execute.

  Current version of planning is in actual an RTK replayer.
  It first loads a recorded trajectory in the initialization and sends the proper segment of the trajectory according to current system time and vehicle position.

## Input
  * Localization
  * Perception (future work)
  * Prediction (future work)
  * Decision (future work)
  * (For current RTK replayer) Recorded RTK trajectory (put into the folder modules/planning/data, and change the gflag file name in planning/common/planning_gflags)

## Output
  * A collision-free and comfortable trajectory. (For current RTK replayer, a segment of the recorded RTK trajectory for execution.)