# Dreamview

## Introduction
  This module provides a web application that helps the users to visualize the current output of relevant autonomous driving modules, e.g. planning trajectory, car localization, chassis status etc. 

## Input
  Currently it monitors the following messages:
  * Localization defined by Protobuf message `LocalizationEstimate`, which can be found in file `localization/proto/localization.proto`.
  * Chassis defined by Protobuf message `Chassis`, which can be found in file `canbus/proto/chassis.proto`.
  * Planning defined by Protobuf message `ADCTrajectory`, which can be found in file `planning/proto/planning.proto`.
  * Monitor defined by Protobuf message `MonitorMessage`, which can be found in file `common/monitor/proto/monitor.proto`.
  * Perception Obstacles defined by Protobuf message `PerceptionObstacles`, which can be found in file `perception/proto/perception_obstacle.proto`.
  * Prediction defined by Protobuf message `PredictionObstacles`, which can be found in file `prediction/proto/prediction_obstacle.proto`.
  * Routing defined by Protobuf message `RoutingResponse`, which can be found in file `routing/proto/routing.proto`.
  
## Output
  A web-based dynamic 3D rendering of the monitored messages in a simulated world.
