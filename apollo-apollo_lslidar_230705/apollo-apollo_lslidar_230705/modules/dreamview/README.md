
# Dreamview

## Introduction
Dreamview or Apollo's HMI module provides a web application that helps developers visualize the output of other relevant autonomous driving modules, e.g. the vehicle's planning trajectory, car localization, chassis status etc.

## Input
  Currently Dreamview monitors the following messages:
  * Localization, defined by Protobuf message `LocalizationEstimate`, which can be found in file `localization/proto/localization.proto`.
  * Chassis, defined by Protobuf message `Chassis`, which can be found in file `canbus/proto/chassis.proto`.
  * Planning, defined by Protobuf message `ADCTrajectory`, which can be found in file `planning/proto/planning.proto`.
  * Monitor, defined by Protobuf message `MonitorMessage`, which can be found in file `common/monitor/proto/monitor.proto`.
  * Perception Obstacles, defined by Protobuf message `PerceptionObstacles`, which can be found in file `perception/proto/perception_obstacle.proto`.
  * Prediction, defined by Protobuf message `PredictionObstacles`, which can be found in file `prediction/proto/prediction_obstacle.proto`.
  * Routing, defined by Protobuf message `RoutingResponse`, which can be found in file `routing/proto/routing.proto`.

## Output
  A web-based dynamic 3D rendering of the monitored messages in a simulated world.

## Related Paper

1. [Xu, J., Luo, Q., Xu, K., Xiao, X., Yu, S., Hu, J., Miao, J. and Wang, J., 2019, November. An Automated Learning-Based Procedure for Large-scale Vehicle Dynamics Modeling on Baidu Apollo Platform. In *2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 5049-5056). IEEE.*](https://ieeexplore.ieee.org/document/8968102)

2. [Jiang, S., Wang, Y., Lin, L., Lin, W., Cao, Y., Miao, J. and Luo, Q., 2020, November. DRF: A Framework for High-Accuracy Autonomous Driving Vehicle Modeling, *arXiv preprint arXiv:2011.00646.*](https://arxiv.org/abs/2011.00646)
