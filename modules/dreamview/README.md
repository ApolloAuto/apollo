
# dreamview

## Introduction
Dreamview or Apollo's HMI module provides a web application that helps developers visualize the output of other relevant autonomous driving modules, e.g. the vehicle's planning trajectory, car localization, chassis status etc.

## Directory Structure
```shell
modules/dreamview
├── backend               // dreamview backend implementation
├── BUILD
├── conf
├── cyberfile.xml
├── frontend              // dreamview frontend implementation 
├── launch
├── main.cc
├── proto
└── README.md
```

## Input
  Currently Dreamview monitors the following messages:
  * Localization, defined by Protobuf message `LocalizationEstimate`, which can be found in file `modules/common_msgs/localization_msgs/localization.proto`.
  * Chassis, defined by Protobuf message `Chassis`, which can be found in file `modules/common_msgs/chassis_msgs/chassis.proto`.
  * Planning, defined by Protobuf message `ADCTrajectory`, which can be found in file `modules/common_msgs/planning_msgs/planning.proto`.
  * Monitor, defined by Protobuf message `MonitorMessage`, which can be found in file `modules/common_msgs/monitor_msgs/monitor.proto`.
  * Perception Obstacles, defined by Protobuf message `PerceptionObstacles`, which can be found in file `modules/common_msgs/perception_msgs/perception_obstacle.proto`.
  * Prediction, defined by Protobuf message `PredictionObstacles`, which can be found in file `modules/common_msgs/prediction_msgs/prediction_obstacle.proto`.
  * Routing, defined by Protobuf message `RoutingResponse`, which can be found in file `modules/common_msgs/routing_msgs/routing.proto`.

## Output
  A web-based dynamic 3D rendering of the monitored messages in a simulated world.

## configs

| file path                                                        | type / struct                            | Description           |
| ---------------------------------------------------------------- | ---------------------------------------- | --------------------- |
| `modules/dreamview/conf/camera_to_lidar_preprocess_table.pb.txt` | `apollo::dreamview::PreprocessTable`     | preprocess table      |
| `modules/dreamview/conf/data_collection_table.pb.txt`            | `apollo::dreamview::DataCollectionTable` | data collection table |
| `modules/dreamview/conf/lidar_to_gnss_preprocess_table.pb.txt`   | `apollo::dreamview::PreprocessTable`     | preprocess table      |
| `modules/dreamview/conf/hmi_modes`                               | `apollo::dreamview::HMIMode`             | HMI mode config       |
| `modules/dreamview/conf/dreamview.conf`                          | `gflags`                                 |   gflags config       |

## Flags

| flagfile                                               | type | Description                 |
| ------------------------------------------------------ | ---- | --------------------------- |
| `modules/dreamview/backend/common/dreamview_gflags.cc` | `cc` | dreamview flags define      |
| `modules/dreamview/backend/common/dreamview_gflags.h`  | `h`  | dreamview flags header      |

## How to use

1. Build apollo in source env or install `dreamview` and `monitor` package in package management env
2. in source env:
```shell
bash scripts/bootstrap.sh
```

in package management env:
```shell
aem bootstrap start
```

## Related Paper

1. [Xu, J., Luo, Q., Xu, K., Xiao, X., Yu, S., Hu, J., Miao, J. and Wang, J., 2019, November. An Automated Learning-Based Procedure for Large-scale Vehicle Dynamics Modeling on Baidu Apollo Platform. In *2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 5049-5056). IEEE.*](https://ieeexplore.ieee.org/document/8968102)

2. [Jiang, S., Wang, Y., Lin, L., Lin, W., Cao, Y., Miao, J. and Luo, Q., 2020, November. DRF: A Framework for High-Accuracy Autonomous Driving Vehicle Modeling, *arXiv preprint arXiv:2011.00646.*](https://arxiv.org/abs/2011.00646)
