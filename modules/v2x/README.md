# v2x

## Introduction
  V2X, known as Vehicle to Everything, i.e., the exchange of information from the vehicle to the outside world. Simply put, the vehicle can analyze real-time traffic information, automatically choose the best driving route, improve the efficiency of access; in addition, it can also sense the surrounding environment, make rapid adjustments to improve driving safety.


## Directory Structure
```shell
modules/v2x/
├── BUILD
├── common          // gflags
├── conf
├── cyberfile.xml
├── dag
├── data
├── fusion
├── launch
├── proto
├── README.md
└── v2x_proxy
```

#### Input

| Name  | Type                                             | Description         |
| ----- | ------------------------------------------------ | ------------------- |
| `msg` | `apollo::localization::LocalizationEstimate`     |  localization msg   |
| `msg` | `apollo::perception::PerceptionObstacles`        |perception obstacles |
| `msg` | `apollo::v2x::V2XObstacles`                      |  v2x obstacles      |

#### Output

| Name  | Type                                             | Description         |
| ----- | ------------------------------------------------ | ------------------- |
| `msg` | `apollo::perception::PerceptionObstacles`        | perception obstacles|


## configs
 
| file path                                              | type / struct        | Description   |
| ------------------------------------------------------ | -------------------- | ------------- |
|  `modules/v2x/conf/v2x_fusion_tracker.conf`            | gflags               |       -       |
|  `modules/v2x/conf/v2x.conf`                           | gflags               |       -       |

#### How to Launch

```bash
cyber_launch start modules/v2x/launch/v2x.launch
```