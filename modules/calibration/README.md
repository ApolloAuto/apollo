# calibration

## Introduction
Apollo currently supports the following vehicle models by default. You can select different vehicle models in HMI mode and run the Apollo autonomous driving module by playing records.

- kitti_140 - KITTI dataset collection vehicle
- mkz_121 - vehicle for test
- mkz_example - vehicle for simulation
- mkz_lgsvl_321 - vehicle for lgsvl simulation
- nuscenes_165 - nuScenes dataset collection vehicle

## Directory Structure

```shell
modules/calibration/
├── BUILD
├── cyberfile.xml
├── data
│   ├── kitti_140
│   ├── mkz_121
│   ├── mkz_example
│   ├── mkz_lgsvl_321
│   └── nuscenes_165
└── README.md
```

## Naming rules
`vehicle model` = `vehicle name` + `_` + `lidar_num` + `camera_num` + `radar_num`.  For example
```
kitti_140

kitti // vehicle name
1     // 1 lidar
4     // 4 camera
0     // 0 radar
```

## Configuration

Different autonomous vehicles have different sensor types, numbers, and installation locations, also have different dynamic parameters.

For a well functioning vehicle, generally you need the following configs.

```text
mkz_121
├── camera_params
├── gnss_params           # Params for GNSS
├── lidar_params          # Params for lidar
├── novatel_localization_extrinsics.yaml
├── perception_conf
├── perception_dag
├── radar_params
├── transform_conf
├── vehicle_param.pb.txt   # Instance of apollo.common.VehicleParam
└── vehicle_params
```

> Need add control params like "calibration_table.pb.txt"
