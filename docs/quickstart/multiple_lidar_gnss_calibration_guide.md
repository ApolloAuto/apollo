# Multiple-LiDAR GNSS Calibration Guide

```
NOTE: Supports upto Apollo 3.0. Apollo 3.5 is not supported yet.
```

Welcome to use the Multiple-LiDAR GNSS calibration tool. This guide will show you the steps to successfully calibrate multiple LiDARs.

## Contents

- Overview
- Preparation
- Using the Calibration Tool
- Results and Validation

## Overview
In many autonomous driving tasks such as HDMap production, the scans from multiple LiDARs need to be registered in a unified coordinate system. In this case, the extrinsic parameters of multiple LiDARs need to be carefully calibrated. This Multiple-LiDAR GNSS calibration tool is developed to solve this problem.

## Preparation

1. Download the [calibration tool](https://github.com/ApolloAuto/apollo/releases/download/v2.5.0/multi_lidar_gnss_calibrator_and_doc.zip), and extract files to `$APOLLO_HOME/modules/calibration`. APOLLO_HOME is the root directory of apollo repository.
2. Choose a calibration place according to the calibration guide provided in Apollo 1.5.
3. Make sure the GNSS is in a good status. To verify this, use `rostopic echo /apollo/sensor/gnss/best_pose` and check the number after keywords `latitude_std_dev`, `longitude_std_dev` and `height_std_dev`. The smaller the deviation, the better the calibration quality. *** We strongly recommend calibrating the sensors when deviations are smaller than 0.02.*** 

## Using the Calibration Tool

### Record Calibration Data

When the LiDARs and GNSS are ready, use `/apollo/modules/calibration/multi_lidar_gnss/record.sh` to record calibration data. Note that, this script is only for recording  velodyne HDL64 and VLP16. For other purpose, some modification of this script is needed or just use rosbag record to do the same thing. Usually, 2 minites length of data is sufficient. After data capture, run `/apollo/modules/calibration/multi_lidar_gnss/calibrate.sh` to calibrate sensors. The script is composed of the following two steps.

### Export Data

Once the calibration bag is recorded, use `/apollo/modules/calibration/exporter/export_msgs --config /apollo/modules/calibration/exporter/conf/export_config.yaml` to get sensor data. The only input of the exporter is a YAML configuration file as follow.

```bash
bag_path: "/apollo/data/bag/calibration/" # The path where the calibration bag is placed.
dump_dir: "/apollo/data/bag/calibration/export/" # The path where the sensor data will be placed using exporter
topics:
    - /apollo/sensor/gnss/odometry: # Odometry topic name
        type: ApolloOdometry        # Odometry type
    - /apollo/sensor/velodyne16/PointCloud2: # vlp16 topic name
        type: PointCloud2                    # vlp16 type
    - /apollo/sensor/velodyne64/PointCloud2: # hdl64 topic name
        type: PointCloud2                    # hdl64 type
```
Other topics of PointCloud2 type also can be exported, if new topics are added to the file in the rule as follow.
```bash
    - TOPIC_NAME: # topic name
        type: PointCloud2
```
Till now, we only support `ApolloOdometry` and `PointCloud2`.  

### Run the Calibration Tool
If all sensor data are exported, run `/apollo/modules/calibration/lidar_gnss_calibrator/multi_lidar_gnss_calibrator --config /apollo/modules/calibration/lidar_gnss_calibrator/conf/multi_lidar_gnss_calibrator_config.yaml` will get the results. The input of the tool is a YAML configuration file as follow.
```bash
# multi-LiDAR-GNSS calibration configurations
data:
    odometry: "/apollo/data/bag/calibration/export/multi_lidar_gnss/_apollo_sensor_gnss_odometry/odometry"
    lidars: 
        - velodyne16: 
            path: "/apollo/data/bag/calibration/export/multi_lidar_gnss/_apollo_sensor_velodyne16_PointCloud2/"
        - velodyne64: 
            path: "/apollo/data/bag/calibration/export/multi_lidar_gnss/_apollo_sensor_velodyne64_PointCloud2/"
    result: "/apollo/data/bag/calibration/export/multi_lidar_gnss/result/"
calibration:
    init_extrinsics:
        velodyne16:
            translation:    
                x: 0.0
                y: 1.77 
                z: 1.1
            rotation:
                x: 0.183014 
                y: -0.183014 
                z: 0.683008 
                w: 0.683008
        velodyne64:
            translation:    
                x: 0.0
                y: 1.57
                z: 1.3
            rotation:
                x: 0.0
                y: 0.0
                z: 0.707
                w: 0.707
    steps: 
        - source_lidars: ["velodyne64"]
          target_lidars: ["velodyne64"]
          lidar_type: "multiple"
          fix_target_lidars: false
          fix_z: true
          iteration: 3
        - source_lidars: ["velodyne16"]
          target_lidars: ["velodyne16"]
          lidar_type: "multiple"
          fix_target_lidars: false
          fix_z: true
          iteration: 3
        - source_lidars: ["velodyne16"]
          target_lidars: ["velodyne64"]
          lidar_type: "multiple"
          fix_target_lidars: true
          fix_z: false
          iteration: 3
```
    
The `data` section tells the tool where to get point clouds and odometry file, and also where to save the results. Note that, the keywords in `lidar` node will be recognized as frame id for the LiDARs. 

The `calibration` section provides initial guess of the extrinsics. ***All extrinsics are from LiDAR to GNSS***, which means this transformation maps the coordinates of a point defined in the LiDAR coordinate system to the coordinates of this point defined in the GNSS coordinate system. The initial guess requires the rotation angle error less than 5 degrees, and the translation error less than 0.1 meter.

The `steps` section specifies the calibration procedure. Each step is defined as follow and their meanings are in comments.
```bash
- source_lidars: ["velodyne16"] # Source LiDAR in point cloud registration.
  target_lidars: ["velodyne64"] # Target LiDAR in point cloud registration.
  lidar_type: "multiple" # "multiple" for multi-beam LiDAR, otherwise "single"
  fix_target_lidars: true # Whether to fix  extrinsics of target LiDARS. Only "true" when align different LiDARs.
  fix_z: false # Whether to fix the z component of translation. Only "false" when align different LiDARs.
  iteration: 3 # Iteration number
```
## Results and Validation

The calibration tool saves the results to `result` path as follow.
```bash
.
└── calib_result
    ├── velodyne16_novatel_extrinsics.yaml
    ├── velodyne16_result.pcd
    ├── velodyne16_result_rgb.pcd
    ├── velodyne64_novatel_extrinsics.yaml
    ├── velodyne64_result.pcd
    └── velodyne64_result_rgb.pcd
```
The two YAML files are extrinsics. To validate the results, use `pcl_viewer *_result.pcd` to check the registration quality. If the sensors are well calibrated, a large amount of details can be identified from the point cloud. For more details, please refer to the calibration guide in Apollo 1.5. 
