# drivers-lidar

## Introduction
This module consists of two parts, the first part is the LIDAR driver, which reads the LIDAR data and parses it when it starts. The second part is the LIDAR parser, which receives the raw radar data published by the driver and publishes the converted point cloud data.

## Directory Structure
```shell
modules/drivers/lidar/
├── BUILD
├── common                      // common function and data structure
├── conf
├── cyberfile.xml
├── dag
├── hesai                       // hesai lidar drivers
├── launch
├── lidar_driver_component.cc
├── lidar_driver_component.h
├── lidar_robosense             
├── proto
├── README.md
├── robosense                   // robosense lidar drivers
└── velodyne                    // velodyne lidar drivers
```

## Modules

### LidarDriverComponent

apollo::drivers::lidar::LidarDriverComponent

#### Input

| Name   | Type                           | Description    |
| ------ | ------------------------------ | -------------- |
| socket |  scan raw packet buffer        |   raw packet   |

#### Output

|     Name    | Type                                          |      Description         |
| ----------- | --------------------------------------------- | ------------------------ |
|    `msg`    |   `apollo::drivers::velodyne::VelodyneScan`   |  scan raw output         |
|    `msg`    |    `apollo::drivers::PointCloud`              |     raw pointcloud       |
|    `msg`    |    `apollo::drivers::PointCloud`              | compensation pointcloud  |

#### configs

| file path                                    | type / struct                    | Description           |
| -------------------------------------------- | -------------------------------- | --------------------- |
| `modules/drivers/lidar/lidar_config.pb.txt`  | `apollo::drivers::lidar::config` |      lidar config     |

#### How to Launch

```bash
cyber_launch start modules/drivers/lidar/launch/driver.launch
```