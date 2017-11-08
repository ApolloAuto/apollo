# Multi-sensor Fusion Localization

## Introduction
  The goal of multi-sensor localization is to provide a robust method which can achieve high localization accuracy and resilience in challenging scenes, such as urban downtown, highways, and tunnels. It adaptively uses information from complementary sensors such as GNSS, LiDAR and IMU.

## Input
  * Point cloud data from LiDAR sensor (ROS topic /apollo/sensor/velodyne64/compensator/PointCloud2)
  * Gnss observation and ephemeris data from Gnss sensor. (ROS topic /apollo/sensor/gnss/rtk_obs, /apollo/sensor/gnss/rtk_eph)
  * Gnss best pose from Gnss sensor. (ROS topic /apollo/sensor/gnss/best_pose)
  * Imu data from IMU sensor (ROS topic /apollo/sensor/gnss/corrected_imu)
  * Localization map (#......localization map path......#)
  * Lidar extrinsics parameter (#......how to get the path........#)
  * Lidar height parameter ((#......how to get the path........#)

## Output
  * IMU pose defined by Protobuf message `LocalizationEstimate`, and the corresponding proto is `` (ROS topic /apollo/localization/pose)
  * We also provide intermediate results: Gnss localization result, LiDAR localization result (ROS topic /apollo/localization/measure_gnss, /apollo/localization/measure_lidar)

## Modes
  This module works in two modes. One is directly to use gnss best pose (`/apollo/sensor/gnss/best_pose`) from INS system, such as Novatel system. In the second mode, the initial gnss observations and ephemeris information (`/apollo/sensor/gnss/rtk_obs` and `/apollo/sensor/gnss/rtk_eph`) are used to calculate gnss localization result.

  You can set `gnss_mode` in `apollo/localization/conf/localization.conf` to decide which mode you want to use. The default mode is gnss best pose.

## Gnerate Localization Map
  Localization map is used for LiDAR-based localization, which is a grid-cell representation of the environment. Each cell stores the statistics of laser reflection intensity and altitude. The map is organized as a group of map nodes. For more information refer to `apollo/modules/localization/msf/local_map`.

  We provide a script (`apollo/scripts/msf_local_map.sh`) to generate localization map. You need to provide a group of point cloud frames (as .pcd file) and corresponding poses file. The format of the poses file is ??? @Li Hao  

## Instructions
  "We need to provide a detailed list to help users to set up their msf loclaization."

## Visualization Tool
  We provide an offline visulazation tool which can show localization map, point cloud, horizontal position and variance circle of LiDAR-localization, Gnss-localization, and fused localization results. Since this visualization tool is an offline version, you need to record rosbag and use our script (`msf/local_tool/monitor_data_exporter.py`) to decode it to pcd and pose files.

  How to use visulaizaiton tool? @Li Hao