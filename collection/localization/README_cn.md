## 定位模块概述

定位模块是Apollo自动驾驶系统的重要组成部分，它负责在车辆行驶过程中对车辆的位置、速度和方向进行精确的测量和估计。它包含3种定位方式：RTK定位、NDT定位和MSF定位，其中MSF定位通过融合GPS、IMU、激光雷达、摄像头等多种传感器的数据，实现了车辆的高精度定位和轨迹估计。

## Channel+消息格式

### RTK定位

输入消息

| 消息名称                           | 通道名称                          | 功能                |
| ---------------------------------- | --------------------------------- | ------------------- |
| apollo::localization::Gps          | /apollo/sensor/gnss/odometry      | GPS消息             |
| apollo::localization::CorrectedImu | /apollo/sensor/gnss/corrected_imu | 组合导航输出IMU消息 |
| apollo::drivers::gnss::InsStat     | /apollo/sensor/gnss/ins_stat      | 组合导航状态        |

输出消息

| 消息名称                                   | 通道名称                        | 功能                         |
| ------------------------------------------ | ------------------------------- | ---------------------------- |
| apollo::localization::LocalizationEstimate | /apollo/localization/pose       | 定位输出车辆的位置和姿态信息 |
| apollo::localization::LocalizationStatus   | /apollo/localization/msf_status | 定位状态                     |

### NDT定位

输入消息

| 消息名称                       | 通道名称                                        | 功能         |
| ------------------------------ | ----------------------------------------------- | ------------ |
| apollo::localization::Gps      | /apollo/sensor/gnss/odometry                    | GPS消息      |
| apollo::drivers::PointCloud    | /apollo/sensor/lidar128/compensator/PointCloud2 | 激光雷达点云 |
| apollo::drivers::gnss::InsStat | /apollo/sensor/gnss/ins_stat                    | 组合导航状态 |

输出消息

| 消息名称                                   | 通道名称                        | 功能                                 |
| ------------------------------------------ | ------------------------------- | ------------------------------------ |
| apollo::localization::LocalizationEstimate | /apollo/localization/pose       | 定位输出车辆的位置和姿态信息         |
| apollo::localization::LocalizationEstimate | /apollo/localization/ndt_lidar  | 激光雷达定位输出车辆的位置和姿态信息 |
| apollo::localization::LocalizationStatus   | /apollo/localization/msf_status | 定位状态                             |

### MSF定位

输入消息

| 消息名称                            | 通道名称                                        | 功能             |
| ----------------------------------- | ----------------------------------------------- | ---------------- |
| apollo::drivers::gnss::Imu          | /apollo/sensor/gnss/imu                         | 原始Imu信息      |
| apollo::drivers::PointCloud         | /apollo/sensor/lidar128/compensator/PointCloud2 | 激光雷达点云信息 |
| apollo::drivers::gnss::GnssBestPose | /apollo/sensor/gnss/best_pose                   | 激光雷达最优姿态 |
| apollo::drivers::gnss::Heading      | /apollo/sensor/gnss/heading                     | 组合导航航向     |

输出消息

| 消息名称                                   | 通道名称                        | 功能                                 |
| ------------------------------------------ | ------------------------------- | ------------------------------------ |
| apollo::localization::LocalizationEstimate | /apollo/localization/pose       | 融合定位输出车辆的位置和姿态信息     |
| apollo::localization::LocalizationEstimate | /apollo/localization/msf_lidar  | 激光雷达定位输出车辆的位置和姿态信息 |
| apollo::localization::LocalizationEstimate | /apollo/localization/msf_gnss   | 组合导航输出                         |
| apollo::localization::LocalizationStatus   | /apollo/localization/msf_status | 融合定位状态                         |

## 参数

### Flag参数

| 参数名                         | 默认值                                                                                             | 功能 |
| ------------------------------ | -------------------------------------------------------------------------------------------------- | ---- |
| flagfile                       | /apollo/modules/common/data/global_flagfile.txt                                                    |      |
| noenable_gps_imu_interprolate  |                                                                                                    |      |
| local_map_name                 | local_map                                                                                          |      |
| enable_gps_imu_compensate      | false                                                                                              |      |
| enable_lidar_localization      | true                                                                                               |      |
| lidar_localization_mode        | 2                                                                                                  |      |
| lidar_yaw_align_mode           | 2                                                                                                  |      |
| lidar_filter_size              | 17                                                                                                 |      |
| point_cloud_step               | 2                                                                                                  |      |
| lidar_height_default           | 1.80                                                                                               |      |
| lidar_imu_lidar_max_delay_time | 0.4                                                                                                |      |
| lidar_map_coverage_theshold    | 0.8                                                                                                |      |
| lidar_debug_log_flag           | false                                                                                              |      |
| integ_ins_can_self_align       | false                                                                                              |      |
| integ_sins_align_with_vel      |                                                                                                    |      |
| integ_sins_state_check         | true                                                                                               |      |
| integ_sins_state_span_time     | 30.0                                                                                               |      |
| integ_sins_state_pos_std       | 1.0                                                                                                |      |
| vel_threshold_get_yaw          | 5.0                                                                                                |      |
| enable_ins_aid_rtk             | false                                                                                              |      |
| eph_buffer_path                | ""                                                                                                 |      |
| gnss_debug_log_flag            | false                                                                                              |      |
| imu_rate                       | 1.0                                                                                                |      |
| if_utm_zone_id_from_folder     | true                                                                                               |      |
| local_utm_zone_id              | 10                                                                                                 |      |
| trans_gpstime_to_utctime       | true                                                                                               |      |
| gnss_mode                      | 0                                                                                                  |      |
| lidar_topic                    | /apollo/sensor/hesai40/compensator/PointCloud2                                                     |      |
| lidar_extrinsics_file          | /apollo/modules/localization/msf/params/velodyne_params/velodyne64_novatel_extrinsics_example.yaml |      |
| imu_coord_rfu                  | true                                                                                               |      |
| gnss_only_init                 | false                                                                                              |      |
| if_imuant_from_file            | true                                                                                               |      |
| imu_to_ant_offset_x            | 0.0                                                                                                |      |
| imu_to_ant_offset_y            | 0.788                                                                                              |      |
| imu_to_ant_offset_z            | 1.077                                                                                              |      |
| imu_to_ant_offset_ux           | 0.05                                                                                               |      |
| imu_to_ant_offset_uy           | 0.05                                                                                               |      |
| imu_to_ant_offset_uz           | 0.08                                                                                               |      |
| if_vehicle_imu_from_file       | true                                                                                               |      |
| imu_vehicle_qx                 | 0.0                                                                                                |      |
| imu_vehicle_qy                 | 0.0                                                                                                |      |
| imu_vehicle_qz                 | 0.0                                                                                                |      |
| imu_vehicle_qw                 | 1.0                                                                                                |      |
| map_visual_dir                 | data/map_visual                                                                                    |      |
| if_use_avx                     | true                                                                                               |      |

### RTK定位参数

| 参数名                      | 默认值                            | 功能 |
| --------------------------- | --------------------------------- | ---- |
| localization_topic          | /apollo/localization/pose         |      |
| localization_status_topic   | /apollo/localization/msf_status   |      |
| imu_topic                   | /apollo/sensor/gnss/corrected_imu |      |
| gps_topic                   | /apollo/sensor/gnss/odometry      |      |
| gps_status_topic            | /apollo/sensor/gnss/ins_stat      |      |
| broadcast_tf_frame_id       | world                             |      |
| broadcast_tf_child_frame_id | localization                      |      |
| imu_frame_id                | imu                               |      |
| imu_list_max_size           | 20                                |      |
| gps_imu_time_diff_threshold | 0.02                              |      |
| map_offset_x                | 0.0                               |      |
| map_offset_y                | 0.0                               |      |
| map_offset_z                | 0.0                               |      |

## 包列表

| 包名                                              | 描述       |
| ------------------------------------------------- | ---------- |
| [localization](modules/localization/README_cn.md) | 定位模块包 |
