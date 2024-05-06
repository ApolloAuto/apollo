# 多激光雷达全球导航卫星系统(Multiple-LiDAR GNSS)校准指南

欢迎使用多激光雷达全球导航卫星系统校准工具。本指南将向您展示如何成功校准多个LiDAR的步骤。

## 内容

-	概述
-	准备
-	使用校准工具
-	结果与验证

## 概述

在许多自动驾驶任务，如HDMap的制作，多个激光雷达扫描结果需要注册在一个统一的坐标系统。在这种情况下，需要对多个LIDARs的外部参数进行仔细校准。为了解决这个问题，开发了多激光雷达GNSS校准工具。

## 准备

下载校准工具，并将文件提取到$APOLLO_HOME/modules /calibration。$APOLLO_HOME是APOLLO repository的根目录。
根据Apollo 1.5提供的校准指南选择校准位置。
确保GNSS处于良好状态。为了验证这一点，使用‘rostopic echo /apollo/sensor/gnss/best_pose’并检查关键词latitude_std_dev, longitude_std_dev 和height_std_dev后的数量，偏差越小，校准质量越好。 我们强烈建议在偏差小于0.02时校准传感器。
## 使用校准工具

### 记录校准数据

当LIDARS和GNSS准备就绪时，使用/apollo/modules/calibration/multi_lidar_gnss/record.sh记录校准数据。请注意，此脚本仅用于记录velodyne HDL64 和VLP16。为了其他目的，需要修改这个脚本，或者只需要使用rosbag record来做同样的事情。通常，2分钟的数据长度就足够了。在数据捕获之后，运行/apollo/modules/calibration/multi_lidar_gnss/calibrate.sh校准传感器。脚本由以下两个步骤组成。

### 出口数据

一旦校准包被记录，使用/apollo/modules/calibration/exporter/export_msgs --config /apollo/modules/calibration/exporter/conf/export_config.yaml获得传感器数据。exporter的唯一输入是一个YAML配置文件，如下所示。
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

如果将新topic按如下的规则添加到文件中，也可以导出PointCloud2 types的其他topic。
```bash
    - TOPIC_NAME: # topic name
        type: PointCloud2
```

到目前为止，我们只支持ApolloOdometry和PointCloud2。

### 运行校准工具

如果输出所有传感器数据，运行/apollo/modules/calibration/lidar_gnss_calibrator/multi_lidar_gnss_calibrator --config /apollo/modules/calibration/lidar_gnss_calibrator/conf/multi_lidar_gnss_calibrator_config.yaml将得到结果。该工具的输入是一个YAML配置文件，如下所示。
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

数据部分告诉工具在哪里获取点云和测距文件，以及在哪里保存结果。注意，LIDAR节点中的关键字将被识别为LiDARs的Frame ID。

校准部分提供了外部信息的初始猜测。所有的外部信息都是从激光雷达到GNSS，这意味着这种变换将激光雷达坐标系中定义的点的坐标映射到GNSS坐标系中定义的这一点的坐标。初始猜测要求旋转角度误差小于5度，平移误差小于0.1米。

步骤部分详细说明了校准过程。每个步骤被如下定义并且它们的含义在注释中。
```bash
- source_lidars: ["velodyne16"] # Source LiDAR in point cloud registration.
  target_lidars: ["velodyne64"] # Target LiDAR in point cloud registration.
  lidar_type: "multiple" # "multiple" for multi-beam LiDAR, otherwise "single"
  fix_target_lidars: true # Whether to fix  extrinsics of target LiDARS. Only "true" when align different LiDARs.
  fix_z: false # Whether to fix the z component of translation. Only "false" when align different LiDARs.
  iteration: 3 # Iteration number
```
## 结果和验证
校准工具将结果保存到结果路径中。
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
这两个YAML文件是外部的。为了验证结果，使用pcl_viewer *_result.pcd检查注册质量。如果传感器校准好了，大量的细节可以从点云中识别出来。欲了解更多详情，请参阅校准指南Apollo 1.5。
