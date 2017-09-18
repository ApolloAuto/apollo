# Perception

## Introduction
  The goal of perception module is to provide the ability of perceiving obstacles given input 3D point cloud data from LiDAR sensor. It detects, segments and tracks obstacles in the ROI defined by high-resolution (HD) map. In addition, it predicts the obstacles’ motion and pose information (e.g., heading, velocity, etc). It consists of four successive sub-modules including **_HDMap ROI Filter_**, **_CNN Segmentation_**, **_MinBox Builder_** and **_HM Object Tracker_**.

## Input
  * Point cloud data from LiDAR sensor (ROS topic _/apollo/sensor/velodyne64/compensator/PointCloud2_)
  * Coordinate frame transformation information over time (ROS topic _/tf_)
  * HD map
  * Extrinsic parameters of LiDAR sensor calibration (ROS topic _/tf_static_)

## Output
  * 3D obstacle tracks with heading and velocity information (ROS topic _/apollo/perception/obstacles_)

## Instruction
  Before running the 3D obstacle perception program, please select the appropriate HD map by setting the option `--map_dir` in the global configuration file `modules/common/data/global_flagfile.txt`. After that, you may setup the general settings in the configuration file `modules/perception/conf/perception.conf`. Then, you can launch the perception program by using the command `./scripts/perception start` or enabling the perception button in HMI. The command of stopping perception is `./scripts/perception stop`.