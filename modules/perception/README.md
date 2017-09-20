# Perception

## Introduction
  The goal of perception module is to provide the ability of perceiving obstacles given input 3D point cloud data from LiDAR sensor. It detects, segments and tracks obstacles in the ROI defined by high-resolution (HD) map. In addition, it predicts the obstacles’ motion and pose information (e.g., heading, velocity, etc). It consists of four successive sub-modules including **_HDMap ROI Filter_**, **_CNN Segmentation_**, **_MinBox Builder_** and **_HM Object Tracker_**. Please see details in [the document of perception](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/3d_obstacle_perception.md).

## Input
  * Point cloud data from LiDAR sensor (ROS topic _/apollo/sensor/velodyne64/compensator/PointCloud2_)
  * Coordinate frame transformation information over time (ROS topic _/tf_)
  * HD map
  * Extrinsic parameters of LiDAR sensor calibration (ROS topic _/tf_static_)

## Output
  * 3D obstacle tracks with heading and velocity information (ROS topic _/apollo/perception/obstacles_)

## Instruction
  Before running the 3D obstacle perception program, please select the appropriate HD map by setting the option `--map_dir` in the global configuration file `modules/common/data/global_flagfile.txt`. After that, you may setup the general settings in the configuration file `modules/perception/conf/perception.conf`. Then, you can launch the perception program by using the command `./scripts/perception.sh start` or enabling the perception button in HMI. The command of stopping perception is `./scripts/perception.sh stop`. In addition we provide some demo data for developers. Please download the demo data from our [Open Data Platform](https://console.bce.baidu.com/apollo/task/download).

  **Note**: It requires Nvidia GPU and CUDA installed to run the perception module with Caffe. We have already installed the CUDA and Caffe libraries in the released docker. However, the pre-installed Nvidia GPU driver in the released docker image may be not compatible to your host machine for offline debugging and simulation, which will make the percetion module fail to run. We suggest to reinstall the exactly same version of Nvidia driver in the docker image as the one installed in the host machine, and build Apollo with GPU option (i.e., using `./apollo.sh build_gpu` or `./apollo.sh build_opt_gpu`). Please see the detailed instruction in [How to Run Perception Module on Your Local Computer](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_run_perception_module_on_your_local_computer.md).