# NDT-based Lidar Localization

## Introduction
  NDT-based lidar localization provide a straightforward, flexible method which combile inspva information and lidar pointcloud to achieve high localization accuracy.

  Currently this localization method only works for x86_64 platform

## Input
  * Point cloud data from LiDAR sensor ( `/apollo/sensor/velodyne64/compensator/PointCloud2`)
  * Inspva message from integrated navigation sensor ( `/apollo/sensor/gnss/odometry`)
  * Localization map (FLAGS_map_dir + "/" + FLAGS_ndt_map_dir + "/" + FLAGS_local_map_name)
  * Parameter config files (velodyne64_novatel_extrinsics_example.yaml, velodyne64_height.yaml, located in `modules/localization/msf/params/`)

## Output
  * Localization result defined by Protobuf message `LocalizationEstimate`, which can be found in file `localization/proto/localization.proto`. ( `/apollo/localization/pose`)

### NDT Localization Setting
under some circumstance, we need to balance the speed and accuracy of the algorithm. So we expose some parameters of NDT matching process, It includes `online_resolution` for online pointcloud, `ndt_max_iterations` for iterative optimization of NDT matching, `ndt_target_resolution` for target resolution, `ndt_line_search_step_size` for searching step size of iteration and `ndt_transformation_epsilon` for convergence condition.

## Generate NDT Localization Map
  NDT Localization map is used for NDT-based localization, which is a voxel-grid representation of the environment. Each cell stores the centroid and relative covariance of the points in the cell. The map is organized as a group of map nodes. For more information, please refer to `apollo/modules/localization/msf/local_map/ndt_map`.

  We provide a script (`apollo/scripts/ndt_local_map_creator.sh`) to generate NDT localization map. You need to provide a group of point cloud frames (as .pcd file), corresponding poses file, and UTM zone id. The format of the poses file is `pcd_number timestamp x y z qx qy qz qw`.

## Visualization Tool
  NDT localization also use the same online visualization tool as msf, which can show localization map, point cloud, horizontal position of localization results. This tool is simply launched by `./scripts/localization_online_visualizer.sh`.
