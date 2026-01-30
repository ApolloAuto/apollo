# Module Name
lidar_segmentation 

## Introduction
The lidar segmentation module uses point cloud of secondary_indices to do object segmentation. Secondary indices is intersetion of non_ground_indices and roi_indices, and remove foreground objects point indices.

## Directory Structure
```
├── lidar_segmentation     // lidar segmentation component
    ├── common             // common codes
    ├── conf               // configuration folder
    ├── dag                // module startup file
    ├── data               // module configuration parameters
    ├── launch             // launch file
    ├── proto              // lidar segmentation module configuration proto file
    ├── segmentor          // point cloud segmentation methods
    ├── lidar_segmentation_component.cc // component entrance
    ├── lidar_segmentation_component.h
    ├── cyberfile.xml      // package management file
    ├── README.md          // readme file
    ├── README_cn.md
    └── BUILD              // compile file
```

## Modules

### LidarSegmentationComponent

apollo::perception::lidar::LidarSegmentationComponent
#### Input
| Name              | Type                            | Description       |
| ----------------- | ------------------------------- | ----------------- |
| `msg`             | `onboard::LidarFrameMessage`    | lidar frame message |

#### Output
| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `frame`           | `onboard::LidarFrameMessage`    | lidar frame message |

#### Configuration

Configurations of graph_segmentation.

| parameter type | parameter name | default value | meaning |
| -------------- | -------------- | ------------- | ------- |
| uint32   | grid_width           |               | grid width |
| uint32   | grid_height          |               | grid height |
| float    | resolution           |               | grid resolution |
| float    | threshold            |               | graph segmentation threshold |
| uint32   | min_pt_number        |               | min point number of cluster |
| uint32   | search_radius        |  3            | grid search radius |
| float    | height_threshold     |  2.0          | height threshold decided by autonomous car |
| float    | xmin        | -30.0           | pointcloud valid distance of front |
| float    | xmax        |  30.0           | pointcloud valid distance of back  |
| float    | ymin        | -10.0           | pointcloud valid distance of right |
| float    | ymax        |  30.0           | pointcloud valid distance of left  |
| float    | semantic_cost |  1.0          | semantic label cost |
| float    | same_semantic_coefficient |  1.0  | edge coefficient for same semantic label |
| float    | diff_semantic_coefficient |  1.0  | edge coefficient for different semantic label |
| float    | same_motion_coefficient   |  1.0  | edge coefficient for same motion label |
| float    | diff_motion_coefficient   |  1.0  | edge coefficient for different motion label |
| float    | split_aspect_ratio |  10.0   | split aspect ratio |
| float    | split_distance     |  5.0    | split distance |

#### How to Launch

1. Add vehicle parameter configuration file to modules/perception/data/params in profile, corresponding frame_id and sensor_name, launch transform
```bash
cyber_launch start /apollo/modules/transform/launch/static_transform.launch
```

2. Modify modules/perception/launch/perception_lidar.launch
- select the dag file to start, add `lidar_segmentation.dag` to perception_lidar.launch
- modify msg_adapter. It is used to wrap messages sent by other steps as /apollo/perception/obstacles, this can be used for individual debugging. Modify relevant channel configurations in modules/perception/data/flag/perception_common.flag

3. Modify parameters of modules/perception/lidar_segmentation/conf/lidar_segmentation_config.pb.txt in profile
- output_channel_name: output channel name
- plugin_param: plugin parameters
  - name: method name
  - config_path: configuration folder
  - config_file: configuration file name

4. Launch pointcloud perception
```bash
cyber_launch start /apollo/modules/perception/launch/perception_lidar.launch
```
