# Module Name
pointcloud_semantics

## Introduction
The pointcloud semantic module assigns a label to each point for pointcloud.

## Directory Structure
```
├── pointcloud_semantics   // pointcloud_semantics component
    ├── conf               // configuration folder
    ├── dag                // module dag file
    ├── data               // module configuration parameters
    ├── interface          // interface of parser
    ├── launch             // launch file
    ├── parser             // parser method
    ├── proto              // pointcloud semantics module configuration proto file
    ├── utils              // utils of pointcloud semantic
    ├── BUILD              // build file
    ├── cyberfile.xml      // package management file
    ├── pointcloud_semantic_component.cc  // component entrance
    ├── pointcloud_semantic_component.h
    └── README.md
    └── README_cn.md
```

## Modules

### PointCloudSemanticComponent

apollo::perception::lidar::PointCloudSemanticComponent
#### Input
| Name              | Type                            | Description       |
| ----------------- | ------------------------------- | ----------------- |
| `msg`             | `onboard::LidarFrameMessage`    | lidar frame message |

#### Output
| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `frame`           | `onboard::LidarFrameMessage`    | lidar frame message |

#### How to Launch

1. Add vehicle parameter configuration file to modules/perception/data/params in profile, corresponding frame_id and sensor_name, launch transform
```bash
cyber_launch start /apollo/modules/transform/launch/static_transform.launch
```

2. Modify modules/perception/launch/perception_lidar.launch
- select the dag file to start, add `pointcloud_semantic.dag` to perception_lidar.launch
- modify msg_adapter. It is used to wrap messages sent by other steps as /apollo/perception/obstacles, this can be used for individual debugging. Modify relevant channel configurations in modules/perception/data/flag/perception_common.flag

3. Modify parameters of modules/perception/pointcloud_semantics/conf/pointcloud_semantic.pb.txt in profile
- output_channel_name: output channel name
- plugin_param: plugin parameters
  - name: method name
  - config_path: configuration folder
  - config_file: configuration file name

4. Launch pointcloud perception
```bash
cyber_launch start /apollo/modules/perception/launch/perception_lidar.launch
```
