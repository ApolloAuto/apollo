# perception-pointcloud-preprocess

## Introduction

The point cloud preprocessing module preprocesses the point cloud data output by the driver. Deleting nan value points,
points that are too far away, points scanned onto the self vehicle, and point that are too high.

## Directory Structure

```
├── pointcloud_preprocess  // point cloud preprocess component
    ├── conf               // configuration folder
    ├── dag                // module startup file
    ├── data               // module configuration parameters
    ├── launch             // launch file
    ├── interface          // preprocess interface
    ├── proto              // preprocess module configuration proto file
    ├── preprocessor       // preprocess method
    ├── pointcloud_preprocess_component.cc // component entrance
    ├── pointcloud_preprocess_component.h
    ├── cyberfile.xml      // package management file
    ├── README.md          // readme file
    └── BUILD              // compile file
```

## Modules

### PointCloudPreprocessComponent

apollo::perception::lidar::PointCloudPreprocessComponent

#### Input

| Name  | Type                          | Description         | Input channal |
| ----- | ----------------------------- | ------------------- | ------------- |
| `msg` | `apollo::drivers::PointCloud` | point cloud message | /apollo/sensor/velodyne64/compensator/PointCloud2 |

Point cloud data from driver: If there is one lidar, output point cloud after motion compensation. If there are multiple
lidars, concatenate the point clouds into one frame after motion compensation. The default trigger channel is `/apollo/sensor/velodyne64/compensator/PointCloud2`. The detailed input channel information is in `modules/perception/pointcloud_preprocess/dag/pointcloud_preprocess.dag` file.

#### Output

| Name    | Type                                             | Description         | Output channal |
| ------- | ------------------------------------------------ | ------------------- | -------------- |
| `frame` | `apollo::perception::onboard::LidarFrameMessage` | lidar frame message | /perception/lidar/pointcloud_preprocess |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/pointcloud_preprocess/conf/pointcloud_preprocess_config.pb.txt` file.

#### How to Launch

1. Add vehicle parameter configuration file to `modules/perception/data/params`, corresponding frame_id and sensor_name,
   launch transform

```bash
cyber_launch start modules/transform/launch/static_transform.launch
```

2. `Modify modules/perception/launch/perception_lidar.launch`

- select the dag file to start, use `modules/perception/pointcloud_preprocess/dag/pointcloud_preprocess.dag` here
- modify msg_adapter. It is used to wrap messages sent by other steps as `/apollo/perception/obstacles`, this can be
  used for individual debugging. Modify relevant channel configurations in
  `modules/perception/data/flag/perception_common.flag`

3. Modify parameters of `modules/perception/pointcloud_preprocess/conf/pointcloud_preprocess_config.pb.txt`

- sensor_name: sensor name
- lidar_query_tf_offset: tf time offset
- output_channel_name: output channel name
- plugin_param: plugin parameters
  - name: method name
  - config_path: configuration file path
  - config_file: configuration file name

4. Launch point cloud preprocess component

```bash
cyber_launch start modules/perception/pointcloud_preprocess/launch/pointcloud_preprocess.launch
```
