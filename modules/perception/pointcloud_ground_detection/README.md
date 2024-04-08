# perception-pointcloud-ground-detection

## Introduction

Point cloud ground detection, detecting ground point, and saving indices of all non ground points.

## Directory Structure

```
├── pointcloud_ground_detection  // ground point detection component
    ├── conf               // configuration folder
    ├── dag                // module startup file
    ├── data               // module configuration parameters
    ├── ground_detector    // ground detection method
    ├── launch             // launch file
    ├── interface          // ground detection interface
    ├── proto              // ground detection module configuration proto file
    ├── pointcloud_ground_detection_component.cc // component entrance
    ├── pointcloud_ground_detection_component.h
    ├── cyberfile.xml      // package management file
    ├── README.md          // readme file
    └── BUILD              // compile file
```

## Modules

### PointCloudGroundDetectComponent

apollo::perception::lidar::PointCloudGroundDetectComponent

#### Input

| Name    | Type                                             | Description         | Input channal |
| ------- | ------------------------------------------------ | ------------------- | ------------- |
| `frame` | `apollo::perception::onboard::LidarFrameMessage` | lidar frame message | /perception/lidar/pointcloud_map_based_roi |

>Note: The input channel is structure type data. The default trigger channel is `/perception/lidar/pointcloud_map_based_roi`. The detailed input channel information is in `modules/perception/pointcloud_ground_detection/dag/pointcloud_ground_detection.dag` file. By default, the upstream components of the messages received by the component include `pointcloud_map_based_roi`.

#### Output

| Name    | Type                                             | Description         | Output channal |
| ------- | ------------------------------------------------ | ------------------- | -------------- |
| `frame` | `apollo::perception::onboard::LidarFrameMessage` | lidar frame message | /perception/lidar/pointcloud_ground_detection |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/pointcloud_ground_detection/conf/pointcloud_ground_detection_config.pb.txt` file.

#### How to Launch

1. Add vehicle parameter configuration file to modules/perception/data/params, corresponding frame_id and sensor_name,
   launch transform

```bash
cyber_launch start modules/transform/launch/static_transform.launch
```

2. Modify `modules/perception/launch/perception_lidar.launch`

- select the dag file to start, use `modules/perception/pointcloud_ground_detection/dag/pointcloud_ground_detection.dag`
  here
- modify msg_adapter. It is used to wrap messages sent by other steps as `/apollo/perception/obstacles`, this can be
  used for individual debugging. Modify relevant channel configurations in
  `modules/perception/data/flag/perception_common.flag`

3. Modify parameters of `modules/perception/pointcloud_ground_detection/conf/pointcloud_ground_detection_config.pb.txt`

- output_channel_name: output channel name
- plugin_param: plugin parameters
  - name: method name
  - config_path: configuration path
  - config_file: configuration file name

4. Launch ground detection component

```bash
cyber_launch start modules/perception/pointcloud_ground_detection/launch/pointcloud_ground_detection.launch
```
