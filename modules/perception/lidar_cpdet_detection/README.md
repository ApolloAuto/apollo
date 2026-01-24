# perception-lidar-detection

## Introduction

Perform 3D object detection based on the point cloud, and output the position, size, and orientation of the detected
objects. Apollo provides 1 lidar detection models: centerpoint

## Structure

```
├── lidar_detection
    ├── conf            // component config file
    ├── dag             // component dag file
    ├── data            // model config files
    ├── detector        // lidar model
    │   ├── cpdet   // CenterPoint model code
    ├── interface       // definition of BaseLidarCPDetector
    ├── proto           // definition of data structure
    ├── lidar_detection_component.cc // component inference
    ├── lidar_detection_component.h
    ├── cyberfile.xml   // package configs
    ├── README.md
    └── BUILD
```

## Modules

### LidarCPDetectionComponent

apollo::perception::lidar::LidarCPDetectionComponent

#### Input

| Name    | Type                                             | Description         | Input channal |
| ------- | ------------------------------------------------ | ------------------- | ------------- |
| `frame` | `apollo::perception::onboard::LidarFrameMessage` | lidar frame message | perception/lidar/pointcloud_ground_detection |

>Note: The input channel is structure type data. The default trigger channel is `/perception/lidar/pointcloud_ground_detection`. The detailed input channel information is in `modules/perception/lidar_cpdet_detection/dag/lidar_detection.dag` file. By default, the upstream components of the messages received by the component include `pointcloud_ground_detection`.

#### Output

| Name    | Type                                             | Description                    | Output channal |
| ------- | ------------------------------------------------ | ------------------------------ | -------------- |
| `frame` | `apollo::perception::onboard::LidarFrameMessage` | LidarFrame's segmented_objects | /perception/lidar/detection |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/lidar_cpdet_detection/conf/lidar_detection_config.pb.txt` file.

#### How to use

1. Add vehicle params configs to `modules/perception/data/params`，keep frame_id and sensor_name consistent, start the
   tf

```bash
cyber_launch start modules/transform/launch/static_transform.launch
```

2. Select the lidar model，download the model files and put them into `modules/perception/data/models` using `amodel`
   tool.

```bash
amodel install https://xxx.zip
```

3. Modify the `modules/perception/lidar_cpdet_detection/dag/lidar_detection.dag`

- config_file_path: path of config path
- reader channel: the name of input channel

4. Modify `modules/perception/lidar_cpdet_detection/conf/lidar_detection_config.pb.txt`

- output_channel_name: the name of output channel
- sensor_name: sensor name
- use_object_builder: whether use object builder
- plugin_param
  - name： the name of lidar model
  - config_path: the path of model config
  - config_file: the name of model config

5. start the lidar component

```bash
cyber_launch start modules/perception/launch/perception_lidar.launch
```

## Reference

1. [Center-based 3D Object Detection and Tracking](https://arxiv.org/abs/2006.11275)
2. [PointPillars: Fast Encoders for Object Detection from Point Clouds](https://arxiv.org/abs/1812.05784)
