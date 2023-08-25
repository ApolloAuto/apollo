# Component Name
lidar_detection

## Introduction
Perform 3D object detection based on the point cloud, and output the position, size, and orientation of the detected objects. Apollo provides 4 lidar detection models: centerpoint, maskpillars, pointpillars, cnnseg.

## Structure
```
├── lidar_detection
    ├── conf            // component config file
    ├── dag             // component dag file
    ├── data            // model config files
    ├── detector        // lidar model
    │   ├── center_point_detection   // CenterPoint model code
    │   ├── cnn_segmentation         // CNNSeg model code
    │   ├── mask_pillars_detection   // MaskPillars model code
    │   └── point_pillars_detection  // PointPillars model code
    ├── interface       // definition of BaseLidarDetector
    ├── proto           // definition of data structure
    ├── lidar_detection_component.cc // component inference
    ├── lidar_detection_component.h
    ├── cyberfile.xml   // package configs
    ├── README.md
    └── BUILD
```

## Input and Output

### Input
| Name              | Type                            | Description          |
| ----------------- | ------------------------------- | -----------------    |
| `frame`         | `LidarFrameMessage`             | lidar frame message  |

### Output
| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `frame`           | `LidarFrameMessage`             | LidarFrame's segmented_objects |

## How to use
1. Add vehicle params configs to modules/perception/data/params，keep frame_id and sensor_name consistent, start the tf
```bash
cyber_launch start modules/transform/launch/static_transform.launch
```

2. Select the lidar model，download the model files and put them into modules/perception/data/models using amodel tool.
```bash
amodel install https://xxx.zip
```

3. Modify the modules/perception/lidar_detection/dag/lidar_detection.dag
- config_file_path: path of config path
- reader channel: the name of input channel

4. Modify modules/perception/lidar_detection/conf/lidar_detection_config.pb.txt
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