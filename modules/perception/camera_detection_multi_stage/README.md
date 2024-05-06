# perception-camera-detection-multi-stage

## Introduction

The camera 2D object detection module is a multitask model developed based on early `yolo`, which can simultaneously
output dozens of dimensional information such as `2d` ,`3d`, and vehicle turn signals. This module completes operations
such as image data preprocessing, detection, result post-processing and so on.

## Directory Structure

```
├── camera_detection_multi_stage // camera detect 2d module
    ├── conf            // module configuration files
    ├── dag             // dag files
    ├── data            // model params
    ├── detector        // main part for detector
    │   ├── yolo        // YoloObstacleDetector
    │   └── ...
    ├── interface       // function interface folder
    ├── proto           // proto files
    ├── camera_detection_multi_stage_component.cc // component interface
    ├── camera_detection_multi_stage_component.h
    ├── cyberfile.xml   // package management profile
    ├── README.md
    └── BUILD
```

## Modules

### CameraDetectionMultiStageComponent

apollo::perception::camera::CameraDetectionMultiStageComponent

#### Input

| Name  | Type                     | Description         | Input channal |
| ----- | ------------------------ | ------------------- | ------------- |
| `msg` | `apollo::drivers::Image` | camera sensor image | /apollo/sensor/camera/front_6mm/image |

>Note: Enter the data type defined by proto. The default trigger camera channel is `/apollo/sensor/camera/front_6mm/image`. The detailed input channel information is in `modules/perception/camera_detection_multi_stage/dag/camera_detection_multi_stage_yolox3d.dag` file.

#### Output

| Name    | Type                                       | Description          | Output channal |
| ------- | ------------------------------------------ | -------------------- | -------------- |
| `frame` | `apollo::perception::onboard::CameraFrame` | camera frame message | /perception/inner/Detection |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/camera_detection_multi_stage/conf/camera_detection_multi_stage_yolox3d_config.pb.txt` file.

#### How to Launch

```bash
cyber_launch start modules/perception/launch/perception_camera_multi_stage.launch
```

## Reference

1. [YOLO: Real-Time Object Detection](https://pjreddie.com/darknet/yolo/)
2. [3D Bounding Box Estimation Using Deep Learning and Geometry](https://arxiv.org/abs/1612.00496)
