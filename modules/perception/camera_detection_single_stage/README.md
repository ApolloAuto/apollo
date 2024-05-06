# perception-camera-detection-single-stage

## Introduction

The camera 3D object detection module includes two models: `caddn` and `smoke`, which can simultaneously output `2d` and
`3d` information at the same time. This module completes operations such as image data preprocessing, detection, and
result postprocessing. This module can directly transfer the results to `camera_tracking` component.

## Directory Structure

```
├── camera_detection_single_stage // camera detect 3d module
    ├── conf            // module configuration files
    ├── dag             // dag files
    ├── data            // model params
    ├── detector        // main part for 3d detector
    │   ├── SMOKE       // SmokeObstacleDetector
    │   └── ...
    ├── interface       // function interface folder
    ├── proto           // proto files
    ├── camera_detection_single_stage_component.cc // component interface
    ├── camera_detection_single_stage_component.h
    ├── cyberfile.xml   // package management profile
    ├── README.md
    └── BUILD
```

## Modules

### CameraDetectionSingleStageComponent

apollo::perception::camera::CameraDetectionSingleStageComponent

#### Input

| Name  | Type                     | Description         | Input channal |
| ----- | ------------------------ | ------------------- | ------------- |
| `msg` | `apollo::drivers::Image` | camera sensor image | /apollo/sensor/camera/front_6mm/image |

>Note: Enter the data type defined by proto. The default trigger camera channel is `/apollo/sensor/camera/front_6mm/image`. The detailed input channel information is in `modules/perception/camera_detection_single_stage/dag/camera_detection_single_stage.dag` file.

#### Output

| Name    | Type                                       | Description          | Output channal |
| ------- | ------------------------------------------ | -------------------- | -------------- |
| `frame` | `apollo::perception::onboard::CameraFrame` | camera frame message | /perception/inner/Detection |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/camera_detection_multi_stage/conf/camera_detection_multi_stage_yolox3d_config.pb.txt` file.

#### How to Launch

```bash
cyber_launch start modules/perception/launch/perception_camera_single_stage.launch
```

## Reference

1. [SMOKE: Single-Stage Monocular 3D Object Detection via Keypoint Estimation](https://arxiv.org/pdf/2002.10111.pdf)
2. [CADDN](https://arxiv.org/abs/2103.01100)
