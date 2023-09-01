# perception-camera-detection-3d

## Introduction

The camera 3D object detection module includes two models: `caddn` and `smoke`, which can simultaneously output `2d` and
`3d` information at the same time. This module completes operations such as image data preprocessing, detection, and
result postprocessing. This module can directly transfer the results to `camera_tracking` component.

## Directory Structure

```
├── camera_detection_3d // camera detect 3d module
    ├── conf            // module configuration files
    ├── dag             // dag files
    ├── data            // model params
    ├── detector        // main part for 3d detector
    │   ├── SMOKE       // SmokeObstacleDetector
    │   └── ...
    ├── interface       // function interface folder
    ├── proto           // proto files
    ├── camera_detection_3d_component.cc // component interface
    ├── camera_detection_3d_component.h
    ├── cyberfile.xml   // package management profile
    ├── README.md
    └── BUILD
```

## Modules

### CameraDetection3dComponent

apollo::perception::camera::CameraDetection3dComponent

#### Input

| Name  | Type                     | Description         |
| ----- | ------------------------ | ------------------- |
| `msg` | `apollo::drivers::Image` | camera sensor image |

#### Output

| Name    | Type                                       | Description          |
| ------- | ------------------------------------------ | -------------------- |
| `frame` | `apollo::perception::onboard::CameraFrame` | camera frame message |

#### How to Launch

```bash
cyber_launch start modules/perception/launch/perception_camera_3d.launch
```

## Reference

1. [SMOKE: Single-Stage Monocular 3D Object Detection via Keypoint Estimation](https://arxiv.org/pdf/2002.10111.pdf)
2. [CADDN](https://arxiv.org/abs/2103.01100)
