# perception-camera-detection-bev

## Introduction

The BEV object detection module is a multi camera `transformer` model trained using `Nuscenes` datasets under the
`paddlepaddle` framework, which can complete the inference of multiple camera data to obtain obstacle targets.

## Directory Structure

```
├── camera_detection_bev // camera bev detector module
    ├── detector         // main part for detector
    │   ├── petr         // BEVObstacleDetector
    │   └── ...
    ├── interface        // function interface folder
    ├── proto            // proto files
    ├── camera_detection_bev_component.cc // component interface
    ├── camera_detection_bev_component.h
    ├── cyberfile.xml    // package management profile
    ├── README.md
    └── BUILD
```

## Modules

### CameraDetectionBevComponent

apollo::perception::camera::CameraDetectionBevComponent

#### Input

| Name  | Type                     | Description         |
| ----- | ------------------------ | ------------------- |
| `msg` | `apollo::drivers::Image` | camera sensor image |

#### Output

| Name    | Type                                       | Description          |
| ------- | ------------------------------------------ | -------------------- |
| `frame` | `apollo::perception::onboard::CameraFrame` | camera frame message |

## Reference

1. [PETR: Position Embedding Transformation for Multi-View 3D Object Detection](https://arxiv.org/abs/2203.05625)
