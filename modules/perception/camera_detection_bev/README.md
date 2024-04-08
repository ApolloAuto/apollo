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

| Name  | Type                     | Description         | Input channel                                                |
| ----- | ------------------------ | ------------------- | ------------------------------------------------------------ |
| `msg` | `apollo::drivers::Image` | camera sensor image | /apollo/sensor/camera/CAM_BACK/image <br/> /apollo/sensor/camera/CAM_FRONT/image <br/> apollo/sensor/camera/CAM_FRONT_RIGHT/image <br/> /apollo/sensor/camera/CAM_FRONT_LEFT/image <br/> /apollo/sensor/camera/CAM_BACK_LEFT/image <br/>/apollo/sensor/camera/CAM_BACK_RIGHT/image |

>Note: Enter the data type defined by proto. This component triggers the proc function based on the main camera sensor data. The trigger camera channel is `/apollo/sensor/camera/CAM_BACK/image`. The detailed input channel information is in `modules/perception/camera_detection_bev/conf /camera_detection_bev_config.pb.txt` file.

#### Output

| Name    | Type                                       | Description          | Output channal |
| ------- | ------------------------------------------ | -------------------- | -------------- |
| `frame` | `apollo::perception::Obstacles` | camera frame message | /apollo/perception/obstacles  |

>Note: If the receiving data type is a custom class or structure, the new component and the upstream and downstream components must be started in the same process to send and receive messages normally. If the received message is of the data type defined by proto, the upstream and downstream components will be started in different processes and can receive the message normally.

#### How to Launch

```bash
mainboard -d modules/perception/camera_detection_bev/dag/camera_detection_bev.dag
```

## Reference

1. [PETR: Position Embedding Transformation for Multi-View 3D Object Detection](https://arxiv.org/abs/2203.05625)
