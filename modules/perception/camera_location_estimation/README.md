# perception-camera-location-estimation

## Introduction

The current module is able to calculate the rotation angle of the target in the camera coordinate system based on the
network predicted 2D target box information, 3D observation angle information, 3D dimension information, and camera
internal parameters Then, combined with the obstacle size template, the module solves the target 3dbbox, and converts
the coordinates to the world coordinate system.

## Directory Structure

```
├── camera_location_estimation  // camera location estimate module
    ├── conf                    // module configuration files
    ├── dag                     // dag files
    ├── data                    // model params
    ├── interface               // function interface folder
    ├── proto                   // proto files
    ├── transformer             // implementation of position estimation algorithm
    │   ├── multicue
    │   └── ...
    ├── camera_location_estimation_component.cc // component interface
    ├── camera_location_estimation_component.h
    ├── cyberfile.xml           // package management profile
    ├── README.md
    └── BUILD
```

## Module

### CameraLocationEstimationComponent

apollo::perception::camera::CameraLocationEstimationComponent

#### Input

| Name    | Type                                       | Description          | Input channal |
| ------- | ------------------------------------------ | -------------------- | ------------- |
| `frame` | `apollo::perception::onboard::CameraFrame` | camera frame message | /perception/inner/Detection |

>Note: The input channel is structure type data. The default trigger channel is `/perception/inner/Detection`. The detailed input channel information is in `modules/perception/camera_location_estimation/dag/camera_location_estimation.dag` file. By default, the upstream components of the messages received by the component include `camera_detection_multi_stage`.

#### Output

| Name    | Type                                       | Description          | Output channal |
| ------- | ------------------------------------------ | -------------------- | -------------- |
| `frame` | `apollo::perception::onboard::CameraFrame` | camera frame message | /perception/inner/location_estimation |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/camera_location_estimation/conf/camera_location_estimation_config.pb.txt` file.

## Reference

1. [3D Bounding Box Estimation Using Deep Learning and Geometry](https://arxiv.org/abs/1612.00496)
