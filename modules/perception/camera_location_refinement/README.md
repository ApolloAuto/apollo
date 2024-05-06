# perception-camera-location-refinement

## Introduction

Post process the obstacles detected in the previous stage. It mainly calculates the relative position and height of the
tuning target and the camera by fitting the ground plane in the camera plane

## Directory Structure

```
├── camera_location_refinement  // object location refinement module
    ├── conf                    // module configuration files
    ├── dag                     // dag files
    ├── data                    // model params
    ├── interface               // function interface folder
    ├── proto                   // proto files
    ├── location_refiner        // implementation of algorithm
    ├── camera_location_refinement_component.cc // component interface
    ├── camera_location_refinement_component.h
    ├── cyberfile.xml           // package management profile
    ├── README.md
    └── BUILD
```

## Modules

### CameraLocationRefinementComponent

apollo::perception::camera::CameraLocationRefinementComponent

#### Input

| Name    | Type                                       | Description          | Input channal |
| ------- | ------------------------------------------ | -------------------- | ------------- |
| `frame` | `apollo::perception::onboard::CameraFrame` | camera frame message | /perception/inner/location_estimation |

>Note: The input channel is structure type data. The default trigger channel is `/perception/inner/location_estimation`. The detailed input channel information is in `modules/perception/camera_location_refinement/dag/camera_location_refinement.dag` file. By default, the upstream components of the messages received by the component include `camera_location_estimation`.

#### Output

| Name    | Type                                       | Description          | Output channal |
| ------- | ------------------------------------------ | -------------------- | -------------- |
| `frame` | `apollo::perception::onboard::CameraFrame` | camera frame message | /perception/inner/location_refinement |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/camera_location_refinement/conf/camera_location_refinement_config.pb.txt` file.