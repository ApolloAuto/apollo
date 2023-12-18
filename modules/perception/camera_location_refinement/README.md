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

| Name    | Type                                       | Description          |
| ------- | ------------------------------------------ | -------------------- |
| `frame` | `apollo::perception::onboard::CameraFrame` | camera frame message |

#### Output

| Name    | Type                                       | Description          |
| ------- | ------------------------------------------ | -------------------- |
| `frame` | `apollo::perception::onboard::CameraFrame` | camera frame message |
