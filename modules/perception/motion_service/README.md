# Introduction
The motion service module provides current motion estimation.

# Directory Structure
```
motion_service
├── BUILD      // bazel build file
├── README.md
├── conf       // dag config file
├── cyberfile.xml  // package description file
├── dag
├── launch
├── motion     // main process
├── motion_service_component.cc   // component
├── motion_service_component.h
└── proto      // proto file
```

## Input
| Channel              | Type                            | Description         |
| ----------------- | ------------------------------- | -----------------   |
| `/apollo/sensor/camera/front_6mm/image`             | `ImageMsgType`        | camera drive message |
| `/apollo/localization/pose`             | `LocalizationMsgType`        | localization message |

## Output
| Channel              | Type                            | Description          |
| ----------------- | ------------------------------- | -------------------- |
| `/apollo/perception/motion_service`           | `MotionService`          | motion service message |

# How to run
You can start the lane detection module with the following command.
```
cyber_launch start modules/perception/motion_service/launch/motion_service.launch
```
