# Introduction
The lane detection module detects lane lines through the camera sensor. Lane lines can be used as an aid to other modules.

# Directory Structure
```
lane_detection
├── BUILD       // bazel build file
├── README.md
├── app         // main process
├── common      // common functions
├── conf        // dag config file
├── cyberfile.xml  // package description file
├── dag
├── data        // lib's configuration file
├── interface
├── lane_detection_component.cc   // component
├── lane_detection_component.h
├── launch
├── lib        // algorithm library
└── proto      // proto file
```

# Input and Output
## Input
| Channel              | Type                            | Description         |
| ----------------- | ------------------------------- | -----------------   |
| `/apollo/sensor/camera/front_6mm/image`             | `drivers::Image`        | camera drive message |
| `/apollo/sensor/camera/front_12mm/image`             | `drivers::Image`        | camera drive message |
| `/apollo/perception/motion_service`             | `MotionServiceMsgType`        | motion service message |

## Output
| Channel              | Type                            | Description          |
| ----------------- | ------------------------------- | -------------------- |
| `/perception/lanes`           | `PerceptionLanes`          | lane line |

# How to run
You can start the lane detection module with the following command.
```
cyber_launch start modules/perception/launch/perception_lane.launch
```
