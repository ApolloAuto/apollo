# Introduction
The radar detection module detects and tracks obstacles through the radar sensor, then sends them to the multi-sensor fusion module.

The radar detection module receives the radar drive message, which is already include the objects detected and tracked. Even so, the radar module has re-tracked the objects.

# Directory Structure
```
radar_detection
├── BUILD     // bazel build file
├── README.md
├── app       // main process
├── common    // utils
├── conf      // dag config file
├── cyberfile.xml  // package description file
├── dag
├── data      // lib's configuration file
├── interface
├── launch
├── lib       // algorithm library
├── proto     // proto file
├── radar_detection_component.cc  // component
└── radar_detection_component.h
```

# Input and Output
## Input
| Channel              | Type                            | Description         |
| ----------------- | ------------------------------- | -----------------   |
| `/apollo/sensor/radar/front`             | `ContiRadar`        | radar drive message |

## Output
| Channel              | Type                            | Description          |
| ----------------- | ------------------------------- | -------------------- |
| `/perception/inner/PrefusedObjects`           | `onboard::SensorFrameMessage`          | frame contains object detection |

# How to run
In most cases, the radar detection module needs to work with the multi-sensor fusion module. If you want to debug the radar module separately and view the detected obstacle information, you can combine the `msg_adapter` module.

Run the command as follows.
```
cyber_launch start modules/perception/launch/perception_radar.launch
```
