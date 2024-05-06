# perception-lane-detection

## Introduction

The lane detection module detects lane lines through the camera sensor. Lane lines can be used as an aid to other
modules.

## Directory Structure

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

## Modules

### LaneDetectionComponent

apollo::perception::onboard::LaneDetectionComponent

#### Input

| Channel                                  | Type                                                                        | Description            |
| ---------------------------------------- | --------------------------------------------------------------------------- | ---------------------- |
| `/apollo/sensor/camera/front_6mm/image`  | `apollo::drivers::Image`                                                    | camera drive message   |
| `/apollo/sensor/camera/front_12mm/image` | `apollo::drivers::Image`                                                    | camera drive message   |
| `/apollo/perception/motion_service`      | `apollo::perception::onboard::LaneDetectionComponent::MotionServiceMsgType` | motion service message |

#### Output

| Channel             | Type                                  | Description |
| ------------------- | ------------------------------------- | ----------- |
| `/perception/lanes` | `apollo::perception::PerceptionLanes` | lane line   |

#### How to run

You can start the lane detection module with the following command.

```bash
cyber_launch start modules/perception/launch/perception_lane.launch
```
