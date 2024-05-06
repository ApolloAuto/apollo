# perception-traffic-light-tracking

## Introduction

In the recognition stage, due to occlusion or flashing traffic lights, the output state may not be the real state, so
the corresponding result should be corrected. There are many types of corrections: First, in the ReviseBySemantic
function, the situation of identifying multiple traffic lights is corrected. If there are multiple traffic lights, the
color of each traffic light will be identified. If a certain color appears, the color count will be increased by 1. If
there is this If the largest number is unique, then this color will be output, if there are two or more color counts
tied for first, then unknown will be output Second, if the recognition result is red or green, it will be output
directly. If black or unknown is received, the fixer checks the state save list. If the semaphore state has been
determined to persist for some time, then the saved state is output. Otherwise black or unknown output will be output.
Third, because of the chronological relationship, yellow will only appear after green and before red, so to be on the
safe side, any yellow that follows red before green will be set to red.

## Directory Structure

```
├── traffic_light_tracking // trafficlight tracker module
    ├── conf            // module configuration files
    ├── dag             // dag files
    ├── data            // tracking params
    ├── interface       // function interface folder
    ├── launch          // launch files
    ├── proto           // proto files
    ├── tracker       // main part for recogn
    ├── traffic_light_tracking_component.cc // component interface
    ├── traffic_light_tracking_component.h
    ├── cyberfile.xml   // package management profile
    ├── README.md
    └── BUILD
```

## Modules

### TrafficLightTrackComponent

apollo::perception::onboard::TrafficLightTrackComponent

#### Input

| Name    | Type                                                | Description          | Input channal |
| ------- | --------------------------------------------------- | -------------------- | ------------- |
| `frame` | `apollo::perception::onboard::TrafficDetectMessage` | trafficlight message | /perception/inner/Tracking |

>Note: The input channel is structure type data. The default trigger channel is `/perception/inner/Tracking`. The detailed input channel information is in `modules/perception/traffic_light_tracking/dag/traffic_light_tracking.dag` file. By default, the upstream components of the messages received by the component include `traffic_light_recognition`.

#### Output

| Name  | Type                                        | Description                 | Output channal |
| ----- | ------------------------------------------- | --------------------------- | -------------- |
| `msg` | `apollo::perception::TrafficLightDetection` | trafficlight output message | /apollo/perception/traffic_light |

>Note: The output channel is proto type data. The detailed output channel information is in `modules/perception/traffic_light_tracking/conf/traffic_light_tracking_config.pb.txt` file.

#### How to Launch

```bash
cyber_launch start modules/perception/traffic_light_tracking/launch/traffic_light_tracking.launch
```
